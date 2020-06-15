#include "ch.h"
#include "hal.h"
#include "at24_eep.h"
#include "simplelink_if.h"
#include "sensorhub.h"
#include "adxl355_dev.h"
#include "adxl355_config.h"
#include "adxl355_cmd.h"
#include "bmi160.h"
#include "bmi160_config.h"
#include "bmi160_cmd.h"
#include "ad7124.h"
#include "ad7124_defs.h"
#include "ad7124_config.h"
#include "ad7124_cmd.h"

#include "filesys.h"
#include "sysParam.h"
#include "exti_cfg.h"
#include "battery_mang.h"
#include "htu2x.h"
#include "peripheral_if.h"
#include "ff.h"
#include "time.h"
#include "comm_if.h"
#include "osi_chibios.h"
#include "serialDriverj.h"

thread_t *mainThread;
thread_t *thdSensorHub;
thread_t *shelltp;
thread_t *shelltp_sd2;
thread_reference_t sensorhub_trp = NULL;
app_param_t appParam;

//static ADXL355Driver adxl;
#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(512)

void tcpConnect(void)
{
  chEvtSignal(thdSensorHub,EV_MAIN_CLIENT_CONNECT);
}

void tcpDisconnect(void)
{
  chEvtSignal(thdSensorHub,EV_MAIN_CLIENT_DISCONNECT);
}

bool serialTxReady(_serial_driver_j *dev)
{
  chBSemSignal(&dev->sem);  
}

bool serialReceived(_serial_driver_j *dev)
{
  size_t sz = dev->pCurRxPtr - dev->rxBuf;
  if(appParam.connState == 0x1){
    if(sz >= CMD_STRUCT_SZ){
      cmd_header_t *header = (cmd_header_t*)dev->rxBuf;
      uint8_t resp[128];

      
      if(header->magic1 != MAGIC1  || header->magic2 != MAGIC2 || !(header->type & MASK_CMD) || header->len < CMD_STRUCT_SZ){
        SDJResetBuffer(dev);
        return false;
      }
      
      if(header->len != sz){
        SDJResetBuffer(dev);
        return false;
      }    
      // apply checksum check if command pid is non-zero
      if(!(header->pid & CMD2_NOCRC_MASK)){
        uint16_t chksum = cmd_checksum(dev->rxBuf,header->len);
        if(header->chksum != chksum){
          SDJResetBuffer(dev);
          return false;
        }
      }
    
      int retSz;
      if((retSz=commif_parse(dev->rxBuf,resp))>0){
        SDJWriteEx(dev,resp,retSz);
      }
      SDJResetBuffer(dev);
      return true;
    }
  }
  

  return true;
}

static const I2CConfig i2ccfg = {
  OPMODE_I2C,
  100000,
  STD_DUTY_CYCLE,
};

static SerialConfig serialcfg = {
  115200,
//  0,
//  0,
//  (1 << 9) |  (1 << 8)      // CTS enable RTS enable 
};

static SerialConfig serialcfg_hs = {
  460800,
//  0,
//  0,
//  (1 << 9) |  (1 << 8)      // CTS enable RTS enable 
};

static SPIConfig spicfg_bmi160 = {
  NULL,
  GPIOA,
  15,
  SPI_CR1_BR_2 | SPI_CR1_BR_1 |SPI_CR1_BR_0
};

static SPIConfig spicfg_ad7124 = {
  NULL,
  NULL,
  NULL,
  SPI_CR1_BR_1 //| SPI_CR1_CPOL | SPI_CR1_CPHA
};
static SPIConfig spicfg_adxl355 = {
  NULL,
  NULL,
  NULL,
  SPI_CR1_BR_1
};

static ad7124_dev_t ad7124;
static _serial_driver_j SDJ1;

void app_start_config();



void bufferValid(uint8_t out)
{
  if(appParam.sdBuffer.length >= appParam.sdBuffer.flushSize){
    appParam.sdBuffer.szToWrite = appParam.sdBuffer.length;
    appParam.sdBuffer.transition++;
    if(appParam.sdBuffer.id == 0){
      appParam.sdBuffer.r = appParam.sdBuffer.buf_a;
      appParam.sdBuffer.w = appParam.sdBuffer.buf_b+CMD_STRUCT_SZ;
      appParam.sdBuffer.length = 0;
      appParam.sdBuffer.id = 1;
    }else{
      appParam.sdBuffer.w = appParam.sdBuffer.buf_a+CMD_STRUCT_SZ;
      appParam.sdBuffer.r = appParam.sdBuffer.buf_b;
      appParam.sdBuffer.length = 0;
      appParam.sdBuffer.id = 0;
    }
    chEvtSignal(appParam.fSThread,EV_SD_WRITE_0);
  }

}


void app_loadDefault()
{
  memcpy((uint8_t*)&moduleParam.adxl355cfg,(uint8_t*)&default_cfg,sizeof(adxl355_config_t));
  memcpy((uint8_t*)&moduleParam.adcConfig.channel,(uint8_t*)&default_channel,sizeof(adc_channel_t)*AD7124_MAX_CHANNEL);
  memcpy((uint8_t*)&moduleParam.adcConfig.config,(uint8_t*)&default_config,sizeof(adc_channel_config_t)*AD7124_NOF_SETUP_CONFIG);
  moduleParam.adcConfig.powermode = AD7124_FULL_POWER;
  
  // bmi160
  memcpy((uint8_t*)&moduleParam.bmi160,(uint8_t*)&bmi160_cfg,sizeof(struct bmi160_dev));
  
}

void app_initConfig(void)
{
  // ad7124
  uint8_t i;
  uint16_t fs;
  adc_channel_t *ch;
  adc_channel_config_t *cfg;
  appParam.maxCh = -1;
  ad7124.config->vBiasMask = 0x0;
  for(i=0;i<8;i++){
    ch = &moduleParam.adcConfig.channel[i];
    cfg = &moduleParam.adcConfig.config[i];
    
    ad7124.config->channels[i].u.enable = ch->enable;
    ad7124.config->channels[i].u.ainp = ch->inp;
    ad7124.config->channels[i].u.ainm = ch->inn;
    ad7124.config->channels[i].u.setup = ch->configId;
    ad7124.config->channels[i].u.rsv = 0;

    ad7124.config->setups[i].u.pga = cfg->pga;
    ad7124.config->setups[i].u.ref_sel = cfg->refSource;
    ad7124.config->setups[i].u.ain_bufm = ADC_BIT_DISABLE;
    ad7124.config->setups[i].u.ain_bufp = ADC_BIT_DISABLE;
    ad7124.config->setups[i].u.ref_bufm = ADC_BIT_DISABLE;
    ad7124.config->setups[i].u.ref_bufp = ADC_BIT_DISABLE;
    ad7124.config->setups[i].u.burnout = ADC_BURNOUT_OFF;
    ad7124.config->setups[i].u.bipolar = ADC_BIT_ENABLE;
    
    ad7124.config->filters[i].u.filter = cfg->filter;
    ad7124.config->filters[i].u.single_cycle = ADC_BIT_ENABLE;
    ad7124.config->filters[i].u.post_filter = 0;
    
    switch(moduleParam.adcConfig.powermode){
    case 0: // low power
      fs = ((75800/cfg->samplerate)-95)>>7;
      break;
    case 1: // mid power
      fs = ((153600/cfg->samplerate)-95)>>7;
      break;
    case 2: // full power
      fs = ((614400/cfg->samplerate)-95)>>7;
      break;
    default:
      fs = ((614400/cfg->samplerate)-95)>>7;
    }

    ad7124.config->filters[i].u.fs = fs;
    
    if(ch->enable){
      appParam.maxCh++;
      appParam.chList[appParam.maxCh] = i;
    }
    
    // vbias mask
    if(ch->driveConfig[2].driveRating == 1){
      ad7124.config->vBiasMask |= (1 << i);
    }
  }
  appParam.maxCh++;
  ad7124.config->iDrv[0].iout = ADC_IOUT_500UA;
  ad7124.config->iDrv[0].idrv_pin = 7;
  ad7124.config->iDrv[1].iout = ADC_IOUT_OFF;
  
  ad7124.control.u.power_mode = moduleParam.adcConfig.powermode;
 
}

enum{
  BT_INIT,
  BT_GET_MAC,
  BT_SET_NAME,
  BT_SET_BAUD
};

static THD_FUNCTION(procBTConnect,p)
{
  _serial_driver_j *dev = (_serial_driver_j*)p;
  uint8_t state = BT_INIT;
  bool isRunning = true;
  msg_t msg = MSG_RESET;
  uint8_t buf[64];
  int16_t ret;
  do{
    switch(state){
    case BT_INIT:
      palSetPadMode(GPIOA,4,PAL_MODE_INPUT_PULLUP); // bt state out 1: connected, 0: not connect
      palSetPadMode(GPIOA,5,PAL_MODE_INPUT_PULLUP); // bt mode out 1: active, 0: sleep
      palSetPadMode(GPIOA,6,PAL_MODE_OUTPUT_PUSHPULL); // bt boot0
      palSetPadMode(GPIOA,7,PAL_MODE_OUTPUT_PUSHPULL); // bt reset
      // remap spiD3 to spiD1 in case of the dma channel restriction
      palSetPadMode(GPIOB,3,PAL_STM32_MODE_ALTERNATE | PAL_STM32_ALTERNATE(5)); // SCK
      palSetPadMode(GPIOB,4,PAL_STM32_MODE_ALTERNATE | PAL_STM32_ALTERNATE(5)); // MOSI
      palSetPadMode(GPIOB,5,PAL_STM32_MODE_ALTERNATE | PAL_STM32_ALTERNATE(5)); // MISO
      
      palClearPad(GPIOA,6);// keep boot-0 low
      // reset bt
      palClearPad(GPIOA,7);
      chThdSleepMilliseconds(50);
      palSetPad(GPIOA,7);
//      chThdSleepSeconds(4);
      // lock wait BT module start
      msg = chBSemWaitTimeout(&dev->sem,S2ST(4));
      if(msg == MSG_OK){
        state = BT_GET_MAC;
      }else{
        isRunning = false;
      }
      break;
    case BT_GET_MAC:
      // clear buffer
      dev->pCurRxPtr = dev->rxBuf;
      SDJWriteEx(dev,"AT+AB GetBDAddress\r\n",20);
      msg = chBSemWaitTimeout(&dev->sem,S2ST(4));
      if(msg == MSG_OK){
        state = BT_SET_NAME;
      }else{
        isRunning = false;
      }
      break;
    case BT_SET_NAME:
      ret = SDJReadEx(dev,buf,64);
      if(ret){
        char macAddr[16];
        sscanf(buf,"AT-AB BDAddress %s\r\n",macAddr);
        ret = chsnprintf(buf,64,"AT+AB LocalName %s-%s\r\n",moduleParam.hub.ssidPrefix,macAddr);
      }
      else{
        ret = chsnprintf(buf,64,"AT+AB LocalName %s-%08x\r\n",moduleParam.hub.ssidPrefix,
                 moduleParam.param.serialNum);
      }
      dev->pCurRxPtr = dev->rxBuf;
      SDJWriteEx(dev,buf,ret);
      msg = chBSemWaitTimeout(&dev->sem,S2ST(4));
      if(msg == MSG_OK){
        state = BT_SET_BAUD;
      }else{
        isRunning = false;
      }
      break;
    case BT_SET_BAUD:
      ret = chsnprintf(buf,64,"AT+AB ChangeBaud 460800\r\n");
      dev->pCurRxPtr = dev->rxBuf;
      SDJWriteEx(dev,buf,ret);
      msg = chBSemWaitTimeout(&dev->sem,S2ST(4));
      msg = MSG_OK;
      isRunning = false;
      // clear input buffer
      ret = SDJReadEx(dev,buf,64);
      SDJChangeSpeed(dev,460800);
      break;
    }
    
  }while(isRunning);
  
  chThdExit(msg);
}

typedef void (*if_start)(void);
if_start f;
static uint8_t rxBuf[64],txBuf[256];
void sensorHubIfInit(void)
{
  sensorhub_param_t *hub = &moduleParam.hub;
  
  POWER_IF_ENABLE();
  uint8_t resp[64];
  int sz;
  thread_t *btConf;
  //hub->commType = COMM_USE_BT;
  // interface initialization
  switch(hub->commType){
  case COMM_USE_BT:
    SDJ1.uart = &UARTD2;
    SDJ1.config.speed = 115200;
    SDJ1.thread = NULL;
    SDJ1.rxBuf = rxBuf;
    SDJ1.txBuf = txBuf;
    SDJ1.receivedCB = serialReceived;
    SDJ1.emptyCB = serialTxReady;
    
    SDJInit_ex(&SDJ1,NULL,0);
//    btConf = chThdCreateStatic(waBt,sizeof(waBt),NORMALPRIO,procBTConnect,&SDJ1);
    btConf = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(256),"BTCONF",NORMALPRIO,procBTConnect,&SDJ1);
    msg_t msg = chThdWait(btConf);
    // enbale bt-connection interrupt
    extChannelEnable(&EXTD1,4);
    
    break;
  case COMM_USE_WIFI:
    osi_simplinkInit(tcpConnect,tcpDisconnect);
    break;
  }
}
void sensorHubInit(void)
{
  spiStop(&SPID2);
  if(moduleParam.hub.commType == COMM_USE_BT)
    spiStop(&SPID1);
  else
    spiStop(&SPID3);
  
  POWER_P7_ENABLE();
  POWER_P8_ENABLE();

  for(uint8_t i=0;i<6;i++){
    switch(moduleParam.hub.sensor[i]){
    case DEV_AD7124:
      if((i==0) || (i == 1)){
        adspi = &SPID2;
        ad7124.chipSel = spi2_cs_0;
      }
      else if((i==2) || (i==3)){
        if(moduleParam.hub.commType == COMM_USE_BT)
          adspi = &SPID1;
        else
          adspi = &SPID3;
        ad7124.chipSel = spi3_cs_0;
      }
      else
        adspi = NULL;
      
      if(adspi){
        spiStart(adspi,&spicfg_ad7124);
        if(ad7124_init(&ad7124) == AD7124_OK){
          ad7124_cmd_init(&moduleParam.ad7124cfg);
          appParam.sensor[i] = DEV_AD7124;
        }else{
          appParam.sensor[i] = DEV_NONE;
        }
      }else{
        appParam.sensor[i] = DEV_NONE;
      }
      break;
    case DEV_ADXL355:
      if((i==0) || (i == 1)){
        adxlspi = &SPID2;
        adxl355.chipSel = spi2_cs_0;
      }
      else if((i==2) || (i==3)){
        adxl355.chipSel = spi3_cs_0;
        if(moduleParam.hub.commType == COMM_USE_BT)
          adxlspi = &SPID1;
        else
          adxlspi = &SPID3;
      }
      else
        adxlspi = NULL;
      if(adxlspi){
        spiStart(adxlspi,&spicfg_adxl355);
        if(adxl355_init(&adxl355) == ADXL355_OK){
          //adxl355_cmd_init(&moduleParam.adxl355cfg);
          appParam.sensor[i] = DEV_ADXL355;
        }else{
          appParam.sensor[i] = DEV_NONE;
        }
      }else{
        appParam.sensor[i] = DEV_NONE;
      }
      break;
    case DEV_BMI160:
      if((i==0) || (i == 1))
        bmispi = &SPID2;
      else if((i==2) || (i==3)){
        if(moduleParam.hub.commType == COMM_USE_BT)
          bmispi = &SPID1;
        else
          bmispi = &SPID3;
      }
      else
        bmispi = NULL;
      if(bmispi){
        bmi160.read = bmi160_read;
        bmi160.write = bmi160_write;
        bmi160.delay_ms = bmi160_delay;
        bmi160.prev_accel_cfg.power = 0;
        bmi160.prev_gyro_cfg.power = 0;
        spiStart(bmispi,&spicfg_bmi160);

        if(bmi160_cmd_init(&bmi160) == BMI160_OK){
          appParam.sensor[i] = DEV_BMI160;
          bmi160_cmd_testRead(&bmi160);
        }else{
          appParam.sensor[i] = DEV_NONE;
        }
      }else{
        appParam.sensor[i] = DEV_NONE;
      }
      break;
    case DEV_HTU21D:
      htu2xinit(&I2CD1);
      appParam.sensor[i] = DEV_HTU21D;
      break;
        
    }
  }
}

static virtual_timer_t vt;

static void timeout_cb(void *arg)
{
  chSysLockFromISR();
  chEvtSignalI(mainThread,EV_ACT_BATT);
  chVTSetI(&vt,S2ST(1),timeout_cb,NULL);
  chSysUnlockFromISR();
}

static void blinker_cb(void *arg)
{
  chSysLockFromISR();
  if(appParam.ledBlinkPeriod == 0){
    LED_OFF();
  }else if(appParam.ledBlinkPeriod == 0xff){
    LED_ON();
  }else{
    LED_TOG();
    chVTSetI(&appParam.blinker,MS2ST(appParam.ledBlinkPeriod*10),blinker_cb,NULL);
  }
  chSysUnlockFromISR();
}


uint32_t intCntr = 0;

void validLogFileName(void)
{
  chsnprintf(appParam.log_file.name,64,"%s_%04d_%02d_%02d_%02d-%02d-%02d.csv\0",
             moduleParam.sdcfg.prefix.log,
             appParam.tim_now.tm_year+1900,
             appParam.tim_now.tm_mon+1,
             appParam.tim_now.tm_mday,
             appParam.tim_now.tm_hour,
             appParam.tim_now.tm_min,
             appParam.tim_now.tm_sec);
  
  FRESULT fres;
  fres = f_open(&appParam.log_file.f,appParam.log_file.name,FA_WRITE | FA_CREATE_NEW);
  if(fres != FR_OK){
    while(1);
  }
  appParam.data_file.lastSync = appParam.data_file.f.fptr;
}
void validDataFileName(void)
{
  chsnprintf(appParam.data_file.name,64,"%s_%04d_%02d_%02d_%02d-%02d-%02d.bin\0",
             moduleParam.sdcfg.prefix.data,
             appParam.tim_now.tm_year+1900,
             appParam.tim_now.tm_mon+1,
             appParam.tim_now.tm_mday,
             appParam.tim_now.tm_hour,
             appParam.tim_now.tm_min,
             appParam.tim_now.tm_sec);
  
  FRESULT fres;
  fres = f_open(&appParam.data_file.f,appParam.data_file.name,FA_READ | FA_WRITE | FA_CREATE_NEW);
  if(fres != FR_OK){
    while(1);
  }
  
  fres = f_lseek(&appParam.data_file.f,512);
  appParam.data_file.lastSync = appParam.data_file.f.fptr;
}
void validDataFileNameHI(void)
{
  chsnprintf(appParam.data_file_hi.name,64,"%s_%04d_%02d_%02d_%02d-%02d-%02d.bin\0",
             moduleParam.sdcfg.prefix.data_hr,
             appParam.tim_now.tm_year+1900,
             appParam.tim_now.tm_mon+1,
             appParam.tim_now.tm_mday,
             appParam.tim_now.tm_hour,
             appParam.tim_now.tm_min,
             appParam.tim_now.tm_sec);
  FRESULT fres;
  fres = f_open(&appParam.data_file_hi.f,appParam.data_file_hi.name,FA_WRITE | FA_CREATE_NEW);
  if(fres != FR_OK){
    while(1);
  }
  
  fres = f_lseek(&appParam.data_file_hi.f,512);
  appParam.data_file_hi.lastSync = appParam.data_file_hi.f.fptr;
}

static THD_WORKING_AREA(waSerial,2048);
static THD_FUNCTION(procSerial ,p)
{
  cmd_header_t *header;
  int16_t sz;

  //reset buffer ptr
  appParam.rbuf.ptr = appParam.rbuf.buffer;
  while(true){
    if(chThdShouldTerminateX()){
      break;
    }
    
    if(SDJReadTimeout(appParam.rbuf.ptr,BUFFER_SZ,50) < CMD_STRUCT_SZ){
      continue;
    }
    header = (cmd_header_t*)appParam.rbuf.ptr;
    if(header->magic1 != MAGIC1  || header->magic2 != MAGIC2 || !(header->type & MASK_CMD) || header->len < CMD_STRUCT_SZ){
      continue;
    }
    
    appParam.rbuf.sz = header->len;
    // apply checksum check if command pid is non-zero
    if(!(header->pid & CMD2_NOCRC_MASK)){
      uint16_t chksum = cmd_checksum(appParam.rbuf.buffer,header->len);
      if(header->chksum != chksum){
        appParam.rbuf.ptr = appParam.rbuf.buffer;
        appParam.rbuf.sz = 0;
        continue;
      }
    }
    header->pid &= ~CMD2_NOCRC_MASK;
    if((sz = commif_parse((char*)appParam.rbuf.buffer,(char*)appParam.rbuf.buffer))){
      SDJWrite(appParam.rbuf.buffer,sz);     
      appParam.rbuf.sz = 0;
      appParam.rbuf.ptr = appParam.rbuf.buffer;
    }
  }  
  chThdExit(MSG_OK);
}


enum{
  REPORT_NONE,
  REPORT_HT,
  REPORT_BATT
};

void closeDataHir()
{
  f_close(&appParam.data_file_hi.f);
}

void closeVSSLogFile()
{
  char buf[512];
  char *ptr = buf;
  size_t sz;
  memset(buf,0,512);
  ptr += chsnprintf(ptr,512,"VSS LOG FILE REV 1.0\n");
  switch(moduleParam.bmi160.accel_cfg.range){
  case BMI160_ACCEL_RANGE_2G:
    ptr += chsnprintf(ptr,512,"ACCEL.RANGE=2G\n");
    break;
  case BMI160_ACCEL_RANGE_4G:
    ptr += chsnprintf(ptr,512,"ACCEL.RANGE=4G\n");
    break;
  case BMI160_ACCEL_RANGE_8G:
    ptr += chsnprintf(ptr,512,"ACCEL.RANGE=8G\n");
    break;
  case BMI160_ACCEL_RANGE_16G:
    ptr += chsnprintf(ptr,512,"ACCEL.RANGE=16G\n");
    break;
  }
  switch(moduleParam.bmi160.gyro_cfg.range){
  case BMI160_GYRO_RANGE_2000_DPS:
    ptr += chsnprintf(ptr,512,"GYRO.RANGE=2000 DPS\n");
    break;
  case BMI160_GYRO_RANGE_1000_DPS:
    ptr += chsnprintf(ptr,512,"GYRO.RANGE=1000 DPS\n");
    break;
  case BMI160_GYRO_RANGE_500_DPS:
    ptr += chsnprintf(ptr,512,"GYRO.RANGE=500 DPS\n");
    break;
  case BMI160_GYRO_RANGE_250_DPS:
    ptr += chsnprintf(ptr,512,"GYRO.RANGE=250 DPS\n");
    break;
  case BMI160_GYRO_RANGE_125_DPS:
    ptr += chsnprintf(ptr,512,"GYRO.RANGE=125 DPS\n");
    break;
  }
  switch(moduleParam.bmi160.accel_cfg.odr){
  case BMI160_ACCEL_ODR_25HZ:
    ptr += chsnprintf(ptr,512,"DATA RATE=25 SPS\n");
    break;
  case BMI160_ACCEL_ODR_50HZ:
    ptr += chsnprintf(ptr,512,"DATA RATE=50 SPS\n");
    break;
  case BMI160_ACCEL_ODR_100HZ:
    ptr += chsnprintf(ptr,512,"DATA RATE=100 SPS\n");
    break;
  case BMI160_ACCEL_ODR_200HZ:
    ptr += chsnprintf(ptr,512,"DATA RATE=200 SPS\n");
    break;
  case BMI160_ACCEL_ODR_400HZ:
    ptr += chsnprintf(ptr,512,"DATA RATE=400 SPS\n");
    break;
  case BMI160_ACCEL_ODR_800HZ:
    ptr += chsnprintf(ptr,512,"DATA RATE=800 SPS\n");
    break;
  case BMI160_ACCEL_ODR_1600HZ:
    ptr += chsnprintf(ptr,512,"DATA RATE=1600 SPS\n");
    break;
  }
  
  DWORD Size;
  
  // flush remaining buffers
  if(appParam.sdBuffer.length){
    char *ptr = appParam.sdBuffer.id==0?appParam.sdBuffer.buf_a:appParam.sdBuffer.buf_b;
    ptr += 8;
    f_write(&appParam.data_file.f,ptr,appParam.sdBuffer.length,&Size);
  }
  
  f_lseek(&appParam.data_file.f,0);
  f_write(&appParam.data_file.f,buf,512,&Size);
  f_close(&appParam.data_file.f);
  
}

void closeLogFile()
{
  f_close(&appParam.log_file.f);
}

void report(uint8_t target)
{
  FRESULT fres;
  DWORD Size;
  DWORD fsize;
  double dv1,dv2;
  uint8_t buf[32];
  cmd_header_t *header = (cmd_header_t*)buf;
  switch(target){
  case REPORT_HT:
    dv1 = appParam.temp;
    dv2 = appParam.humidity;
    if((appParam.dataPath & DATA_PATH_LOGSD)
        && (appParam.sdSize > 0)){
      char logMsg[64];
      chsnprintf(logMsg,64,"%04d/%02d/%02d,%02d:%02d:%02d,%08.3f,%08.3f\n",
                 appParam.tim_now.tm_year+1900,
                 appParam.tim_now.tm_mon+1,
                 appParam.tim_now.tm_mday,
                 appParam.tim_now.tm_hour,
                 appParam.tim_now.tm_min,
                 appParam.tim_now.tm_sec,
                 dv1,
                 dv2);
      //chMtxLock(&appParam.mtx_fs);
      //int fsize = writeLogFile(appParam.fileName.log,logMsg,strlen(logMsg));
      f_write(&appParam.log_file.f,logMsg,strlen(logMsg),&Size);
      //chMtxUnlock(&appParam.mtx_fs);
      fsize = appParam.log_file.f.obj.objsize;
      if((fsize > 0) && (fsize > moduleParam.sdcfg.szConstrain.log_size)){
        closeLogFile();
        validLogFileName();
      }
    }
      if(appParam.dataPath & DATA_PATH_TRANMIT){
        if(moduleParam.hub.mode == MODE_VSS){
  //        header = (cmd_header_t*)appParam.wbuf.ptr;
          header->magic1 = MAGIC1;
          header->magic2 = MAGIC2;
          header->type = MASK_DATA | DATA_ENV;
          header->len = 16 + CMD_STRUCT_SZ;
          header->pid = 0;
          // send packet direct
         
  //        memcpy((appParam.wbuf.ptr+CMD_STRUCT_SZ),(uint8_t*)&dv1,8);
  //        memcpy((appParam.wbuf.ptr+CMD_STRUCT_SZ+8),(uint8_t*)&dv2,8);
          memcpy((buf+CMD_STRUCT_SZ),(uint8_t*)&dv1,8);
          memcpy((buf+CMD_STRUCT_SZ+8),(uint8_t*)&dv2,8);
          header->chksum = cmd_checksum(buf,header->len);
          //appParam.wbuf.ptr += 24;
          //appParam.wbuf.sz += 24;
          //bufferValid(0);
          SDJWrite(buf,24);
        }
      }
    
    break;
  case REPORT_BATT:
      if(appParam.dataPath & DATA_PATH_TRANMIT){
        if(moduleParam.hub.mode == MODE_VSS){
          //header = (cmd_header_t*)appParam.wbuf.ptr;
          header->magic1 = MAGIC1;
          header->magic2 = MAGIC2;
          header->type = MASK_DATA | DATA_BATTERY;
          header->len = CMD_STRUCT_SZ;
          header->pid = appParam.battery.soc;
          header->chksum = cmd_checksum(buf,header->len);
          SDJWrite(buf,8);
          //appParam.wbuf.ptr += CMD_STRUCT_SZ;
          //appParam.wbuf.sz += CMD_STRUCT_SZ;
          //bufferValid(0);
        }
      }
    break;
  //default:
  }
  //chMtxUnlock(&appParam.mtx);
}
static THD_WORKING_AREA(waWriteFS,1024);
static THD_FUNCTION(procWriteFS ,p)
{
  FIL f;
  FRESULT fres;
  DWORD Size;
  cmd_header_t h;
  cmd_header_t *header = &h;
  char msg[512];
  while(!chThdShouldTerminateX()){
    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
//    if((appParam.dataPath & DATA_PATH_LOGSD) && (appParam.sdSize > 0) ){
    if((appParam.sdSize > 0) ){
      fres = f_write(&appParam.data_file.f,(appParam.sdBuffer.r+ CMD_STRUCT_SZ),appParam.sdBuffer.szToWrite,&Size);
      f_sync(&appParam.data_file.f);
      Size = appParam.data_file.f.obj.objsize;
      if((Size > 0) && (Size > moduleParam.sdcfg.szConstrain.data_size)){
        chMtxLock(&appParam.mtx_fs);
        closeVSSLogFile();
        chMtxUnlock(&appParam.mtx_fs);
        validDataFileName();
      }
    }
  }
  chThdRelease(appParam.fSThread);
  appParam.fSThread = NULL;
  chThdExit((msg_t)0);
}

//static THD_WORKING_AREA(waFsWrite,1024);
//static THD_FUNCTION(procFsWrite ,p)
//{
//  FRESULT fres;
//  DWORD Size;
//  char msg[512];
//  //thread_t *t;
//  while(!chThdShouldTerminateX()){
//    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
//    fres = f_write(&appParam.data_file.f,appParam.sdBuffer.r,SD_BUFFER_SIZE,&Size);
//    
//    if(fres != FR_OK)
//      while(1);
//    if(Size != SD_BUFFER_SIZE)
//      while(1);
//    f_sync(&appParam.data_file.f);
//    
//  }
//  chThdRelease(appParam.fSThread);
//  appParam.fSThread = NULL;
//  chThdExit((msg_t)0);
//}

/**
  working thread
*/

void app_stop_config()
{
  switch(moduleParam.hub.mode){
  case MODE_DOORSPEED:
    extChannelDisable(&EXTD1,14);
    ad7124_stop(&ad7124);
    break;
  case MODE_VSS_HI:
    extChannelDisable(&EXTD1,1);
    adxl355_powerdown(&adxl355);
    adxl355.config->intmask = 0;
    adxl355_set_interrupt(&adxl355);
    closeDataHir();
    closeLogFile();
    break;
  case MODE_VSS:
  default:
    extChannelDisable(&EXTD1,7);
    bmi160_cmd_deconfig_int(&bmi160);
    closeVSSLogFile();
    closeLogFile();
    break;
  }
}

void app_start_config(void)
{
  uint16_t odr;
  //appParam.wbuf.ptr = appParam.wbuf.buffer;
  //appParam.wbuf.pendSz = appParam.wbuf.sz = 0;    
  switch(moduleParam.hub.mode){
  case MODE_DOORSPEED:
    appParam.pid[DATA] = 0;
    // doorspeed, 1200 sample/second
    // 10-byte (6-byte inertial & 4-byte adc) data per record.
    // 10006 byte/second
    // for 50 ms/packet, flush size = 10106/20 = 500 bytes
    if(appParam.dataPath == DATA_PATH_TRANMIT)
      appParam.sdBuffer.flushSize = 250;
    else
      appParam.sdBuffer.flushSize = 250;
    // start adc in contineoue mode, BMI160 in single read 
    // mode triggered by ADC
    app_initConfig();
    ad7124_init(&ad7124);
    // force BMI160 to 800 Hz
    bmi160.accel_cfg.odr = 0xb;
    // update sensor config
    bmi160_reconfig_by_app(&bmi160);
    
    extChannelEnable(&EXTD1,14);
    ad7124_setmode(AD7124_MODE_CONTINUE,&ad7124);
    break;
  case MODE_VSS_HI:
    if(appParam.dataPath == DATA_PATH_TRANMIT){
      appParam.sdBuffer.flushSize = (10 - adxl355.config->outputrate) * 90;
      if(appParam.sdBuffer.flushSize == 0) appParam.sdBuffer.flushSize = 90;
      if(appParam.sdBuffer.flushSize > 900) appParam.sdBuffer.flushSize = 900;
    }
    else{
      appParam.sdBuffer.flushSize = 250;
      validDataFileNameHI();
    }
    appParam.pid[DATA_HIR] = 0;
    adxl355_powerdown(&adxl355);
    adxl355.config->intmask = ADXL355_INT_FULL_EN1;
    adxl355_set_interrupt(&adxl355);
    adxl355_set_filter(&adxl355);
    adxl355_set_full_scale(&adxl355);
    extChannelEnable(&EXTD1,1);
    adxl355_powerup(&adxl355);
  break;
  case MODE_VSS_V2:
    adxl355_powerdown(&adxl355);
    adxl355.config->intmask = ADXL355_INT_FULL_EN1;
    adxl355_set_interrupt(&adxl355);
    adxl355_set_filter(&adxl355);
    adxl355_set_full_scale(&adxl355);
    extChannelEnable(&EXTD1,1);
    adxl355_powerup(&adxl355);
    appParam.pid[DATA_HIR] = 0;
  case MODE_VSS:
  default:
    
    odr = (1<<(moduleParam.bmi160.accel_cfg.odr - 6))*25;
    odr /= 20; // 20 Hz packet rate
    if(appParam.dataPath == DATA_PATH_TRANMIT){
      appParam.sdBuffer.flushSize = 600;
      appParam.pid[DATA] = 0;
      appParam.sdBuffer.flushSize = odr * 12;
      if(appParam.sdBuffer.flushSize > 600)
        appParam.sdBuffer.flushSize = 600;
    }
    else{
      appParam.sdBuffer.flushSize = SD_BUFFER_SIZE - CMD_STRUCT_SZ;
      validLogFileName();
      validDataFileName();
    }

    // update sensor config
    bmi160_reconfig_by_app(&bmi160);
    // start bmi160 in fifo mode
    bmi160_cmd_config_int(&bmi160);
    extChannelEnable(&EXTD1,7);
  }
}

//static       union bmi160_int_status interrupt;
static THD_WORKING_AREA(waWorking,512);
static THD_FUNCTION(procWorking ,p)
{
  cmd_header_t *header;
  size_t sz;
  int32_t data[96];
  uint8_t *p_src,*p_dst;
  uint16_t bsz;
  systime_t t_start;
  appParam.sdBuffer.w = appParam.sdBuffer.buf_a+CMD_STRUCT_SZ;
  appParam.sdBuffer.length = 0;
  appParam.sdBuffer.id = 0;
  appParam.sdBuffer.transition = 0;
  app_start_config();
  while(!chThdShouldTerminateX()){
    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
    if(evt & EV_ADXL_FIFO_FULL){
      intCntr++;
      // read fifo data
      size_t sz; 
      //adxl.vmt_adxl355->get_fifo_size(&adxl,&sz);
      adxl355_get_fifo_size(&adxl355,&sz);
      sz = (sz/3)*3;
      if(sz){
        bsz = sz * 3;
        adxl355_read_fifo(&adxl355,bsz);
        memcpy((void*)appParam.sdBuffer.w,adxl355.buffer,bsz);
        appParam.sdBuffer.length += bsz;
        appParam.sdBuffer.w += bsz;
      }      
    }
    if(evt & EV_BMI_INT1){
      bmi160_get_fifo_data(&bmi160);
      int8_t v;
      uint16_t j;
      appParam.lig_file_size[2] = bmi160.fifo->length;
      appParam.lig_file_size[1] += bmi160.fifo->length;
      // copy data to temperary buffer, signal transfer thread
      // to decide write to SD card or transfer via communication interface
      
      memcpy((void*)appParam.sdBuffer.w,bmi160.fifo->data,bmi160.fifo->length);
      appParam.sdBuffer.length += bmi160.fifo->length;
      appParam.sdBuffer.w += bmi160.fifo->length;
    }
    if(evt & EV_P7_EXINT){
      ad7124_int_handler(&ad7124);
    }
    
    // send data
    if(appParam.sdBuffer.length >= appParam.sdBuffer.flushSize){
      appParam.sdBuffer.szToWrite = appParam.sdBuffer.length;
      appParam.sdBuffer.transition++;
      if(appParam.sdBuffer.id == 0){
        appParam.sdBuffer.r = appParam.sdBuffer.buf_a;
        appParam.sdBuffer.w = appParam.sdBuffer.buf_b+CMD_STRUCT_SZ;
        appParam.sdBuffer.length = 0;
        appParam.sdBuffer.id = 1;
      }else{
        appParam.sdBuffer.w = appParam.sdBuffer.buf_a+CMD_STRUCT_SZ;
        appParam.sdBuffer.r = appParam.sdBuffer.buf_b;
        appParam.sdBuffer.length = 0;
        appParam.sdBuffer.id = 0;
      }
      if(appParam.dataPath == DATA_PATH_TRANMIT){
        // prepare data transmit
        header = (cmd_header_t*)appParam.sdBuffer.r;
        header->magic1 = MAGIC1;
        header->magic2 = MAGIC2;
        switch(moduleParam.hub.mode){
        case MODE_VSS:
          header->type = MASK_DATA | DATA_VSS;
          break;
        case MODE_DOORSPEED:
          header->type = MASK_DATA | DATA_DOORSPEED;
          break;
        case MODE_VSS_HI:
          header->type = MASK_DATA | DATA_VSS_HI;
          break;
        }
        header->len = appParam.sdBuffer.szToWrite + CMD_STRUCT_SZ;
        header->pid = appParam.pid[0]++;
        if(moduleParam.hub.commType == COMM_USE_WIFI){
          //appParam.sock.msgLen = appParam.sdBuffer.szToWrite;
          socketSend2(appParam.sdBuffer.r,header->len);
        }
        else if(moduleParam.hub.commType == COMM_USE_BT){
          SDJWriteEx2(&SDJ1,appParam.sdBuffer.r,appParam.sdBuffer.szToWrite+ CMD_STRUCT_SZ);
        }     
      }else{
        chEvtSignal(appParam.fSThread,(msg_t)1);
      }
    }
  }
  app_stop_config(); 
  //chThdRelease(appParam.workingThread);
  appParam.workingThread = NULL;
  chThdExit((msg_t)0);  
}

static void startWorker(void)
{
  appParam.workingThread = chThdCreateStatic(waWorking,sizeof(waWorking),NORMALPRIO+1,procWorking,NULL); 
  if((appParam.ledBlinkPeriod == 0) || (appParam.ledBlinkPeriod == 0xff)){
    appParam.ledBlinkPeriod = 50;
    chVTSet(&appParam.blinker,MS2ST(appParam.ledBlinkPeriod*10),blinker_cb,NULL);
  }
  

}

static void stopWorker(void)
{
  if(appParam.workingThread){
    chThdTerminate(appParam.workingThread);
    //chEvtSignal(appParam.workingThread,0x8000);
    chThdWait(appParam.workingThread);
//    if(chThdTerminatedX(appParam.workingThread))
//      appParam.workingThread = NULL;
  }
  appParam.ledBlinkPeriod = 0;
}


static void startSerialIF(void)
{
  appParam.serialThread = chThdCreateStatic(waSerial,sizeof(waSerial),NORMALPRIO,procSerial,NULL);
}

static void stopSerialIF(void)
{
  if(appParam.serialThread){
    chThdTerminate(appParam.serialThread);
    chThdWait(appParam.serialThread);
    if(chThdTerminatedX(appParam.serialThread))
      appParam.serialThread = NULL;
  }
}


static THD_WORKING_AREA(waSensorHub,2048);
static THD_FUNCTION(procSensorHub ,p)
{
  uint8_t msg[64];
 //uint8_t buf[96];
  uint16_t odr;
  battInit(&I2CD1,thdSensorHub);
  cmd_header_t *header;
//  battSetCapacity(2000);

  chThdResume(&sensorhub_trp,MSG_OK);
  chVTObjectInit(&vt);
  chVTSet(&vt,S2ST(1),timeout_cb,NULL);
  chVTObjectInit(&appParam.blinker);
  appParam.ledBlinkPeriod = 0xff;
  //chVTSet(&appParam.blinker,MS2ST(appParam.ledBlinkPeriod),blinker_cb,NULL);
   extChannelEnable(&EXTD1,8);

  while(true){
    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
    if(evt & EV_CMD_EXTINT){
      
    }
    if(evt & EV_ACT_BATT){
      RTCDateTime timespec;
      rtcGetTime(&RTCD1,&timespec);
      rtcConvertDateTimeToStructTm(&timespec,&appParam.tim_now,NULL);
      appParam.runSecs++;
      if(!appParam.workingThread){
        battPollState();
        appParam.temp = sen_htu2xx_read_temp();
        appParam.humidity = sen_htu2xx_read_humidity();
      }

      //if(appParam.dataPath){
#if 0
        if(moduleParam.hub.ht_report_interval){
          if((appParam.runSecs % moduleParam.hub.ht_report_interval) == 0){
            report(REPORT_HT);
          }
        }
        if(moduleParam.battery.reportInterval){
          if((appParam.runSecs % moduleParam.battery.reportInterval) == 0){
            report(REPORT_BATT);
          }
        }
#endif
      //}
      if(palReadPad(GPIOB,8) == PAL_LOW){
        appParam.user_pressed_sec += appParam.user_press_inc;
        if(appParam.user_pressed_sec > 3){
          appParam.user_press_inc = 0;
          appParam.user_pressed_sec = 0;
          if(appParam.workingThread)
            continue;     
          else{
            appParam.user_cfg_mode = (appParam.user_cfg_mode==0)?1:0;
            if(appParam.user_cfg_mode == 0){ // leave wifi/bt config mode
              sysSaveParams(); 
            }
            LED_OFF();
            for(uint8_t f=0;f<3;f++){
              LED_ON();
              chThdSleepMilliseconds(250);
              LED_OFF();
              chThdSleepMilliseconds(250);
            }
          }
        }
      }else{
        appParam.user_press_inc = 0;
        appParam.user_pressed_sec = 0;
      }
    }
    
    if(evt & EV_CMD_READ_BATT){
      battPollState();
      header = (cmd_header_t*)msg;
      header->magic1 = MAGIC1;
      header->magic2 = MAGIC2;
      header->type = MASK_DATA | DATA_BATTERY;
      header->len = CMD_STRUCT_SZ;
      header->pid = appParam.battery.soc;
      header->chksum = cmd_checksum(appParam.wbuf.ptr,header->len);
      SDJWrite(msg,header->len);
    }
    
    if(evt & EV_CMD_READ_ENV){
      header = (cmd_header_t*)msg;
      header->magic1 = MAGIC1;
      header->magic2 = MAGIC2;
      header->type = MASK_DATA | DATA_ENV;
      header->len = 16 + CMD_STRUCT_SZ;
      header->pid = 0;
      // send packet direct
     
      //appParam.temp = sen_htu2xx_read_temp();
      //appParam.humidity = sen_htu2xx_read_humidity();
      memcpy((msg+CMD_STRUCT_SZ),(uint8_t*)&appParam.temp,8);
      memcpy((msg+CMD_STRUCT_SZ+8),(uint8_t*)&appParam.humidity,8);
      header->chksum = cmd_checksum(msg,header->len);
      SDJWrite(msg,header->len);      
    }

    if(evt & EV_CMD_SINGLE){
      if(appParam.workingThread) return;
      switch(appParam.singleRead){
      case SENSOR_AD7124:
        break;
      case SENSOR_ADXL355:
        break;
      case SENSOR_BMI160:
        bmi160_cmd_acquire_one(&bmi160,msg);
        //sdWrite(&SD2,msg,12);
        break;
      case SENSOR_HTU21D:
        break;
      }
    }
    if(evt & EV_USER_BUTTON_0){
      chThdSleepMilliseconds(500);
      if(palReadPad(GPIOB,8) == PAL_HIGH){
        if(appParam.user_cfg_mode){
          moduleParam.hub.commType = (moduleParam.hub.commType == COMM_USE_BT)?COMM_USE_WIFI:COMM_USE_BT;
          LED_OFF();
          for(uint8_t f=0;f<(moduleParam.hub.commType+1);f++){
            LED_ON();
            chThdSleepMilliseconds(500);
            LED_OFF();
            chThdSleepMilliseconds(500);
          }
          
        }
        else{
          if(appParam.workingThread)
            evt |= EV_CMD_STOP;
          else{
            evt |= EV_CMD_RUN;
            appParam.dataPath = DATA_PATH_LOGSD;
          }
        }
      }
      else{
        appParam.user_press_inc = 1;
      }
    }
    // run command received
    if(evt & EV_CMD_RUN){
      if(appParam.workingThread)
        continue;
        startWorker();
    }
    
    if(evt & EV_CMD_STOP){
      stopWorker();
      if(appParam.dataPath == DATA_PATH_TRANMIT){
        // send response packet
        cmd_header_t *h_resp = (cmd_header_t*)msg;
        h_resp->magic1 = MAGIC1;
        h_resp->magic2 = MAGIC2;
        h_resp->type = MASK_CMD_RET_OK;
        h_resp->pid = 0;
        h_resp->len = CMD_STRUCT_SZ;    
        h_resp->chksum = cmd_checksum(appParam.wbuf.buffer,h_resp->len);
        SDJWrite(msg,CMD_STRUCT_SZ);
      }
    }
    
    if(evt & EV_BUFFER_FULL){
      //bufFlush();
      bufferValid(0);
    }   
    
    if(evt & EV_P7_EXINT){
    }
    
    if(evt & EV_SAVE_PARAM){
      sysSaveParams();
      sensorHubInit();
    }
    if(evt & EV_LOAD_PARAM){
      app_loadDefault();
      defaultParams();
      sysSaveParams();
      sensorHubInit();
    }
    if(evt & EV_BMI160_CMD_FOC){
      bmi160_cmd_startfoc(&bmi160);
    }
    if(evt & EV_CMD_RESET){
      chSysDisable();
      NVIC_SystemReset();
    }

    if(evt & EV_SDMMC_INSERT){
      sdCheck();
      if(!appParam.sdSize)
        appParam.ledBlinkPeriod = 0xff;
      extChannelEnable(&EXTD1,9);  
    }
    
    if(evt & EV_MAIN_CLIENT_CONNECT){
      appParam.connState = 0x1;
    }
    
    if(evt & EV_MAIN_CLIENT_DISCONNECT){
      if(appParam.dataPath == DATA_PATH_TRANMIT){
        appParam.connState = 0x0;
        // stop working thread if any
        if(appParam.workingThread){
          stopWorker();
        }
      }
    }
    
    if(evt & EV_BT_EXTINT){
      if(palReadPad(GPIOA,4) == PAL_HIGH){ // connected
        SDJResetBuffer(&SDJ1);
        appParam.connState = 0x1;
      }else{
        appParam.connState = 0;
        //if(appParam.serialThread){
          if(appParam.dataPath == DATA_PATH_TRANMIT){
            // stop working thread if any
            if(appParam.workingThread){
              stopWorker();
            }
          }
          //stopSerialIF();
            
        //}
      }
      extChannelEnable(&EXTD1,4);

    }
  }
  
}



int main(void)
{
  halInit();
  chSysInit();
  
  chThdSleepMilliseconds(50);
  i2cStart(&I2CD1,&i2ccfg);
  //i2cStart(&I2CD2,&i2ccfg);
  POWER_IF_DISABLE();
  POWER_P7_DISABLE();
  POWER_P8_DISABLE();
  
 
  at24eep_init(&I2CD1,32,1024,0x50,2);
  sysParamInit();

  if(palReadPad(GPIOB,8) == PAL_LOW){
    uint8_t cntr = 0;
    for(cntr=0;cntr<100;cntr++){
      if(palReadPad(GPIOB,8) == PAL_HIGH){
        continue;
      }      
      chThdSleepMilliseconds(100);
    }
    if(cntr > 30){ // press > 3 sec
      defaultParams();
      sysSaveParams();
      LED_OFF();
      for(uint8_t f=0;f<3;f++){
        LED_ON();
        chThdSleepMilliseconds(250);
        LED_OFF();
        chThdSleepMilliseconds(250);
      }
      NVIC_SystemReset();
    }
  }
  


  moduleParam.param.verNum = SW_VERSION_NUMBER;
  extStart(&EXTD1,&extcfg);

  extChannelEnable(&EXTD1,9);  
  sdcStart(&SDCD1,NULL);
  sdCheck();
  //sdStop(&SD2);

  sensorHubIfInit();
  //chThdSleepMilliseconds(100);
  sensorHubInit();
  //appParam.stream = NULL;
  appParam.activeFile.dirChanged = 0;
  appParam.activeFile.path[0]='/';
  appParam.activeFile.path[1] = 0x0;
  appParam.dataPath = 0;
  appParam.activeFile.dirChanged = 1;

  appParam.user_pressed_sec = 0;
  appParam.user_press_inc = 0;
  appParam.user_cfg_mode = 0;
  
  chMtxObjectInit(&appParam.mtx);
  chMtxObjectInit(&appParam.mtx_fs);
 
  thdSensorHub = chThdCreateStatic(waSensorHub,sizeof(waSensorHub),NORMALPRIO,procSensorHub,NULL);
  mainThread = thdSensorHub;

  //appParam.workingThread = chThdCreateStatic(waBufferHandle,sizeof(waBufferHandle),NORMALPRIO-1,procBufferHandler,NULL);
  appParam.fSThread = chThdCreateStatic(waWriteFS,sizeof(waWriteFS),NORMALPRIO-1,procWriteFS,NULL);
  
  // lock until thread finishing initial task
  chSysLock();
  chThdSuspendS(&sensorhub_trp);
  chSysUnlock();
  
  commif_init(mainThread);
  while(1){
    chThdSleepMilliseconds(100);
  }
   
  
}

void adxl_int_handler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  if(appParam.workingThread)
    chEvtSignalI(appParam.workingThread, EV_ADXL_FIFO_FULL);
  chSysUnlockFromISR();
}

void bmi160_int1_handler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  if(appParam.workingThread)
    chEvtSignalI(appParam.workingThread,EV_BMI_INT1);
  chSysUnlockFromISR();  
}
void bmi160_int2_handler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  if(appParam.workingThread)
    chEvtSignalI(appParam.workingThread,EV_BMI_INT1);
  chSysUnlockFromISR();  
}


void adc_int_handler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  extChannelDisableI(&EXTD1,14);
  if(appParam.workingThread)
    chEvtSignalI(appParam.workingThread,EV_P7_EXINT);
  chSysUnlockFromISR();
}

void adcEnableInterupt(void *p)
{
    extChannelEnable(&EXTD1,14);  
}

int8_t ad7124_conversion_done(void *p)
{
#define g (struct AD7124_dev*)p
  int8_t ret = AD7124_OK;
  /**
    doorspeed use single channel adc and gyro scope,
    acquire one data from BMI160
  */
  if(moduleParam.hub.mode == MODE_DOORSPEED){
    memcpy(appParam.sdBuffer.w, (uint8_t*)&((ad7124_dev_t*)p)->results[0],4);
    appParam.sdBuffer.w += 4;
    appParam.sdBuffer.length += 4;
    bmi160_get_regs(BMI160_GYRO_DATA_ADDR, appParam.sdBuffer.w, 12, &bmi160);
    appParam.sdBuffer.w += 12;
    appParam.sdBuffer.length += 12;
  }
  
  return ret;
#undef g
}




int8_t adxl355_conversion_done(struct ADXL355_dev *p)
{
  
  return 0;
}
int8_t adxl355_buffer_cb(struct ADXL355_dev *p)
{
 
  return 0;
}

void user_button_handler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  chEvtSignalI(thdSensorHub,EV_USER_BUTTON_0);
  chSysUnlockFromISR();  
}
void sdmmc_int_handler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  extChannelDisableI(&EXTD1,9);
  chEvtSignalI(thdSensorHub,EV_SDMMC_INSERT);
  chSysUnlockFromISR();
}

void btlink_int_handler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  extChannelDisableI(&EXTD1,4);
  chEvtSignalI(thdSensorHub,EV_BT_EXTINT);
  chSysUnlockFromISR();
}

