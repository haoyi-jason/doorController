#include "ch.h"
#include "hal.h"
#include "sysParam.h"
#include "sensorhub.h"
#include "comm_if.h"
#include "battery_mang.h"
#include "fileSys.h"

static thread_t *mainThread=0;

uint16_t cmd_checksum(uint8_t *data, uint16_t length)
{
    uint16_t i;
    uint16_t sum = 0;

    for (i = 0; i < length; i++){
      if((i==6) || (i==7)) continue;
      sum += data[i];
    }
    return sum;
}

int16_t parseControlCmd(char *in, char *out)
{
  cmd_header_t *header = (cmd_header_t*)in;
  cmd_header_t *h_resp = (cmd_header_t*)out;
  h_resp->magic1 = header->magic1;
  h_resp->magic2 = header->magic2;
  h_resp->pid = header->pid;
  h_resp->len = CMD_STRUCT_SZ;
  //header->pid &= 0x7f;  // mask NOCRC mask
  uint8_t cmd = header->pid & 0x7f;
  appParam.dataPath = cmd;
  switch(cmd){
  case CMD2_CONTROL_STOP:
    h_resp->len = 0;
    h_resp->type = MASK_CMD_RET_OK;
    chEvtSignal(mainThread,EV_CMD_STOP);
    break;
  case CMD2_CONTROL_LOG:
  case CMD2_CONTROL_START:
  //case (CMD2_CONTROL_LOG | CMD2_CONTROL_START):
    h_resp->type = MASK_CMD_RET_OK;
    chEvtSignal(mainThread, EV_CMD_RUN);
    break;
  case CMD2_CONTROL_READ_SOC:
    h_resp->type = MASK_CMD_RET_OK;
    chEvtSignal(mainThread, EV_CMD_READ_BATT);
    break;
  case CMD2_CONTROL_READ_ENV:
    h_resp->type = MASK_CMD_RET_OK;
    chEvtSignal(mainThread, EV_CMD_READ_ENV);
    break;
  default:
    h_resp->type = MASK_CMD_RET_ERR;
  }
  
  return h_resp->len;
}

int16_t parseSetupCmd(char *in, char *out)
{
  RTCDateTime timespec;
  struct tm tim;
  cmd_header_t *header = (cmd_header_t*)in;
  cmd_header_t *h_resp = (cmd_header_t*)out;
  char *resp = (char*)(out + CMD_STRUCT_SZ);
  char *data = (char*)(in + CMD_STRUCT_SZ);
  h_resp->magic1 = header->magic1;
  h_resp->magic2 = header->magic2;
  h_resp->pid = header->pid;
  h_resp->type = MASK_CMD_RET_CFG | CMD_CMD1_SETUP;  
  header->pid &= 0x0f;
  h_resp->pid = header->pid;
  

  switch(header->pid){
  case CMD2_SETUP_ACC_HI:
    if(header->len == 8){
      h_resp->type = MASK_CMD_RET_CFG | CMD_CMD1_SETUP;  
      resp[0] = moduleParam.adxl355cfg.outputrate;
      resp[1] = moduleParam.adxl355cfg.fullscale-1;
      resp[2] = moduleParam.adxl355cfg.highpassfilter>>4;
      h_resp->len = 3;        
    }else{
      switch(data[0]){
      case 0x1:
        moduleParam.adxl355cfg.outputrate =(adxl355_odr_t)(data[1]);
        h_resp->len = 0;
        break;
      case 0x2:
        moduleParam.adxl355cfg.fullscale = (adxl355_fs_t)(data[1]+1);
        h_resp->len = 0;
        break;
      case 0x3:
        moduleParam.adxl355cfg.highpassfilter = (adxl355_hpf_t)(data[1]<<4);
        h_resp->len = 0;
        break;
      case 4: // set rate/range/hpf at once
        moduleParam.adxl355cfg.outputrate = (adxl355_odr_t)(data[1]+1);
        moduleParam.adxl355cfg.fullscale = (adxl355_fs_t)data[2];
        moduleParam.adxl355cfg.highpassfilter = (adxl355_hpf_t)(data[3]<<4);
        h_resp->len = 0;
        break;
      default:
        h_resp->type = MASK_CMD_RET_ERR | CMD_CMD1_SETUP;  
        h_resp->len = 0;
      }
    }
    break;
  case CMD2_SETUP_INERTIAL:
    if(header->len == 8){
      h_resp->type = MASK_CMD_RET_CFG | CMD_CMD1_SETUP;  
      resp[0] = moduleParam.bmi160.accel_cfg.odr;
      switch(moduleParam.bmi160.accel_cfg.range){
      case 0x03:
        resp[1] = 0;
        break;
      case 0x05:
        resp[1] = 1;
        break;
      case 0x08:
        resp[1] = 2;
        break;
      case 0x0c:
        resp[1] = 3;
        break;
      }
      resp[2] = moduleParam.bmi160.gyro_cfg.odr;
      resp[3] = moduleParam.bmi160.gyro_cfg.range;
      h_resp->len = 4;        
    }else{
      switch(data[0]){
      case 0x1:
        moduleParam.bmi160.accel_cfg.odr = data[1];
        moduleParam.bmi160.gyro_cfg.odr = data[2];
        h_resp->len = 0;
        break;
      case 0x2:
        switch(data[1]){
        case 0:
          moduleParam.bmi160.accel_cfg.range = 0x3;
          break;
        case 1:
          moduleParam.bmi160.accel_cfg.range = 0x5;
          break;
        case 2:
          moduleParam.bmi160.accel_cfg.range = 0x8;
          break;
        case 3:
          moduleParam.bmi160.accel_cfg.range = 0xc;
          break;
        }
        moduleParam.bmi160.gyro_cfg.range = data[2];
        h_resp->len = 0;
        break;
      case 0x3:
        switch(data[2]){
        case 0:
          moduleParam.bmi160.accel_cfg.range = 0x3;
          break;
        case 1:
          moduleParam.bmi160.accel_cfg.range = 0x5;
          break;
        case 2:
          moduleParam.bmi160.accel_cfg.range = 0x8;
          break;
        case 3:
          moduleParam.bmi160.accel_cfg.range = 0xc;
          break;
        }
        moduleParam.bmi160.accel_cfg.odr = data[1];
        moduleParam.bmi160.gyro_cfg.odr = data[3];
        moduleParam.bmi160.gyro_cfg.range = data[4];
        h_resp->len = 0;
        break;
      case 0xff:
        //chEvtSignal(mainThread,EV_BMI160_CMD_FOC);
        h_resp->len = 0;
        break;
      default:
        h_resp->type = MASK_CMD_RET_ERR | CMD_CMD1_SETUP;  
        h_resp->len = 0;
      }
    }
    break;
  case CMD2_SETUP_ADC:
    if(header->len == 9){ // query data
      h_resp->type = MASK_CMD_RET_CFG | CMD_CMD1_SETUP;  
      resp[0] = data[0];
      switch(data[0]){
      case 0x1: // enable/disable & setup number
        for(uint8_t i=0;i<8;i++){
          resp[i+1] = moduleParam.adcConfig.channel[i].configId;
          if(moduleParam.adcConfig.channel[i].enable)
            resp[i+1] |= 0x80;
        }
        h_resp->len = 9;
        break;
      case 0x2:// channel pin, P/N
        for(uint8_t i=0;i<8;i++){
          resp[i*2+2] = moduleParam.adcConfig.channel[i].inp;
          resp[i*2+3] = moduleParam.adcConfig.channel[i].inn;
        }
        h_resp->len = 17;
        break;
      case 0x3: // current 0/1 drive pad & strength
        //for(uint8_t i=0;i<8;i++){
          for(uint8_t j=0;j<2;j++){
            resp[2+j*2] = moduleParam.adcConfig.channel[data[1]].driveConfig[j].drivePad;
            resp[2+j*2+1] = moduleParam.adcConfig.channel[data[1]].driveConfig[j].driveRating;
          }
        //}
        h_resp->len = 5;
        break;
      case 0x4:
        for(uint8_t i=0;i<8;i++){
          resp[i*4+1] = moduleParam.adcConfig.config[i].samplerate & 0xff;
          resp[i*4+2] = (moduleParam.adcConfig.config[i].samplerate >> 8);
          resp[i*4+3] = moduleParam.adcConfig.config[i].pga;
          resp[i*4+4] = moduleParam.adcConfig.config[i].refSource;
        }
        h_resp->len = 33;
        break;
      case 0x5:
        resp[2] = moduleParam.adcConfig.powermode;
        h_resp->len = 2;
        break;
      }
    }else{
      switch(data[0]){
      case 1: // channel register
        for(uint8_t i=0;i<8;i++){
          if(data[i+1] & 0x80)
            moduleParam.adcConfig.channel[i].enable = 1;
          else
            moduleParam.adcConfig.channel[i].enable = 0;
          moduleParam.adcConfig.channel[i].configId = data[i+1] & 0xf;
        }
        h_resp->type = MASK_CMD_RET_OK;  
        h_resp->len = 0;
        break;
      case 2: // input pin config
        moduleParam.adcConfig.channel[data[1]].inp = data[2];
        moduleParam.adcConfig.channel[data[1]].inn = data[3];
        h_resp->type = MASK_CMD_RET_OK;  
        h_resp->len = 0;
        break;
      case 3: // iout  
        moduleParam.adcConfig.channel[data[1]].driveConfig[0].drivePad = data[2];
        moduleParam.adcConfig.channel[data[1]].driveConfig[0].drivePad = data[3];
        moduleParam.adcConfig.channel[data[1]].driveConfig[1].drivePad = data[4];
        moduleParam.adcConfig.channel[data[1]].driveConfig[1].drivePad = data[5];
        h_resp->type = MASK_CMD_RET_OK;  
        h_resp->len = 0;
        break;
      case 4:
        moduleParam.adcConfig.config[data[1]].samplerate = (data[2] | (data[3]<<8));
        moduleParam.adcConfig.config[data[1]].pga = data[4];
        moduleParam.adcConfig.config[data[1]].refSource = data[5];
        h_resp->type = MASK_CMD_RET_OK;  
        h_resp->len = 0;
        break;
      case 5:
        moduleParam.adcConfig.powermode = data[2];
        h_resp->type = MASK_CMD_RET_OK;  
        h_resp->len = 0;
        break;
      default:
        h_resp->type = MASK_CMD_RET_ERR | CMD_CMD1_SETUP;  
        h_resp->len = 0;
      }
    }
    break;
  case CMD2_SETUP_HT:
    if(header->len == 8){
      h_resp->type = MASK_CMD_RET_CFG | CMD_CMD1_SETUP;
      h_resp->pid = CMD2_SETUP_HT;
      resp[0] = moduleParam.hub.ht_report_interval>>8;
      resp[1] = moduleParam.hub.ht_report_interval&0xff;
      h_resp->len = 8;
    }
    else{
      switch(data[0]){
      case 0x1:
        moduleParam.hub.ht_report_interval = (data[1]<<8 | data[2]);
        h_resp->type = MASK_CMD_RET_OK | CMD_CMD1_SETUP;
        break;
      default:
        h_resp->type = MASK_CMD_RET_ERR | CMD_CMD1_SETUP;
      }
      h_resp->len = 0;
    }
    break;
  case CMD2_SETUP_BATT:
    if(header->len == 8){
      h_resp->type = MASK_CMD_RET_CFG | CMD_CMD1_SETUP;
      h_resp->pid = CMD2_SETUP_BATT;
      resp[0] = appParam.battery.capacity >> 8;
      resp[1] = appParam.battery.capacity & 0xff;
      resp[2] = appParam.battery.capRemain >> 8;
      resp[3] = appParam.battery.capRemain & 0xff;
      resp[4] = appParam.battery.sealed;
      resp[5] = appParam.battery.chgState;
      resp[6] = appParam.battery.voltage >> 8;
      resp[7] = appParam.battery.voltage & 0xff;
      h_resp->len = 8;
    }else{
      switch(data[0]){
      case 0x1:
        if(data[1] == 0)
          battStopCharge();
        else
          battStartCharge();
        break;
      case 0x3:
        battSetCapacity((data[1]<<8 | data[2]));
        h_resp->type = MASK_CMD_RET_OK | CMD_CMD1_SETUP;
          break;
      default:
        h_resp->type = MASK_CMD_RET_ERR | CMD_CMD1_SETUP;
      }
      h_resp->len = 0;
    }
    break;
  case CMD2_SETUP_COMM:
    break;
  case CMD2_SETUP_SYS:
    if(header->len == 9){
      h_resp->type = MASK_CMD_RET_CFG | CMD_CMD1_SETUP;
      h_resp->pid = CMD2_SETUP_SYS;
      switch(data[0]){
      case CMD2_SYS_TYPE:
        resp[0] = data[0];
        resp[1] = moduleParam.hub.sensor[0];
        resp[2] = moduleParam.hub.sensor[1];
        resp[3] = moduleParam.hub.sensor[2];
        resp[4] = moduleParam.hub.sensor[3];
        resp[5] = moduleParam.hub.sensor[4];
        resp[6] = moduleParam.hub.sensor[5];
        resp[7] = appParam.sensor[0];
        resp[8] = appParam.sensor[1];
        resp[9] = appParam.sensor[2];
        resp[10] = appParam.sensor[3];
        resp[11] = appParam.sensor[4];
        resp[12] = appParam.sensor[5];
        h_resp->len = 13;
        break;
      case CMD2_SYS_COMM:
        resp[0] = data[0];
        resp[1] = moduleParam.hub.commType;
        memcpy(&resp[2],moduleParam.hub.ssidPrefix,16);
        h_resp->len = 18;
        break;
      case CMD2_SYS_RTC:
        rtcGetTime(&RTCD1,&timespec);
        rtcConvertDateTimeToStructTm(&timespec,&tim,NULL);
        resp[0] = data[0];
        resp[1] = tim.tm_year - 100;
        resp[2] = tim.tm_mon + 1;
        resp[3] = tim.tm_mday;
        resp[4] = tim.tm_hour;
        resp[5] = tim.tm_min;
        resp[6] = tim.tm_sec;
        h_resp->len = 7;
        break;
      case CMD2_SYS_MODE:
        resp[0] = data[0];
        resp[1] = moduleParam.hub.mode;
        h_resp->len = 2;
        break;
      case CMD2_SYS_SD_SAVE:
        resp[0] = data[0];
        resp[1] = moduleParam.sdcfg.savdSd;
        h_resp->len = 2;
        break;
      case CMD2_SYS_SD_DP:
        resp[0] = data[0];
        memcpy(&resp[1],moduleParam.sdcfg.prefix.data,32);
        h_resp->len = 33;
        break;
      case CMD2_SYS_SD_DPH:
        resp[0] = data[0];
        memcpy(&resp[1],moduleParam.sdcfg.prefix.data_hr,32);
        h_resp->len = 33;
        break;
      case CMD2_SYS_SD_DPL:
        resp[0] = data[0];
        memcpy(&resp[1],moduleParam.sdcfg.prefix.log,32);
        h_resp->len = 33;
        break;
      case CMD2_SYS_SD_DATA:
        resp[0] = data[0];
        memcpy(&resp[1],(uint8_t*)&moduleParam.sdcfg.szConstrain.data_size,4);
        memcpy(&resp[5],(uint8_t*)&moduleParam.sdcfg.szConstrain.data_hr_size,4);
        memcpy(&resp[9],(uint8_t*)&moduleParam.sdcfg.szConstrain.log_size,4);
        h_resp->len = 13;
        break;
      case CMD2_SETUP_AP_PASS:
        resp[0] = data[0];
        memcpy(&resp[1],moduleParam.hub.passwdAP,16);
        h_resp->len = 17;
        break;
      case CMD2_SETUP_STA_SSID:
        resp[0] = data[0];
        memcpy(&resp[1],moduleParam.hub.ssid,32);
        h_resp->len = 33;
        break;
      case CMD2_SETUP_STA_ST:
        resp[0] = data[0];
        resp[1] = moduleParam.hub.secType;
        break;
      case CMD2_SETUP_STA_PASS:
        resp[0] = data[0];
        memcpy(&resp[1],moduleParam.hub.passwd,16);
        h_resp->len = 17;
        break;
      case CMD2_SETUP_STA_TIMEOUT:
        resp[0] = data[0];
        resp[1] = moduleParam.hub.connectionTimeout;
        h_resp->len = 2;
        break;
      case CMD2_SETUP_WIFI_TYPE:
        resp[0] = data[0];
        resp[1] = moduleParam.hub.boot;
        h_resp->len = 2;
        break;
      case CMD2_SETUP_VERNUM:
        resp[0] = data[0];
        memcpy(&resp[1],(uint8_t*)&moduleParam.param.verNum,4);
        h_resp->len = 5;
        break;
      case CMD2_SETUP_SERNUM:
        resp[0] = data[0];
        memcpy(&resp[1],(uint8_t*)&moduleParam.param.serialNum,4);
        h_resp->len = 5;
        break;
      case CMD2_SETUP_USRSTR:
        resp[0] = data[0];
        memcpy(&resp[1],moduleParam.param.user,32);
        h_resp->len = 33;
        break;
      case CMD2_SETUP_VNDSTR:
        resp[0] = data[0];
        memcpy(&resp[1],moduleParam.param.vender,32);
        h_resp->len = 33;
        break;
      default:
        h_resp->type = MASK_CMD_RET_ERR | CMD_CMD1_SETUP;
        h_resp->len = 0;
      }
      
    }else{
      h_resp->type = MASK_CMD_RET_OK | CMD_CMD1_SETUP;
      switch(data[0]){
      case CMD2_SYS_TYPE:
        for(uint8_t i=0;i<6;i++)
          moduleParam.hub.sensor[i] = (dev_type_t)data[i+1];
        break;
      case CMD2_SYS_COMM:
        h_resp->type = MASK_CMD_RET_OK | CMD_CMD1_SETUP;
        moduleParam.hub.commType = (comm_use_t)data[1];
        memcpy(&moduleParam.hub.ssidPrefix,&data[2],header->len-10);
        moduleParam.hub.ssidPrefix[header->len-10]=0;
        break;
      case CMD2_SYS_RTC:
        h_resp->type = MASK_CMD_RET_OK | CMD_CMD1_SETUP;
        h_resp->len = CMD_STRUCT_SZ;
        tim.tm_year = data[1] + 100;
        tim.tm_mon = data[2]-1;
        tim.tm_mday = data[3];
        tim.tm_hour = data[4];
        tim.tm_min = data[5];
        tim.tm_sec = data[6];
        rtcConvertStructTmToDateTime(&tim,0,&timespec);
        rtcSetTime(&RTCD1,&timespec);
        break;
      case CMD2_SYS_MODE:
        moduleParam.hub.mode = (sensor_func_t)data[1];
        break;
      case CMD2_SYS_SD_SAVE:
        moduleParam.sdcfg.savdSd = data[1];
        break;
      case CMD2_SYS_SD_DP:
        memcpy(&moduleParam.sdcfg.prefix.data,&data[1],header->len-9);
        break;
      case CMD2_SYS_SD_DPH:
        memcpy(&moduleParam.sdcfg.prefix.data_hr,&data[1],header->len-9);
        break;
      case CMD2_SYS_SD_DPL:
        memcpy(&moduleParam.sdcfg.prefix.log,&data[1],header->len-9);
        break;
      case CMD2_SYS_SD_DATA:
        memcpy(&moduleParam.sdcfg.szConstrain.data_size,&data[1],4);
        memcpy(&moduleParam.sdcfg.szConstrain.data_hr_size,&data[5],4);
        memcpy(&moduleParam.sdcfg.szConstrain.log_size,&data[9],4);
        break;
      case CMD2_SETUP_AP_PASS:
        memcpy(&moduleParam.hub.passwdAP,&data[1],header->len - CMD_STRUCT_SZ-1);
        break;
      case CMD2_SETUP_STA_SSID:
        memcpy(&moduleParam.hub.ssid,&data[1],header->len - CMD_STRUCT_SZ-1);
        break;
      case CMD2_SETUP_STA_ST:
        moduleParam.hub.secType = data[1];
        break;
      case CMD2_SETUP_STA_PASS:
        memcpy(&moduleParam.hub.passwd,&data[1],header->len - CMD_STRUCT_SZ-1);
        break;
      case CMD2_SETUP_STA_TIMEOUT:
        moduleParam.hub.connectionTimeout = data[1];
        break;
      case CMD2_SETUP_WIFI_TYPE:
        moduleParam.hub.boot = (wifi_boot_t)data[1];
        break;
      case CMD2_SETUP_VERNUM:
        memcpy(&moduleParam.param.verNum,&data[1],header->len - CMD_STRUCT_SZ-1);
        break;
      case CMD2_SETUP_SERNUM:
        memcpy(&moduleParam.param.serialNum,&data[1],header->len - CMD_STRUCT_SZ-1);
        break;
      case CMD2_SETUP_USRSTR:
        memcpy(&moduleParam.param.user,&data[1],header->len - CMD_STRUCT_SZ-1);
        break;
      case CMD2_SETUP_VNDSTR:
        memcpy(&moduleParam.param.vender,&data[1],header->len - CMD_STRUCT_SZ-1);
        break;
      default:
        h_resp->type = MASK_CMD_RET_ERR | CMD2_SETUP_SYS;
      }
      h_resp->len = 0;
    }
    break;
  case CMD2_SETUP_SAVE:
    chEvtSignal(mainThread,EV_SAVE_PARAM);
    break;
  }
  h_resp->len += CMD_STRUCT_SZ;

  return h_resp->len;
}

int16_t parseFileCmd(char *in, char *out)
{
  cmd_header_t *header = (cmd_header_t*)in;
  cmd_header_t *h_resp = (cmd_header_t*)out;
  char *data = (char*)(in + CMD_STRUCT_SZ);
  char *resp = (char*)(out + CMD_STRUCT_SZ);
  h_resp->type = MASK_CMD_RET_OK | CMD_CMD1_FILE;
  uint16_t dataLen = header->len - CMD_STRUCT_SZ;
  //h_resp->len = 0;
  //h_resp->pid = header->pid & 0x7f;
  switch(header->pid){
  case CMD2_FILE_LIST:
    h_resp->type = MASK_DATA | DATA_FILE;
    h_resp->pid |= CMD2_FILE_OP_OK;
    h_resp->len = fs_list(appParam.activeFile.path,resp,appParam.activeFile.dirChanged);
    if(appParam.activeFile.dirChanged)
      appParam.activeFile.dirChanged = 0;
    break;
  case CMD2_FILE_OPEN:
    if(header->len > CMD_STRUCT_SZ){
      if(!appParam.workingThread){
        h_resp->type = MASK_DATA | DATA_FILE;
        memcpy(appParam.activeFile.fileName,data,dataLen);
        appParam.activeFile.fileName[dataLen] = 0x0;
        // try to open file
        appParam.activeFile.nofBytes = fileSize(appParam.activeFile.fileName);
        appParam.activeFile.offset = 0;
        h_resp->pid |= CMD2_FILE_OP_OK;
        //h_resp->len = 0;
      }else{
        h_resp->type = MASK_CMD_RET_BUSY | CMD_CMD1_FILE;
        h_resp->pid |= CMD2_FILE_OP_ERR;
        h_resp->len = 0;
      }
    }else{
      h_resp->type = MASK_CMD_RET_ERR | CMD_CMD1_FILE;
      //h_resp->pid |= CMD_FILE_OP_ERR
      h_resp->len = 0;
    }
    break;
  case CMD2_FILE_READ:
    if(header->len >= 8){
      if(!appParam.workingThread){
        h_resp->type = MASK_DATA | DATA_FILE;
        if(appParam.activeFile.nofBytes){
          h_resp->pid |= CMD2_FILE_OP_OK;
          if(header->len == CMD_STRUCT_SZ){
            h_resp->len = fsRead(appParam.activeFile.fileName,resp,appParam.activeFile.offset,512);
            appParam.activeFile.offset += h_resp->len;
          }else if(header->len == (CMD_STRUCT_SZ + 8)){
            memcpy((void*)&appParam.activeFile.offset,&data[0],4);
            memcpy((void*)&appParam.activeFile.byteToRead,&data[4],4);
            h_resp->len = fsRead(appParam.activeFile.fileName,resp,appParam.activeFile.offset,appParam.activeFile.byteToRead);
            appParam.activeFile.offset += h_resp->len;
          }
        }else{
          h_resp->pid |= CMD2_FILE_OP_ERR;
          h_resp->len = 0;
        }
      }else{
        h_resp->type = MASK_CMD_RET_BUSY | CMD_CMD1_FILE;
        h_resp->len = 0;
      }
    }
    else{
      h_resp->type = MASK_CMD_RET_ERR | CMD_CMD1_FILE;
      h_resp->len = 0;
    }
    break;
  case CMD2_FILE_WRITE:
    break;
  case CMD2_FILE_REMOVE:
    if(!appParam.workingThread){
      if(appParam.activeFile.nofBytes){
        if(fsRemove(appParam.activeFile.fileName)==0){
          h_resp->type = MASK_CMD_RET_OK | CMD_CMD1_FILE;
          h_resp->pid |= CMD2_FILE_OP_OK;
          h_resp->len = 0;
        }else{
          h_resp->type = MASK_CMD_RET_OK | CMD_CMD1_FILE;
          h_resp->pid |= CMD2_FILE_OP_ERR;
          h_resp->len = 0;
        }
        appParam.activeFile.nofBytes = 0;
      }
    }else{
      h_resp->type = MASK_CMD_RET_BUSY | CMD_CMD1_FILE;
      h_resp->len = 0;
    }
    break;
  case CMD2_FILE_CHDIR:
    if(header->len > CMD_STRUCT_SZ){
      if(!appParam.workingThread){
        memcpy(appParam.activeFile.path,data,header->len - CMD_STRUCT_SZ);
        if(fsChdir(appParam.activeFile.path)==0){
          h_resp->type = MASK_CMD_RET_ERR | CMD_CMD1_FILE;
          h_resp->pid |= CMD2_FILE_OP_OK;
          appParam.activeFile.dirChanged = 1;
          h_resp->len = 0;
        }else{
          h_resp->type = MASK_CMD_RET_OK | CMD_CMD1_FILE;
          h_resp->pid |= CMD2_FILE_OP_ERR;
          h_resp->len = 0;
        }
      }else{
        h_resp->type = MASK_CMD_RET_BUSY | CMD_CMD1_FILE;
        h_resp->len = 0;
      }
    }
    else{
      h_resp->type = MASK_CMD_RET_ERR | CMD_CMD1_FILE;
      h_resp->len = 0;
    }
    break;
  default:
    h_resp->type = MASK_CMD_RET_ERR | CMD_CMD1_FILE;
    h_resp->len = 0;
  }
  h_resp->len += CMD_STRUCT_SZ;
  return h_resp->len;  
}
int16_t commif_parse(char *buf,char *resp)
{
  cmd_header_t *header = (cmd_header_t*)buf;
  cmd_header_t *h_resp = (cmd_header_t*)resp;

//    uint8_t status = 0;
//    uint32_t tmp32;
    switch(header->type){
    case MASK_CMD | CMD_CMD1_CONTROL:
      parseControlCmd(buf,resp);
      break;
    case MASK_CMD | CMD_CMD1_SETUP:
      parseSetupCmd(buf,resp);
      break;
    case MASK_CMD | CMD_CMD1_FILE:
      parseFileCmd(buf,resp);
      break;
    case MASK_CMD | CMD_CMD1_CRITICAL:
      h_resp->magic1 = header->magic1;
      h_resp->magic2 = header->magic2;
      h_resp->type = MASK_CMD_RET_OK | CMD_CMD1_CRITICAL;
      h_resp->len = 0;
      h_resp->pid = header->pid;
      if(header->pid == CMD2_CRITICAL_DEFAULT){
        chEvtSignal(mainThread,EV_LOAD_PARAM);
      }
      
      break;
    default:
      h_resp->type = MASK_CMD_RET_ERR;
      h_resp->len = 0;
    }
    h_resp->chksum = cmd_checksum((uint8_t*)resp,h_resp->len);
    return h_resp->len;
}


int8_t commif_init(thread_t *t)
{
  mainThread = t;
  return 0;
}
