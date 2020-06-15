/**
  @file modbus_gateway
  @description modbus gateway application, configuration:
USART1 : MODBUS RTU Slave for PLC Interface
USART2 : simply ASCII command to control audio play
USART3: MODBUS ASCII Master to control door controller
USART6: ASCII command to control level display

*/

#include "ch.h"
#include "hal.h"
#include "modbus_gateway.h"
#include "sysparam.h"
#include "mbTask.h"
#include "string.h"
#include "ff.h"
#include "lwipthread.h"
#include "segger_rtt.h"
#include "math.h"
#include "wavePlayer.h"


#define DBG_PRINT(a,b)  SEGGER_RTT_printf(0,a,b)


#define I2S_BUF_SIZE    256
static uint16_t i2s_rx_buf[I2S_BUF_SIZE];
static uint16_t i2s_tx_buf[I2S_BUF_SIZE];
static void i2scallback(I2SDriver *i2sp,  size_t offset, size_t n);
static const I2SConfig i2scfg = {
  i2s_tx_buf,
  i2s_rx_buf,
  I2S_BUF_SIZE,
  i2scallback,
  0,
  12
};

static void i2scallback(I2SDriver *i2sp, size_t offset, size_t n)
{
  (void)i2sp;
  (void)offset;
  (void)n;
  DBG_PRINT("i2scallback:offset=%d\n",offset);
}

static char *playList[] = {
  "0001.wav",
  "sound2.wav",
  "sound3.wav"  
};

static uint16_t wavePlayIndex = 0;

thread_t *thdGateway;
thread_reference_t gateway_trp = NULL;

static SerialConfig serialcfg_level = {
  9600,
};

uint8_t localMACAddr[] = {0xcc,0xdd,0xee,0x55,0x66,0x77};
lwipthread_opts_t lwipOpts = {
  (uint8_t*)localMACAddr,
  0xe600a8c0,
  0x00ffffff,
  0x0100a8c0
};

static uint8_t levelDispPacket2[7] = {0xfa,0x0,0x0,0x0,0x0,0x0,0xfb};
static THD_WORKING_AREA(waMBGW,2048);
static THD_FUNCTION(thMBGW ,p)
{
  sdStart(&SD6,&serialcfg_level);
  uint8_t *ptr;
  uint16_t amp = 1;
  uint8_t i;
//  for(i=0;i<0xff;i++){
//    i2s_tx_buf[i] = amp*sin(i*1440/255)+32768;
//    i2s_rx_buf[i] = i;
//  }
  
//  i2sStart(&I2SD3,&i2scfg);
//  
//  i2sStartExchange(&I2SD3);
  playerInit();
  playFile(playList[0]);
  chThdResume(&gateway_trp,MSG_OK);
  
  while(true){
    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
    if((evt & EV_MBGW_LDISP_CMDRCVD)){
      // get packet
      levelDispGetPacket(ptr);
      // write to port
      sdWrite(&SD6,ptr,7);
    }
    
    if(evt & EV_MBGW_PLAYWAVE){
      playFile(playList[wavePlayIndex]);
    }
    //sdWrite(&SD6,levelDispPacket2,7);
    chThdSleepMilliseconds(1000);
  }
}

void modbusGateway_init(void)
{
  
//  ffTest();
  
  lwipInit(&lwipOpts);
  
  
  mbPortInit(NULL);
  thdGateway = chThdCreateStatic(waMBGW,sizeof(waMBGW),NORMALPRIO,thMBGW,NULL);
  // lock until thread finishing initial task
  chSysLock();
  chThdSuspendS(&gateway_trp);
  chSysUnlock();

  
}