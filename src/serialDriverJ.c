#include "ch.h"
#include "hal.h"
#include "serialdriverj.h"
#include <string.h>

static uint8_t rxbuf[SERIAL_BUFFER_RX];
//static uint8_t txbuf[SERIAL_BUFFER_TX];
static uint8_t *rxPtr;
static uint8_t timeout_ms = 10;

static thread_t *thdUart;

static UARTDriver *drv;
static virtual_timer_t vt;

static void txend1(UARTDriver *uartp);
static void txend2(UARTDriver *uartp);
static void rxerr(UARTDriver *uartp,uartflags_t e);
static void rxchar(UARTDriver *uartp, uint16_t c);
static void rxend(UARTDriver *uartp);

static mutex_t mtx;

#define NOF_DEV 2
_serial_driver_j *devList[NOF_DEV];
static uint8_t nofDevice = 0;

//SDJ_DEF(SDJ1,0,1,512);
//memory_heap_t heap;
//uint8_t heap_buff[256];

typedef struct _write_msg_s{
  char *ptr;
  uint16_t size;
}_write_msg_t;

static void timeout_cb(void *arg)
{
  _serial_driver_j *drv = (_serial_driver_j*)arg;
  chSysLockFromISR();
  chEvtSignalI(drv->thread,EVT_PERIODIC);
  chSysUnlockFromISR();
}

static UARTConfig uart_cfg = {
  txend1,
  txend2,
  rxend,
  rxchar,
  rxerr,
  115200,
  0,
  USART_CR2_LINEN,
  0
};

static UARTConfig uart_cfg_460 = {
  txend1,
  txend2,
  rxend,
  rxchar,
  rxerr,
  460800,
  0,
  USART_CR2_LINEN,
  0
};


uint16_t SDJWrite(char *buf, size_t sz)
{
  _write_msg_t msg;
  msg.ptr = buf;
  msg.size = sz;
  chMsgSend(thdUart,(msg_t)&msg);
  return 0;
}

int16_t SDJReadEx(_serial_driver_j *dev,char *buf, size_t sz)
{
  int16_t szInBuf = dev->usCurRxLen;
  if(sz > szInBuf){
    memcpy(buf,dev->rxBuf,szInBuf);
    dev->pCurRxPtr = dev->rxBuf;
    dev->usCurRxLen = 0;
    return szInBuf;
  }
  else{
    memcpy(buf,dev->rxBuf,sz);
    dev->pCurRxPtr = dev->rxBuf;
    dev->usCurRxLen = 0;
    return sz;
  }
}

int16_t SDJWriteEx(_serial_driver_j *dev,char *buf, size_t sz)
{
  memcpy(dev->txBuf,buf,sz);
  dev->pCurTxPtr = dev->txBuf;
  dev->usCurTxLen = sz;
  chEvtSignal(dev->thread,EVT_TXREQUEST);

  return 0;
}

int16_t SDJWriteEx2(_serial_driver_j *dev,char *buf, size_t sz)
{
  //memcpy(dev->txBuf,buf,sz);
  dev->pCurTxPtr = buf;
  dev->usCurTxLen = sz;
  chEvtSignal(dev->thread,EVT_TXREQUEST);

  return 0;
}

int16_t SDJRead(char *buf,uint16_t sz)
{
  if(rxPtr == rxbuf){
    return 0;
  }
  
  int16_t SZ = rxPtr - rxbuf;
  if(SZ > sz)   
    memcpy(buf,rxbuf,sz);
  else
    memcpy(buf,rxbuf,SZ);
  rxPtr = rxbuf;
  return SZ;
}


int16_t SDJReadTimeout(char *buf,uint16_t sz, uint32_t ms)
{
  chThdSleepMilliseconds(ms);
  return SDJRead(buf,sz);
}




static int32_t vv=0;
static THD_WORKING_AREA(waUSART,256);
static THD_FUNCTION(procUSART,p)
{
  _serial_driver_j *dev = (_serial_driver_j*)p;
  bool bRunning = true;
  chVTObjectInit(&dev->vt);
  chVTSet(&dev->vt,MS2ST(50),timeout_cb,dev);
  do{
    eventmask_t evt =chEvtWaitAny(ALL_EVENTS);
    if(evt & EVT_SHUTDOWN){
      bRunning = false;
      uartStop(dev->uart);
    }
    if(evt & EVT_RECEIVED){
      
    }
    if(evt & EVT_EOF_FRAME){
      
    }
    if(evt & EVT_TXREQUEST){
      
      uartStartSend(dev->uart,dev->usCurTxLen,dev->pCurTxPtr);
    }
    if(evt & EVT_TXREADY){

    }
    if(evt & EVT_DEVRESTART){
      uartStop(dev->uart);
      uartStart(dev->uart,&dev->config);
      // reset buffer...
    }
    if(evt & EVT_PERIODIC){
      vv++;
      if(dev->newRxChar){
        dev->newRxChar = false;
      }else{ // rx idle, check size and call handler
        if(dev->pCurRxPtr != dev->rxBuf){
          if(dev->receivedCB){
            dev->receivedCB(dev);
            chBSemSignal(&dev->sem);
          }
        }
        //dev->pCurRxPtr = dev->rxBuf;              
      }
      chVTSet(&dev->vt,MS2ST(200),timeout_cb,dev);
    }
  }while(bRunning);
  chThdRelease(dev->thread);
  dev->thread = NULL;
  chThdExit((msg_t)0);
}

int8_t SDJRequestTX(_serial_driver_j *drv)
{
  if(!drv) return -1;
  if(!drv->thread) return -2;
  if(!drv->uart) return -3;
  
  chEvtSignal(drv->thread,EVT_TXREQUEST);
  
  return 0;
}

int8_t SDJStop(_serial_driver_j *drv)
{
  if(!drv) return -1;
  chEvtSignal(drv->thread,EVT_SHUTDOWN);
  return 0;
}

int8_t SDJInit_ex(_serial_driver_j *drv,stkalign_t *wa,size_t waSize)
{
  if(!drv) return -1;
  
//  if(drv->uart->state != UART_UNINIT){
//    return -1;
//  }
  
  if(nofDevice == (NOF_DEV - 1))
    return -2;
//  if(nofDevice == 0){
//    chHeapObjectInit(&heap,heap_buff,256);
//  }
  drv->config.txend1_cb = txend1;
  drv->config.txend2_cb = txend2;
  drv->config.rxend_cb = rxend;
  drv->config.rxchar_cb = rxchar;
  drv->config.rxerr_cb = rxerr;
  drv->pCurRxPtr = drv->rxBuf;
  drv->pCurTxPtr = drv->txBuf;
  
  uartStart(drv->uart,&drv->config);
  chBSemObjectInit(&drv->sem,true);
  // create working thread
  drv->thread = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(256),"123",NORMALPRIO,procUSART,drv);
  //drv->thread = chThdCreateStatic(waUSART,512,NORMALPRIO,procUSART,drv);
  devList[nofDevice++] = drv;
  
  return 0;
}

void SDJRestart(_serial_driver_j *dev)
{
  if((dev) && (dev->uart)){
    chEvtSignal(dev->thread,EVT_DEVRESTART);
  }
}

void SDJInit(UARTDriver *u, UARTConfig *f, uint32_t baud)
{
//  if(!u) return;
//  drv = u;
//  if(!f){          
//    uart_cfg.speed = baud;
//    uartStart(drv,&uart_cfg);
//  }else{
//    uartStart(drv,f);
//  }
//  rxPtr = rxbuf;
//  chVTObjectInit(&vt);
//  thdUart = chThdCreateStatic(waUART,sizeof(waUART),NORMALPRIO,procUART,NULL); 
}

void SDJChangeSpeed(_serial_driver_j *dev,uint32_t speed)
{
  dev->config.speed = speed;
  SDJRestart(dev);
}



static void txend1(UARTDriver *uartp)
{
//  chSysLockFromISR();
//  chMtxUnlock(&mtx);
//  chSysUnlockFromISR();
}

static void txend2(UARTDriver *uartp)
{
  chSysLockFromISR();
  for(uint8_t i=0;i<NOF_DEV;i++){
    if(devList[i]->uart == uartp)
      chEvtSignalI(devList[i]->thread,EVT_TXREADY);
  }
  chSysUnlockFromISR();
}

static void rxerr(UARTDriver *uartp,uartflags_t e)
{
  chSysLockFromISR();
  for(uint8_t i=0;i<NOF_DEV;i++){
    if(devList[i]->uart == uartp)
      chEvtSignalI(devList[i]->thread,EVT_DEVRESTART);
  }
  chSysUnlockFromISR();  
}

static void rxchar(UARTDriver *uartp, uint16_t c)
{
  chSysLockFromISR();
  for(uint8_t i=0;i<NOF_DEV;i++){
    if(devList[i]->uart == uartp){
      *devList[i]->pCurRxPtr = (uint8_t)c;
      devList[i]->pCurRxPtr++;
      devList[i]->usCurRxLen++;
      devList[i]->newRxChar = true;
    }
  }
  chSysUnlockFromISR(); 
}

static void rxend(UARTDriver *uartp)
{
  
}

int16_t SDJResetBuffer(_serial_driver_j *dev)
{
  dev->pCurRxPtr = dev->rxBuf;
//  dev->pCurTxPtr = dev->txBuf;
  dev->usCurRxLen = 0;
//  dev->usCurTxLen = 0;
  return 0;
}