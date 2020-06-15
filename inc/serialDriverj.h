#ifndef _SERIALDRIVERJ_
#define _SERIALDRIVERJ_

#include "hal.h"

#define EVT_SHUTDOWN    EVENT_MASK(0)
#define EVT_RECEIVED    EVENT_MASK(1)
#define EVT_EOF_FRAME   EVENT_MASK(2)
#define EVT_TXREADY     EVENT_MASK(3)
#define EVT_DEVRESTART  EVENT_MASK(4)
#define EVT_PERIODIC    EVENT_MASK(5)
#define EVT_TXREQUEST   EVENT_MASK(6)
#define EVT_TXREQUESTEX   EVENT_MASK(7)

#define SERIAL_BUFFER_TX        512
#define SERIAL_BUFFER_RX        512

typedef struct _serial_driver_s _serial_driver_j;
typedef bool (*transmitterEmptyCB)(_serial_driver_j *dev);
typedef bool (*receiverIdleCB)(_serial_driver_j *dev);


struct _serial_driver_s{
  thread_t *thread;
  uint8_t index;
  UARTDriver *uart;
  UARTConfig config;
  uint8_t *rxBuf;
  uint8_t *txBuf;
  uint8_t *pCurRxPtr;
  uint8_t *pCurTxPtr;
  uint16_t usCurRxLen;
  uint16_t usCurTxLen;
  systime_t idleTime;
  transmitterEmptyCB emptyCB;
  receiverIdleCB receivedCB;
  bool newRxChar;       // set true if rxisr receive new char
  virtual_timer_t vt;
  binary_semaphore_t sem;
};


#define SDJ_DEF(_name,id,port,sz) \
static uint8_t _name##_rxBuf[512]; \
static uint8_t _name##_txBuf[512]; \
static _serial_driver_j _name= { \
  id, \
  &(UARTD##port), \
  0, \
  _name##_rxBuf, \
  _name##_txBuf, \
  _name##_rxBuf, \
  _name##_txBuf, \
  0, \
  0 \
};

void SDJInit(UARTDriver *u, UARTConfig *f, uint32_t baud);
int16_t SDJRead(char *buf,uint16_t szsz);
uint16_t SDJWrite(char *buf, size_t sz);
int16_t SDJReadTimeout(char *buf,uint16_t sz, uint32_t ms);

int8_t SDJRequestTX(_serial_driver_j *drv);
int8_t SDJInit_ex(_serial_driver_j *drv,stkalign_t *wa,size_t waSize);
void SDJRestart(_serial_driver_j *dev);

int16_t SDJWriteEx(_serial_driver_j *dev,char *buf, size_t sz);
int16_t SDJWriteEx2(_serial_driver_j *dev,char *buf, size_t sz);
int16_t SDJReadEx(_serial_driver_j *dev,char *buf, size_t sz);
int16_t SDJResetBuffer(_serial_driver_j *dev);
int8_t SDJStop(_serial_driver_j *drv);
void SDJChangeSpeed(_serial_driver_j *dev,uint32_t speed);
#endif
