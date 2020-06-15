/* 
 * MODBUS Library: STM32 CubeMX, FreeRTOS port
 * Copyright (c)  2016 Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * $Id: mbportserial.c 1068 2016-08-26 16:05:09Z cwalter $
 */
/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>
#include <string.h>
#include <ch.h>
#include <hal.h>
#include <math.h>

/* ----------------------- Platform includes --------------------------------*/
#include "mbport.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "common/mbtypes.h"
#include "common/mbframe.h"
#include "common/mbutils.h"
#include "common/mbportlayer.h"

#include "sysparam.h"
#include "app_doorcontrol.h"

#if defined(APP_COMM_MBRTU) && defined(APP_USE_RSI_BT)

/* ----------------------- Defines ------------------------------------------*/
#define MBP_SERIAL_TASK_PRIORITY          	( MBP_TASK_PRIORITY )
#define MBP_SERIAL_TASK_STACKSIZE           ( 256 )
#define MBP_SERIAL_BUFFER_SIZE	            ( 128 )

#define IDX_INVALID                         ( 255 )

#define PORT_0_INIT_FUNC										MX_USART4_UART_Init
#define PORT_0_DEINIT_FUNC									HAL_UART_MspDeInit

#define TXRX_BUFFER_LEN											( 64 )
#define RX_FIFO_LEN													( 16 )

#define HDL_RESET( x )						do { \
	( x )->ubIdx = IDX_INVALID; \
	( x )->xMBHdl = FALSE; \
	( x )->pbMBPTransmitterEmptyFN = NULL; \
	( x )->pvMBPReceiveFN = NULL; \
	( x )->pvMBPTimerExpiredFN = NULL; \
	( x )->xQueue = NULL; \
	( x )->pubCurrentRXPtr = NULL; \
	( x )->usCurrentRXLen = 0; \
	( x )->pubCurrentWRPtr = NULL; \
	( x )->usCurrentWRLen = 0; \
	memset( ( x )->arubTXRXDoubleBuffers, 0, sizeof( ( x )->arubTXRXDoubleBuffers ) ); \
	memset( ( x )->arubRXFIFO, 0, sizeof( ( x )->arubRXFIFO ) ); \
	( x )->usIdleTime = 0xFFFFU; \
	( x )->usIdleTimeMax = 0xFFFFU; \
	( x )->usFIFORxPos = 0; \
        ( x )->usRxIsrPos = 0; \
} while( 0 )


/* ----------------------- Type definitions ---------------------------------*/
//typedef enum
//{
#define    MSG_END_OF_FRAME             EVENT_MASK(0)
#define    MSG_NEW_TX_FRAME             EVENT_MASK(1)
#define    MSG_DMA_SEND_COMPLETE        EVENT_MASK(2)
#define    MSG_DMA_RECEIVE_COMPLETE     EVENT_MASK(3)
#define    MSG_SHUTDOWN                 EVENT_MASK(4)
#define    EVT_PERIODIC                 EVENT_MASK(5)
#define    EVT_DATA_RX                  EVENT_MASK(6)
//} xQueueMsg;

typedef struct
{
    UBYTE           ubIdx;
    xMBHandle       xMBHdl;
    pbMBPSerialTransmitterEmptyAPIV2CB pbMBPTransmitterEmptyFN;
    pvMBPSerialReceiverAPIV2CB pvMBPReceiveFN;
    pbMBPTimerExpiredCB pvMBPTimerExpiredFN;

    thread_t   *xQueue;
    /* These buffers are used to communicate new received data to the serial handler task.
     * The current write buffer (pubCurrentWRPtr) is always filled with new data from the FIFO
     * whereas the current read buffer (pubCurrentRXPtr) can be read by the task. The buffers
     * always point to one of the elements within the double buffer arubTXRXDoubleBuffers
     */
    UBYTE          *pubCurrentRXPtr;
    USHORT          usCurrentRXLen;
    UBYTE          *pubCurrentWRPtr;
    USHORT          usCurrentWRLen;
    UBYTE           arubTXRXDoubleBuffers[2][TXRX_BUFFER_LEN];

    /* The STM32 supports a circular DMA. The content of the FIFO is checked every 750us
     * within a very short ISR and if there is new data it is taken from the FIFO and buffered
     * for later use by the MODBUS stack. It has to be ensured that the FIFO has enough space
     * because at 115200 baud the FIFO already receives around 8 characters.
     */
    UBYTE           arubRXFIFO[RX_FIFO_LEN];    /* this is written by the DMA of the UART */
    USHORT          usIdleTime; /* number of intervals (750us) in which no character has been received */
    USHORT          usIdleTimeMax;      /* T3.5 character timeout */
    USHORT          usFIFORxPos;        /* read position for FIFO written by DMA */
    USHORT          usRxIsrPos;
} xSerialHandle;

typedef struct
{
    UARTDriver  *usart;
    UARTConfig *config;
    uint8_t rxBuf[RX_FIFO_LEN];
    uint16_t rxPos;
} xSerialHWHandle;

/* ----------------------- Static functions ---------------------------------*/
STATIC void     vMBPSerialHandlerTask( void *pvArg );
STATIC void     vMBPNextRxBuffer( UBYTE ucPort, BOOL wakeupTask );
void vMBPHandleUART( UBYTE ucPort );
void handleDataArrived(UBYTE ucPort);

/* ----------------------- Static variables ---------------------------------*/
STATIC BOOL     bIsInitalized = FALSE;


STATIC xSerialHWHandle xSerialHWHdls[1]; 

STATIC xSerialHandle xSerialHdls[MB_UTILS_NARRSIZE( xSerialHWHdls )];

static void vMBPortSerialNewData(uint8_t ucPort);
static void vMBPortSerialDataSent(uint8_t ucPort);

static    virtual_timer_t vt;
static void timeout_cb(void *arg)
{
  thread_t *t = (thread_t*)arg;
  chSysLockFromISR();
  //chEvtSignalI(t,EVT_PERIODIC);
  vMBPHandleUART(0);
  chVTSetI(&vt,US2ST(750),timeout_cb,t);
  chSysUnlockFromISR();
}


/* ----------------------- Start implementation -----------------------------*/
static uint32_t tx_cntr,txend_cntr;
static THD_WORKING_AREA(waSerialTask,512);
static THD_FUNCTION(procSerialTask,p)
{
    xSerialHandle  *pxSerialIntHdl = (xSerialHandle*)p;
    BOOL            bIsRunning = TRUE;
    thread_t *self = chThdGetSelfX();
    chVTObjectInit(&vt);
    tx_cntr = txend_cntr = 0;
    chVTSet(&vt,US2ST(750),timeout_cb,pxSerialIntHdl->xQueue);
    
    do
    {
      eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
      switch ( evt )
      {
      case MSG_SHUTDOWN:
          MBP_ENTER_CRITICAL_SECTION(  );
          bIsRunning = FALSE;
          MBP_EXIT_CRITICAL_SECTION(  );
          
          break;
      case MSG_DMA_RECEIVE_COMPLETE:
          MBP_ENTER_CRITICAL_SECTION(  );
          if( NULL != pxSerialIntHdl->pvMBPReceiveFN )
          {
              pxSerialIntHdl->pvMBPReceiveFN( pxSerialIntHdl->xMBHdl, pxSerialIntHdl->pubCurrentRXPtr, pxSerialIntHdl->usCurrentRXLen );
          }
          MBP_EXIT_CRITICAL_SECTION(  );
          /*memcpy( &arubRXTXBuffer[usCnt], pxSerialIntHdl->pubCurrentRXPtr, pxSerialIntHdl->usCurrentRXLen );
             usCnt += pxSerialIntHdl->usCurrentRXLen; */
          break;
      case MSG_END_OF_FRAME:
          MBP_ENTER_CRITICAL_SECTION(  );
          if( NULL != pxSerialIntHdl->pvMBPReceiveFN )
          {
              if( pxSerialIntHdl->usCurrentRXLen )
              {
                  pxSerialIntHdl->pvMBPReceiveFN( pxSerialIntHdl->xMBHdl, pxSerialIntHdl->pubCurrentRXPtr, pxSerialIntHdl->usCurrentRXLen );
              }
              if( NULL != pxSerialIntHdl->pvMBPTimerExpiredFN )
              {
                  pxSerialIntHdl->pvMBPTimerExpiredFN( pxSerialIntHdl->xMBHdl );
              }
          }
          MBP_EXIT_CRITICAL_SECTION(  );
          break;
      case MSG_DMA_SEND_COMPLETE:
//          MBP_ENTER_CRITICAL_SECTION(  );
//          if( NULL != pxSerialIntHdl->pbMBPTransmitterEmptyFN )
//          {
//              USHORT          usCnt = 0;
//              if( !pxSerialIntHdl->
//                  pbMBPTransmitterEmptyFN( pxSerialIntHdl->xMBHdl, &pxSerialIntHdl->arubTXRXDoubleBuffers[0][0], TXRX_BUFFER_LEN, &usCnt ) )
//              { 
//                  pxSerialIntHdl->pbMBPTransmitterEmptyFN = NULL;
//                  txend_cntr++;
//              }
//              
//          }
//          MBP_EXIT_CRITICAL_SECTION(  );
//          break;
      case MSG_NEW_TX_FRAME:
          MBP_ENTER_CRITICAL_SECTION(  );
          if( NULL != pxSerialIntHdl->pbMBPTransmitterEmptyFN )
          {
              USHORT          usCnt = 0;
              if( !pxSerialIntHdl->
                  pbMBPTransmitterEmptyFN( pxSerialIntHdl->xMBHdl, &pxSerialIntHdl->arubTXRXDoubleBuffers[0][0], TXRX_BUFFER_LEN, &usCnt ) )
              { 
                  pxSerialIntHdl->pbMBPTransmitterEmptyFN = NULL;
              }
              
              if( usCnt > 0 )
              {                
                rsi_app_bt_send(pxSerialIntHdl->ubIdx, &pxSerialIntHdl->arubTXRXDoubleBuffers[0][0] ,usCnt);
                tx_cntr++;
              }
          }
          MBP_EXIT_CRITICAL_SECTION(  );
          break;
      case EVT_PERIODIC:
      case EVT_DATA_RX:
        //handleDataArrived(pxSerialIntHdl->ubIdx);
        break;
      }
    }
    while( bIsRunning );
    chThdRelease(self);
    HDL_RESET( pxSerialIntHdl );
    chThdExit( MSG_OK );
}


eMBErrorCode
eMBPSerialInit( xMBPSerialHandle * pxSerialHdl, UCHAR ucPort, ULONG ulBaudRate,
                UCHAR ucDataBits, eMBSerialParity eParity, UCHAR ucStopBits, xMBHandle xMBHdl, pbMBPTimerExpiredCB pbFrameTimeoutFN, eMBSerialMode eMode )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    UBYTE           ubIdx;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( !bIsInitalized )
    {
        for( ubIdx = 0; ubIdx < MB_UTILS_NARRSIZE( xSerialHdls ); ubIdx++ )
        {
            HDL_RESET( &xSerialHdls[ubIdx] );
        }
        bIsInitalized = TRUE;
    }

    if( ( ucPort < MB_UTILS_NARRSIZE( xSerialHdls ) ) && ( IDX_INVALID == xSerialHdls[ucPort].ubIdx ) )
    {
        HDL_RESET( &xSerialHdls[ucPort] );
        eStatus = MB_ENOERR;
       
        xSerialHWHdls[ucPort].rxPos = 0;

        float timeout_us = 11.0f / ( float )ulBaudRate * 3.5f * 1E6f;
        if( timeout_us < 1500.0f )
        {
            timeout_us = 1500.0f;
        }
        xSerialHdls[ucPort].usIdleTimeMax = roundf( timeout_us / 750.0f );
//        xSerialHdls[ucPort].usIdleTimeMax = 1;

        if(1)
        {
//          if(NULL==( xSerialHdls[ucPort].xQueue= chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(32),"TASK",NORMALPRIO-1,procSerialTask,&xSerialHdls[ucPort]))){
          if( NULL ==(xSerialHdls[ucPort].xQueue = chThdCreateStatic(waSerialTask,sizeof(waSerialTask),NORMALPRIO,procSerialTask,&xSerialHdls[ucPort]))){
            eStatus = MB_EPORTERR;
          }else{
            rsi_app_bt_cb_data_rx(vMBPortSerialNewData);
            rsi_app_bt_cb_data_sent(vMBPortSerialDataSent);
          //rsi_app_bt_cb_client_connect(vMBPortTCPConnected);
            vMBPNextRxBuffer( ucPort, FALSE );
            xSerialHdls[ucPort].pvMBPTimerExpiredFN = pbFrameTimeoutFN;
            xSerialHdls[ucPort].xMBHdl = xMBHdl;
            xSerialHdls[ucPort].ubIdx = ucPort;
            *pxSerialHdl = &xSerialHdls[ucPort];
            eStatus = MB_ENOERR;
          }

          if( MB_ENOERR != eStatus )
          {
              HDL_RESET( &xSerialHdls[ucPort] );
          }
        }        
    }
    else
    {
        eStatus = MB_ENORES;
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode
eMBPSerialClose( xMBPSerialHandle xSerialHdl )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    xSerialHandle  *pxSerialIntHdl = xSerialHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
//    if( ( NULL != pxSerialIntHdl ) && MB_IS_VALID_HDL( pxSerialIntHdl, xSerialHdls ) )
//    {
//        chEvtSignal(pxSerialIntHdl->xQueue,MSG_SHUTDOWN);
//        eStatus = MB_ENOERR;
//    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode
eMBPSerialTxEnable( xMBPSerialHandle xSerialHdl, pbMBPSerialTransmitterEmptyCB pbMBPTransmitterEmptyFN )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xSerialHandle  *pxSerialIntHdl = xSerialHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( MB_IS_VALID_HDL( pxSerialIntHdl, xSerialHdls ) )
    {
        if( NULL != pbMBPTransmitterEmptyFN )
        {
          pxSerialIntHdl->pbMBPTransmitterEmptyFN = pbMBPTransmitterEmptyFN;
          chEvtSignal(pxSerialIntHdl->xQueue,MSG_NEW_TX_FRAME);
        }
        else
        {
            pxSerialIntHdl->pbMBPTransmitterEmptyFN = NULL;
        }
        eStatus = MB_ENOERR;
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode
eMBPSerialRxEnable( xMBPSerialHandle xSerialHdl, pvMBPSerialReceiverCB pvMBPReceiveFN )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xSerialHandle  *pxSerialIntHdl = xSerialHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( MB_IS_VALID_HDL( pxSerialIntHdl, xSerialHdls ) )
    {
        if( NULL != pvMBPReceiveFN )
        {
            pxSerialIntHdl->pvMBPReceiveFN = pvMBPReceiveFN;
        }
        else
        {
            pxSerialIntHdl->pvMBPReceiveFN = NULL;
        }
        eStatus = MB_ENOERR;
    }
    else{
      while(1);
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}


static void
vMBPNextRxBuffer( UBYTE ucPort, BOOL copyToRx )
{
    if( copyToRx )
    {
        xSerialHdls[ucPort].pubCurrentRXPtr = xSerialHdls[ucPort].pubCurrentWRPtr;
        xSerialHdls[ucPort].usCurrentRXLen = xSerialHdls[ucPort].usCurrentWRLen;
    }

    if( xSerialHdls[ucPort].pubCurrentWRPtr == &xSerialHdls[ucPort].arubTXRXDoubleBuffers[0][0] )
    {
        xSerialHdls[ucPort].pubCurrentWRPtr = &xSerialHdls[ucPort].arubTXRXDoubleBuffers[1][0];
    }
    else
    {
        xSerialHdls[ucPort].pubCurrentWRPtr = &xSerialHdls[ucPort].arubTXRXDoubleBuffers[0][0];
    }
    xSerialHdls[ucPort].usCurrentWRLen = 0;
}

void handleDataArrived(UBYTE ucPort)
{
    if( IDX_INVALID != xSerialHdls[ucPort].ubIdx )
    {
      if(rsi_bt_buffer_size() == 0){
        if( xSerialHdls[ucPort].usIdleTime < xSerialHdls[ucPort].usIdleTimeMax )
        {
            xSerialHdls[ucPort].usIdleTime++;
        }
        else if( xSerialHdls[ucPort].usIdleTime == xSerialHdls[ucPort].usIdleTimeMax )
        {
            vMBPNextRxBuffer( ucPort, TRUE );
            chEvtSignal(xSerialHdls[ucPort].xQueue,MSG_END_OF_FRAME);
            xSerialHdls[ucPort].usIdleTime = 0xFFFFU;
        }
      }
      else{
        xSerialHdls[ucPort].usIdleTime = 0;
        if( NULL != xSerialHdls[ucPort].pvMBPReceiveFN )
          xSerialHdls[ucPort].usCurrentWRLen = rsi_app_bt_read(0,xSerialHdls[ucPort].pubCurrentWRPtr,TXRX_BUFFER_LEN);
      }
    }
  
}

void
vMBPHandleUART( UBYTE ucPort )
{
    if( IDX_INVALID != xSerialHdls[ucPort].ubIdx )
    {
        USHORT  usFIFOWrPos = xSerialHdls[ucPort].usRxIsrPos;
        /* No character has been received. Increment idle time counter by 1 (corresponds to 750us) */
        if( xSerialHdls[ucPort].usFIFORxPos == usFIFOWrPos )
        {
            if( xSerialHdls[ucPort].usIdleTime < xSerialHdls[ucPort].usIdleTimeMax )
            {
                xSerialHdls[ucPort].usIdleTime++;
            }
            else if( xSerialHdls[ucPort].usIdleTime == xSerialHdls[ucPort].usIdleTimeMax )
            {
                vMBPNextRxBuffer( ucPort, TRUE );
                chEvtSignalI(xSerialHdls[ucPort].xQueue,MSG_END_OF_FRAME);
                xSerialHdls[ucPort].usIdleTime = 0xFFFFU;
            }
        }
        else
        {
            xSerialHdls[ucPort].usIdleTime = 0;
            while( xSerialHdls[ucPort].usFIFORxPos != usFIFOWrPos )
            {
                if( NULL != xSerialHdls[ucPort].pvMBPReceiveFN )
                {
                    xSerialHdls[ucPort].pubCurrentWRPtr[xSerialHdls[ucPort].usCurrentWRLen] = xSerialHdls[ucPort].arubRXFIFO[xSerialHdls[ucPort].usFIFORxPos];
                    xSerialHdls[ucPort].usCurrentWRLen++;
                    if( xSerialHdls[ucPort].usCurrentWRLen >= TXRX_BUFFER_LEN )
                    {
                        vMBPNextRxBuffer( ucPort, TRUE );
                        chEvtSignalI(xSerialHdls[ucPort].xQueue,MSG_DMA_RECEIVE_COMPLETE);
                    }
                }
                xSerialHdls[ucPort].usFIFORxPos++;
                if( RX_FIFO_LEN == xSerialHdls[ucPort].usFIFORxPos )
                {
                    xSerialHdls[ucPort].usFIFORxPos = 0;
                }
            }
        }
    }
}

static void vMBPortSerialNewData(uint8_t ucPort)
{
  UBYTE           ubIdx;
  uint8_t c;
  for( UBYTE ucPort = 0; ucPort < MB_UTILS_NARRSIZE( xSerialHdls ); ucPort++ )
  {
      if( ( IDX_INVALID != xSerialHdls[ucPort].ubIdx ) )
      {
        chSysLock();
        for(uint8_t i=0;i<appParam.sdBuffer.length;i++){
          xSerialHdls[ucPort].arubRXFIFO[xSerialHdls[ucPort].usRxIsrPos++] = (UBYTE)appParam.sdBuffer.buf_a[i];
          if(xSerialHdls[ucPort].usRxIsrPos == RX_FIFO_LEN)
            xSerialHdls[ucPort].usRxIsrPos = 0;
        }
        chSysUnlock();
      }
  }
  
//  chEvtSignal(xSerialHdls[ucPort].xQueue,EVT_DATA_RX);
//  xSerialHdls[port].usCurrentRXLen = rsi_app_bt_read(0,xSerialHdls[port].arubRXFIFO,RX_FIFO_LEN);
//  xSerialHWHdls[port].rxPos = rsi_app_bt_read(port,xSerialHdls[port].arubRXFIFO,RX_FIFO_LEN);
//  if(NULL != xSerialHdls[port].pvMBPReceiveFN){
//    xSerialHdls[port].usCurrentRXLen = rsi_app_bt_read(0,xSerialHdls[port].arubRXFIFO,RX_FIFO_LEN);
//    xSerialHdls[port].pvMBPReceiveFN(xSerialHdls[port].xMBHdl,xSerialHdls[port].pubCurrentRXPtr, xSerialHdls[port].usCurrentRXLen );
//    xSerialHdls[port].pvMBPReceiveFN(xSerialHdls[port].xMBHdl,xSerialHdls[port].pubCurrentRXPtr, xSerialHdls[port].usCurrentRXLen );
//    xSerialHdls[port].pvMBPTimerExpiredFN( xSerialHdls[port].xMBHdl );    
//  }
  
//  if(rsi_app_bt_read_byte(&c)){
//    xSerialHdls[port].arubRXFIFO[xSerialHWHdls[port].rxPos++] = (UBYTE)c;
//    if(xSerialHWHdls[port].rxPos == RX_FIFO_LEN)
//      xSerialHWHdls[port].rxPos = 0;
//  }
//    
//  }
//  vMBPHandleUART(port);
  
}

static void vMBPortSerialDataSent(uint8_t ucPort)
{
  //chSysLock();
  chEvtSignal(xSerialHdls[ucPort].xQueue,MSG_DMA_SEND_COMPLETE);
  //chSysUnlock();
}

void vMBPortSerialConnected(uint8_t port)
{
  UBYTE           ubIdx;
  //int32_t *sock = (int32_t*)(socket);
  eMBErrorCode  eStatus;
  //xMBPTCPIntSlaveHandle *pxTCPIntSlaveHdl = &xMBTCPSlaveHdls[0];
//  for( ubIdx = 0; ubIdx < MB_UTILS_NARRSIZE(pxTCPIntSlaveHdl->xClientCons ); ubIdx++ )
//  {
//    if( -1 == pxTCPIntSlaveHdl->xClientCons[ubIdx].iSocket )
//    {
      //MBP_ASSERT( NULL != pxTCPIntSlaveHdl->eMBPTCPClientConnectedFN );
     // pxTCPIntSlaveHdl->xClientCons[ubIdx].iSocket = *sock;
//      if( MB_ENOERR !=
//          ( eStatus = pxTCPIntSlaveHdl->eMBPTCPClientConnectedFN( pxTCPIntSlaveHdl->xMBHdl, &( pxTCPIntSlaveHdl->xClientCons[ubIdx] ) ) ) )
//      {
//          /* Do not close socket in this function because we close it
//           * using the bDropClient flag.
//           */
//          vMBTCPClientHandleReset( &( pxTCPIntSlaveHdl->xClientCons[ubIdx] ), FALSE );
//      }
//      break;
//    }
//
//  }
}

void vMBPortSerialDisconnect(uint8_t port)
{

  
}

#endif // APP_USE_RSI_BT