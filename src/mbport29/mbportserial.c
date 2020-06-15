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

#if defined(APP_COMM_MBRTU) && defined(APP_USE_SERIAL)

/* ----------------------- Defines ------------------------------------------*/
#define MBP_SERIAL_TASK_PRIORITY          	( MBP_TASK_PRIORITY )
#define MBP_SERIAL_TASK_STACKSIZE           ( 256 )
#define MBP_SERIAL_BUFFER_SIZE	            ( 128 )

#define IDX_INVALID                         ( 255 )

#define PORT_0_INIT_FUNC										MX_USART4_UART_Init
#define PORT_0_DEINIT_FUNC									HAL_UART_MspDeInit

#define TXRX_BUFFER_LEN											( 128 )
#define RX_FIFO_LEN													( 32 )

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

/* ----------------------- Static variables ---------------------------------*/
STATIC BOOL     bIsInitalized = FALSE;


static void txend1(UARTDriver *uartp);
static void txend2(UARTDriver *uartp);
static void rxerr(UARTDriver *uartp,uartflags_t e);
static void rxchar(UARTDriver *uartp, uint16_t c);
static void rxend(UARTDriver *uartp);
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

STATIC xSerialHWHandle xSerialHWHdls[] = { 
  {.config = &uart_cfg,.usart = &UARTD3}, 
};

STATIC xSerialHandle xSerialHdls[MB_UTILS_NARRSIZE( xSerialHWHdls )];

static    virtual_timer_t vt;
static void timeout_cb(void *arg)
{
  thread_t *t = (thread_t*)arg;
  chSysLockFromISR();
  vMBPHandleUART(0);
  chVTSetI(&vt,US2ST(750),timeout_cb,t);
  chSysUnlockFromISR();
}


/* ----------------------- Start implementation -----------------------------*/

static THD_WORKING_AREA(waSerialTask,1024);
static THD_FUNCTION(procSerialTask,p)
{
    xSerialHandle  *pxSerialIntHdl = (xSerialHandle*)p;
    BOOL            bIsRunning = TRUE;
    thread_t *self = chThdGetSelfX();
    chVTObjectInit(&vt);
    chVTSet(&vt,US2ST(750),timeout_cb,pxSerialIntHdl->xQueue);
    
    do
    {
      eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
      switch ( evt )
      {
      case MSG_SHUTDOWN:
          MBP_ENTER_CRITICAL_SECTION(  );
          bIsRunning = FALSE;
          uartStop(xSerialHWHdls[pxSerialIntHdl->ubIdx].usart);
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
                  uartStartSend( xSerialHWHdls[pxSerialIntHdl->ubIdx].usart, usCnt, &pxSerialIntHdl->arubTXRXDoubleBuffers[0][0] );
              }
          }
          MBP_EXIT_CRITICAL_SECTION(  );
          break;
      case EVT_PERIODIC:
        //vMBPHandleUART(pxSerialIntHdl->ubIdx);
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

        //xSerialHWHdls[ucPort].driver = xSerialHWHdls[ucPort].usart;
        xSerialHWHdls[ucPort].config->speed = ulBaudRate;
       
        switch ( ucDataBits )
        {
        case 7:
            xSerialHWHdls[ucPort].config->cr1 &= ~USART_CR1_M;
            break;
        case 8:
            xSerialHWHdls[ucPort].config->cr1 &= ~USART_CR1_M;
            break;
        default:
            eStatus = MB_EINVAL;
            break;
        }
        switch ( moduleParam.serial.stop )
        {
        case SSTOP_1:
            xSerialHWHdls[ucPort].config->cr2 &= ~USART_CR2_STOP;
            break;
        case SSTOP_1_5:
            xSerialHWHdls[ucPort].config->cr2 &= ~USART_CR2_STOP;
            xSerialHWHdls[ucPort].config->cr2 |= (USART_CR2_STOP_0 | USART_CR2_STOP_1);
            break;
        case SSTOP_2:
            xSerialHWHdls[ucPort].config->cr2 &= ~USART_CR2_STOP;
            xSerialHWHdls[ucPort].config->cr2 |= USART_CR2_STOP_1;
            break;
        default:
            xSerialHWHdls[ucPort].config->cr2 &= ~USART_CR2_STOP;
            break;
        }
        switch ( eParity )
        {
        case MB_PAR_NONE:
            xSerialHWHdls[ucPort].config->cr1 &= ~USART_CR1_PCE;
            break;
        case MB_PAR_ODD:
            xSerialHWHdls[ucPort].config->cr1 &= ~USART_CR1_PCE;
//            xSerialHWHdls[ucPort].config->cr1 |= USART_CR1_PCE;
//            xSerialHWHdls[ucPort].config->cr1 |= USART_CR1_PS;
            break;
        case MB_PAR_EVEN:
            xSerialHWHdls[ucPort].config->cr1 &= ~USART_CR1_PCE;
//            xSerialHWHdls[ucPort].config->cr1 |= USART_CR1_PCE;
//            xSerialHWHdls[ucPort].config->cr1 &= ~USART_CR1_PS;
            break;
        default:
            eStatus = MB_EINVAL;
            break;
        }
        xSerialHWHdls[ucPort].rxPos = 0;
//        xSerialHWHdls[ucPort].handle->Init.Mode = UART_MODE_TX_RX;
//        xSerialHWHdls[ucPort].handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
//        xSerialHWHdls[ucPort].handle->Init.OverSampling = UART_OVERSAMPLING_16;
//        xSerialHWHdls[ucPort].handle->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//        xSerialHWHdls[ucPort].handle->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

        float timeout_us = 11.0f / ( float )ulBaudRate * 3.5f * 1E6f;
        if( timeout_us < 1500.0f )
        {
            timeout_us = 1500.0f;
        }
        xSerialHdls[ucPort].usIdleTimeMax = roundf( timeout_us / 750.0f );

//        if( HAL_OK == HAL_UART_Init( xSerialHWHdls[ucPort].handle ) )
        if(1)
        {
            uartStart(xSerialHWHdls[ucPort].usart,xSerialHWHdls[ucPort].config);
            vMBPNextRxBuffer( ucPort, FALSE );
            if( NULL ==(xSerialHdls[ucPort].xQueue = chThdCreateStatic(waSerialTask,sizeof(waSerialTask),NORMALPRIO,procSerialTask,&xSerialHdls[ucPort])))
            {
                eStatus = MB_EPORTERR;
            }
            else
            {
                xSerialHdls[ucPort].pvMBPTimerExpiredFN = pbFrameTimeoutFN;
                xSerialHdls[ucPort].xMBHdl = xMBHdl;
                xSerialHdls[ucPort].ubIdx = ucPort;
                *pxSerialHdl = &xSerialHdls[ucPort];
                eStatus = MB_ENOERR;
            }

            if( MB_ENOERR != eStatus )
            {
                //HAL_UART_DeInit( xSerialHWHdls[ucPort].handle );
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
    eMBErrorCode    eStatus = MB_EINVAL;
    xSerialHandle  *pxSerialIntHdl = xSerialHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( ( NULL != pxSerialIntHdl ) && MB_IS_VALID_HDL( pxSerialIntHdl, xSerialHdls ) )
    {
        chEvtSignal(pxSerialIntHdl->xQueue,MSG_SHUTDOWN);
        eStatus = MB_ENOERR;
    }
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

void
vMBPHandleUART( UBYTE ucPort )
{
    if( IDX_INVALID != xSerialHdls[ucPort].ubIdx )
    {
//        USHORT  usFIFOWrPos = RX_FIFO_LEN - xSerialHdls[ucPort].usFIFORxPos;
        USHORT  usFIFOWrPos = xSerialHdls[ucPort].usRxIsrPos;
        //USHORT usFIFOWrPos;
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


static void txend1(UARTDriver *uartp)
{
  chSysLockFromISR();
  for( UBYTE ucPort = 0; ucPort < MB_UTILS_NARRSIZE( xSerialHdls ); ucPort++ )
  {
      if( ( IDX_INVALID != xSerialHdls[ucPort].ubIdx ) && ( xSerialHWHdls[ucPort].usart == uartp ) )
      {
          chEvtSignalI(xSerialHdls[ucPort].xQueue,MSG_DMA_SEND_COMPLETE);
      }
  }
  chSysUnlockFromISR();
}
static void txend2(UARTDriver *uartp)
{
  chSysLockFromISR();
  for( UBYTE ucPort = 0; ucPort < MB_UTILS_NARRSIZE( xSerialHdls ); ucPort++ )
  {
      if( ( IDX_INVALID != xSerialHdls[ucPort].ubIdx ) && ( xSerialHWHdls[ucPort].usart == uartp ) )
      {
          chEvtSignalI(xSerialHdls[ucPort].xQueue,MSG_DMA_SEND_COMPLETE);
      }
  }
  chSysUnlockFromISR();
}
static void rxerr(UARTDriver *uartp,uartflags_t e)
{
}
static void rxchar(UARTDriver *uartp, uint16_t c)
{
  for( UBYTE ucPort = 0; ucPort < MB_UTILS_NARRSIZE( xSerialHdls ); ucPort++ )
  {
      if( ( IDX_INVALID != xSerialHdls[ucPort].ubIdx ) && ( xSerialHWHdls[ucPort].usart == uartp ) )
      {
        chSysLockFromISR();
        xSerialHdls[ucPort].arubRXFIFO[xSerialHdls[ucPort].usRxIsrPos++] = (UBYTE)c;
        if(xSerialHdls[ucPort].usRxIsrPos == RX_FIFO_LEN)
          xSerialHdls[ucPort].usRxIsrPos = 0;
        chSysUnlockFromISR();
      }
  }
}

static void rxend(UARTDriver *uartp)
{
  
}

#endif