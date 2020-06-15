/* 
 * MODBUS Library: lwIP/Linux/FreeRTOS port
 * Copyright (c) 2014 Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * Implemenation notes:
 *  
 * $Id: mbporttcp.c,v 1.7 2014-08-23 09:49:22 embedded-solutions.cwalter Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>
#include <string.h>

/* ----------------------- Platform includes --------------------------------*/
#include "mbport.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "common/mbtypes.h"
#include "common/mbframe.h"
#include "common/mbutils.h"
#include "common/mbportlayer.h"
   
#include "app_doorcontrol.h"
#ifdef APP_USE_RSI_WIFI
#include "rsi_app_wlan.h"
#endif
   
/* ----------------------- Defines ------------------------------------------*/
#define MBP_TCP_MASTER_TASK_PRIORITY        ( MBP_TASK_PRIORITY )
#define MBP_TCP_MASTER_TASK_STACKSIZE       ( 256 )
#define MBP_TCP_SLAVE_TASK_PRIORITY         ( MBP_TASK_PRIORITY )
#define MBP_TCP_SLAVE_TASK_STACKSIZE        ( 196 )
#define MBP_TCP_MASTER_QUEUE_SIZE           ( 1 )
#define MBP_TCP_MASTER_THREAD_TIMEOUT       ( 10000 / portTICK_RATE_MS )
#define MAX_MASTER_HDLS                     ( 1 )

#define MAX_SLAVE_HDLS                      ( 1 )       /* Should be equal to MBS_TCP_MAX_INSTANCES */
#define MAX_SLAVE_CLIENT_HDLS               ( 2 )       /* Should be equal to MBS_TCP_MAX_CLIENTS */
#define IDX_INVALID                         ( 255 )

#ifndef MBP_TCP_DEBUG
#define MBP_TCP_DEBUG                       ( 11 )
#endif

#define MBP_TCP_HDL_COMMON \
    UBYTE           ubIdx; \
    xMBTCPIntHandleType eType; \
    BOOL            bIsRunning; \
    xMBHandle       xMBHdl


/* ----------------------- Type definitions ---------------------------------*/

typedef struct
{
    int             iSocket;
    BOOL            bDelete;
} xMBPTCPIntClientHandle;

typedef enum
{
    TCP_MODBUS_UNKNOWN,
    TCP_MODBUS_MASTER,
    TCP_MODBUS_SLAVE
} xMBTCPIntHandleType;

typedef struct
{
    MBP_TCP_HDL_COMMON;
} xMBPTCPIntCommonHandle;

typedef struct
{
    MBP_TCP_HDL_COMMON;
    xMBPTCPIntClientHandle xClientCon;
    peMBPTCPClientNewDataCB eMBPTCPClientNewDataFN;
    peMBPTCPClientDisconnectedCB eMBPTCPClientDisconnectedFN;
} xMBPTCPIntMasterHandle;

typedef struct
{
    MBP_TCP_HDL_COMMON;
    xMBPTCPIntClientHandle xServerCon;  /* Listening socket */
    xMBPTCPIntClientHandle xClientCons[MAX_SLAVE_CLIENT_HDLS];  /* Active clients */
    peMBPTCPClientNewDataCB eMBPTCPClientNewDataFN;
    peMBPTCPClientDisconnectedCB eMBPTCPClientDisconnectedFN;
    peMBPTCPClientConnectedCB eMBPTCPClientConnectedFN;
} xMBPTCPIntSlaveHandle;

/* ----------------------- Static variables ---------------------------------*/
STATIC xMBPTCPIntMasterHandle xMBTCPMasterHdls[MAX_MASTER_HDLS];
STATIC xMBPTCPIntSlaveHandle xMBTCPSlaveHdls[MAX_SLAVE_HDLS];
STATIC BOOL     bIsInitalized = FALSE;

/* ----------------------- Static functions ---------------------------------*/
STATIC void     vMBPTCPInit( void );
STATIC void     vMBTCPCommonHandleReset( xMBPTCPIntCommonHandle * pxTCPHdl, BOOL bClose );
//STATIC void     vMBTCPMasterHandleReset( xMBPTCPIntMasterHandle * pxTCPMasterHdl, BOOL bClose, BOOL bFullReset );
STATIC void     vMBTCPMasterHandlerThread( void *pvArg );
STATIC void     vMBTCPClientHandleReset( xMBPTCPIntClientHandle * pxClientHdl, BOOL bClose );
STATIC void     vMBTCPSlaveHandleReset( xMBPTCPIntSlaveHandle * pxTCPSlaveHdl, BOOL bClose );
STATIC void     vMBTCPSlaveHandlerThread( void *pvArg );

#if defined( MBP_ENABLE_DEBUG_FACILITY ) && ( MBP_ENABLE_DEBUG_FACILITY == 1 )
STATIC const char *pszMBTCPHdlType2Str( xMBTCPIntHandleType );
#endif

void vMBPortTCPNewData(void *socket);
void vMBPortTCPConnected(void *socket);

/* ----------------------- Start implementation -----------------------------*/
/* --------------------------------------------------------------------------*/
/* ----------------------- MODBUS SLAVE CODE --------------------------------*/
/* --------------------------------------------------------------------------*/
eMBErrorCode
eMBPTCPServerInit( xMBPTCPHandle * pxTCPHdl, CHAR * pcBindAddress,
                   USHORT usTCPPort,
                   xMBHandle xMBHdlArg,
                   peMBPTCPClientNewDataCB eMBPTCPClientNewDataFNArg,
                   peMBPTCPClientDisconnectedCB eMBPTCPClientDisconnectedFNArg, peMBPTCPClientConnectedCB eMBPTCPClientConnectedFNArg )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xMBPTCPIntSlaveHandle *pxTCPIntSlaveHdl = NULL;
    UBYTE           ubIdx;
    int             iSockAddr;
    int             iDontBlock = 1;

    vMBPTCPInit(  ); // reset handles only
    if( NULL != pxTCPHdl )
    {
        MBP_ENTER_CRITICAL_SECTION(  );
        // check if room availiable
        for( ubIdx = 0; ubIdx < ( UBYTE ) MB_UTILS_NARRSIZE( xMBTCPSlaveHdls ); ubIdx++ )
        {
            if( IDX_INVALID == xMBTCPSlaveHdls[ubIdx].ubIdx )
            {
                pxTCPIntSlaveHdl = &xMBTCPSlaveHdls[ubIdx];
                vMBTCPSlaveHandleReset( pxTCPIntSlaveHdl, FALSE );
                pxTCPIntSlaveHdl->ubIdx = ubIdx;
                break;
            }
        }
        MBP_EXIT_CRITICAL_SECTION(  );
        if( NULL != pxTCPIntSlaveHdl )
        {

          //pxTCPIntSlaveHdl->xServerCon.iSocket = appParam.clientSocket;
          pxTCPIntSlaveHdl->eMBPTCPClientNewDataFN = eMBPTCPClientNewDataFNArg;
          pxTCPIntSlaveHdl->eMBPTCPClientDisconnectedFN = eMBPTCPClientDisconnectedFNArg;
          pxTCPIntSlaveHdl->eMBPTCPClientConnectedFN = eMBPTCPClientConnectedFNArg;
          pxTCPIntSlaveHdl->xMBHdl = xMBHdlArg;
          pxTCPIntSlaveHdl->bIsRunning = TRUE;
#ifdef APP_USE_RSI_WIFI
          rsi_app_wlan_cb_data_rx(vMBPortTCPNewData);
          rsi_app_wlan_cb_client_connect(vMBPortTCPConnected);
#endif
          eStatus = MB_ENOERR;
        }
        else
        {
            eStatus = MB_ENORES;
        }
    }
    return eStatus;
}

STATIC void
vMBTCPSlaveHandleReset( xMBPTCPIntSlaveHandle * pxTCPSlaveHdl, BOOL bClose )
{
    UBYTE           ubIdx;

    MBP_ASSERT( NULL != pxTCPSlaveHdl );
    for( ubIdx = 0; ubIdx < ( UBYTE ) MB_UTILS_NARRSIZE( pxTCPSlaveHdl->xClientCons ); ubIdx++ )
    {
        vMBTCPClientHandleReset( &( pxTCPSlaveHdl->xClientCons[ubIdx] ), bClose );
    }
    vMBTCPClientHandleReset( &( pxTCPSlaveHdl->xServerCon ), bClose );
    vMBTCPCommonHandleReset( ( xMBPTCPIntCommonHandle * ) pxTCPSlaveHdl, bClose );
    pxTCPSlaveHdl->eMBPTCPClientNewDataFN = NULL;
    pxTCPSlaveHdl->eMBPTCPClientDisconnectedFN = NULL;
    pxTCPSlaveHdl->eMBPTCPClientConnectedFN = NULL;
    pxTCPSlaveHdl->eType = TCP_MODBUS_SLAVE;
}
//                pxTCPIntSlaveHdl->xClientCons[ubIdx].iSocket = iClientSocket;

eMBErrorCode
eMBTCPServerClose( xMBPTCPHandle xTCPHdl )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xMBPTCPIntSlaveHandle *pxTCPIntSlaveHdl = xTCPHdl;
    pxTCPIntSlaveHdl->bIsRunning = FALSE;


    return eStatus;
}

/* --------------------------------------------------------------------------*/
/* ----------------------- COMMON CODE --------------------------------------*/
/* --------------------------------------------------------------------------*/

STATIC void
vMBPTCPInit(  )
{
    UBYTE           ubIdx;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( !bIsInitalized )
    {
//        for( ubIdx = 0; ubIdx < ( UBYTE ) MB_UTILS_NARRSIZE( xMBTCPMasterHdls ); ubIdx++ )
//        {
//            vMBTCPMasterHandleReset( &xMBTCPMasterHdls[ubIdx], FALSE, TRUE );
//        }
        for( ubIdx = 0; ubIdx < ( UBYTE ) MB_UTILS_NARRSIZE( xMBTCPSlaveHdls ); ubIdx++ )
        {
            vMBTCPSlaveHandleReset( &xMBTCPSlaveHdls[ubIdx], FALSE );
        }
        bIsInitalized = TRUE;
    }
    MBP_EXIT_CRITICAL_SECTION(  );
}

eMBErrorCode
eMBPTCPConClose( xMBPTCPHandle xTCPHdl, xMBPTCPClientHandle xTCPClientHdl )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xMBPTCPIntCommonHandle *pxTCPIntCommonHdl = xTCPHdl;
    xMBPTCPIntClientHandle *pxTCPIntClientHdl = xTCPClientHdl;

    ( void )pxTCPIntCommonHdl;

    if( NULL != xTCPClientHdl )
    {
        MBP_ENTER_CRITICAL_SECTION(  );
        pxTCPIntClientHdl->bDelete = TRUE;
        MBP_EXIT_CRITICAL_SECTION(  );
        eStatus = MB_ENOERR;
    }
    return eStatus;
}

void
vMBTCPClientHandleReset( xMBPTCPIntClientHandle * pxClientHdl, BOOL bClose )
{
    if( bClose && ( -1 != pxClientHdl->iSocket ) )
    {
//        if( close( pxClientHdl->iSocket ) < 0 )
//        {
//#if defined( MBP_ENABLE_DEBUG_FACILITY ) && ( MBP_ENABLE_DEBUG_FACILITY == 1 )
//            if( bMBPPortLogIsEnabled( MB_LOG_ERROR, MB_LOG_PORT_TCP ) )
//            {
//                vMBPPortLog( MB_LOG_ERROR, MB_LOG_PORT_TCP, "[MBS/MBM=?] Close failed on socket %d with error: %d\n", pxClientHdl->iSocket, errno );
//            }
//#endif
//        }
//        else
//        {
//#if defined( MBP_ENABLE_DEBUG_FACILITY ) && ( MBP_ENABLE_DEBUG_FACILITY == 1 )
//            if( bMBPPortLogIsEnabled( MB_LOG_DEBUG, MB_LOG_PORT_TCP ) )
//            {
//                vMBPPortLog( MB_LOG_DEBUG, MB_LOG_PORT_TCP, "[MBS/MBM=?] Closed socket %d.\n", pxClientHdl->iSocket );
//            }
//#endif
//        }
    }
    pxClientHdl->iSocket = -1;
    pxClientHdl->bDelete = FALSE;
}

STATIC void
vMBTCPCommonHandleReset( xMBPTCPIntCommonHandle * pxTCPCommonHdl, BOOL bClose )
{
    pxTCPCommonHdl->ubIdx = IDX_INVALID;
    pxTCPCommonHdl->eType = TCP_MODBUS_UNKNOWN;
    pxTCPCommonHdl->bIsRunning = FALSE;
    pxTCPCommonHdl->xMBHdl = MB_HDL_INVALID;
}

eMBErrorCode
eMBPTCPConRead( xMBPTCPHandle xTCPHdl, xMBPTCPClientHandle xTCPClientHdl, UBYTE * pubBuffer, USHORT * pusBufferLen, USHORT usBufferMax )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xMBPTCPIntCommonHandle *pxTCPIntCommonHdl = xTCPHdl;
    xMBPTCPIntClientHandle *pxTCPIntClientHdl = xTCPClientHdl;

    int             iErr;

    
    ( void )pxTCPIntCommonHdl;
    if( NULL != xTCPClientHdl )
    {
        *pusBufferLen = 0;
        //iErr = lwip_read( pxTCPIntClientHdl->iSocket, pubBuffer, usBufferMax );
#ifdef APP_USE_RSI_WIFI
        iErr = rsi_read(0,pubBuffer,usBufferMax);
#endif
        if(iErr > 0){
            eStatus = MB_ENOERR;
            *pusBufferLen = iErr;
        }
        else{
          eStatus = MB_EIO;
        }
    }
    return eStatus;
}

eMBErrorCode
eMBPTCPConWrite( xMBPTCPHandle xTCPHdl, xMBPTCPClientHandle xTCPClientHdl, const UBYTE * pubBuffer, USHORT usBufferLen )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xMBPTCPIntCommonHandle *pxTCPIntCommonHdl = xTCPHdl;
    xMBPTCPIntClientHandle *pxTCPIntClientHdl = xTCPClientHdl;

    ( void )pxTCPIntCommonHdl;
    if( NULL != xTCPClientHdl )
    {
#ifdef APP_USE_RSI_WIFI
      if(usBufferLen >0){
        rsi_app_wlan_send(pxTCPIntClientHdl->iSocket,(uint8_t*)pubBuffer,usBufferLen);
      }
#endif
      eStatus = MB_ENOERR;
    }
    return eStatus;
}

void vMBPortTCPNewData(void *socket)
{
  UBYTE           ubIdx;
  int32_t *sock = (int32_t*)(socket);
  xMBPTCPIntSlaveHandle *pxTCPIntSlaveHdl = &xMBTCPSlaveHdls[0];
  for( ubIdx = 0; ubIdx < MB_UTILS_NARRSIZE( pxTCPIntSlaveHdl->xClientCons ); ubIdx++ )
  {
      if( -1 != pxTCPIntSlaveHdl->xClientCons[ubIdx].iSocket )
      {
          /* Recheck delete flag since data callback could mark connection as dead. */
          if( !pxTCPIntSlaveHdl->xClientCons[ubIdx].bDelete && (pxTCPIntSlaveHdl->xClientCons[ubIdx].iSocket== (*sock)))
          {
                  MBP_ASSERT( NULL != pxTCPIntSlaveHdl->eMBPTCPClientNewDataFN );
                  ( void )pxTCPIntSlaveHdl->eMBPTCPClientNewDataFN( pxTCPIntSlaveHdl->xMBHdl, &( pxTCPIntSlaveHdl->xClientCons[ubIdx] ) );
          }
      }
  }
}

void vMBPortTCPConnected(void *socket)
{
  UBYTE           ubIdx;
  int32_t *sock = (int32_t*)(socket);
  eMBErrorCode  eStatus;
  xMBPTCPIntSlaveHandle *pxTCPIntSlaveHdl = &xMBTCPSlaveHdls[0];
  for( ubIdx = 0; ubIdx < MB_UTILS_NARRSIZE(pxTCPIntSlaveHdl->xClientCons ); ubIdx++ )
  {
    if( -1 == pxTCPIntSlaveHdl->xClientCons[ubIdx].iSocket )
    {
      MBP_ASSERT( NULL != pxTCPIntSlaveHdl->eMBPTCPClientConnectedFN );
      pxTCPIntSlaveHdl->xClientCons[ubIdx].iSocket = *sock;
      if( MB_ENOERR !=
          ( eStatus = pxTCPIntSlaveHdl->eMBPTCPClientConnectedFN( pxTCPIntSlaveHdl->xMBHdl, &( pxTCPIntSlaveHdl->xClientCons[ubIdx] ) ) ) )
      {
          /* Do not close socket in this function because we close it
           * using the bDropClient flag.
           */
          vMBTCPClientHandleReset( &( pxTCPIntSlaveHdl->xClientCons[ubIdx] ), FALSE );
      }
      break;
    }

  }
}

void vMBPortTCPDisconnect(void *socket)
{

  
}