//*****************************************************************************
// osi_chibios.c
//
// Interface APIs for ChiBIOS function calls
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "sl_common.h"
#include "simplelink_if.h"
#include "simplelink.h"
#include "wlan.h"
#include "simplelink_porting.h"
#include "mmsSocket.h"
#include "sysParam.h"
#include "comm_if.h"
#include "osi_chibios.h"
#include "sensorhub.h"

static thread_t *thdSimplelink;
static thread_t *thdSpawn;
thread_reference_t simplelink_trp;

static connect_cb cb_connect=NULL, cb_disconnect=NULL;

int socketRecv(char *buf,int len);
int socketSend(int id);
void clientConnect(void);
void clientDisconnect(void);

void *xSimpleLinkSpawnQueue = NULL;
//static mailbox_t mbSimplelink;

#define MB_SIZE      4
#define MSG_SIZE    sizeof(tSimpleLinkSpawnMsg)
//tSimpleLinkSpawnMsg mb_buffer[MB_SIZE];
static msg_t msg_buf[MB_SIZE];
static tSimpleLinkSpawnMsg smsg[MB_SIZE];
static MAILBOX_DECL(mbSimplelink,msg_buf,MB_SIZE);
uint8_t mbIndex = 0;

#define NUM_BUFFER  4
//#define BUF_SIZE    sizeof(tSimpleLinkSpawnMsg)
//#define BUF_SIZE    32
//static char buffer[NUM_BUFFER][BUF_SIZE];
//static msg_t buffer_queue;
//static mailbox_t mblink;

thread_t *xSimpleLinkSpawnTaskHndl = NULL;
// Queue size 
#define slQUEUE_SIZE				( 3 )
#define portMAX_DELAY   100     // ms
#define pdTRUE true


#define THREADS_STACK_SIZE      96
static MEMORYPOOL_DECL(mp1, THD_WORKING_AREA_SIZE(THREADS_STACK_SIZE), NULL);
tSimpleLinkSpawnMsg pool[NUM_BUFFER];
//static char pool[THREADS_STACK_SIZE];

static THD_WORKING_AREA(waSimplelinkSpawn,512);
static THD_FUNCTION(procSimplelinkSpawn ,p)
{
  tSimpleLinkSpawnMsg *Msg;
  while(1){
    msg_t ret = chMBFetch(&mbSimplelink,(msg_t*)&Msg,TIME_INFINITE);    
    if(ret == MSG_OK){
      if(Msg->pEntry == 0x0)
        while(1);
      Msg->pEntry(Msg->pValue);
      chPoolFree(&mp1,Msg);
//      chHeapFree(Msg);
    }
  }
}
/*!
	\brief 	This function call the pEntry callback from a different context

	\param	pEntry		-	pointer to the entry callback function

	\param	pValue		- 	pointer to any type of memory structure that would be
							passed to pEntry callback from the execution thread.

	\param	flags		- 	execution flags - reserved for future usage

	\return upon successful registration of the spawn the function should return 0
			(the function is not blocked till the end of the execution of the function
			and could be returned before the execution is actually completed)
			Otherwise, a negative value indicating the error code shall be returned
	\note
	\warning
*/

OsiReturnVal_e osi_Spawn(P_OSI_SPAWN_ENTRY pEntry , void* pValue , unsigned long flags)
{
  tSimpleLinkSpawnMsg *Msg;
  OsiReturnVal_e ret = OSI_FAILURE;
//  if(flags == SL_SPAWN_FLAG_FROM_SL_IRQ_HANDLER){
//    int index = mbSimplelink.cnt;
//    if(index < MB_SIZE){
//      Msg = (tSimpleLinkSpawnMsg*)&smsg[index];
//      Msg->pEntry = pEntry;
//      Msg->pValue = pValue;
//      (void)chMBPostI(&mbSimplelink,(msg_t)Msg);
//      ret = OSI_OK;
//    }
//    else{
//      ret = OSI_FAILURE;
//    }
//  }else{
//    int index = mbSimplelink.cnt;
//    if(index < MB_SIZE){
//      Msg = (tSimpleLinkSpawnMsg*)&msg_buf[index];
//      Msg->pEntry = pEntry;
//      Msg->pValue = pValue;
//        (void)chMBPost(&mbSimplelink,(msg_t)Msg,TIME_IMMEDIATE);
//      ret = OSI_OK;
//    }
//    else{
//      ret = OSI_FAILURE;
//    }
//  }
  
  if(flags == SL_SPAWN_FLAG_FROM_SL_IRQ_HANDLER){
    Msg = chPoolAllocI(&mp1);
    if(Msg){
      Msg->pEntry = pEntry;
      Msg->pValue = pValue;    
      if(chMBPostI(&mbSimplelink,(msg_t)Msg) == MSG_OK)
        ret = OSI_OK;
    }
  }
  else{
    Msg = chPoolAlloc(&mp1);
    if(Msg){
      Msg->pEntry = pEntry;
      Msg->pValue = pValue;    
      if(chMBPost(&mbSimplelink,(msg_t)Msg,TIME_IMMEDIATE) == MSG_OK)
        ret = OSI_OK;
    }
  }
  if(ret != OSI_OK)
    while(1);
  
  return ret;
}


/*!
	\brief 	This function creates a sync object

	The sync object is used for synchronization between different thread or ISR and
	a thread.

	\param	pSyncObj	-	pointer to the sync object control block

	\return upon successful creation the function should return 0
			Otherwise, a negative value indicating the error code shall be returned
	\note
	\warning
*/
OsiReturnVal_e osi_SyncObjectCreate(OsiSyncObj_t* pSyncObj)
{
    //Check for NULL
    if(NULL == pSyncObj)
    {
        return OSI_INVALID_PARAMS;
    }
    
    chBSemObjectInit((binary_semaphore_t*)pSyncObj,false);

    return OSI_OK; 

}

/*!
	\brief 	This function deletes a sync object

	\param	pSyncObj	-	pointer to the sync object control block

	\return upon successful deletion the function should return 0
			Otherwise, a negative value indicating the error code shall be returned
	\note
	\warning
*/
OsiReturnVal_e osi_SyncObjectDelete(OsiSyncObj_t* pSyncObj)
{

  chBSemReset((binary_semaphore_t*)pSyncObj,false);
    return OSI_OK;
}

/*!
	\brief 		This function generates a sync signal for the object.

	All suspended threads waiting on this sync object are resumed

	\param		pSyncObj	-	pointer to the sync object control block

	\return 	upon successful signaling the function should return 0
				Otherwise, a negative value indicating the error code shall be returned
	\note		the function could be called from ISR context
	\warning
*/
OsiReturnVal_e osi_SyncObjectSignal(OsiSyncObj_t* pSyncObj)
{
	//Check for NULL
	if(NULL == pSyncObj)
	{
		return OSI_INVALID_PARAMS;
	}
        
          chBSemSignal((binary_semaphore_t*)pSyncObj);
    return OSI_OK;
}
/*!
	\brief 		This function generates a sync signal for the object
				from ISR context.

	All suspended threads waiting on this sync object are resumed

	\param		pSyncObj	-	pointer to the sync object control block

	\return 	upon successful signalling the function should return 0
				Otherwise, a negative value indicating the error code shall be returned
	\note		the function is called from ISR context
	\warning
*/
OsiReturnVal_e osi_SyncObjectSignalFromIRQ(OsiSyncObj_t* pSyncObj)
{
	//Check for NULL
	if(NULL == pSyncObj)
	{
		return OSI_INVALID_PARAMS;
	}
        
          chBSemSignalI((binary_semaphore_t*)pSyncObj);
		return OSI_OK;
}

/*!
	\brief 	This function waits for a sync signal of the specific sync object

	\param	pSyncObj	-	pointer to the sync object control block
	\param	Timeout		-	numeric value specifies the maximum number of mSec to
							stay suspended while waiting for the sync signal
							Currently, the simple link driver uses only two values:
								- OSI_WAIT_FOREVER
								- OSI_NO_WAIT

	\return upon successful reception of the signal within the timeout window return 0
			Otherwise, a negative value indicating the error code shall be returned
	\note
	\warning
*/
OsiReturnVal_e osi_SyncObjectWaitTimeout(OsiSyncObj_t* pSyncObj , OsiTime_t Timeout)
{
	//Check for NULL
	if(NULL == pSyncObj)
	{
		return OSI_INVALID_PARAMS;
	}
          if(chBSemWaitTimeout((binary_semaphore_t*)pSyncObj,Timeout)!= MSG_OK)
            return OSI_FAILURE;
        return OSI_OK;
}

/*!
	\brief 	This function clears a sync object

	\param	pSyncObj	-	pointer to the sync object control block

	\return upon successful clearing the function should return 0
			Otherwise, a negative value indicating the error code shall be returned
	\note
	\warning
*/
OsiReturnVal_e osi_SyncObjClear(OsiSyncObj_t* pSyncObj)
{
	//Check for NULL
	if(NULL == pSyncObj)
	{
		return OSI_INVALID_PARAMS;
	}

          chBSemReset((binary_semaphore_t*)pSyncObj,false);
        return OSI_OK;
}

/*!
	\brief 	This function creates a locking object.

	The locking object is used for protecting a shared resources between different
	threads.

	\param	pLockObj	-	pointer to the locking object control block

	\return upon successful creation the function should return 0
			Otherwise, a negative value indicating the error code shall be returned
	\note
	\warning
*/
OsiReturnVal_e osi_LockObjCreate(OsiLockObj_t* pLockObj)
{
  if(pLockObj){
    chMtxObjectInit(pLockObj);
    return OSI_OK;
  }
  return OSI_FAILURE;
}


/*!
	\brief 	This function deletes a locking object.

	\param	pLockObj	-	pointer to the locking object control block

	\return upon successful deletion the function should return 0
			Otherwise, a negative value indicating the error code shall be returned
	\note
	\warning
*/
OsiReturnVal_e osi_LockObjDelete(OsiLockObj_t* pLockObj)
{
    return OSI_OK;
}

/*!
	\brief 	This function locks a locking object.

	All other threads that call this function before this thread calls
	the osi_LockObjUnlock would be suspended

	\param	pLockObj	-	pointer to the locking object control block
	\param	Timeout		-	numeric value specifies the maximum number of mSec to
							stay suspended while waiting for the locking object
							Currently, the simple link driver uses only two values:
								- OSI_WAIT_FOREVER
								- OSI_NO_WAIT


	\return upon successful reception of the locking object the function should return 0
			Otherwise, a negative value indicating the error code shall be returned
	\note
	\warning
*/
OsiReturnVal_e osi_LockObjLock(OsiLockObj_t* pLockObj , OsiTime_t Timeout)
{
    //Check for NULL
    if(NULL == pLockObj)
    {
      return OSI_INVALID_PARAMS;
    }
    //Take Mutex
    chMtxLock(pLockObj);
    return OSI_OK;
//    if(chMtxTryLock(pLockObj))
//      return OSI_OK;
//    else
//      return OSI_OPERATION_FAILED;
}

/*!
	\brief 	This function unlock a locking object.

	\param	pLockObj	-	pointer to the locking object control block

	\return upon successful unlocking the function should return 0
			Otherwise, a negative value indicating the error code shall be returned
	\note
	\warning
*/
OsiReturnVal_e osi_LockObjUnlock(OsiLockObj_t* pLockObj)
{
  //Check for NULL
  if(NULL == pLockObj)
  {
          return OSI_INVALID_PARAMS;
  }
  //Release Semaphore
  chMtxUnlock(pLockObj);
  return OSI_OK;
}



OsiReturnVal_e VStartSimpleLinkSpawnTask(uint8_t  uxPriority)
{
	xSimpleLinkSpawnTaskHndl = chThdCreateStatic(waSimplelinkSpawn,sizeof(waSimplelinkSpawn),NORMALPRIO,procSimplelinkSpawn,NULL);
	
	if(xSimpleLinkSpawnTaskHndl)
		return OSI_OK;
    return OSI_OPERATION_FAILED;
}

/*!
	\brief 	This is the API to delete SL spawn task and delete the SL queue

	\param	none

	\return void
	\note
	\warning
*/
void VDeleteSimpleLinkSpawnTask( void )
{
	if(0 != xSimpleLinkSpawnTaskHndl)
	{
		osi_TaskDelete( xSimpleLinkSpawnTaskHndl );
		xSimpleLinkSpawnTaskHndl = 0;
	}

}


void * mem_Malloc(unsigned long Size)
{
  
    return ( void * ) pvPortMalloc( (size_t)Size );
}

/*!
	\brief 	This function to call the memory de-allocation function of the FREERTOS

	\param	pMem		-	pointer to the memory which needs to be freed
	
	\return - void 
	\note
	\warning
*/
void mem_Free(void *pMem)
{
    chHeapFree(pMem );
}

/*!
	\brief 	This function call the memset function
	\param	pBuf	     -	 pointer to the memory to be fill
        \param  Val          -   Value to be fill
        \param  Size         -   Size of the memory which needs to be fill
	\return - void 
	\note
	\warning
*/

void  mem_set(void *pBuf,int Val,size_t Size)
{
    memset( pBuf,Val,Size);
  
}

/*!
      \brief 	This function call the memcopy function
      \param	pDst	-	pointer to the destination
      \param pSrc     -   pointer to the source
      \param Size     -   Size of the memory which needs to be copy
      
      \return - void 
      \note
      \warning
*/
void  mem_copy(void *pDst, void *pSrc,size_t Size)
{
    memcpy(pDst,pSrc,Size);
}


/*!
	\brief 	This function use to entering into critical section
	\param	void		
	\return - void 
	\note
	\warning
*/

unsigned long osi_EnterCritical(void)
{
	chSysLock();
    return 0;
}

/*!
	\brief 	This function use to exit critical section
	\param	void		
	\return - void 
	\note
	\warning
*/

void osi_ExitCritical(unsigned long ulKey)
{
	chSysUnlock();
}
/*!
	\brief 	This function used to start the scheduler
	\param	void
	\return - void
	\note
	\warning
*/
void osi_start()
{
}
/*!
	\brief 	This function used to suspend the task for the specified number of milli secs
	\param	MilliSecs	-	Time in millisecs to suspend the task
	\return - void
	\note
	\warning
*/
void osi_Sleep(unsigned int MilliSecs)
{
  chThdSleepMilliseconds(MS2ST(MilliSecs));
}


/*!
	\brief 	This function used to disable the tasks
	\param	- void
	\return - Key with the suspended tasks
	\note
	\warning
*/
unsigned long osi_TaskDisable(void)
{
   vTaskSuspendAll();

   return OSI_OK;
}


/*!
	\brief 	This function used to resume all the tasks
	\param	key	-	returned from suspend tasks
	\return - void
	\note
	\warning
*/
void osi_TaskEnable(unsigned long key)
{
   xTaskResumeAll();
}

/*!
	\brief 	This function used to save the OS context before sleep
	\param	void
	\return - void
	\note
	\warning
*/
void osi_ContextSave()
{

}
/*!
	\brief 	This function used to restore the OS context after sleep
	\param	void
	\return - void
	\note
	\warning
*/
void osi_ContextRestore()
{

}


static _i32 establishConnectionWithAP()
{
    SlSecParams_t secParams = {0};
    _i32 retVal = 0;

    secParams.Key = (_i8 *)PASSKEY;
    secParams.KeyLen = pal_Strlen(PASSKEY);
    secParams.Type = SEC_TYPE;

    retVal = sl_WlanConnect((_i8 *)SSID_NAME, pal_Strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(retVal);

    /* Wait */
 //   while((!IS_CONNECTED(g_Status)) || (!IS_IP_ACQUIRED(g_Status)));

    return SUCCESS;
}

//static THD_WORKING_AREA(waWlanAP,2048);
//static THD_FUNCTION(thWlanAP ,p)
int16_t coinfigToApMode(char *ssid, char *passwd)
{
//  (void)p;
  int32_t retVal = -1;
  uint8_t SecType;
  char ssidMac[32];
  retVal = configureSimpleLinkToDefaultState();
  if(retVal < 0){
    chThdResume(&simplelink_trp,MSG_RESET);
    chThdExit(MSG_RESET);
  }
  // start 
  retVal = sl_Start(0,0,0);
  if((retVal < 0)){
    return retVal;
  }
  
  
    unsigned char macAddressVal[12];
    unsigned char macAddressLen;
    sl_NetCfgGet(SL_MAC_ADDRESS_GET,NULL,&macAddressLen,(unsigned char *)macAddressVal);
  for(uint8_t i=0;i<macAddressLen;i++){
    appParam.macAddr[i] = macAddressVal[i];
  }

    if((ROLE_AP == retVal)){
    // wait 
    while(!IS_IP_ACQUIRED(g_Status)){
      chThdYield();
      return 0;
      //chThdSleepMilliseconds(100);
    }
  }
  else{
    /* Configure CC3100 to start in AP mode */
    retVal = sl_WlanSetMode(ROLE_AP);
    if(retVal < 0)
      return retVal;


    /* Configure the SSID of the CC3100 */
    //int len = strlen(ssid);
    int len = chsnprintf(ssidMac,32,"%s-%02X%02X%02X%02X%02X%02X",moduleParam.hub.ssidPrefix,
                         appParam.macAddr[0],
                         appParam.macAddr[1],
                         appParam.macAddr[2],
                         appParam.macAddr[3],
                         appParam.macAddr[4],
                         appParam.macAddr[5]);
    retVal = sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_SSID,
            len, ssidMac);
    if(retVal < 0)
      return retVal;

    SecType = SEC_TYPE_AP_MODE;
    /* Configure the Security parameter the AP mode */
    retVal = sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_SECURITY_TYPE, 1,
            (_u8 *)&SecType);
    if(retVal < 0)
      return retVal;

    len = strlen(passwd);
    retVal = sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_PASSWORD, len,(_u8 *)passwd);
    if(retVal < 0)
      return retVal;

    retVal = sl_Stop(SL_STOP_TIMEOUT);
    if(retVal < 0)
      return retVal;

    CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

    retVal = sl_Start(0, 0, 0);
    if (ROLE_AP == retVal)
    {
        /* If the device is in AP mode, we need to wait for this event before doing anything */
        while(!IS_IP_ACQUIRED(g_Status)) { 
          chThdYield(); 
        }
        return 0;
    }
    else
    {
        return -1;
    }
  }
  
    return retVal;
}

static void SimpleLinkPingReport(SlPingReport_t *pPingReport)
{
    SET_STATUS_BIT(g_Status, STATUS_BIT_PING_DONE);

    if(pPingReport == NULL)
    {
        CLI_Write((_u8 *)" [PING REPORT] NULL Pointer Error\r\n");
        return;
    }

    g_PingPacketsRecv = pPingReport->PacketsReceived;
}

static _i32 checkInternetConnection()
{
    SlPingStartCommand_t pingParams = {0};
    SlPingReport_t pingReport = {0};

    _u32 ipAddr = 0;

    _i32 retVal = -1;

    CLR_STATUS_BIT(g_Status, STATUS_BIT_PING_DONE);
    g_PingPacketsRecv = 0;

    /* Set the ping parameters */
    pingParams.PingIntervalTime = PING_INTERVAL;
    pingParams.PingSize = PING_PKT_SIZE;
    pingParams.PingRequestTimeout = PING_TIMEOUT;
    pingParams.TotalNumberOfAttempts = NO_OF_ATTEMPTS;
    pingParams.Flags = 0;
    pingParams.Ip = g_GatewayIP;

    /* Check for Internet connection */
    retVal = sl_NetAppDnsGetHostByName((_i8 *)HOST_NAME, pal_Strlen(HOST_NAME), &ipAddr, SL_AF_INET);
    ASSERT_ON_ERROR(retVal);

    /* Replace the ping address to match HOST_NAME's IP address */
    pingParams.Ip = ipAddr;

    /* Try to ping HOST_NAME */
    retVal = sl_NetAppPingStart( (SlPingStartCommand_t*)&pingParams, SL_AF_INET,
                                 (SlPingReport_t*)&pingReport, SimpleLinkPingReport);
    ASSERT_ON_ERROR(retVal);

    /* Wait */
    while(!IS_PING_DONE(g_Status)) { _SlNonOsMainLoopTask(); }

    if (0 == g_PingPacketsRecv)
    {
        /* Problem with internet connection*/
        ASSERT_ON_ERROR(INTERNET_CONNECTION_FAILED);
    }

    /* Internet connection is successful */
    return SUCCESS;
}

static _i32 checkLanConnection()
{
    SlPingStartCommand_t pingParams = {0};
    SlPingReport_t pingReport = {0};

    _i32 retVal = -1;

    CLR_STATUS_BIT(g_Status, STATUS_BIT_PING_DONE);
    g_PingPacketsRecv = 0;

    /* Set the ping parameters */
    pingParams.PingIntervalTime = PING_INTERVAL;
    pingParams.PingSize = PING_PKT_SIZE;
    pingParams.PingRequestTimeout = PING_TIMEOUT;
    pingParams.TotalNumberOfAttempts = NO_OF_ATTEMPTS;
    pingParams.Flags = 0;
    pingParams.Ip = g_GatewayIP;

    /* Check for LAN connection */
    retVal = sl_NetAppPingStart( (SlPingStartCommand_t*)&pingParams, SL_AF_INET,
                                 (SlPingReport_t*)&pingReport, SimpleLinkPingReport);
    ASSERT_ON_ERROR(retVal);

    /* Wait */
    while(!IS_PING_DONE(g_Status)) { _SlNonOsMainLoopTask(); }

    if(0 == g_PingPacketsRecv)
    {
        /* Problem with LAN connection */
        ASSERT_ON_ERROR(LAN_CONNECTION_FAILED);
    }

    /* LAN connection is successful */
    return SUCCESS;
}

int16_t coinfigToStaMode(char *ssid, char *passwd)
{
//  (void)p;
  int32_t retVal = -1;
  uint8_t SecType;
  retVal = configureSimpleLinkToDefaultState();
  if(retVal < 0){
    return retVal;
  }
  // start 
  retVal = sl_Start(0,0,0);
  if((retVal < 0) || (ROLE_STA != retVal)){
    return retVal;
  }

  unsigned char macAddressLen;
  unsigned char macAddressVal[12];
  sl_NetCfgGet(SL_MAC_ADDRESS_GET,NULL,&macAddressLen,(unsigned char *)macAddressVal);
  for(uint8_t i=0;i<macAddressLen;i++){
    appParam.macAddr[i] = macAddressVal[i];
  }
  
  
  SlSecParams_t secParams = {0};
  
  secParams.Key = (_i8*)passwd;
  secParams.KeyLen = pal_Strlen(passwd);
  secParams.Type = SEC_TYPE;
  retVal = sl_WlanConnect((_i8*)ssid,pal_Strlen(ssid),0,&secParams,0);
  // wait
  int16_t delay_sec = 0;
  while((!IS_CONNECTED(g_Status)) || (!IS_IP_ACQUIRED(g_Status))){
    chThdSleepMilliseconds(1000);
    if(delay_sec++ > moduleParam.hub.connectionTimeout){
      return -1;
    }
  }
  return 0;
}

static virtual_timer_t vt;

static void timeout_cb(void *arg)
{
  chSysLockFromISR();
  chEvtSignalI(thdSimplelink,SL_EV_POLL);
  chVTSetI(&vt,MS2ST(50),timeout_cb,NULL);
  chSysUnlockFromISR();
}

static uint8_t *bufPtr;
static uint16_t bufSize;

static THD_WORKING_AREA(waSimplelink,1800);
static THD_FUNCTION(procSimplelink ,p)
{
  int32_t retVal = -1;
  int32_t iCount = 0;
  int32_t iNumBroker = 0;
  int32_t iConnBroker = 0;
  //osi_messages RecvQue;
  CC3100_Disable();
  
  chVTObjectInit(&vt);

  char ssid[32],passwd[16];
//  moduleParam.hub.boot = BOOT_STATION;
  moduleParam.hub.boot  = BOOT_AP;
  
  if(moduleParam.hub.boot == BOOT_AP){
    chsnprintf(ssid,32,"%s%08x",moduleParam.hub.ssidPrefix,moduleParam.param.serialNum);
    chsnprintf(passwd,16,"%s",moduleParam.hub.passwd);
    coinfigToApMode(ssid,passwd);
//    coinfigToApMode("TEST","53290921");
  }else{
    chsnprintf(ssid,32,"%s",moduleParam.hub.ssid);
    chsnprintf(passwd,32,"%s",moduleParam.hub.passwd);
    if(coinfigToStaMode(ssid,passwd) < 0){ // ssid lookup fail
      sl_Stop(SL_STOP_TIMEOUT);
      chsnprintf(ssid,32,"%s%08x",moduleParam.hub.ssidPrefix,moduleParam.param.serialNum);
      chsnprintf(passwd,16,"%s",moduleParam.hub.passwdAP);
      coinfigToApMode(ssid,passwd);
    }
  }
  
  // register mDNS service
  retVal = sl_NetAppMDNSUnRegisterService(MDNS_SERVICE_1,strlen(MDNS_SERVICE_1));
//  retVal = sl_NetAppMDNSUnRegisterService(MDNS_SERVICE_2,strlen(MDNS_SERVICE_2));
//  retVal = sl_NetAppMDNSUnRegisterService(MDNS_SERVICE_3,strlen(MDNS_SERVICE_3));
  int len = chsnprintf(ssid,32,"%s-%02X%02X%02X%02X%02X%02X",moduleParam.hub.ssidPrefix,
                       appParam.macAddr[0],
                       appParam.macAddr[1],
                       appParam.macAddr[2],
                       appParam.macAddr[3],
                       appParam.macAddr[4],
                       appParam.macAddr[5]);
  
  retVal = sl_NetAppMDNSRegisterService(ssid,len,
                                        MDNS_TEXT_1,pal_Strlen(MDNS_TEXT_1),
                                        MDNS_PORT_1, TTLIVE, UNIQUE_SERVICE);
//  retVal = sl_NetAppMDNSRegisterService(MDNS_SERVICE_2,pal_Strlen(MDNS_SERVICE_2),
//                                        MDNS_TEXT_2,pal_Strlen(MDNS_TEXT_2),
//                                        MDNS_PORT_2, TTLIVE, UNIQUE_SERVICE);
//  retVal = sl_NetAppMDNSRegisterService(MDNS_SERVICE_3,pal_Strlen(MDNS_SERVICE_3),
//                                        MDNS_TEXT_3,pal_Strlen(MDNS_TEXT_3),
//                                        MDNS_PORT_3, TTLIVE, UNIQUE_SERVICE);


  // create socket server
    uint16_t *port = (uint16_t*)p;
  
  SlSockAddrIn_t addr;
  SlSockAddrIn_t LocalAddr;
  
  appParam.sock.fd = appParam.sock.clientfd = -1;
  appParam.sock.msgLen = 0;
  appParam.sock.port = 5001;
  appParam.sock.sockDomain = SL_AF_INET;
  appParam.sock.sockType = SL_SOCK_STREAM;
  appParam.sock.recvfcn = socketRecv;
  appParam.sock.sendfcn = socketSend;
  appParam.sock.connect = clientConnect;
  appParam.sock.disconnect = clientDisconnect;
  appParam.sock.in = appParam.rbuf.buffer;
  appParam.sock.out = appParam.wbuf.buffer;
  //socket.close = 
  mmsSocketInit(&appParam.sock);
  cmd_header_t *header;
  uint8_t cmdBuffer[32];
  int16_t retSz;
  int ret;
  chThdResume(&simplelink_trp,MSG_OK);
 
  chVTSet(&vt,MS2ST(50),timeout_cb,NULL);
  
  for(;;){
//    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS,MS2ST(50));
    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
    if(evt & SL_EV_POLL){
      mmsSocketPoll(&appParam.sock);
//      if(ret=mmsSocketPoll(&appParam.sock)<0)
//        while(1);
    }
  }
  
}

void osi_simplinkInit(connect_cb conn,connect_cb disconn)
{
  palSetPadMode(CC3100_IRQ_PORT,CC3100_IRQ_PAD,PAL_MODE_INPUT ); // interrupt
  palSetPadMode(CC3100_HIB_PORT,CC3100_HIB_PAD,PAL_MODE_OUTPUT_PUSHPULL); // nHIB
  palSetPad(CC3100_HIB_PORT,CC3100_HIB_PAD);
  
  palSetPadMode(GPIOA,9,PAL_MODE_OUTPUT_PUSHPULL); // 
  palSetPadMode(GPIOA,10,PAL_MODE_OUTPUT_PUSHPULL); // 
  chPoolObjectInit(&mp1, NUM_BUFFER, NULL);
  chPoolLoadArray(&mp1, pool, NUM_BUFFER);
  for(uint8_t i=0;i<NUM_BUFFER;i++){
    chPoolFree(&mp1,&pool[i]);
  }
  //chPoolInit(&mp1,sizeof(pool),NULL);
  //chMBObjectInit(&mbSimplelink, (msg_t*)buffer, NUM_BUFFER);
  
  xSimpleLinkSpawnTaskHndl = chThdCreateStatic(waSimplelinkSpawn,sizeof(waSimplelinkSpawn),NORMALPRIO,procSimplelinkSpawn,NULL);

  thdSimplelink = chThdCreateStatic(waSimplelink,sizeof(waSimplelink),NORMALPRIO-1,procSimplelink,NULL);
  chSysLock();
  chThdSuspendS(&simplelink_trp);
  chSysUnlock();
  
  cb_connect = conn;
  cb_disconnect = disconn;
  
}


int socketRecv(char *buf,int len)
{
  cmd_header_t *header = (cmd_header_t*)appParam.sock.in;
      if(header->magic1 != MAGIC1  || header->magic2 != MAGIC2 || !(header->type & MASK_CMD) || header->len < CMD_STRUCT_SZ){
        return -1;
      }
      
      if(header->len != appParam.sock.msgLen){
          return -1;
      }    
      // apply checksum check if command pid is non-zero
      if(!(header->pid & CMD2_NOCRC_MASK)){
        uint16_t chksum = cmd_checksum(appParam.sock.in,appParam.sock.msgLen);
        if(header->chksum != chksum){
          return -1;
        }
      }
      int retSz;
      if((retSz=commif_parse(appParam.sock.in,appParam.wbuf.buffer))>0){
        //sl_Send(appParam.sock.clientfd,appParam.sock.out,retSz,0);
        socketSend2(appParam.wbuf.buffer,retSz);
//        appParam.sock.txLen = retSz;
//        appParam.sock.out = appParam.wbuf.buffer;
      }
  return 0;
}
int socketSend(int id)
{
  //int retv;
  chEvtSignal(thdSimplelink,SL_EV_SEND);
  return 0;
}

int socketSend2(uint8_t *p, uint16_t sz)
{
//  bufPtr = p;
//  bufSize = sz;
  appParam.sock.out = p;
  appParam.sock.txLen = sz;
  chEvtSignal(thdSimplelink,SL_EV_POLL);
  return 0;
}

void clientConnect(void)
{
  chEvtSignal(thdSimplelink,SL_EV_CLIENTCONNEDT);
  if(cb_connect)
    cb_connect();
}

void clientDisconnect(void)
{
  chEvtSignal(thdSimplelink,SL_EV_CLIENTDISCONNEDT);
  if(cb_disconnect)
    cb_disconnect();
}