#include "ch.h"
#include "hal.h"
#include "simplelink.h"
#include "sl_common.h"
#include "simplelink_porting.h"
#include "sensorhub.h"

#define APPLICATION_VERSION "1.2.0"

#define SL_STOP_TIMEOUT 0xff

/* Use bit 32: Lower bits of status variable are used for NWP events
 *      1 in a 'status_variable', the device has completed the ping operation
 *      0 in a 'status_variable', the device has not completed the ping operation
 */
#define STATUS_BIT_PING_DONE  31

#define CONFIG_IP       SL_IPV4_VAL(192,168,0,1)    /* Static IP to be configured */
#define CONFIG_MASK     SL_IPV4_VAL(255,255,255,0)  /* Subnet Mask for the station */
#define CONFIG_GATEWAY  SL_IPV4_VAL(192,168,0,1)    /* Default Gateway address */
#define CONFIG_DNS      SL_IPV4_VAL(192,168,0,1)    /* DNS Server Address */

#define DHCP_START_IP    SL_IPV4_VAL(192,168,0,100) /* DHCP start IP address */
#define DHCP_END_IP      SL_IPV4_VAL(192,168,0,200) /* DHCP End IP address */

#define IP_LEASE_TIME        3600

#define PING_INTERVAL        1000
#define PING_SIZE            20
#define PING_REQUEST_TIMEOUT 3000
#define PING_ATTEMPT         3

///* Application specific status/error codes */
//typedef enum{
//    LAN_CONNECTION_FAILED = -0x7D0,        /* Choosing this number to avoid overlap with host-driver's error codes */
//    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
//    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,
//
//    STATUS_CODE_MAX = -0xBB8
//}e_AppStatusCodes;

#define IS_PING_DONE(status_variable)           GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_PING_DONE)
/*
 * GLOBAL VARIABLES -- Start
 */
uint32_t  g_Status = 0;
uint32_t  g_PingPacketsRecv = 0;
uint32_t  g_GatewayIP = 0;
uint32_t  g_StationIP = 0;
/*
 * GLOBAL VARIABLES -- End
 */

void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(pWlanEvent == NULL)
    {
        CLI_Write((_u8 *)" [WLAN EVENT] NULL Pointer Error \n\r");
        return;
    }
    
    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);

            /*
             * Information about the connected AP (like name, MAC etc) will be
             * available in 'slWlanConnectAsyncResponse_t' - Applications
             * can use it if required
             *
             * slWlanConnectAsyncResponse_t *pEventData = NULL;
             * pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
             *
             */
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            /* If the user has initiated 'Disconnect' request, 'reason_code' is SL_USER_INITIATED_DISCONNECTION */
            if(SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                CLI_Write((_u8 *)" Device disconnected from the AP on application's request \n\r");
            }
            else
            {
                CLI_Write((_u8 *)" Device disconnected from the AP on an ERROR..!! \n\r");
            }
        }
        break;

        case SL_WLAN_STA_CONNECTED_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_STA_CONNECTED);
        }
        break;

        case SL_WLAN_STA_DISCONNECTED_EVENT:
        {
            CLR_STATUS_BIT(g_Status, STATUS_BIT_STA_CONNECTED);
            CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_LEASED);
        }
        break;

        default:
        {
            CLI_Write((_u8 *)" [WLAN EVENT] Unexpected event \n\r");
        }
        break;
    }
}

void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(pNetAppEvent == NULL)
    {
        CLI_Write((_u8 *)" [NETAPP EVENT] NULL Pointer Error \n\r");
        return;
    }
 
    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
            g_GatewayIP = pEventData->gateway;
            appParam.gwAddr = pEventData->gateway;
            appParam.ipAddr = pEventData->ip;
        }
        break;

        case SL_NETAPP_IP_LEASED_EVENT:
        {
            g_StationIP = pNetAppEvent->EventData.ipLeased.ip_address;
            SET_STATUS_BIT(g_Status, STATUS_BIT_IP_LEASED);
        }
        break;

        default:
        {
            CLI_Write((_u8 *)" [NETAPP EVENT] Unexpected event \n\r");
        }
        break;
    }
}

void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse)
{

  
}

void sl_GeneralEvtHdlr(SlDeviceEvent_t *pSlDeviceEvent)
{
  
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

void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    /*
     * Most of the general errors are not FATAL are are to be handled
     * appropriately by the application
     */
    CLI_Write((_u8 *)" [GENERAL EVENT] \n\r");
}

void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    /*
     * This application doesn't work with socket - Hence these
     * events are not handled here
     */
    CLI_Write((_u8 *)" [SOCK EVENT] Unexpected event \n\r");
}

static _i32 initializeAppVariables()
{
    g_Status = 0;
    g_PingPacketsRecv = 0;
    g_StationIP = 0;

    return SUCCESS;
}

static void displayBanner()
{
    CLI_Write((_u8 *)"\n\r\n\r");
    CLI_Write((_u8 *)" Getting started with WLAN access-point application - Version ");
    CLI_Write((_u8 *)APPLICATION_VERSION);
    CLI_Write((_u8 *)"\n\r*******************************************************************************\n\r");
}

int32_t configureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    _u8           val = 1;
    _u8           configOpt = 0;
    _u8           configLen = 0;
    _u8           power = 0;

    _i32          retVal = -1;
    _i32          mode = -1;

    mode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(mode);

    /* If the device is not in station-mode, try configuring it in station-mode */
    if (ROLE_STA != mode)
    {
        if (ROLE_AP == mode)
        {
            /* If the device is in AP mode, we need to wait for this event before doing anything */
//            while(!IS_IP_ACQUIRED(g_Status)) { _SlNonOsMainLoopTask(); }
            while(!IS_IP_ACQUIRED(g_Status)) { chThdSleepMilliseconds(MS2ST(10)); }
        }

        /* Switch to STA role and restart */
        retVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Stop(SL_STOP_TIMEOUT);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(retVal);

        /* Check if the device is in station again */
        if (ROLE_STA != retVal)
        {
            /* We don't want to proceed if the device is not coming up in station-mode */
            ASSERT_ON_ERROR(DEVICE_NOT_IN_STATION_MODE);
        }
    }

    /* Get the device's version-information */
    configOpt = SL_DEVICE_GENERAL_VERSION;
    configLen = sizeof(ver);
    retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (_u8 *)(&ver));
    ASSERT_ON_ERROR(retVal);

    /* Set connection policy to Auto + SmartConfig (Device's default connection policy) */
    retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove all profiles */
    retVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(retVal);

    /*
     * Device in station-mode. Disconnect previous connection if any
     * The function returns 0 if 'Disconnected done', negative number if already disconnected
     * Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
     */
    retVal = sl_WlanDisconnect();
    if(0 == retVal)
    {
        /* Wait */
        while(IS_CONNECTED(g_Status)) { chThdSleepMilliseconds(MS2ST(10)); }
    }

    /* Enable DHCP client*/
    retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&val);
    ASSERT_ON_ERROR(retVal);

    /* Disable scan */
    configOpt = SL_SCAN_POLICY(0);
    retVal = sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Set Tx power level for station mode
       Number between 0-15, as dB offset from maximum power - 0 will set maximum power */
    power = 0;
    retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (_u8 *)&power);
    ASSERT_ON_ERROR(retVal);

    /* Set PM policy to normal */
    retVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Unregister mDNS services */
    retVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove  all 64 filters (8*8) */
    pal_Memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    retVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(retVal);

    retVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(retVal);

    retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);

    return retVal; /* Success */
}



void simplelink_init(void)
{
  int32_t mode = ROLE_STA;
  int32_t retVal = -1;
  
  retVal = configureSimpleLinkToDefaultState();
  if(retVal < 0){
    if(DEVICE_NOT_IN_STATION_MODE == retVal)
      CLI_Write((uint8_t*)" Failed to configure the device in its default state\r\n");
    
    LOOP_FOREVER();
  }
  
  CLI_Write((_u8 *)" Device is configured in default state \n\r");

  mode = sl_Start(0,0,0);
  
  if(ROLE_AP == mode){
    
  }
  else{
    retVal = sl_WlanSetMode(ROLE_AP);
    //if(retVal < 0)
      
    retVal = sl_WlanSet(SL_WLAN_CFG_AP_ID,WLAN_AP_OPT_SSID,pal_Strlen(SSID_AP_MODE),(uint8_t*)(SSID_AP_MODE));
    
  }
  
}

