
#include "ch.h"
#include "hal.h"
#include <stdio.h>
#include <rsi_common_apis.h>
#include "app_doorcontrol.h"

#ifdef RSI_BT_ENABLE
#include <rsi_bt_apis.h>
#include <rsi_bt_common_apis.h>
#include <rsi_bt_common.h>
#include <rsi_bt_config.h>
#include <rsi_bt.h>
#include "rsi_app_bt_spp_slave.h"
#endif

#ifdef RSI_WLAN_ENABLE
#include "rsi_app_wlan.h"
#endif


#if defined(APP_USE_RSI)

#define RSI_WLAN_BT_TASK_PRIORITY 	64

//! Wlan task priority
#define RSI_WLAN_TASK_PRIORITY   64

//! BT task priority
#define RSI_BT_TASK_PRIORITY   NORMALPRIO

//! Wireless driver task priority
#define RSI_DRIVER_TASK_PRIORITY   NORMALPRIO

//! Memory length for driver


#ifdef RSI_BT_ENABLE
#define GLOBAL_BUFF_LEN       8848
#endif
#ifdef RSI_WLAN_ENABLE
#define GLOBAL_BUFF_LEN    7000
#endif

//! Wlan BT task stack size
#define RSI_WLAN_BT_TASK_STACK_SIZE 1024

//! Wlan task stack size
#define RSI_WLAN_TASK_STACK_SIZE  768

//! BT task stack size
#define RSI_BT_TASK_STACK_SIZE 768

//! Wireless driver task stack size
#define RSI_DRIVER_TASK_STACK_SIZE  256




uint8_t global_buf[GLOBAL_BUFF_LEN] = {0};

//! Wlan access point mode 
#define RSI_WLAN_AP_MODE    6

int32_t rsi_app_init()
{
  int32_t status;
  //! Driver initialization
  status = rsi_driver_init(global_buf, GLOBAL_BUFF_LEN);
  if((status < 0) || (status > GLOBAL_BUFF_LEN))
  {
    return status;
  }
  //! Redpine module intialisation
  status = rsi_device_init(RSI_LOAD_IMAGE_I_FW);
// status = rsi_device_init(LOAD_NWP_FW);
  if(status != RSI_SUCCESS)
  {
    return status;
  }
  
  //! Task created for Driver task
  rsi_task_create(rsi_wireless_driver_task, "driver_task",RSI_DRIVER_TASK_STACK_SIZE, NULL, RSI_DRIVER_TASK_PRIORITY, &appParam.rsi_handle.rsi_driver);

  //! WC initialization

  
#ifdef RSI_BT_ENABLE
  status = rsi_wireless_init(RSI_WLAN_AP_MODE, RSI_OPERMODE_WLAN_BT_CLASSIC);
  if(status != RSI_SUCCESS)
  {
    return 0;
  }
  rsi_task_create(rsi_bt_spp_slave, "bt_task", RSI_BT_TASK_STACK_SIZE, NULL, RSI_BT_TASK_PRIORITY, &appParam.rsi_handle.rsi_bt);
  appParam.transThread = appParam.rsi_handle.rsi_bt;
#endif
#ifdef RSI_WLAN_ENABLE
  status = rsi_wireless_init(RSI_WLAN_AP_MODE, RSI_OPERMODE_WLAN_ONLY);

  if(status != RSI_SUCCESS)
  {
    return 0;
  }
  rsi_task_create(rsi_wlan_ap_app_task, "wlan_task", RSI_WLAN_TASK_STACK_SIZE, NULL, RSI_BT_TASK_PRIORITY, &appParam.rsi_handle.rsi_wlan);
  appParam.transThread = appParam.rsi_handle.rsi_wlan;
#endif
  return status;
}

#endif