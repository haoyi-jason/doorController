#ifndef _RSI_APP_BT_SPP_SLAVE_
#define _RSI_APP_BT_SPP_SLAVE_

#include "ch.h"
#include "hal.h"

//! application defines 
#ifdef RSI_DEBUG_PRINTS
#define LOG_PRINT                     printf
#else 
#define LOG_PRINT
#endif
#define RSI_BT_LOCAL_NAME             "SPP_SLAVE"
#define PIN_CODE                      "4321"

//! application events list
#define RSI_APP_EVENT_CONNECTED       EVENT_MASK(0)
#define RSI_APP_EVENT_PINCODE_REQ     EVENT_MASK(1)
#define RSI_APP_EVENT_LINKKEY_SAVE    EVENT_MASK(2)
#define RSI_APP_EVENT_AUTH_COMPLT     EVENT_MASK(3)
#define RSI_APP_EVENT_DISCONNECTED    EVENT_MASK(4)
#define RSI_APP_EVENT_LINKKEY_REQ     EVENT_MASK(5)
#define RSI_APP_EVENT_SPP_CONN        EVENT_MASK(6)
#define RSI_APP_EVENT_SPP_DISCONN     EVENT_MASK(7)
#define RSI_APP_EVENT_SPP_RX          EVENT_MASK(8)
#define RSI_APP_EVENT_SPP_TX          EVENT_MASK(9)
/** ssp related defines********/
#define RSI_APP_EVENT_PASSKEY_DISPLAY EVENT_MASK(10)
#define RSI_APP_EVENT_PASSKEY_REQUEST EVENT_MASK(11)
#define RSI_APP_EVENT_SSP_COMPLETE    EVENT_MASK(12)
#define RSI_APP_EVENT_CONFIRM_REQUEST EVENT_MASK(13)
#define RSI_APP_EVENT_SPP_RX_REMAIN   EVENT_MASK(14)
#define RSI_APP_EVENT_SPP_TXPTR         EVENT_MASK(15)
#define RSI_APP_EVENT_SPP_TXCMD         EVENT_MASK(16)


//! Memory length for driver
//#define BT_GLOBAL_BUFF_LEN            9000

#ifdef RSI_WITH_OS
//! BLE task stack size
//#define RSI_BT_TASK_STACK_SIZE 768

//! BT task priority
//#define RSI_BT_TASK_PRIORITY   65

//! Number of packet to send or receive
//#define NUMBER_OF_PACKETS 1000
#define NUMBER_OF_PACKETS 100

//! Wireless driver task priority
//#define RSI_DRIVER_TASK_PRIORITY   63

//! Wireless driver task stack size
//#define RSI_DRIVER_TASK_STACK_SIZE  256

//void rsi_wireless_driver_task(void*p);

#endif
void rsi_bt_spp_slave (void *p);
void rsi_app_bt_send(uint8_t port, uint8_t *buf, uint16_t sz);
void rsi_app_bt_sendcmd(uint8_t port);
void rsi_app_bt_sendptr(uint8_t port,uint8_t *buf, uint16_t sz);
//int32_t rsi_app_bt_spp_slave_init();
void rsi_app_bt_cb_data_rx(void (*cb)(uint8_t p));
void rsi_app_bt_cb_client_connect(void (*cb)(uint8_t p));
void rsi_app_bt_cb_client_disconnect(void (*cb)(uint8_t p));
void rsi_app_bt_cb_data_sent(void(*cb)(uint8_t p));
int16_t rsi_app_bt_read(void *socket,int8_t *buffer, uint16_t MaxSz);
int16_t rsi_app_bt_read_byte(uint8_t *c);
uint16_t rsi_bt_buffer_size(void);

void rsi_app_bt_sendptr2(uint8_t port,uint8_t *buf, uint16_t sz);
#endif