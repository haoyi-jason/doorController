#ifndef _RSI_APP_WLAN_
#define _RSI_APP_WLAN_

void  rsi_wlan_ap_app_task(void* p);
void rsi_app_wlan_send(int32_t sockId, uint8_t *buf, uint16_t sz);

void rsi_app_wlan_cb_data_rx(void (*cb)(void *p));
void rsi_app_wlan_cb_client_connect(void (*cb)(void *p));
void rsi_app_wlan_cb_client_disconnect(void (*cb)(void *p));
int16_t rsi_read(void *socket,int8_t *buffer, uint16_t MaxSz);
void rsi_app_wlan_sendptr(int32_t sodkId);
#endif