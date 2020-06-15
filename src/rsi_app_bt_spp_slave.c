//! BT include file to refer BT APIs
#include "ch.h"
#include "app_doorcontrol.h"
#include <stdio.h>

#include "sysparam.h"

#ifdef RSI_BT_ENABLE
#include <rsi_bt_apis.h>
#include <rsi_bt_common_apis.h>
#include <rsi_bt_common.h>
#include <rsi_bt_config.h>
#include <rsi_bt.h>
#include "rsi_app_bt_spp_slave.h"

thread_t *self;
static uint8_t *ptr;
static uint16_t length;
static uint8_t linkkey_saved = 0;

static void (*dataArrived)  (uint8_t p) = NULL;
static void (*data_sent_cb)(uint8_t p) = NULL;
static void (*connected_cb) (uint8_t p) = NULL;
static void (*disconnect_cb)(uint8_t p) = NULL;
static mutex_t mtx_tx;

static void rsi_bt_app_init_events()
{
	appParam.rsi_bt.rsi_app_async_event_map = 0;
	return;
}

static void rsi_bt_app_set_event(uint32_t event_num)
{
	appParam.rsi_bt.rsi_app_async_event_map |= BIT(event_num);
  chEvtSignal(self,event_num);
  return;
}

static void rsi_bt_app_clear_event(uint32_t event_num)
{
	appParam.rsi_bt.rsi_app_async_event_map &= ~BIT(event_num);
	return;
}

static int32_t rsi_bt_app_get_event(void)
{
	uint32_t  ix;

	for (ix = 0; ix < 32; ix++)
	{
		if (appParam.rsi_bt.rsi_app_async_event_map & (1 << ix))
		{
		   return ix;
		}
	}

	return (RSI_FAILURE);
}

void rsi_bt_app_on_conn (uint16_t resp_status, rsi_bt_event_bond_t *conn_event)
{
  rsi_6byte_dev_address_to_ascii((int8_t *)appParam.rsi_bt.str_conn_bd_addr, conn_event->dev_addr);
  LOG_PRINT ("on_conn: str_conn_bd_addr: %s\r\n", appParam.rsi_bt.str_conn_bd_addr);
	rsi_bt_app_set_event (RSI_APP_EVENT_CONNECTED);
}


void rsi_bt_app_on_pincode_req(uint16_t resp_status, rsi_bt_event_user_pincode_request_t *user_pincode_request)
{
  rsi_6byte_dev_address_to_ascii((int8_t *)appParam.rsi_bt.str_conn_bd_addr, user_pincode_request->dev_addr);
  LOG_PRINT ("on_pin_coe_req: str_conn_bd_addr: %s\r\n", appParam.rsi_bt.str_conn_bd_addr);
	rsi_bt_app_set_event (RSI_APP_EVENT_PINCODE_REQ);
}

void rsi_ble_app_on_linkkey_req (uint16_t status, rsi_bt_event_user_linkkey_request_t  *user_linkkey_req)
{
  rsi_6byte_dev_address_to_ascii((int8_t *)appParam.rsi_bt.str_conn_bd_addr, user_linkkey_req->dev_addr);
  LOG_PRINT ("linkkey_req: str_conn_bd_addr: %s\r\n", appParam.rsi_bt.str_conn_bd_addr);
	rsi_bt_app_set_event (RSI_APP_EVENT_LINKKEY_REQ);
}

void rsi_ble_app_on_linkkey_save (uint16_t status, rsi_bt_event_user_linkkey_save_t *user_linkkey_save)
{
  rsi_6byte_dev_address_to_ascii((int8_t *)appParam.rsi_bt.str_conn_bd_addr, user_linkkey_save->dev_addr);
  uint8_t m = 0;
  for(uint8_t i=0;i<16;i++){
    if(user_linkkey_save->linkKey[i] != appParam.linkKey[i]) m=1;
  }
  if(m){
    memcpy(appParam.linkKey,user_linkkey_save->linkKey,16);
    rsi_bt_app_set_event (RSI_APP_EVENT_LINKKEY_SAVE);
    linkkey_saved = 1;
  }
  LOG_PRINT ("linkkey_save: str_conn_bd_addr: %s\r\n", appParam.rsi_bt.str_conn_bd_addr);
}

void rsi_bt_app_on_auth_complete (uint16_t resp_status, rsi_bt_event_auth_complete_t *auth_complete)
{
  rsi_6byte_dev_address_to_ascii((int8_t *)appParam.rsi_bt.str_conn_bd_addr, auth_complete->dev_addr);
  LOG_PRINT ("auth_complete: str_conn_bd_addr: %s\r\n", appParam.rsi_bt.str_conn_bd_addr);
  if(resp_status == 0)
  {
    rsi_bt_app_set_event (RSI_APP_EVENT_AUTH_COMPLT);
  }
}

void rsi_bt_app_on_disconn (uint16_t resp_status, rsi_bt_event_disconnect_t *bt_disconnected)
{
  rsi_6byte_dev_address_to_ascii((int8_t *)appParam.rsi_bt.str_conn_bd_addr, bt_disconnected->dev_addr);
  LOG_PRINT ("on_disconn: str_conn_bd_addr: %s\r\n", appParam.rsi_bt.str_conn_bd_addr);
	rsi_bt_app_set_event (RSI_APP_EVENT_DISCONNECTED);
}

void rsi_bt_app_on_spp_connect (uint16_t resp_status, rsi_bt_event_spp_connect_t *spp_connect)
{
  rsi_6byte_dev_address_to_ascii((int8_t *)appParam.rsi_bt.str_conn_bd_addr, spp_connect->dev_addr);
  LOG_PRINT ("spp_conn: str_conn_bd_addr: %s\r\n", appParam.rsi_bt.str_conn_bd_addr);
	rsi_bt_app_set_event (RSI_APP_EVENT_SPP_CONN);
}

void rsi_bt_app_on_spp_disconnect (uint16_t resp_status, rsi_bt_event_spp_disconnect_t *spp_disconn)
{
  rsi_6byte_dev_address_to_ascii((int8_t *)appParam.rsi_bt.str_conn_bd_addr, spp_disconn->dev_addr);
  LOG_PRINT ("spp_disconn: str_conn_bd_addr: %s\r\n", appParam.rsi_bt.str_conn_bd_addr);
	rsi_bt_app_set_event (RSI_APP_EVENT_SPP_DISCONN);
}



void rsi_bt_on_passkey_display (uint16_t resp_status, rsi_bt_event_user_passkey_display_t *bt_event_user_passkey_display)
{
	uint8_t ix; 
  //rsi_6byte_dev_address_to_ascii(str_conn_bd_addr, bt_event_user_passkey_display->passkey);
  memcpy (appParam.sdBuffer.buf_a,bt_event_user_passkey_display->passkey ,4);
  for (ix = 0; ix < 4; ix++)
  {
    LOG_PRINT(" 0x%02x,", bt_event_user_passkey_display->passkey[ix]);
  }
	
  LOG_PRINT ("\r\n");
  LOG_PRINT ("passkey: %d", *((uint32_t *)bt_event_user_passkey_display->passkey));
  LOG_PRINT ("\r\n");
	rsi_bt_app_set_event (RSI_APP_EVENT_PASSKEY_DISPLAY);
}

void rsi_bt_on_passkey_request (uint16_t resp_status, rsi_bt_event_user_passkey_request_t *user_passkey_request)
{
  rsi_6byte_dev_address_to_ascii((int8_t *)appParam.rsi_bt.str_conn_bd_addr, user_passkey_request->dev_addr);
  LOG_PRINT ("passkey_request: str_conn_bd_addr: %s\r\n", appParam.rsi_bt.str_conn_bd_addr);
	rsi_bt_app_set_event (RSI_APP_EVENT_PASSKEY_REQUEST);
}

void rsi_bt_on_ssp_complete (uint16_t resp_status, rsi_bt_event_ssp_complete_t *ssp_complete)
{
  rsi_6byte_dev_address_to_ascii((int8_t *)appParam.rsi_bt.str_conn_bd_addr, ssp_complete->dev_addr);
  LOG_PRINT ("ssp_complete: str_conn_bd_addr: %s\r\n",appParam.rsi_bt.str_conn_bd_addr);
	rsi_bt_app_set_event (RSI_APP_EVENT_SSP_COMPLETE);
}

void rsi_bt_on_confirm_request (uint16_t resp_status, rsi_bt_event_user_confirmation_request_t *user_confirmation_request)
{
	uint8_t ix; 

  memcpy (appParam.sdBuffer.buf_a,user_confirmation_request->confirmation_value ,4);
  chEvtSignal(self,RSI_APP_EVENT_CONFIRM_REQUEST);
  for (ix = 0; ix < 4; ix++)
  {
    LOG_PRINT (" 0x%02x,", user_confirmation_request->confirmation_value[ix]);
  }
  LOG_PRINT ("\r\n");
  LOG_PRINT ("data: %s",user_confirmation_request->confirmation_value );
	rsi_bt_app_set_event (RSI_APP_EVENT_CONFIRM_REQUEST);
}
int16_t rsi_app_bt_read_byte(uint8_t *c)
{
//  if(appParam.rbuf.sz == 0) return -1;
//  *c = *(appParam.rbuf.ptr++);
//  appParam.rbuf.sz--;
  return 1;
  
}

int16_t rsi_app_bt_read(void *socket,int8_t *buffer, uint16_t MaxSz)
{
  int16_t ret = 0;
//  int16_t readSz = MaxSz <= appParam.rbuf.sz?MaxSz:appParam.rbuf.sz;
//  if(readSz){
//    memcpy(buffer,appParam.rbuf.ptr,readSz);
//    appParam.rbuf.sz -= readSz;
//    appParam.rbuf.ptr += readSz;
//    ret = readSz;
//    if(appParam.rbuf.sz)
//      chEvtSignal(self,RSI_APP_EVENT_SPP_RX_REMAIN);
//  }
  return ret;
}

uint16_t rsi_bt_buffer_size(void)
{
  return appParam.sdBuffer.length;
}

void rsi_bt_app_on_spp_data_rx (uint16_t resp_status, rsi_bt_event_spp_receive_t *spp_receive)
{
  uint16_t ix;

  appParam.rxBuf.sz = spp_receive->data_len;
  memcpy (appParam.rxBuf.buffer, spp_receive->data, spp_receive->data_len);
  
  //rsi_bt_app_set_event (RSI_APP_EVENT_SPP_RX);
  chEvtSignal(self,RSI_APP_EVENT_SPP_RX);
  //chEvtSignal(appParam.mainThread,EV_CMD_RX);
//  LOG_PRINT ("spp_rx: data_len: %d, data: ", spp_receive->data_len);
//  for (ix = 0; ix < spp_receive->data_len; ix++)
//  {
//    LOG_PRINT (" 0x%02x,", spp_receive->data[ix]);
//  }
//  LOG_PRINT ("\r\n");
//  LOG_PRINT ("data: %s", spp_receive->data);
}

void rsi_app_bt_send(uint8_t port,uint8_t *buf, uint16_t sz)
{
  //memcpy (spp_receive->data, appParam.rbuf.buffer,sz);
  //spp_receive->data_len = sz;
  memcpy(appParam.sdBuffer.buf_b,buf,sz);
  appParam.sdBuffer.szToWrite = sz;
  appParam.sdBuffer.r = appParam.sdBuffer.buf_b;
  chEvtSignal(self,RSI_APP_EVENT_SPP_TX);


}


void rsi_app_bt_sendptr(uint8_t port,uint8_t *buf, uint16_t sz)
{
//  ptr = buf;
//  length = sz;
  chEvtSignal(self,RSI_APP_EVENT_SPP_TX);
}

void rsi_app_bt_sendcmd(uint8_t port)
{
  chEvtSignal(self,RSI_APP_EVENT_SPP_TXCMD);
}

void rsi_app_bt_sendptr2(uint8_t port,uint8_t *buf, uint16_t sz)
{
  //ptr = buf;
  //length = sz;
}

void rsi_bt_spp_slave (void *p)
{
  (void)p;
  int32_t status = 0;
  int32_t temp_event_map = 0;
  uint8_t str_bd_addr[18] = {0};
  uint8_t eir_data[64] = {2,1,0};

  self = chThdGetSelfX();
  //! BT register GAP callbacks:
  rsi_bt_gap_register_callbacks(
      NULL, //role_change
      rsi_bt_app_on_conn, 
      NULL, //
      rsi_bt_app_on_disconn,
      NULL,//scan_resp
      NULL,//remote_name_req
      rsi_bt_on_passkey_display,//passkey_display
      NULL,//remote_name_req+cancel
      rsi_bt_on_confirm_request,//confirm req
      rsi_bt_app_on_pincode_req,
      rsi_bt_on_passkey_request,//passkey request
      NULL,//inquiry complete
      rsi_bt_app_on_auth_complete,
      rsi_ble_app_on_linkkey_req,//linkkey request
      rsi_bt_on_ssp_complete,//ssp coplete
      rsi_ble_app_on_linkkey_save,
      NULL, //get services
      NULL,
      NULL,
      NULL); //search service

  //! get the local device address(MAC address).
  status = rsi_bt_get_local_device_address(appParam.rsi_bt.local_dev_addr);
  if(status != RSI_SUCCESS)
  {
    return ;
  }
  rsi_6byte_dev_address_to_ascii ((int8_t *)str_bd_addr, appParam.rsi_bt.local_dev_addr);
  //LOG_PRINT ("\r\nlocal_bd_address: %s\r\n", str_bd_addr);

  //! set the local device name
  char str[64];
  uint8_t lenSz;
//  lenSz = sprintf(str,"%s-%s",moduleParam.hub.ssidPrefix,str_bd_addr);
  lenSz = sprintf(str,"%s-%s","DoorController",str_bd_addr);
  status = rsi_bt_set_local_name(str);
  if(status != RSI_SUCCESS)
  {
    return ;
  }

  //! get the local device name
  status = rsi_bt_get_local_name(&appParam.rsi_bt.local_name);
  if(status != RSI_SUCCESS)
  {
    return ;
  }
  //LOG_PRINT ("\r\nlocal_name: %s\r\n", appParam.rsi_bt.local_name.name);
  
  //! prepare Extended Response Data 
  eir_data[3] = lenSz + 1;//strlen (RSI_BT_LOCAL_NAME) + 1;
  eir_data[4] = 9;
//  strcpy ((char *)&eir_data[5], RSI_BT_LOCAL_NAME);
  strcpy ((char *)&eir_data[5], str);
  //! set eir data
  rsi_bt_set_eir_data (eir_data, lenSz + 5);
  //! start the discover mode
  status = rsi_bt_start_discoverable();
  if(status != RSI_SUCCESS)
  {
    return ;
  }

  //! start the connectability mode
  status = rsi_bt_set_connectable();
  if(status != RSI_SUCCESS)
  {
    return ;
  }
  
  //***//
  status = rsi_bt_set_ssp_mode(1,0);
  if(status != RSI_SUCCESS)
  {
    return ;
  }
  
  rsi_bt_set_local_class_of_device(0);
//  rsi_bt_set_local_class_of_device(0x240404);

  //! initilize the SPP profile
  status = rsi_bt_spp_init ();
  if(status != RSI_SUCCESS)
  {
    return ;
  }

  //! register the SPP profile callback's
  rsi_bt_spp_register_callbacks (rsi_bt_app_on_spp_connect,
                                 rsi_bt_app_on_spp_disconnect,
                                 rsi_bt_app_on_spp_data_rx);

  chMtxObjectInit(&mtx_tx);
  while(1)
  {
    //! Application main loop
    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);

    //! if any event is received, it will be served.
        if(evt & RSI_APP_EVENT_CONNECTED)
        {
          //int8_t resp;
          //! remote device connected event
          //status = rsi_bt_set_local_device_role(str_conn_bd_addr,1,&resp);
          //! clear the connected event.
          rsi_bt_app_clear_event (RSI_APP_EVENT_CONNECTED);

        }
        
        if(evt & RSI_APP_EVENT_PINCODE_REQ)
        {
          //! pincode request event
          uint8_t *pin_code = (uint8_t *)PIN_CODE;

          //! clear the pincode request event.
          rsi_bt_app_clear_event(RSI_APP_EVENT_PINCODE_REQ);

          //! sending the pincode requet reply
          status = rsi_bt_pincode_request_reply((int8_t *)appParam.rsi_bt.str_conn_bd_addr, pin_code, 1);
          if(status != RSI_SUCCESS)
          {
            return ;
          }
        }
        if(evt & RSI_APP_EVENT_LINKKEY_SAVE)
        {
          //! linkkey save event

          //! clear the likkey save event.
          rsi_bt_app_clear_event (RSI_APP_EVENT_LINKKEY_SAVE);
          //chEvtSignal(appParam.mainThread,EV_SYS_SAVEPARAM);
        }
        if(evt & RSI_APP_EVENT_AUTH_COMPLT)
        {
          //! authentication complete event

          //! clear the authentication complete event.
          rsi_bt_app_clear_event (RSI_APP_EVENT_AUTH_COMPLT);
        }
        if(evt & RSI_APP_EVENT_DISCONNECTED)
        {
          //! remote device connected event

          //! clear the disconnected event.
          rsi_bt_app_clear_event (RSI_APP_EVENT_DISCONNECTED);
        }
        if(evt & RSI_APP_EVENT_LINKKEY_REQ)
        {
          //! linkkey request event

          //! clear the linkkey request event.
          rsi_bt_app_clear_event (RSI_APP_EVENT_LINKKEY_REQ);

          LOG_PRINT ("linkkey_req: %s\r\n", appParam.rsi_bt.str_conn_bd_addr);
          //! sending the linkkey request negative reply
          if(linkkey_saved==0)
            rsi_bt_linkkey_request_reply ((int8_t *)appParam.rsi_bt.str_conn_bd_addr, NULL, 0);
          else
            rsi_bt_linkkey_request_reply ((int8_t *)appParam.rsi_bt.str_conn_bd_addr, appParam.linkKey, 1);
        }

        if(evt & RSI_APP_EVENT_SPP_CONN)
        {
          //! spp connected event

          //! clear the spp connected event.
          rsi_bt_app_clear_event (RSI_APP_EVENT_SPP_CONN);
          chEvtSignal(appParam.mainThread,EV_CLIENT_CONNECT);
        }
        if(evt & RSI_APP_EVENT_SPP_DISCONN)
        {
          //! spp disconnected event

          //! clear the spp disconnected event.
          rsi_bt_app_clear_event (RSI_APP_EVENT_SPP_DISCONN);
          chEvtSignal(appParam.mainThread,EV_CLIENT_DISCONNECT);
        }
        if(evt & RSI_APP_EVENT_SPP_RX)
        {
          //! spp receive event

          //! clear the spp receive event.
          rsi_bt_app_clear_event (RSI_APP_EVENT_SPP_RX);
          if(dataArrived)
            dataArrived(0);
          else
            chEvtSignal(appParam.mainThread,EV_CMD_RX);

          //! SPP data transfer (loop back)
          //rsi_bt_spp_transfer (appParam.rsi_bt.str_conn_bd_addr, appParam.rbuf.buffer, appParam.rbuf.sz);
        }
        if(evt & RSI_APP_EVENT_SPP_RX_REMAIN)
        {
          //! clear the spp receive event.
          rsi_bt_app_clear_event (RSI_APP_EVENT_SPP_RX_REMAIN);
          if(dataArrived)
            dataArrived(0);
          else
            chEvtSignal(appParam.mainThread,EV_CMD_RX);

          //! SPP data transfer (loop back)
          //rsi_bt_spp_transfer (appParam.rsi_bt.str_conn_bd_addr, appParam.rbuf.buffer, appParam.rbuf.sz);
        }
        if(evt & RSI_APP_EVENT_SPP_TX)
        {
          //! spp receive event

          //! clear the spp receive event.
          rsi_bt_app_clear_event (RSI_APP_EVENT_SPP_TX);

          //! SPP data transfer (loop back)
          //chMtxLock(&mtx_tx);
          status = rsi_bt_spp_transfer (appParam.rsi_bt.str_conn_bd_addr, appParam.sdBuffer.r,appParam.sdBuffer.szToWrite);
          if(status == RSI_SUCCESS){
            if(data_sent_cb)
              data_sent_cb(0);
          }else{
//            while(1);
          }
          //chMtxUnlock(&mtx_tx);
        }
        if(evt & RSI_APP_EVENT_SPP_TXPTR)
        {
          //! spp receive event

          //! clear the spp receive event.
          rsi_bt_app_clear_event (RSI_APP_EVENT_SPP_TX);

          //! SPP data transfer (loop back)
          //chMtxLock(&mtx_tx);
          status = rsi_bt_spp_transfer (appParam.rsi_bt.str_conn_bd_addr, ptr,length);
          //chMtxUnlock(&mtx_tx);
//          if(status != RSI_SUCCESS)
//            while(1);
        }
        if(evt & RSI_APP_EVENT_SPP_TXCMD)
        {
          status = rsi_bt_spp_transfer (appParam.rsi_bt.str_conn_bd_addr,appParam.txBuf.buffer,appParam.txBuf.sz);
          if(status != RSI_SUCCESS)
            while(1);
        }
        if(evt & RSI_APP_EVENT_PASSKEY_DISPLAY)
        {
          //! clear the ssp receive event.
          rsi_bt_app_clear_event (RSI_APP_EVENT_PASSKEY_DISPLAY);
        }
        if(evt & RSI_APP_EVENT_PASSKEY_REQUEST)
        {
          //! clear the ssp receive event.
          rsi_bt_app_clear_event (RSI_APP_EVENT_PASSKEY_REQUEST);
        }
        if(evt & RSI_APP_EVENT_SSP_COMPLETE)
        {
          //! clear the ssp receive event.
          rsi_bt_app_clear_event (RSI_APP_EVENT_SSP_COMPLETE);
					LOG_PRINT(" SSP conection completed\n");
        }
        if(evt & RSI_APP_EVENT_CONFIRM_REQUEST)
        {
          //! clear the ssp receive event.
          rsi_bt_app_clear_event (RSI_APP_EVENT_CONFIRM_REQUEST);
					LOG_PRINT("Confirmation is requested\n");
					rsi_bt_accept_ssp_confirm((int8_t *)appParam.rsi_bt.str_conn_bd_addr);

        }
    
  }

  return ;
}

void rsi_app_bt_cb_data_rx(void(*cb)(uint8_t p))
{
  dataArrived = cb;
}

void rsi_app_bt_cb_data_sent(void(*cb)(uint8_t p))
{
  data_sent_cb = cb;
}

void rsi_app_bt_cb_client_connect(void (*cb)(uint8_t p))
{
  connected_cb = cb;
}

void rsi_app_bt_cb_client_disconnect(void (*cb)(uint8_t p))
{
  disconnect_cb = cb;
}



#endif
















