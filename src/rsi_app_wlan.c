
//! Driver Header file
#include "rsi_driver.h"

//! WLAN include file to refer wlan APIs
#include "rsi_wlan_apis.h"

#include "rsi_app_wlan.h"

#include "app_doorcontrol.h"

#include "sysparam.h"

//! Configuration parameters 

#ifdef RSI_WLAN_ENABLE 
//! Access point SSID to connect
#define SSID               "REDPINE_AP"

//! Channel Number
#define CHANNEL_NO             11 

//! Security Type
#define SECURITY_TYPE        RSI_WPA2

//! Encryption Type
#define ENCRYPTION_TYPE      RSI_CCMP

//! Password
#define PSK                "12345678"

//! Beacon Interval
#define BEACON_INTERVAL        100

//! DTIM Count
#define DTIM_COUNT             4  

#define DHCP_MODE       1

//! IP address of the module 
//! E.g: 0x650AA8C0 == 192.168.10.1
#define DEVICE_IP          0x0A64A8C0

//! IP address of Gateway
//! E.g: 0x010AA8C0 == 192.168.10.1
#define GATEWAY            0x0164A8C0

//! IP address of netmask
//! E.g: 0x00FFFFFF == 255.255.255.0
#define NETMASK            0x00FFFFFF 

//! Device port number
#define DEVICE_PORT        5001

//! Remote port number
#define REMOTE_PORT        5001

//! Remote IP address. Should be in reverse long format
//! E.g: 0x640AA8C0 == 192.168.10.100
#define REMOTE_IP_ADDRESS 0x6364A8C0



//! Standard Defines

//!   ethernet header offset
#define ETHERNET_HEADER_OFFSET    14

//! IP header offset
#define IP_HEADER_OFFSET         (14 + 20)

//! UDP header offset
#define UDP_HEADER_OFFSET        (14 + 20 + 8)

//!ARP REQUEST CODE
#define ARP_OPTION_REQUEST       0x0100

//! ARP RESPONSE CODE
#define ARP_OPTION_RESPONSE      0x0200

//! ARP Message size
#define ARP_MESSAGE_SIZE         28

//! MAC address size
#define MAC_ADDRESS_SIZE         6

//! IP header size 
#define IP_HEADER_SIZE           20

//! Protocol type of UDP
#define UDP_PROTOCOL             17

//!Time to live
#define TIME_TO_LIVE             64

//! ARP PACKET 
#define RSI_ARP_PACKET          0x0608

//! IP PACKET
#define RSI_IP_PACKET           0x0008

//! BT event: set when a packet from BT received 
#define RSI_BT_EVENT            BIT(0)

//! WLAN event: set when a  packet from WLAN device received
#define RSI_WLAN_EVENT           BIT(1)

#define RSI_SOCK_EVENT           BIT(2)

//! Application buffers size
#define RSI_APP_BUF_SIZE        128

#define  ICMP_PING_REPLY    0

#define  ICMP_PING_REQUEST  8

#define  CHECKSUM_LENGTH    2

#define  IP_ADDRESS_SIZE    4

#define  MAC_ADDRESS_SIZE   6

#define  PROTO_ICMP         0x1


//! Memory length for driver
#define APP_BUFF_LEN   300
thread_t *self;

//int8_t      send_buffer[APP_BUFF_LEN] = {0};


//void rsi_wlan_send_ping_response(uint8_t *data, uint32_t length);
extern void rsi_wlan_get_mac_address(uint8_t *buffer, uint32_t length);
//extern void prepare_udp_pkt_header(uint8_t *buffer, uint32_t length);
extern void rsi_udp_socket_create(uint16_t port_no);
extern int32_t rsi_wlan_check_mac_address(uint8_t *buffer, uint32_t length);
//extern void rsi_wlan_set_ipaddress(uint32_t ip_addr,uint32_t network_mask, uint32_t gateway);
int32_t rsi_wlan_set_ipaddress(uint32_t ip_addr,uint32_t network_mask, uint32_t gateway);
extern int32_t rsi_wlan_check_packet_type(uint8_t *buffer, uint32_t length);
//extern void rsi_wlan_send_arp_response(uint8_t *buffer, uint32_t length);
extern void rsi_wlan_send_arp_request(uint8_t *buffer, uint32_t length);
extern void udp_socket_create(uint16_t port_no);
extern int32_t udp_send(uint16_t remote_port, uint8_t *buffer, uint32_t length);
//extern void udp_recv(uint8_t *buffer, uint32_t length);
extern void rsi_wlan_ap_app_task(void*);
extern int32_t rsi_bt_app_send_to_wlan(uint8_t msg_type,uint8_t *buffer, uint32_t length);
extern void rsi_wlan_app_callbacks_init(void);
extern void rsi_stations_connect_notify_handler(uint16_t status,uint8_t *buffer, const uint32_t length);
extern void rsi_stations_disconnect_notify_handler(uint16_t status, uint8_t *buffer, const uint32_t length);
extern void rsi_packet_receive_notify_handler(uint16_t status, uint8_t *buffer, uint32_t length);
//extern void rsi_prepare_udp_packet_header(uint8_t *buffer, uint16_t length);
extern uint16_t rsi_data_send_checksum(uint32_t csum, uint8_t * buff_ptr, uint32_t data_len);
extern uint32_t rsi_udp_start_checksum(uint8_t proto, uint16_t len, uint16_t port1, uint16_t port2,
                uint8_t *srcaddr, uint8_t *dstaddr);
extern uint16_t rsi_udp_end_checksum(uint32_t csum);
//uint16_t convert_le_be(uint16_t num);

uint8_t null_mac_addr[6] ={0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t broadcast_mac_addr[6] ={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
//! stations count
uint8_t rsi_stations_count;

//! device static ip address
uint32_t    ip_addr      = DEVICE_IP;

//! Network mask
uint32_t    network_mask = NETMASK;

//! Gateway
uint32_t    gateway      = GATEWAY;



typedef struct rsi_ether_header_s 
{
  //! destination mac address
  uint8_t ether_dest[6];

  //! source mac address
  uint8_t ether_source[6];

  //! ethernet type
  uint16_t ether_type;

} rsi_ether_header_t;

//! Enumeration for commands used in application
typedef enum rsi_app_cmd_e
{
  RSI_DATA = 0
}rsi_app_cmd_t;

//! IP header structure
typedef struct rsi_ip_header_s 
{
  //! version 
  uint8_t ver_ihl;

  //! type of service
  uint8_t tos;

  //! length higher byte
  uint8_t len_hb;

  //! length lower byte
  uint8_t len_lb;

  //! id higher byte
  uint8_t id_hb;

  //! id lower byte
  uint8_t id_lb;

  //! fragmentation higher byte
  uint8_t frag_hb;

  //! fragmentation lower byte
  uint8_t frag_lb;

  //! time to live
  uint8_t ttl;

  //! protocol
  uint8_t proto;

  //! check sum higher byte
  uint8_t csum_hb;

  //! check sum lower byte
  uint8_t csum_lb;

  //! source ip address
  uint8_t srcaddr[4];

  //! destination ip address
  uint8_t dstaddr[4];

} rsi_ip_header_t;


typedef struct rsi_ip_header_ping_s
{
  uint8_t       ip_header_word_0[4];
  uint8_t       ip_header_word_1[4];
  uint8_t       ip_header_word_2[4];
  /* Define the source IP address.  */
  uint8_t       ip_header_source_ip[4];
  /* Define the destination IP address.  */
  uint8_t       ip_header_destination_ip[4];
}rsi_ip_header_ping_t;


typedef struct rsi_eth_header_ping_s
 {
   uint8_t  src_mac_addr[6];
   uint8_t  dst_mac_addr[6];
   uint8_t  pkt_type[2];
 }rsi_eth_header_ping_t;


//! Enumeration for states in applcation 
typedef enum rsi_wlan_app_state_e
{
  RSI_WLAN_INITIAL_STATE              = 0,
  RSI_WLAN_AP_UP_STATE                = 1,
  RSI_WLAN_SOCKET_SERVER_LISTEN_STATE = 2,
  RSI_WLAN_SOCKET_CONNECTED_STATE     = 3

}rsi_wlan_app_state_t;

//! UDP header structure
typedef struct rsi_udp_header_s 
{
  //! source port
  uint16_t src_port;

  //! destination port
  uint16_t dest_port;

  //! payload length
  uint16_t length;

  //! check sum
  uint16_t check_sum;

}rsi_udp_header_t;

//! UDP pkt total header, excluding payload 
typedef struct rsi_udp_pkt_header_s
{
  //! ethernet header
  rsi_ether_header_t ether_header;

  //! ip header
  rsi_ip_header_t  ip_header;

  //! udp header
  rsi_udp_header_t udp_header;

}rsi_udp_pkt_header_t;

//! APR packet structure
typedef struct rsi_arp_packet_s
{
  //! hardware type
  uint16_t hardware_type;

  //! protocol type
  uint16_t protocol_type;

  //! hardware size
  uint8_t  hardware_size;

  //! protocol size
  uint8_t  protocol_size;

  //! protocol opcode
  uint16_t opcode;

  //!sender physical address
  uint8_t  sender_mac_addr[6];

  //! sender ip address
  uint8_t  sender_ip_addr[4];

  //! target physical address
  uint8_t  target_mac_addr[6];

  //! target ip address
  uint8_t  target_ip_addr[4];

}rsi_arp_packet_t;

//! wlan application control block

typedef struct rsi_wlan_device_params_s
{
  //! device static ip address
  uint32_t    module_ip_addr;

  //! udp source port no
  uint16_t source_port;

  //! remote device ip address
  uint32_t    sender_ip_addr;

  //! udp remote port no
  uint16_t remote_port;

  //! module physical address
  uint8_t module_mac_addr[6];

  //! target physical address
  uint8_t sender_mac_addr[6];

}rsi_wlan_device_params_t;

//! wlan application control block
typedef struct rsi_wlan_app_cb_s
{
  //! wlan application state 
  rsi_wlan_app_state_t state;

  //! length of buffer to copy
  uint32_t bt_pkt_length;

  //! application buffer
  uint8_t bt_buffer[RSI_APP_BUF_SIZE];

  //! to check application buffer availability
  uint8_t bt_buf_in_use;

  //! length of buffer to copy
  uint32_t wlan_pkt_length;

  //! application buffer
  uint8_t wlan_buffer[RSI_APP_BUF_SIZE];

  //! to check application buffer availability
  uint8_t wlan_buf_in_use;

  //! application events bit map 
  uint32_t event_map;

  //! wlan device details
  rsi_wlan_device_params_t device_params;
  
  uint32_t socket_evt_map;

}rsi_wlan_app_cb_t;

#define  SOCKET_CONNECT         BIT(0)
#define  SOCKET_DISCONNECTED    BIT(1)
#define  SOCKET_TX              BIT(2)
#define  SOCKET_RX              BIT(3)
#define  SOCKET_READ              BIT(4)

//! application control block
rsi_wlan_app_cb_t rsi_wlan_app_cb;

uint32_t currentSocket;

static void (*dataArrived)(void *p) = NULL;
static void (*connected_cb)(void *p) = NULL;
static void (*disconnect_cb)(void *p) = NULL;

int16_t rsi_read(void *socket,int8_t *buffer, uint16_t MaxSz)
{
  int16_t ret = 0;
  if(MaxSz <= appParam.sdBuffer.length){
    memcpy(buffer,appParam.sdBuffer.r,MaxSz);
    appParam.sdBuffer.length -= MaxSz;
    appParam.sdBuffer.r += MaxSz;
//    dataArrived(&appParam.clientSocket);
    ret = MaxSz;
    if(appParam.sdBuffer.length)
      chEvtSignal(self,SOCKET_READ);
  }
  return ret;
}

void socket_async_recive(uint32_t sock_no, uint8_t *buffer, uint32_t length)
{
  memcpy(appParam.rxBuf.buffer,buffer,length);
  appParam.rxBuf.sz = length;
  chEvtSignal(self,SOCKET_RX);
//  rsi_wlan_app_cb.socket_evt_map |= SOCKET_RX;
//  rsi_wlan_app_cb.event_map |= RSI_SOCK_EVENT;
}

void remote_socket_terminated(uint16_t status, uint8_t *buffer, uint32_t length)
{
  //rsi_wlan_app_cb.socket_evt_map |= SOCKET_DISCONNECTED;
  //rsi_wlan_app_cb.event_map |= RSI_SOCK_EVENT;
  chEvtSignal(self,SOCKET_DISCONNECTED);
}

void rsi_wlan_app_callbacks_init(void)
{

  //! Initialze station connect notify  call back
  rsi_wlan_register_callbacks(RSI_STATIONS_CONNECT_NOTIFY_CB, rsi_stations_connect_notify_handler);

  //! Initialze station disconnect notify call back
  rsi_wlan_register_callbacks(RSI_STATIONS_DISCONNECT_NOTIFY_CB, rsi_stations_disconnect_notify_handler);

  //! Initialize packet receive notify call back
  rsi_wlan_register_callbacks(RSI_WLAN_DATA_RECEIVE_NOTIFY_CB, rsi_packet_receive_notify_handler);

  rsi_wlan_register_callbacks(RSI_REMOTE_SOCKET_TERMINATE_CB, remote_socket_terminated);
}

void rsi_app_wlan_send(int32_t sockId, uint8_t *buf, uint16_t sz)
{
  // use sdbuffer
  memcpy(appParam.sdBuffer.buf_b,buf,sz);
  appParam.sdBuffer.szToWrite = sz;
  appParam.sdBuffer.r = appParam.sdBuffer.buf_b;
  chEvtSignal(self,SOCKET_TX);
//  int32_t status;
//  if(!rsi_wlan_app_cb.wlan_buf_in_use)
//  {
//    //! if not in use
//
//    //! copy the buffer to wlan app cb wlan buffer
//    memcpy(rsi_wlan_app_cb.wlan_buffer, buf, sz); 
//
//    //! hold length information
//    rsi_wlan_app_cb.wlan_pkt_length = sz;
//
//    //! make buffer in use
//    rsi_wlan_app_cb.wlan_buf_in_use = 1;
//
//    chEvtSignal(self,SOCKET_TX);
//    //! raise event to wlan app task
//  }
  
}

void rsi_app_wlan_sendptr(int32_t sodkId)
{
  chEvtSignal(self,SOCKET_TX);
}

int32_t     status;
void  rsi_wlan_ap_app_task(void *p)
{
  (void)p;
  int32_t     status = RSI_SUCCESS;
  int32_t pkt_type = 0; 
  int32_t server_socket,new_socket;
  struct      rsi_sockaddr_in server_addr, client_addr; 
  int8_t recv_buffer[128];
  uint32_t recv_size;
  int32_t addr_size;
  uint16_t     tcp_keep_alive_time = 1000 ; 
  
  self = chThdGetSelfX();
  while(1)
  {
    chThdSleepMilliseconds(5);
  switch(rsi_wlan_app_cb.state)
  {
    case RSI_WLAN_INITIAL_STATE:
      {
        //! register call backs
        rsi_wlan_app_callbacks_init();

        //! Configure IP 
        rsi_wlan_set_ipaddress(ip_addr, network_mask, gateway);

        //! Get MAC address of the Access point
        status = rsi_wlan_get(RSI_MAC_ADDRESS, rsi_wlan_app_cb.device_params.module_mac_addr, MAC_ADDRESS_SIZE);

        if(status != RSI_SUCCESS)
        {
          break;
        }
        else
        {
          //! update wlan application state
          rsi_wlan_app_cb.state = RSI_WLAN_AP_UP_STATE; 
        }

        //! Start Access point
        int8_t str_bd_addr[18],str[32],psk[32];
        rsi_6byte_dev_address_to_ascii ((int8_t *)str_bd_addr, rsi_wlan_app_cb.device_params.module_mac_addr);
        str_bd_addr[17] = 0x0;
        uint8_t lenSz;
        uint8_t *wptr = str;
        uint8_t *p = rsi_wlan_app_cb.device_params.module_mac_addr+5;
//        lenSz = sprintf(str,"%s-%s\0",moduleParam.hub.ssidPrefix,str_bd_addr);
//        lenSz = sprintf(str,"%s-%02X%02X%02X%02X%02X%02X\0",moduleParam.hub.ssidPrefix,*(p+5),*(p+4),*(p+3),*(p+2),*(p+1),*p);
        wptr += sprintf(wptr,"%s",moduleParam.hub.ssidPrefix);
        uint8_t digits = moduleParam.hub.commType & 0xf;
        if(digits){
          wptr += sprintf(wptr,"-");
          for(uint8_t i=0;i<digits;i++){
            wptr += sprintf(wptr,"%02X",*p--);
          }
        }
        lenSz = wptr - str;
        str[lenSz] = 0x0;
        memcpy(psk,moduleParam.hub.passwdAP,strlen(moduleParam.hub.passwdAP));
        psk[strlen(moduleParam.hub.passwdAP)] = 0x0;
        if(psk[0] == 0x0){
          sprintf(psk,"53290921\0");
        }
//        status =  rsi_wlan_ap_start((int8_t *)SSID, CHANNEL_NO, SECURITY_TYPE, ENCRYPTION_TYPE, PSK, BEACON_INTERVAL, DTIM_COUNT);
        status =  rsi_wlan_ap_start((int8_t *)str, CHANNEL_NO, SECURITY_TYPE, ENCRYPTION_TYPE, psk, BEACON_INTERVAL, DTIM_COUNT);
        if(status != RSI_SUCCESS)
        {
          break;
        }

      }
    case RSI_WLAN_AP_UP_STATE:
      { 
        //! create UDP socket at the Access point
        server_socket = rsi_socket_async(AF_INET,SOCK_STREAM,0,socket_async_recive);
        
        if(server_socket < 0)
          while(1);
                
        memset(&server_addr,0,sizeof(server_addr));
        server_addr.sin_family = AF_INET;
#ifdef APP_COMM_MBTCP
        server_addr.sin_port = htons(502);
#else
        server_addr.sin_port = htons(DEVICE_PORT);
#endif
        status = rsi_bind(server_socket,(struct rsi_sockaddr*)&server_addr,sizeof(server_addr));
        if(status != RSI_SUCCESS){
          status = rsi_wlan_get_status();
          rsi_shutdown(server_socket,0);
          while(1);
        }        

        status = rsi_listen(server_socket,1);
        if(status != RSI_SUCCESS){
          status = rsi_wlan_get_status();
          rsi_shutdown(server_socket,0);
          while(1);
        }        

        rsi_wlan_app_cb.state = RSI_WLAN_SOCKET_SERVER_LISTEN_STATE; 
      }
    case RSI_WLAN_SOCKET_SERVER_LISTEN_STATE:
      {

        addr_size = sizeof(server_socket);
        //! Socket accept
        new_socket = rsi_accept(server_socket, (struct rsi_sockaddr *)&client_addr, &addr_size);
//        new_socket = rsi_accept_async(server_socket, (struct rsi_sockaddr *)&client_addr, &addr_size);
        
        if(new_socket < 0)
        {
          status = rsi_wlan_get_status();
          rsi_shutdown(server_socket, 0);
          while(1);
        }
        appParam.clientSocket = new_socket;
        if(connected_cb){
          connected_cb(&new_socket);
        }
        //! update wlan application state
        rsi_wlan_app_cb.state = RSI_WLAN_SOCKET_CONNECTED_STATE; 
        //rsi_app_wlan_send(new_socket,(int8_t *)"Hello from TCP client 1!!!", (sizeof("Hello from TCP client!!!") - 1));
      }
    case RSI_WLAN_SOCKET_CONNECTED_STATE:
      {
        eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
        if(evt & SOCKET_CONNECT){
          rsi_wlan_app_cb.socket_evt_map &= ~SOCKET_CONNECT;
        }
        if(evt & SOCKET_DISCONNECTED){
          rsi_wlan_app_cb.socket_evt_map &= ~SOCKET_DISCONNECTED;
          rsi_wlan_app_cb.state = RSI_WLAN_SOCKET_SERVER_LISTEN_STATE; 
          //status = rsi_shutdown(appParam.clientSocket,0);
          chEvtSignal(appParam.mainThread,EV_CLIENT_DISCONNECT);
        }
        if(evt & SOCKET_TX){
          rsi_wlan_app_cb.socket_evt_map &= ~SOCKET_TX;
//          status = rsi_send(new_socket,rsi_wlan_app_cb.wlan_buffer, rsi_wlan_app_cb.wlan_pkt_length, 0);
          status = rsi_send(new_socket,appParam.sdBuffer.r, appParam.sdBuffer.szToWrite, 0);
          rsi_wlan_app_cb.wlan_buf_in_use = 0;
          // check if fail
          if(status < 0){
            chEvtSignal(appParam.mainThread,EV_CLIENT_DISCONNECT);
          }
        }
        if(evt & SOCKET_RX){
          rsi_wlan_app_cb.socket_evt_map &= ~SOCKET_RX;
          if(dataArrived)
            dataArrived(&new_socket);
          else
            chEvtSignal(appParam.mainThread,EV_CMD_RX);
        }
        if(evt & SOCKET_READ){
          rsi_wlan_app_cb.socket_evt_map &= ~SOCKET_RX;
          if(dataArrived)
            dataArrived(&new_socket);
          else
            chEvtSignal(appParam.mainThread,EV_CMD_RX);
        }
      }
    default:
      break;
  }

  }

}

int32_t rsi_bt_app_send_to_wlan(uint8_t msg_type, uint8_t *buffer, uint32_t length)
{
	int32_t status=0;
  switch(msg_type)
  {
    case RSI_DATA:
      {
        //! buffer is in use or not
        if(!rsi_wlan_app_cb.bt_buf_in_use)
        {
          //! if not in use

          //! copy the buffer to wlan app cb bt buffer
          memcpy(rsi_wlan_app_cb.bt_buffer, buffer, length); 

          //! hold length information
          rsi_wlan_app_cb.bt_pkt_length = length;

          //! make buffer in use
          rsi_wlan_app_cb.bt_buf_in_use = 1;

          //! raise event to wlan app task
          rsi_wlan_app_cb.event_map |= RSI_BT_EVENT;
#ifdef RSI_WITH_OS
          //status = rsi_semaphore_post(&wlan_thread_sem);
#endif
        }
        else
          //!if buffer is in use
        {
#ifdef RSI_WITH_OS
          //rsi_semaphore_post(&wlan_thread_sem);
#endif
          return -1;
          //! return error 
        }
      }
  }
  return 0;
}


//! callback functions

//! stations connect notify call back handler in AP mode
void rsi_stations_connect_notify_handler(uint16_t status, uint8_t *buffer, const uint32_t length)
{

  //! increment connected stations count
  rsi_stations_count++;

}

//! stations disconnect notify call back handler in AP mode
void rsi_stations_disconnect_notify_handler(uint16_t status,  uint8_t *buffer, const uint32_t length)
{
  //! decrement connected stations count
  rsi_stations_count--;
}

//! packet receive notify call back handler in AP mode
void rsi_packet_receive_notify_handler(uint16_t status, uint8_t *buffer, uint32_t length)
{

  //! check destination mac address of the received packet 
  status = rsi_wlan_check_mac_address(buffer, length);

  if(status != RSI_SUCCESS)
  {
    return;
  }
  else
  {
    if(!rsi_wlan_app_cb.wlan_buf_in_use)
    {
      //! if not in use

      //! copy the buffer to wlan app cb wlan buffer
      memcpy(rsi_wlan_app_cb.wlan_buffer, buffer, length); 

      //! hold length information
      rsi_wlan_app_cb.wlan_pkt_length = length;

      //! make buffer in use
      rsi_wlan_app_cb.wlan_buf_in_use = 1;

      //! raise event to wlan app task
      rsi_wlan_app_cb.event_map |= RSI_WLAN_EVENT;


    }
    else
      //!if buffer is in use
    {
      return;
    }


  }
}


//! Function to check destination mac address of the recieved packet
int32_t rsi_wlan_check_mac_address(uint8_t *buffer, uint32_t length)
{
  rsi_ether_header_t *ether_header;

  ether_header = (rsi_ether_header_t *)buffer;

  if ((memcmp(ether_header->ether_dest, rsi_wlan_app_cb.device_params.module_mac_addr,MAC_ADDRESS_SIZE) == 0) ||
      (ether_header->ether_dest[0] == 0xFF))
  {
    return RSI_SUCCESS;
  }
  return RSI_FAILURE;

}

int32_t rsi_wlan_set_ipaddress(uint32_t ip_addr, uint32_t network_mask,uint32_t gateway)
{
  //! Configure IP 
  int32_t status = rsi_config_ipaddress(RSI_IP_VERSION_4, RSI_STATIC, (uint8_t *)&ip_addr, (uint8_t *)&network_mask, (uint8_t *)&gateway, NULL, 0,0);
  if(status != RSI_SUCCESS)
  {
    return status;
  }
  //! hold ip address of the Access point in a variable
  rsi_wlan_app_cb.device_params.module_ip_addr = ip_addr;
  return RSI_SUCCESS;
}


int32_t rsi_wlan_check_packet_type(uint8_t *buffer, uint32_t length)
{
  rsi_arp_packet_t *arp_packet;
  rsi_ip_header_ping_t rsi_ip_header_ping;
  rsi_ether_header_t *ether_header;
  rsi_ip_header_t   *ip_header;
  rsi_udp_header_t   *udp_header;

  ether_header = (rsi_ether_header_t *)buffer;

  if (ether_header->ether_type == RSI_ARP_PACKET)
  {
    /* Setup a pointer to the ARP message.  */
    arp_packet = (rsi_arp_packet_t *)(buffer  + ETHERNET_HEADER_OFFSET);

    /* Determine if the ARP message type is valid.  */
    if (arp_packet->opcode == ARP_OPTION_REQUEST)
    {
      return ARP_OPTION_REQUEST; 
    }
    else if (arp_packet->opcode == ARP_OPTION_RESPONSE)
    {
      return ARP_OPTION_RESPONSE; 
    }
    else
    {
      return RSI_FAILURE; 
    }
  }
  else if (ether_header->ether_type == RSI_IP_PACKET) 
  {
    memcpy((uint8_t *)&rsi_ip_header_ping ,&buffer[14], sizeof(rsi_ip_header_ping_t));
    /* Setup a pointer to the IP message.  */
    ip_header = (rsi_ip_header_t *)(buffer  + ETHERNET_HEADER_OFFSET);

    /* Determine what protocol the current IP datagram is.  */

    if((rsi_bytes4R_to_uint32(ip_header->dstaddr) == rsi_wlan_app_cb.device_params.module_ip_addr)&&(ip_header->proto == UDP_PROTOCOL))
    {

      udp_header = (rsi_udp_header_t *)(buffer  + IP_HEADER_OFFSET);

//      if(rsi_wlan_app_cb.device_params.source_port == convert_le_be(udp_header->dest_port))
//      {
//        //! copy sender ip address in a global variable
//        rsi_wlan_app_cb.device_params.sender_ip_addr = rsi_bytes4R_to_uint32(ip_header->srcaddr); 
//        
//        //! copy remote port in a global variable
//        rsi_wlan_app_cb.device_params.remote_port = convert_le_be(udp_header->src_port); 
//      }
      return RSI_IP_PACKET; 

    }
    //! Check for IP Packet
    //! Check for ICMP Protocol
    else if (rsi_ip_header_ping.ip_header_word_2[1] == PROTO_ICMP) 
    {
      //rsi_wlan_send_ping_response(rsi_wlan_app_cb.wlan_buffer, rsi_wlan_app_cb.wlan_pkt_length);
    }
  }
  return RSI_FAILURE; 
}

//! Function to get mac address from arp response
void rsi_wlan_get_mac_address(uint8_t *buffer, uint32_t length)
{
  rsi_arp_packet_t *arp_packet;
  rsi_ether_header_t *ether_header;


  ether_header = (rsi_ether_header_t *)buffer;

  //! Setup a pointer to the ARP message.  
  arp_packet = (rsi_arp_packet_t *)(buffer  + ETHERNET_HEADER_OFFSET);

  //! copy Access point mac address to source address of ARP response packet
  if(memcmp(ether_header->ether_dest, rsi_wlan_app_cb.device_params.module_mac_addr, MAC_ADDRESS_SIZE)== 0)
  { 
    //! Pick up the sender's physical address from the message.  
    memcpy(rsi_wlan_app_cb.device_params.sender_mac_addr,(arp_packet->sender_mac_addr),MAC_ADDRESS_SIZE); 
  }

}

void rsi_app_wlan_cb_data_rx(void(*cb)(void *p))
{
  dataArrived = cb;
}

void rsi_app_wlan_cb_client_connect(void (*cb)(void *p))
{
  connected_cb = cb;
}

void rsi_app_wlan_cb_client_disconnect(void (*cb)(void *p))
{
  disconnect_cb = cb;
}

#endif