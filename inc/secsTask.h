#ifndef _SECSTASK_H_
#define _SECSTASK_H_

#include "lwip/api.h"
#include "lwip/tcp.h"
#include "lwip/sockets.h"

// SECS format code
#define FMT_LIST    (0x0)
#define FMT_BIN     (0x8<<2)
#define FMT_BOOL    (0x9<<2)
#define FMT_ASC     (0x10<<2)
#define FMT_JIS8    (0x11<<2)
#define FMT_CHAR2   (0x12<<2)
#define FMT_I8   (0x18<<2)
#define FMT_I1    (0x19<<2)
#define FMT_I2   (0x1A<<2)
#define FMT_I4   (0x1B<<2)
#define FMT_F8    (0x20<<2)
#define FMT_F4    (0x24<<2)
#define FMT_U8  (0x28<<2)
#define FMT_U1   (0x29<<2)
#define FMT_U2  (0x2A<<2)
#define FMT_U4  (0x2B<<2)

enum secs_role_e{
  SECS_ACTIVE,
  SECS_PASSIVE
};

typedef enum {
  S_NOT_CONNECTED,
  S_CONNECTED,
  S_NOT_SELECTED,
  S_SELECTED
}secs_connection_state_t;

typedef enum {
  ST_DATA,
  ST_SELECT_REQ,
  ST_SELECT_RSP,
  ST_DESELECT_REQ,
  ST_DESELECT_RSP,
  ST_LINKTEST_REQ,
  ST_LINKTEST_RSP,
  ST_REJECT_REQ,
  ST_RESERVED,
  ST_SEPERATE_REQ,
  ST_RESERVED2
}secs_stype_t;


typedef struct
{
  struct tcp_pcb *pxPCBServer;
  struct tcp_pcb *pxPCBClient;
  uint32_t serverAddr;
  uint16_t serverPortNum;
  uint16_t myPortNum;
  uint8_t autoConnect;
  uint8_t sendRespOnConnected;
  uint8_t timeoutT1;
  uint8_t timeoutT2;
  uint8_t timeoutT3;
  uint8_t timeoutT4;
  uint8_t timeoutT5;
  uint8_t timeoutT6;
  uint8_t timeoutT7;
  uint8_t timeoutT8;
  uint8_t selected;
}_hsms_config_t;

typedef struct varMap_s{
	struct varMap_s *next;
	char *name;
	void *value;
}varMap_t;

typedef struct hsms_data_s
{
    uint8_t format;
    uint32_t nofBytes;
    uint8_t *dptr;
    struct hsms_data_s *next;
}hsms_data_t;

typedef struct valueField_s{
	struct valueField_s *parent;
	struct valueField_s *next;
	struct valueField_s *child;
	varMap_t *varList;
	uint8_t nofParams;
	uint8_t secsType;
	uint8_t LengthBytes; // 1,2 or 3
	uint32_t textSize;	
}valueField_t;

typedef struct{
  uint32_t msgLen;
  uint16_t sessionID;
  uint8_t stream;
  uint8_t function;
  uint8_t PType;
  uint8_t SType;
  uint8_t sysByte[4];
}hsms_message_header_t;

typedef struct{
	hsms_message_header_t header;
	char *msgBody;
}hsms_message_body_t;



typedef struct{
  
  
}secs_struct_t;


#endif  // _secstask_h_