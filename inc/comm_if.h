#ifndef _COMM_IF_
#define _COMM_IF_

#define MAGIC   0xabba
#define MAGIC1  0xab
#define MAGIC2  0xba

#define MASK_CMD                0xa0
#define MASK_CMD_RET_OK         0x50
#define MASK_CMD_RET_ERR        0x70
#define MASK_CMD_RET_BUSY       0x90
#define MASK_DATA               0x30
#define MASK_CMD_RET_CFG        0x10

#define CMD_CMD1_CONTROL        0x01
#define CMD_CMD1_SETUP          0x02
#define CMD_CMD1_FILE           0x03
#define CMD_CMD1_CRITICAL       0x0C

#define CMD2_NOCRC_MASK         0x80
#define CMD2_CONTROL_STOP       0x00
#define CMD2_CONTROL_LOG        0x01
#define CMD2_CONTROL_START      0x02
#define CMD2_CONTROL_SINGLE     0x40
#define CMD2_CONTROL_READ_SOC   0x10
#define CMD2_CONTROL_READ_ENV   0x20

#define CMD2_SETUP_ACC_HI        0x01
#define CMD2_SETUP_INERTIAL      0x02
#define CMD2_SETUP_ADC           0x03
#define CMD2_SETUP_HT            0x04        
#define CMD2_SETUP_BATT          0x0C
#define CMD2_SETUP_COMM          0x0D
#define CMD2_SETUP_SYS           0x0E
#define CMD2_SETUP_SAVE          0x0F
#define CMD2_SETUP_AP_PASS       0x20
#define CMD2_SETUP_STA_SSID     0x21
#define CMD2_SETUP_STA_ST       0x22
#define CMD2_SETUP_STA_PASS     0x23
#define CMD2_SETUP_STA_TIMEOUT  0x24
#define CMD2_SETUP_WIFI_TYPE    0x25
#define CMD2_SETUP_VERNUM       0x91
#define CMD2_SETUP_SERNUM       0x92
#define CMD2_SETUP_USRSTR       0x93
#define CMD2_SETUP_VNDSTR       0x94

#define CMD2_FILE_OP_OK         0x10
#define CMD2_FILE_OP_ERR        0x20;
#define CMD2_FILE_LIST          0x01
#define CMD2_FILE_OPEN          0x02
#define CMD2_FILE_READ          0x03
#define CMD2_FILE_WRITE         0x04
#define CMD2_FILE_REMOVE        0x05
#define CMD2_FILE_CHDIR         0x06
#define CMD2_FILE_CLOSE         0x07

#define CMD2_CRITICAL_DEFAULT   0x01    // load default


// used for type low nibble with MASK_DATA
#define DATA_VSS        0x1
#define DATA_VSS_HI     0x2
#define DATA_DOORSPEED  0x3
#define DATA_BATTERY    0x4
#define DATA_ENV        0x5
#define DATA_FILE       0x6

#define CMD2_SYS_TYPE    0x1
#define CMD2_SYS_COMM    0x2
#define CMD2_SYS_RTC     0x3
#define CMD2_SYS_MODE    0x4
#define CMD2_SYS_SD_SAVE      0x5
#define CMD2_SYS_SD_DP   0x6
#define CMD2_SYS_SD_DPH  0x7
#define CMD2_SYS_SD_DPL  0x8
#define CMD2_SYS_SD_DATA 0xA


#define DATA_BUF_SIZE   1024

#define CMD_STRUCT_SZ   8
#define CMD_STRUCT_ACTSZ   6

enum{
  COMM_IF_CMD_START,
  COMM_IF_CMD_STOP,
  COMM_IF_CMD_SINGLE,
};

typedef struct {
  uint8_t magic1;
  uint8_t magic2;
  uint8_t type;
  uint8_t pid;
  uint16_t len;
  uint16_t chksum;
} cmd_header_t;

int8_t commif_init(thread_t *t);
int16_t commif_parse(char *buf,char *resp);
uint16_t cmd_checksum(uint8_t *data, uint16_t length);

#endif