#ifndef _SENSOR_HUB_
#define _SENSOR_HUB_

#include "sysParam.h"
#include "mmssocket.h"
#include "battery_mang.h"
#include "socket.h"
#include "ff.h"

#define SW_VERSION_NUMBER       0x19042401
#define DATA_PATH_TRANMIT       0x01
#define DATA_PATH_LOGSD         0x02

#define BUFFER_SZ       1024

#define EV_ADXL_FIFO_FULL EVENT_MASK(0)

#define EV_EXT_INT              EVENT_MASK(0)
#define EV_TIMEOUT              EVENT_MASK(1)
#define EV_CMD_RUN              EVENT_MASK(2)
#define EV_CMD_STOP             EVENT_MASK(3)
#define EV_ADC_RESTART          EVENT_MASK(4)
#define EV_ACT_BATT             EVENT_MASK(5)
#define EV_CMD_SINGLE           EVENT_MASK(6)
#define EV_BMI_INT1             EVENT_MASK(7)
#define EV_BMI_INT2             EVENT_MASK(8)
#define EV_BUFFER_FULL          EVENT_MASK(9)
#define EV_P7_EXINT             EVENT_MASK(10)
#define EV_BT_EXTINT             EVENT_MASK(11)
#define EV_SAVE_PARAM           EVENT_MASK(12)
#define EV_LOAD_PARAM           EVENT_MASK(13)
#define EV_CMD_RESET           EVENT_MASK(14)
#define EV_CMD_EXTINT           EVENT_MASK(15)
#define EV_SDMMC_INSERT         EVENT_MASK(16)
#define EV_USER_BUTTON_0        EVENT_MASK(17)
#define EV_CMD_READ_BATT        EVENT_MASK(18)
#define EV_CMD_READ_ENV        EVENT_MASK(19)

#define EV_SD_WRITE_0           EVENT_MASK(20)
#define EV_SD_WRITE_1           EVENT_MASK(21)

#define EV_FSWRITE_DATA         EVENT_MASK(22)
#define EV_FSWRITE_DATA_HI         EVENT_MASK(23)
#define EV_FSWRITE_LOG         EVENT_MASK(24)
#define EV_MAIN_CLIENT_CONNECT  EVENT_MASK(25)
#define EV_MAIN_CLIENT_DISCONNECT  EVENT_MASK(26)


#define POWER_IF_ENABLE()       palClearPad(GPIOC,GPIOC_PWR_EN_IF)
#define POWER_IF_DISABLE()       palSetPad(GPIOC,GPIOC_PWR_EN_IF)
#define POWER_P7_ENABLE()       palClearPad(GPIOC,GPIOC_PWR_EN_P7)
#define POWER_P7_DISABLE()       palSetPad(GPIOC,GPIOC_PWR_EN_P7)
#define POWER_P8_ENABLE()       palClearPad(GPIOC,GPIOC_PWR_EN_P8)
#define POWER_P8_DISABLE()       palSetPad(GPIOC,GPIOC_PWR_EN_P8)

typedef int8_t (*write_func)(uint8_t*,uint16_t n);

typedef enum{
  SEN_IDLE,
  SEN_SINGLE,
  SEN_RUNNING
}sensor_state_t;


typedef struct{
  int32_t x;
  int32_t y;
  int32_t z;
}adxl_data_t;

typedef struct{
  ioportid_t port;
  uint16_t pad;
}io_config_t;

typedef struct{
  char data[64];
  char data_hr[64];
  char log[64];
}log_filename_t;

typedef struct{
  FIL f;
  char name[64];
  uint32_t lastSync;
}log_file_info_t;

enum {
  DATA,
  DATA_HIR,
  LOG
};

typedef struct{
  char path[256];
  char fileName[64];
  uint32_t offset;
  uint32_t nofBytes;
  uint32_t byteToRead;
  uint8_t dirChanged;
}file_op_t;



typedef struct{
  uint16_t sz;
  uint16_t pendSz;      // pending size that not calculate checksum
  uint8_t pendType;
  uint8_t *ptr;
  uint8_t buffer[BUFFER_SZ];
}buffer_t;


#define SD_BUFFER_SIZE  3072+8 // push CMD_STRUCT_SZ in the front of buffer
typedef struct{
  uint8_t *w,*r;
  uint8_t id;
  uint16_t length;
  uint16_t transition;
  uint16_t flushSize;
  uint16_t szToWrite;
  uint8_t buf_a[SD_BUFFER_SIZE],buf_b[SD_BUFFER_SIZE];
}sd_buffer_t;

typedef struct{
  uint8_t *dptr;
  size_t sz;
}_fs_msg_t;



typedef struct{
  bsdSocket_t sock;
  uint32_t sdSize;
  uint8_t ledBlinkPeriod;
  virtual_timer_t blinker;
  thread_t *workingThread;
  thread_t *serialThread;
  thread_t *fSThread;
  struct tm tim_now;
  dev_type_t sensor[6];
  log_filename_t fileName;
  uint8_t pid[3];
  file_op_t activeFile;
  uint8_t dataPath;
  int8_t tmp;
  uint16_t flushSize;
  buffer_t wbuf,rbuf;
  uint16_t wrIndex;
  uint32_t ipAddr;
  uint32_t gwAddr;
  uint8_t macAddr[6];
  uint32_t runSecs;
  int8_t maxCh;
  int8_t chList[8];
  mutex_t mtx;
  mutex_t mtx_fs;
  //mutex_t mtx_uart;
  uint8_t singleRead;
  batt_spec_t battery;
  double temp, humidity;
  uint8_t user_press_inc;
  uint8_t user_pressed_sec;
  uint8_t user_cfg_mode;
  size_t lig_file_size[3];
  log_file_info_t data_file,data_file_hi,log_file;
  systime_t writeTime;
  sd_buffer_t sdBuffer;
  uint8_t connState;
}app_param_t;


extern thread_t *mainThread;
extern app_param_t appParam;

void cmdSetup(BaseSequentialStream *chp, int argc, char *argv[]);
void cmdControl(BaseSequentialStream *chp, int argc, char *argv[]);
void cmdNull(BaseSequentialStream *chp, int argc, char *argv[]);

void app_loadDefault();
void sensorHubInit(void);
#endif