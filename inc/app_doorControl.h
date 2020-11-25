#ifndef _APP_DOORCONTROL_
#define _APP_DOORCONTROL_

#include "ch.h"
#include "hal.h"
#include "rsi_bt_common.h"
#include "rsi_bt_config.h"

#define VERSION 0x2011
#define VERSION_2       0x2501

typedef void (*rsi_interrupt_cb)(void);
typedef void (*lora_interrupt_cb)(void *);

#define EV_OPEN_DOOR1   EVENT_MASK(0)
#define EV_OPEN_DOOR2   EVENT_MASK(1)
#define EV_DOOR_LOCK   EVENT_MASK(2)
#define EV_DOOR_UNLOCK   EVENT_MASK(3)
#define EV_TG1_TRIGGER   EVENT_MASK(4)
#define EV_TG2_TRIGGER   EVENT_MASK(5)
#define EV_USR_TRIGGER   EVENT_MASK(6)
#define EV_CLIENT_CONNECT   EVENT_MASK(7)
#define EV_CLIENT_DISCONNECT   EVENT_MASK(8)
#define EV_CMD_RX   EVENT_MASK(9)
#define EV_TG1_MBTRG   EVENT_MASK(10)

#define EV_SYS_CLEAR_ERROR EVENT_MASK(29)
#define EV_SYS_RESET EVENT_MASK(30)
#define EV_SYS_SAVE_PARAM EVENT_MASK(31)
// DIP swith definition
#define MODEL_DOOR_LEFT       0x01
#define LOCK_ENABLED           0x02
#define DUAL_DOOR               0x04

#define VDDA_MV         3250  // vref = 2.5*(1+3.3/10) = 3325, measured 3250
#define LSB_MV          VDDA_MV/4096
#define CURRENT_SHUNT   220
#define MV_PER_MA      CURRENT_SHUNT*0.0024 // 0.24% mirror currrent
#define MA_PER_LSB      MV_PER_MA/LSB_MV         

enum do_map{
  MC1_EN,
  MC1_CTRL1,
  MC1_CTRL2,
  MC2_EN,
  MC2_CTRL1,
  MC2_CTRL2,
  M1_ON,
  M2_ON,
  M3_ON,
  BUZZER,
  DOOR_CLOSED,
  DOOR_ABNORMAL,
  DOOR_OPEN_DONE,
  ID_LATCH,
  ID_EN,
};

enum di_map{
  ULOCK_INP,
  LLOCK_INP,
  DOOR1_OPEN,
  DOOR2_OPEN,
  DOOR1_INP,
  DOOR2_INP,
  ANG1_ICP,
  ANG2_ICP,
  USR_BTN,
  TG1,
  TG2
};

enum motor_map{
  MOTOR_MAIN,
  MOTOR_SUB,
  MOTOR_LOCK
};

enum motor_dir{
  DIR_NEG,
  DIR_POS,
};

enum beep_map{
  BEEP_NORMAL,
  BEEP_ERROR,
  BEEP_ERROR2
};

enum {
  ERR_LOCKED,
  ERR_OVERCURRENT,
  ERR_INPOSITION,
  ERR_LOCK_FAIL=4,
  ERR_LOCK_FB_NONE,
  ERR_LOCK_FB_DUAL,
  ERR_LOCK_REVERSED,
  ERR_TIMEOUT=8
};

enum alarm_code{
  ALM_NONE,
  ALM_DOOR_BLOCKING,
  ALM_DOOR_OC,
  ALM_LOCK,
  ALM_INP
};

#define ERR_M1_MASK     0x10
#define ERR_M2_MASK     0x20

// possible motor/door error
enum{
  ERR_NOERROR,
  ERR_MOTOR_POL,        // polarity error
  ERR_MOTOR_WIR,        // drive motor without angle change
  ERR_MOTOR_ANGS,       // angle sensor @5% or 95%
  ERR_MOTOR_POSS,       // Photocoupler not work 
  ERR_MOTOR_OC,         // over current
  ERR_MOTOR_LC,         // locked
};

// possible LOCK error
enum{
  ERR_LOCK_DIR = 1,
  ERR_LOCK_LOST
};

enum err_n{
  RPT_MMOTOR_POLARITY,
  RPT_MMOTOR_WIRING,
  RPT_MANG_SENSOR,
  RPT_MPOS_SENSOR,
  RPT_MMOTOR_OC,
  RPT_SMOTOR_POLARITY,
  RPT_SMOTOR_WIRING,
  RPT_SANG_SENSOR,
  RPT_SPOS_SENSOR,
  RPT_SMOTOR_OC,
  RPT_WRONG_PARAM,
  RPT_LOCK
};

//#define SET_ERR(b,x)            (b |= (1<<x))
#define SET_ERR(b,x)            (b = x)
#define CLR_ERR(b,x)            (b &= ~(1<<x))
//#define IS_ERR(b,x)             (b & (1<<x))
#define IS_ERR(b,x)             (b ==x)

#define M1_ERR(b,x)     (b = ERR_M1_MASK | x)
#define M2_ERR(b,x)     (b = ERR_M2_MASK | x)


#define ERR_M1_CURRENT  0x01
#define ERR_M2_CURRENT  0x02
#define ERR_M3_CURRENT  0x03
#define ERR_M1_LOCKED   0x10
#define ERR_M2_LOCKED   0x20
#define ERR_ANG1_ERROR   0x100
#define ERR_ANG2_ERROR   0x200
#define ERR_ANG_MIXED    0x400
//#define ERR_LOCK_FAIL   0x800
//#define ERR_LOCK_FB_NONE        0x1000
//#define ERR_LOCK_FB_DUAL        0x2000
//#define ERR_LOCK_REVERSED       0x4000

enum motor_msg{
  MOTOR_OK,
  MOTOR_1_OVC,
  MOTOR_2_OVC,
  MOTOR_3_OVC,
  DOOR_1_LOCK,
  DOOR_2_LOCK
};

typedef struct{
  uint16_t on;
  uint16_t off;
  int8_t cycles_to_run;
  uint8_t state; // ON of OFF
  int8_t cycles;
}beep_pattern_t;

#define INVALID_PAD     0xff

typedef int16_t (*angleMapFunc)(int32_t);

typedef struct{
  ioportid_t gate_port;
  uint16_t gate_pin;
  uint16_t value;
}_output_config;


typedef struct dio_map_s{
  ioportid_t port;
  uint16_t pad;
  void (*set)(struct dio_map_s*);
  void (*clear)(struct dio_map_s*);
  uint8_t (*read)(struct dio_map_s*);
}dio_map_t;

typedef struct {
  uint8_t id;
  dio_map_t *gate; // relay control pad
  dio_map_t *en; // map ID to do_map
  uint8_t ctrl1; // 
  uint8_t ctrl2;
  dio_map_t *pel; // positive end limit
  dio_map_t *mel; // negative end limit
  uint8_t dir;
  uint8_t speed; // 0~100 pwm duty
  int16_t angle; // current angle
  int16_t angle_last;
  float dc_current; // current milli-amp
  bool running;
  uint8_t active_delay; // seconds
  PWMDriver *posDriver;
  uint8_t posChannel;
  PWMDriver *negDriver;
  uint8_t negChannel;
  uint16_t holdingTicks;
  uint16_t posSldAngle;
  uint16_t negSldAngle;
  uint8_t inhome;
  bool inPos;
  uint16_t waitCycles;
  uint8_t currentDir;
  //int16_t angleHistory[16]; // angle history, record every cycle
  //uint16_t ang_index;
  uint8_t ang_keep_counts;
  uint8_t posDir;
  uint8_t negDir;
  uint8_t orgFound;
  uint8_t newSpeed;
  uint8_t newDir;
}_motor_config_t;

typedef struct{
  uint16_t ms_on;
  uint16_t ms_off;
  uint8_t times;
}beep_t;

typedef struct{
  systime_t last;
  uint32_t period;
  uint32_t width;
  float percent;
  float degree;
}_icu_value_t;

typedef struct{
  uint16_t idle_current;
  uint16_t op_current;
  uint16_t max_current;
  uint32_t lockTimes;
  uint32_t opTimes;
}_door_state_t;

typedef struct{
  uint32_t rsi_app_async_event_map;
#ifdef RSI_BT_ENABLE
  rsi_bt_resp_get_local_name_t local_name;
  uint8_t local_dev_addr[RSI_DEV_ADDR_LEN];
#endif
  uint8_t str_conn_bd_addr[18];
}rsi_bt_data_t;

typedef struct{
  thread_t *rsi_bt;
  thread_t *rsi_wlan;
  thread_t *rsi_driver;
}rsi_handle_t;

typedef struct{
  uint8_t *w,*r;
  uint8_t id;
  uint16_t length;
  uint16_t transition;
  uint16_t flushSize;
  uint16_t szToWrite;
  //uint8_t buf_a[SD_BUFFER_SIZE],buf_b[SD_BUFFER_SIZE];
  uint8_t *buf_a,*buf_b;
}sd_buffer_t;

#ifndef BUFFER_SZ
#define BUFFER_SZ       64
#endif

typedef struct{
  int16_t sz;
  uint8_t buffer[BUFFER_SZ];
}buffer_t;


typedef struct{
  uint32_t runSecs;
  dio_map_t *doMap;
  dio_map_t *diMap;
  _icu_value_t icu[2]; 
  uint16_t boardID;
  _door_state_t doorState[3];
  uint16_t vr6;
  _motor_config_t *motorConfig;
  thread_t *mainThread,*m1Thread,*m2Thread,*m3Thread;
  thread_t *transThread;
  thread_t *beepThread;
  thread_t *closingThread;
  beep_pattern_t *beep;
  virtual_timer_t vtBeep;
  uint8_t userInPress;
  uint8_t userPressedTime;
  beep_t buzzer;
  uint8_t beepStage;
  rsi_bt_data_t rsi_bt;
  rsi_handle_t rsi_handle;
  rsi_interrupt_cb rsi_int;
  int32_t clientSocket;
  uint8_t linkKey[16];
  sd_buffer_t sdBuffer;
  buffer_t rxBuf,txBuf;
  uint16_t errState;
  uint8_t closeByTimeout;
  uint8_t openTimes;
  uint8_t doorClosing;
  uint8_t doorReopen;
}_appParam_t;

extern _appParam_t appParam;

void updateConstrain();
void reset_buzzer(void);
static msg_t doorOpenCtrl(void);
static msg_t doorCloseCtrl(void);

#endif