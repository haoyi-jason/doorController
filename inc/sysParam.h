#ifndef _SYSPARAM_
#define _SYSPARAM_
#include <time.h>




#define EEP_HEADING              0x45503031
#define EEP_STORE_OFFSET        0x100

typedef enum{
  SPAR_NONE,
  SPAR_EVEN,
  SPAR_ODD,
}com_parity_t;

typedef enum{
  SSTOP_1,
  SSTOP_1_5,
  SSTOP_2
}com_stopbit_t;

typedef enum{
  DATA_7 = 7,
  DATA_8
}com_databit_t;

typedef struct{
  uint8_t baudrate_id;
  uint32_t baudrate_val;
  com_parity_t parity;
  com_stopbit_t stop;
  com_databit_t data;
}serial_setting_t;

typedef struct{
  uint8_t ip[6];
  uint8_t mask[6];
  uint8_t gateway[6];
  uint8_t macaddr[6];
}lan_setting_t;

typedef struct{
  uint32_t flag;
  uint32_t verNum;
  uint32_t serialNum;
  uint8_t vender[32];
  uint8_t user[32];
}module_setting_t;

typedef struct{
  int32_t min;
  int32_t max;
}_boundary_t;

enum motor_type
{
  TYPE_NONE,
  TYPE_DOOR,
  TYPE_LOCK
};


typedef struct{
  int16_t lockRetryIdleCycles;
  uint8_t lockRetry;
  uint8_t triggerAngle; // angle to trigger start, 1 to 30 deg
  float degree_percent;
  uint8_t angleDiff;    // 雙開角度差
  uint8_t lockActiveTime; // 電鎖作動時間, 0.1s
  uint8_t waitTimeToClose;      // 開門等待時間.
  int16_t doorFreeAngle;        //  open angle to ignore follow criteria
  int16_t actionDelay;  //作動時間間隔
  int16_t angleValidDeg;         // 阻擋判斷角度差
  int16_t angleValidCycles;     // 阻擋判斷時間(cycles)
  uint16_t ramp;                        // 馬達加/減速值
  uint16_t adSampleIgnore;      // 啟動取樣延時
}_door_global_config_t;

typedef struct{
  uint8_t type;
  uint8_t normalSpeed;     // max speed %
  uint8_t slowSpeed;    // slow speed %
  uint8_t openAngle;    // in degree, 0 to 180
  uint8_t sldOpenAngle;   // slow down angle 
  uint8_t sldCloseAngle;   // slow down angle
  uint8_t openRevSpeed; // speed before open door, %
  uint8_t openRevTime;
  uint8_t closeFwdSpeed; // speed after door opened
  uint8_t closeFwdTime;  // holding time after door closed
  uint16_t normalMaxCurrent;  // in mA
  uint16_t slowMaxCurrent;
  uint8_t zero_angle_error;     // 關門允許誤差
  uint32_t lock_times; // 阻擋次數記錄
  uint16_t max_working_current;
  float zeroAngle;
}_door_config_t;


typedef struct module_param_s{
  module_setting_t param;
  serial_setting_t serial;
  lan_setting_t lan;
  _door_global_config_t doorConfig;
  _boundary_t doorAng[2];
  _door_config_t door[3];
  uint16_t action_delay_time;   // delay time between actions
  uint16_t angle_check_time; // 
  uint16_t angle_check_diff; //
  uint16_t ramp;
}module_params_t;

extern module_params_t moduleParam;

void sysSaveParams(void);
void sysParamInit(void);
void defaultParams(void);

#endif
