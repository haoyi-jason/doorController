#ifndef _BATTERY_MANG_H
#define _BATTERY_MANG_H
#include "hal.h"

#define EV_BATT_BASE    16
#define EV_BATT_CLI     EVENT_MASK(EV_BATT_BASE + 0)

typedef enum{
  CHARGER_OFF,
  CHARGER_ON
}charge_control_t;
typedef enum{
  BATT_DISCHARGE,
  BATT_CHARGE,
}current_direction_t;

typedef struct{
  int16_t overTemp;
  int16_t underTemp;
  int16_t tempHys;
}temp_monitor_t;

typedef enum{
  UNSEALED,
  SEALED,
}battery_seal_t;

typedef struct{
  int8_t tca_set;
  int8_t tca_clear;
  int8_t fc_set;
  int8_t fc_clear;
  int16_t deltaT;
}charge_termination_t;

typedef struct{
  int8_t voltage;
  int8_t standby;
  int16_t load;
}data_t;

typedef struct{
  uint8_t soc1_set;
  uint8_t soc1_clear;
  uint8_t socf_set;
  uint8_t socf_clear;
}discharge_t;

typedef struct{
  uint8_t batFlag;
  charge_control_t  chgCtrl;
  bool sealed;
  current_direction_t chgState;
  uint16_t capacity;    // maH, 1 to 65535
  uint16_t capRemain;
  int16_t soc;
  int16_t soh;
  uint16_t voltage;
  temp_monitor_t tempMon;
  charge_termination_t chargMon;
  data_t chargeData;
  discharge_t dischargeMon;
  uint8_t lbtThreshold;
}batt_spec_t;

// serial CLI
void cmdBattery(BaseSequentialStream *chp, int argc, char *argv[]);

msg_t battStartCharge();
msg_t battStopCharge();
uint16_t battGetPercent();
int16_t battGetPower();
msg_t battSetCapacity(uint16_t cap);
msg_t battInit(I2CDriver *devp, thread_t *mainThread);
msg_t battPollState(void);
#endif