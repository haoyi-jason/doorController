/**
file: battery_mang.c
description: support Li-ion battery charge and column-counter

charger: BQ24075
column-counter: BQ27441

support function:
1. charger on-off control
2. column-counter seal/unseal control
3. battery soc

*/

#include "ch.h"
#include "hal.h"
#include <string.h>
#include "chprintf.h"
#include "battery_mang.h"
#include "charger/bq27441.h"
#include "sensorhub.h"

#define CHARGER_ON()  palClearPad(GPIOC,GPIOC_CHG_NCE)
#define CHARGER_OFF()  palSetPad(GPIOC,GPIOC_CHG_NCE)
//#define SYSTEM_ON()  palClearPad(GPIOC,GPIOC_CHG_OFF)
//#define SYSTEM_OFF()  palSetPad(GPIOC,GPIOC_CHG_OFF)
#define SYSTEM_ON()  
#define SYSTEM_OFF()  
#define POWER_GOOD()    (palReadPad(GPIOC,GPIOC_CHG_PG) == PAL_LOW)
#define CHARGING()      (palReadPad(GPIOC,GPIOC_CHG_CHG) == PAL_LOW)

thread_t *sysThread = NULL;

msg_t battStartCharge()
{
 // SYSTEM_ON();
  CHARGER_ON();
  chThdSleepMilliseconds(100);
  // check if power good
  if(POWER_GOOD())
    return MSG_OK;
  else
    return MSG_RESET;
      
}

msg_t battStopCharge()
{
  CHARGER_OFF();
  //SYSTEM_OFF();
  return (msg_t)0;
}

uint16_t battGetPercent()
{
  return bq27441_soc(FILTERED);
}

int16_t battGetPower()
{
  return bq27441_power();
}

msg_t battSetCapacity(uint16_t cap)
{
  unseal();
  bq27441_setCapacity(cap);
  seal();
  return MSG_OK;
}

msg_t battPollState(void)
{
  appParam.battery.soc = battGetPercent();
  appParam.battery.voltage = bq27441_voltage();
  appParam.battery.capRemain = bq27441_capacity(REMAIN);
  appParam.battery.capacity = bq27441_capacity(FULL);
  
  if(POWER_GOOD()){
    if(appParam.battery.chgState == BATT_DISCHARGE && appParam.battery.soc < 95){
      appParam.battery.chgState = BATT_CHARGE;
      battStartCharge();
    }
  }else{
    if(appParam.battery.chgState == BATT_CHARGE){
      appParam.battery.chgState = BATT_DISCHARGE;
      battStopCharge();
    }
  }
  appParam.battery.chgCtrl = CHARGING()?BATT_CHARGE:BATT_DISCHARGE;
  return (msg_t)0;
}

msg_t battInit(I2CDriver *devp, thread_t *mainThread)
{
  msg_t ret = MSG_OK;
  if(!devp) 
    ret = MSG_RESET;
  else{
    ret = bq27441Init(devp);
    sysThread = mainThread;
    appParam.battery.chgCtrl = CHARGER_OFF;
    appParam.battery.sealed = sealed();
    appParam.battery.chgState = BATT_DISCHARGE;
    appParam.battery.capacity = bq27441_capacity(FULL);
    appParam.battery.soc = battGetPercent();
    //battery.soh
    appParam.battery.voltage = bq27441_voltage();
    appParam.battery.lbtThreshold = 10;  // percent
  }
  return ret;
}

/*
battery CLI, B ...
B: command prefix

B C 0/1: charger off/on
B S 0/1: sealoff/on
B V   : view SOC
B c x: set capacity (maH)

*/

void cmdBattery(BaseSequentialStream *chp, int argc, char *argv[])
{
  int16_t v1,v2;
  if(argc < 1) return;
  switch(*argv[0]){
  case 'C':
    if(*argv[1] == '0'){
      appParam.battery.chgCtrl = CHARGER_OFF;
      battStopCharge();
    }
    else if(*argv[1] == '1'){
      appParam.battery.chgCtrl = CHARGER_ON;
      battStartCharge();
    }
    else
      chprintf(chp,"b C %d\r\n",appParam.battery.chgCtrl);
    break;
  case 'S':
    if(*argv[1] == '0'){
      appParam.battery.sealed = !unseal();
    }
    else if(*argv[1] == '1'){
      appParam.battery.sealed = seal();
    }
    else
      chprintf(chp,"Battery %s\r\n",appParam.battery.sealed?"SEALED":"UNSEALED");
    break;
  case 'V':
    //battery.soc = battGetPercent();
    chprintf(chp,"SOC=%d VOLT=%d CAP(F)=%d CAP(R)=%d%\r\n",
             appParam.battery.soc,appParam.battery.voltage,appParam.battery.capacity,appParam.battery.capRemain);
    break;
  case 'c':
    if(*argv[1] == '?'){
      v1 = bq27441_capacity(REMAIN);
      v2 = bq27441_capacity(FULL);
      chprintf(chp,"Capacity Full/Remain:%d/%d\r\n",v2,v1);
    }
    else{
      v1 = strtol(argv[1],NULL,10);
      appParam.battery.capacity = v1;
      battSetCapacity(appParam.battery.capacity);
    }
    break;
//  default:
    
  }
  
}