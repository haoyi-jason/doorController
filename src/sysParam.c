#include "ch.h"
#include "hal.h"
#include "sysParam.h"
#include "at24_eep.h"
#include "string.h"
#

module_params_t moduleParam;
const serial_setting_t serial_default = {
  0,
  9600,
  SPAR_NONE,
  SSTOP_1,
  DATA_8
};

const lan_setting_t lan_default = {
  {192,168,0,240,0,0},
  {255,255,255,0,0,0},
  {192,168,0,1,0,0},
  {0x70,0xb1,0xff,0xff,0xff,0xff}
};

const module_setting_t module_default = {
  EEP_HEADING,
  0x01000000,
  0x00000001,
  "Grididea.com",
  "USER"
};

const _door_global_config_t default_door_config = {
  1000,
  3,
  10,
  333./80.,
  20,
  5,
  10,
  30,
  5,
  5,
  10,
  20,
  5, // 500ms
};

//const _door_config_t door_default[] = {
//  {TYPE_DOOR,80,20,95,85,15,25,5,25,1,750,450,5,5,5,0,333./80.,0,1,0,5,5,30},
//  {TYPE_DOOR,80,20,95,85,15,25,5,25,1,750,450,5,5,5,0,333./80.,0,0,1,5,5,30},
//  {TYPE_LOCK,10,10,255,255,255,255,255,255,255,850,250,5,5,5,255,255,255,1,0,1,0,1},
//};

const _door_config_t door_default[] = {
  {TYPE_DOOR,80,40,95,70,20,25,5,25,10,750,450,3,0,0},
  {TYPE_DOOR,80,40,95,70,20,25,5,25,10,750,450,3,0,0},
  {TYPE_LOCK,15,15,255,255,255,255,255,25,10,750,450,3,0,0},
};

void defaultParams(void)
{
  app_loadDefault();
  memcpy((uint8_t*)&moduleParam.param,(uint8_t*)&module_default,sizeof(module_setting_t));
  memcpy((uint8_t*)&moduleParam.serial,(uint8_t*)&serial_default,sizeof(serial_setting_t));
  memcpy((uint8_t*)&moduleParam.lan,(uint8_t*)&lan_default,sizeof(lan_setting_t));
  memcpy((uint8_t*)&moduleParam.doorConfig,(uint8_t*)&default_door_config,sizeof(_door_global_config_t));
  memcpy((uint8_t*)&moduleParam.door,(uint8_t*)&door_default,sizeof(_door_config_t)*3);
  
  moduleParam.action_delay_time = 10;
  moduleParam.angle_check_time = 10; // cycle
  moduleParam.angle_check_diff = 5; // degree
  moduleParam.ramp = 5; // pwm duty increase/decrease speed
  
}

void sysSaveParams(void)
{
  eepromWrite(EEP_STORE_OFFSET,sizeof(module_params_t),(uint8_t*)&moduleParam);
}

void sysParamInit()
{
  eepromRead(EEP_STORE_OFFSET,sizeof(module_params_t),(uint8_t*)&moduleParam);
  
  if(moduleParam.param.flag != EEP_HEADING){
    defaultParams();
    sysSaveParams();
  }
  
  if(moduleParam.doorConfig.adSampleIgnore == 0)
    moduleParam.doorConfig.adSampleIgnore = 5;
}
