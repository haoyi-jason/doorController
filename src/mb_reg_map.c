#include "app_doorcontrol.h"
#include "mb_reg_map.h"
#include "sysparam.h"


void mapMBWord(uint8_t *dptr, uint8_t *val)
{
  *dptr = *(val+1);
  *(dptr+1) = *val;
}

void mapMBFloat(uint8_t *dptr, uint8_t *val)
{
  *(dptr+0) = *(val+1);
  *(dptr+1) = *(val+0);
  *(dptr+2) = *(val+3);
  *(dptr+3) = *(val+2);
}

/**

EG_ADDR        Value           Size    Type

  
*/


int8_t read40000(uint16_t offset, uint8_t *dptr)
{
  int16_t val = 0;
  switch(offset){
  case 0:
    val = appParam.motorConfig[0].speed;
    break;
  case 1:
    val = appParam.motorConfig[0].angle;
    break;
  case 2:
    val = (int16_t)appParam.icu[0].degree;
    break;
  case 3:
    val = (int16_t)appParam.motorConfig[0].dc_current;
    break;
  case 4:
    if(appParam.motorConfig[0].speed != 0)
      val = appParam.motorConfig[0].currentDir == appParam.motorConfig[0].posDir?1:2;
    else
      val = 0;
    break;
  case 5:
    val = appParam.motorConfig[0].inhome;
    break;
  case 10:
    val = appParam.motorConfig[1].speed;
    break;
  case 11:
    val = appParam.motorConfig[1].angle;
    break;
  case 12:
    val = (int16_t)appParam.icu[1].degree;
    break;
  case 13:
    val = appParam.motorConfig[1].dc_current;
    break;
  case 14:
    if(appParam.motorConfig[0].speed != 0)
      val = appParam.motorConfig[1].currentDir == appParam.motorConfig[1].posDir?1:2;
    else
      val = 0;
    break;
  case 15:
    val = appParam.motorConfig[1].inhome;
    break;
//  case 16:
//    val = appParam.motorConfig[2].speed;
//    break;
//  case 17:
//    val = appParam.motorConfig[2].angle;
//    break;
//  case 18:
//    val = (int16_t)appParam.icu[2].degree;
//    break;
  case 20:
    val = appParam.motorConfig[2].dc_current;
    break;
  case 21:
    if(appParam.motorConfig[2].speed != 0)
      val = appParam.motorConfig[1].currentDir == appParam.motorConfig[1].posDir?1:2;
    else
      val = 0;
    break;
  case 22:
    if(appParam.motorConfig[2].pel->read(appParam.motorConfig[2].pel))
      val = 1;
    if(appParam.motorConfig[2].mel->read(appParam.motorConfig[2].mel))
      val = 2;
    break;
  case 30:
    val = appParam.errState;
    break;
  case 31:
    val = opState.openTimes + appParam.openTimes;
    break;
  case 33:
    val = moduleParam.doorConfig.freq1;
    break;
  case 34:
    val = moduleParam.doorConfig.freq2;
    break;
//  case 31:
//    val = moduleParam.action_delay_time;
//    break;
//  case 28:
//    val = moduleParam.angle_check_time;
//    break;
//  case 29:
//    val = moduleParam.angle_check_diff;
//    break;
  default:
    val = 0;
  }
  mapMBWord((void*)dptr,(void*)(&val));
  return 1;
}


int8_t write40000(uint16_t offset, uint8_t *dptr)
{
  int16_t val;
  mapMBWord((void*)(&val),(void*)dptr);
  switch(offset){
  case 30:
    appParam.errState = 0;
    chEvtSignal(appParam.mainThread,EV_SYS_CLEAR_ERROR);
    break;
  case 31:// save parameter
    if(val == 99)
      chEvtSignal(appParam.mainThread,EV_SYS_SAVE_PARAM);
    else if(val == 88)
      chEvtSignal(appParam.mainThread,EV_SYS_RESET);
    break;
  case 32: // door open
    if(val == 1)
      chEvtSignal(appParam.mainThread,EV_TG1_MBTRG);
    else if(val == 2)
      chEvtSignal(appParam.mainThread,EV_TG2_TRIGGER);
    break;
  case 33:
    moduleParam.doorConfig.freq1 = val;
    break;
  case 34:
    moduleParam.doorConfig.freq2 = val;
    break;
//  case 27:
//    moduleParam.action_delay_time = val;
//    break;
//  case 28:
//    moduleParam.angle_check_time = val;
//    break;
//  case 29:
//    moduleParam.angle_check_diff = val;
//    break;
  }
  return 1;
}


int8_t read40200(uint16_t offset, uint8_t *dptr)
{
  int16_t val = 0;
  uint8_t id = offset / 20;
  uint8_t index = offset % 20;
  
  switch(index){
  case 0:
    val = moduleParam.door[id].normalSpeed;
    break;
  case 1:
    val = moduleParam.door[id].slowSpeed;
    break;
  case 2:
    val = moduleParam.door[id].openAngle;
    break;
  case 3:
    val = moduleParam.door[id].sldOpenAngle;
    break;
  case 4:
    val = moduleParam.door[id].sldCloseAngle;
    break;
  case 5:
    val = moduleParam.door[id].zero_angle_error;
    break;
  case 6:
    val = moduleParam.door[id].openRevTime;
    break;
  case 7:
    val = moduleParam.door[id].openRevSpeed;
    break;
  case 8:
    val = moduleParam.door[id].closeFwdTime;
    break;
  case 9:
    val = moduleParam.door[id].closeFwdSpeed;
    break;
  case 10:
    val = moduleParam.door[id].lock_times;
    break;
  case 11:
    val = moduleParam.door[id].max_working_current;
    break;  
  case 12:
    val = moduleParam.door[id].normalMaxCurrent;
    break;
  case 13:
    val = moduleParam.door[id].slowMaxCurrent;
    break;
  default:
    val = 0;
    break;
  }
  mapMBWord((void*)dptr,(void*)(&val));
  return 1;
}  

int8_t write40200(uint16_t offset, uint8_t *dptr)
{
  int16_t val;
  uint8_t id = offset / 20;
  uint8_t index = offset % 20;
  mapMBWord((void*)(&val),(void*)dptr);
  switch(index){
  case 0:
    moduleParam.door[id].normalSpeed = val;
    break;
  case 1:
    moduleParam.door[id].slowSpeed = val;
    break;
  case 2:
    moduleParam.door[id].openAngle = val;
    break;
  case 3:
    moduleParam.door[id].sldOpenAngle = val;
    break;
  case 4:
    moduleParam.door[id].sldCloseAngle = val;
    break;
  case 5:
    moduleParam.door[id].zero_angle_error = val;
    break;
  case 6:
    moduleParam.door[id].openRevTime = val;
    break;
  case 7:
    moduleParam.door[id].openRevSpeed = val;
    break;
  case 8:
    moduleParam.door[id].closeFwdSpeed = val;
    break;
  case 9:
    moduleParam.door[id].closeFwdTime = val;
    break;
  case 10:
    moduleParam.door[id].lock_times = val;
    break;
  case 11:
    moduleParam.door[id].max_working_current = val;
    break;
  case 12:
    moduleParam.door[id].normalMaxCurrent = val;
    break;
  case 13:
    moduleParam.door[id].slowMaxCurrent = val;
    break;
  default:
    break;
  }
  
  return 1;
}

int8_t read40300(uint16_t offset, uint8_t *dptr)
{
  int16_t val = 0;
  uint8_t id = offset / 3;
  uint8_t index = offset;
  switch(offset){
  case 0: // start map door_global_config
    val = moduleParam.doorConfig.lockRetryIdleCycles;
    break;
  case 1:
    val = moduleParam.doorConfig.lockRetry;
    break;
  case 2:
    val = moduleParam.doorConfig.triggerAngle;
    break;
  case 3:
    val = (int16_t)(moduleParam.doorConfig.degree_percent*100);
    break;
  case 4:
    val = moduleParam.doorConfig.angleDiff;
    break;
  case 5:
    val = moduleParam.doorConfig.lockActiveTime;
    break;
  case 6:
    val = moduleParam.doorConfig.waitTimeToClose;
    break;
  case 7:
    val = moduleParam.doorConfig.doorFreeAngle;
    break;
  case 8:
    val = moduleParam.doorConfig.actionDelay;
    break;
  case 9:
    val = moduleParam.doorConfig.angleValidDeg;
    break;
  case 10:
    val = moduleParam.doorConfig.angleValidCycles;
    break;
  case 11:
    val = moduleParam.doorConfig.ramp;
    break;
  case 12:
    val = moduleParam.doorConfig.adSampleIgnore;
    break;
  case 13:
    val = VERSION;
    break;
  case 14:
    val = VERSION_2;
    break;
  default:
    val = 0;
  }
  mapMBWord((void*)dptr,(void*)(&val));
  return 1;
}
int8_t write40300(uint16_t offset, uint8_t *dptr)
{
  int16_t val;
  mapMBWord((void*)(&val),(void*)dptr);
  switch(offset){
  case 0:
    moduleParam.doorConfig.lockRetryIdleCycles = val;
    break;
  case 1:
    moduleParam.doorConfig.lockRetry = val;
    break;
  case 2:
    moduleParam.doorConfig.triggerAngle = val;
    break;
  case 3:
    moduleParam.doorConfig.degree_percent = (float)(val)/100.;
    break;
  case 4:
    moduleParam.doorConfig.angleDiff = val;
    break;
  case 5:
    moduleParam.doorConfig.lockActiveTime = val;
    break;
  case 6:
    moduleParam.doorConfig.waitTimeToClose = val;
    break;
  case 7:
    moduleParam.doorConfig.doorFreeAngle = val;
    break;
  case 8:
    moduleParam.doorConfig.actionDelay = val;
    break;
  case 9:
    moduleParam.doorConfig.angleValidDeg = val;
    break;
  case 10:
    moduleParam.doorConfig.angleValidCycles = val;
    break;
  case 11:
    moduleParam.doorConfig.ramp = val;
    break;
  case 12:
    moduleParam.doorConfig.adSampleIgnore = val;
    break;
  default:break;
  }
  
  return 1;
}

int8_t read40513(uint16_t offset, uint8_t *dptr)
{
  int16_t val = 0;
//  switch(offset){
//  case 0: 
//    val = moduleParam.door[0].normalSpeed;
//    break;
//  case 1:
//    val = moduleParam.door[0].slowSpeed;
//    break;
//  case 2:
//    val = moduleParam.door[0].openAngle;
//    break;
//  case 3:
//    val = moduleParam.door[1].normalSpeed;
//    break;
//  case 4:
//    val = moduleParam.door[1].slowSpeed;
//    break;
//  case 5:
//    val = moduleParam.door[1].openAngle;
//    break;
//  case 6:
//    //val = moduleParam.door[0].openDelay;
//    break;
//  case 7:
//    val = moduleParam.door[0].sldOpenAngle;
//    break;
//  case 8:
//    val = moduleParam.door[0].sldCloseAngle;
//    break;
//  case 9:
//    val = moduleParam.door[0].zero_angle_error;
//    break;
//  case 10:
//    val = 0;
//    break;
//  case 11:
//    val = 0;
//    break;
//  case 12:
//    val = moduleParam.doorConfig.lockTimeout;
//    break;
//  case 13:
////    val = moduleParam.door[2].working_time;
//    break;
//  case 14:
////    val = moduleParam.door[0].lockFreeTime;
//    break;
//  case 15:
////    val = moduleParam.door[0].lockHoldingTime;
//    break;
//  case 16:
//    val = 0;
//    break;
//  case 17:
//    val = moduleParam.door[2].normalSpeed;
//    break;
//  case 18:
//    val = 0;
//    val |= appParam.motorConfig[0].running?0x1:0x0;
//    val |= appParam.motorConfig[1].running?0x2:0x0;
//    val |= appParam.motorConfig[2].running?0x4:0x0;
//    break;
//  case 19:
//    val = 0;
//    break;
//  case 20:
//    val = 0;
//    break;
//  case 21:
//    val = appParam.motorConfig[0].angle;
//    break;
//  case 22:
//    val = appParam.motorConfig[1].angle;
//    break;
//  case 23:
//    val = 0;
//    break;
//  case 24:
//    val = 0;
//    break;
//  case 25:
//    val = appParam.boardID;
//    break;
//  case 26:
//    val = 0;
//    break;
//  case 27:
//    val = appParam.motorConfig[0].speed;
//    break;
//  case 28:
//    val = appParam.motorConfig[1].speed;
//    break;
//  case 29:
//    val = moduleParam.door[0].openRevSpeed;
//    break;
//  case 30:
//    val = moduleParam.door[0].closeFwdSpeed;
//    break;
//  case 31:
//    val = moduleParam.door[0].closeFwdTime;
//    break;
//  case 32:
//    val = moduleParam.action_delay_time;
//    break;
//  case 33:
//    val = moduleParam.angle_check_time;
//    break;
//  case 34:
//    val = moduleParam.angle_check_diff;
//    break;
//  case 35:
//    val = appParam.motorConfig[0].dc_current;
//    break;
//  case 36:
//    val = appParam.motorConfig[1].dc_current;
//    break;
//  case 37:
//    val = appParam.motorConfig[2].dc_current;
//    break;
//  default:
//    val = 0;
//  }
//  
//  mapMBWord((void*)dptr,(void*)(&val));
  return 1;
}


int8_t write40513(uint16_t offset, uint8_t *dptr)
{
  int16_t val;
//  mapMBWord((void*)(&val),(void*)dptr);
//  switch(offset){
//  case 0: 
//    moduleParam.door[0].normalSpeed = val;
//    break;
//  case 1:
//    moduleParam.door[0].slowSpeed = val;
//    break;
//  case 2:
//    moduleParam.door[0].openAngle = val;
//    break;
//  case 3:
//    moduleParam.door[1].normalSpeed = val;
//    break;
//  case 4:
//    moduleParam.door[1].slowSpeed = val;
//    break;
//  case 5:
//    moduleParam.door[1].openAngle = val;
//    break;
//  case 6:
////    moduleParam.door[0].openDelay = val;
////    moduleParam.door[1].openDelay = val;
//    break;
//  case 7:
//    moduleParam.door[0].sldOpenAngle = val;
//    moduleParam.door[1].sldOpenAngle = val;
//    break;
//  case 8:
//    moduleParam.door[0].sldCloseAngle = val;
//    moduleParam.door[1].sldCloseAngle = val;
//    break;
//  case 9:
//    moduleParam.door[0].zero_angle_error = val;
//    moduleParam.door[1].zero_angle_error = val;
//    break;
//  case 10:
//    val = 0;
//    break;
//  case 11:
//    val = 0;
//    break;
//  case 12:
////    moduleParam.door[0].lockStayTime = val;
////    moduleParam.door[1].lockStayTime = val;
//    break;
//  case 13:
////    moduleParam.door[2].working_time = val;
//    break;
//  case 14:
////    moduleParam.door[0].lockFreeTime = val;
////    moduleParam.door[1].lockFreeTime = val;
//    break;
//  case 15:
////    moduleParam.door[0].lockHoldingTime = val;
////    moduleParam.door[1].lockHoldingTime = val;
//    break;
//  case 16:
//    val = 0;
//    break;
//  case 17:
//    moduleParam.door[2].normalSpeed = val;
//    break;
//  case 29:
//    moduleParam.door[0].openRevSpeed = val;
//    break;
//  case 30:
//    moduleParam.door[0].closeFwdSpeed = val;
//    break;
//  case 31:
//    moduleParam.door[0].closeFwdTime = val;
//    break;
//  case 32:
//    moduleParam.action_delay_time = val;
//    break;
//  case 33:
//    moduleParam.angle_check_time = val;
//    break;
//  case 34:
//    moduleParam.angle_check_diff = val;
//    break;
//  default:
//    val = 0;
//  }
  //updateConstrain();
  return 1;
}


mb_reg_map_t mb_reg_map_func[] = {
  {0,35,read40000,write40000},
  {200,260,read40200,write40200},
  {300,340,read40300,write40300},
  //{513,550,read40513,write40513},
};


