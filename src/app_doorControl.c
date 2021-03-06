#include "ch.h"
#include "hal.h"
#include "app_doorControl.h"
#include "exti_cfg.h"
#include "sysparam.h"
#include "iir.h"
#ifdef APP_USE_RSI
#include "rsi_app_main.h"
#include "rsi_app_bt_spp_slave.h"
#endif
#include "modbus_binding.h"
enum {
  STG_PRE_PRESS,
  STG_DOOR_OPEN,
  STG_POST_PRESS,
};

enum moveType{
  MV_TIME,  // timed move
  MV_POS,       // position move
};
// struct for control task
typedef struct{
  _motor_config_t* m;
  uint8_t moveType;
  uint8_t dir;
  uint16_t speed[2]; // slow/normal speed
  int16_t args[4]; // time in ms or position in degre
}_motor_control_t;

#define NUM_MT_BUFFER   8
#define MT_BUFFER_SIZE sizeof(_motor_control_t)

static const I2CConfig i2ccfg = {
  OPMODE_I2C,
  100000,
  STD_DUTY_CYCLE,
};

//static SPIConfig spicfg_rsi = {
//  NULL,
//  GPIOA,
//  15,
//  SPI_CR1_BR_2 |SPI_CR1_BR_1 //|SPI_CR1_BR_0
////  SPI_CR1_BR_2  | SPI_CR1_CPHA | SPI_CR1_CPOL
//};

// driving frequency = PWM_FREQ/PWM_PERIOD, Hz
#define PWM_FREQ        1000000
#define PWM_PERIOD      400
_appParam_t appParam;

void setPad(struct dio_map_s *p)
{
  palSetPad(p->port,p->pad);
}

void clrPad(struct dio_map_s *p)
{
  palClearPad(p->port,p->pad);
}

uint8_t readPad(struct dio_map_s *p)
{
  return (palReadPad(p->port, p->pad)==PAL_HIGH)?1:0;
}

uint8_t readLatch(struct dio_map_s *p)
{
  return ((palReadLatch(p->port)>>p->pad) & 0x1)==PAL_HIGH?1:0;
}

dio_map_t do_map[]={
  {GPIOB,1,setPad,clrPad, readLatch},
  {GPIOB,6,setPad,clrPad, readLatch},
  {GPIOB,7,setPad,clrPad, readLatch},
  {GPIOA,5,setPad,clrPad, readLatch},
  {GPIOA,2,setPad,clrPad, readLatch},
  {GPIOA,3,setPad,clrPad, readLatch},
  {GPIOC,7,setPad,clrPad, readLatch},
  {GPIOC,8,setPad,clrPad, readLatch},
  {GPIOD,2,setPad,clrPad, readLatch},
  {GPIOA,8,setPad,clrPad, readLatch},
  {GPIOC,2,setPad,clrPad, readLatch},
  {GPIOC,3,setPad,clrPad, readLatch},
  {GPIOC,4,setPad,clrPad, readLatch},
  {GPIOB,15,setPad,clrPad,readLatch},
};

dio_map_t di_map[] = {
  {GPIOC,0,setPad,clrPad,readPad},
  {GPIOC,1,setPad,clrPad,readPad},
  {GPIOC,5,setPad,clrPad,readPad},
  {GPIOC,6,setPad,clrPad,readPad},
  {GPIOB,2,setPad,clrPad,readPad},
  {GPIOB,3,setPad,clrPad,readPad},
  {GPIOB,0,setPad,clrPad,readPad},
  {GPIOB,4,setPad,clrPad,readPad},
  {GPIOC,13,setPad,clrPad,readPad},
};

_motor_config_t motors[] = {
  {0,&do_map[M1_ON],&do_map[MC1_EN],MC1_CTRL1,MC1_CTRL2,NULL,&di_map[DOOR1_INP],DIR_NEG, 100,0,0,0,2000,false,&PWMD4,0,&PWMD4,1},
  {1,&do_map[M2_ON],&do_map[MC2_EN],MC2_CTRL1,MC2_CTRL2,NULL,&di_map[DOOR2_INP],DIR_NEG, 100,0,0,0,2000,false,&PWMD5,2,&PWMD5,3},
  {2,&do_map[M3_ON],&do_map[MC2_EN],MC2_CTRL1,MC2_CTRL2,&di_map[ULOCK_INP],&di_map[LLOCK_INP],DIR_NEG,50,0,0,0,2000,false,&PWMD5,2,&PWMD5,3},
};

beep_pattern_t beepPattern[] = {
  {1000,1000,20},
  {500,1000,20},
  {1000,500,20},
};

static SPIConfig spicfg = {
  NULL,
  GPIOA,
  4,
  SPI_CR1_BR_2 | SPI_CR1_BR_1 |SPI_CR1_BR_0 | SPI_CR1_LSBFIRST
};


static void pwmcb(PWMDriver *pwmp)
{
  if(pwmp == &PWMD4){
    
  }
}

static void pwmM1FWcb(PWMDriver *pwmp) 
{
  (void)pwmp;
}

static void pwmM1RVcb(PWMDriver *pwmp) 
{
  (void)pwmp;
}
static void pwmM2FWcb(PWMDriver *pwmp) 
{
  (void)pwmp;
}
static void pwmM2RVcb(PWMDriver *pwmp) 
{
  (void)pwmp;
}
static PWMConfig pwmcfg_t4= {
  PWM_FREQ,
  PWM_PERIOD,
  NULL,
  {
    {PWM_OUTPUT_ACTIVE_HIGH,pwmM1FWcb}, // M1 1
    {PWM_OUTPUT_ACTIVE_HIGH,pwmM1RVcb}, // M1.2
    {PWM_OUTPUT_DISABLED,NULL},
    {PWM_OUTPUT_DISABLED,NULL}
  },
  0,
  0
};
static PWMConfig pwmcfg_t5= {
  PWM_FREQ,
  PWM_PERIOD,
  NULL,
  {
    {PWM_OUTPUT_DISABLED,NULL},
    {PWM_OUTPUT_DISABLED,NULL},
    {PWM_OUTPUT_ACTIVE_HIGH,pwmM2FWcb}, // M2.1
    {PWM_OUTPUT_ACTIVE_HIGH,pwmM2RVcb}  // M2.2
  },
  0,
  0
};

static PWMConfig pwmcfg_t1= {
  80000,
  20,
  NULL,
  {
    {PWM_OUTPUT_ACTIVE_HIGH,NULL}, // Buzzer
    {PWM_OUTPUT_DISABLED,NULL},
    {PWM_OUTPUT_DISABLED,NULL},
    {PWM_OUTPUT_DISABLED,NULL},
  },
  0,
  0
};

#define ADC_GRP1_NUM_CHANNELS   3
#define ADC_GRP1_BUF_DEPTH      8
static adcsample_t samples[ADC_GRP1_NUM_CHANNELS*ADC_GRP1_BUF_DEPTH];

static void adccallback(ADCDriver *adcp, adcsample_t *buffer,size_t n)
{
  (void)adcp;
  uint16_t chSum[3] = {0,0,0};
  for(uint8_t i=0;i<ADC_GRP1_BUF_DEPTH;i++){
    chSum[0] += samples[i*3];
    chSum[1] += samples[i*3+1];
    chSum[2] += samples[i*3+2];
  }
  chSum[0] >>= 3;
  chSum[1] >>= 3;
  chSum[2] >>= 3;
 
  appParam.doorState[0].op_current = chSum[0];
  appParam.doorState[1].op_current = chSum[1];
  appParam.vr6 = 4095 - chSum[2]; // reverse
  
  appParam.motorConfig[0].dc_current = (chSum[0] - appParam.doorState[0].idle_current) * 1.13;
  appParam.motorConfig[1].dc_current = (chSum[1] - appParam.doorState[1].idle_current) * 1.13;
  if(appParam.motorConfig[0].dc_current < 0) appParam.motorConfig[0].dc_current = 0;
  if(appParam.motorConfig[1].dc_current < 0) appParam.motorConfig[1].dc_current = 0;
  
  if(appParam.motorConfig[0].dc_current < 0){
    chSum[0]++;
  }
}

static void adcerror(ADCDriver *adcp, adcerror_t err)
{
  (void)adcp;
  while(1){}
  
}

static const ADCConversionGroup adcgrpcfg = {
  TRUE,
  ADC_GRP1_NUM_CHANNELS,
  NULL,
  NULL,
  0,
  ADC_CR2_SWSTART,
  0,
  ADC_SMPR2_SMP_AN0(ADC_SAMPLE_3) | ADC_SMPR2_SMP_AN1(ADC_SAMPLE_3)| ADC_SMPR2_SMP_AN2(ADC_SAMPLE_3),
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
  0,
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1) | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN6)
};


uint16_t read_board_id(void)
{
  uint8_t tx[2],rx[2];
  
  spiStart(&SPID3,&spicfg);
  // toggle SH/LD
  do_map[ID_LATCH].clear(&do_map[ID_LATCH]);
  chThdSleepMilliseconds(50);
  do_map[ID_LATCH].set(&do_map[ID_LATCH]);
  chThdSleepMicroseconds(200);
  // read 
  spiSelect(&SPID3);
  spiReceive(&SPID3,2,rx);
  spiUnselect(&SPID3);
  spiStop(&SPID3);
  
  uint16_t ret = 0;
  ret |= (uint8_t)(~(rx[0]));
  spiStop(&SPID3);
  return ret;
}

enum{
  STATE_NONE,
  STATE_REVERSE,
  STATE_UNLOCK,
  STATE_OPEN_D2,
  STATE_OPEN_INIT,
  STATE_OPEN,
  STATE_OPEN_DONE,
  STATE_OPEN_WAIT,
  STATE_CLOSE,
  STATE_CLOSING,
  STATE_CLOSE_FORWARD,
  STATE_CLOSED
};

void handleADCCalculation(void)
{
  uint16_t chSum[3] = {0,0,0};
  //chSysLock();
  for(uint8_t i=0;i<ADC_GRP1_BUF_DEPTH;i++){
    chSum[0] += samples[i*3];
    chSum[1] += samples[i*3+1];
    chSum[2] += samples[i*3+2];
  }
  chSum[0] >>= 3;
  chSum[1] >>= 3;
  chSum[2] >>= 3;
 
  
  appParam.doorState[0].op_current = chSum[0];
  appParam.doorState[1].op_current = chSum[1];
  appParam.vr6 = 4095 - chSum[2]; // reverse
  
  appParam.motorConfig[0].dc_current = (chSum[0] - appParam.doorState[0].idle_current) * 1.13;
  appParam.motorConfig[1].dc_current = (chSum[1] - appParam.doorState[1].idle_current) * 1.13;
  if(appParam.motorConfig[0].dc_current < 0) appParam.motorConfig[0].dc_current = 0;
  if(appParam.motorConfig[1].dc_current < 0) appParam.motorConfig[1].dc_current = 0;
  
  //chSysUnlock();

  if(appParam.motorConfig[0].dc_current < 0){
    chSum[0]++;
  }
}


void motorControlP(bool enable, uint8_t dir, _motor_config_t *m,uint16_t speed)
{
  if(enable){
    m->speed = speed;
    if(m->running){
      if(dir == DIR_POS){
        if(dir == m->currentDir){
          pwmEnableChannel(m->posDriver, m->posChannel, PWM_PERCENTAGE_TO_WIDTH(m->posDriver, speed*100));
        }
        else{
          pwmEnableChannel(m->negDriver, m->negChannel, PWM_PERCENTAGE_TO_WIDTH(m->negDriver, 0));
          chThdSleepMilliseconds(5);
          pwmEnableChannel(m->posDriver, m->posChannel, PWM_PERCENTAGE_TO_WIDTH(m->posDriver, speed*100));
        }
      }
      else if(dir == DIR_NEG){
        if(dir == m->currentDir){
          pwmEnableChannel(m->negDriver, m->negChannel, PWM_PERCENTAGE_TO_WIDTH(m->negDriver, speed*100));
        }
        else{
          pwmEnableChannel(m->posDriver, m->posChannel, PWM_PERCENTAGE_TO_WIDTH(m->posDriver, 0));
          chThdSleepMilliseconds(5);
          pwmEnableChannel(m->negDriver, m->negChannel, PWM_PERCENTAGE_TO_WIDTH(m->negDriver, speed*100));
        }
      }
    }
    else{
      m->gate->set(m->gate);
      m->en->set(m->en);
      if(dir == DIR_POS){
        pwmEnableChannel(m->posDriver, m->posChannel, PWM_PERCENTAGE_TO_WIDTH(m->posDriver, speed*100));
      }
      else{
        pwmEnableChannel(m->negDriver, m->negChannel, PWM_PERCENTAGE_TO_WIDTH(m->negDriver, speed*100));
      }
      
    }
    m->currentDir = dir;
    m->running = true;
  }
  else{
    if(m->running){
      pwmEnableChannel(m->posDriver, m->posChannel, PWM_PERCENTAGE_TO_WIDTH(m->posDriver, 0));
      pwmEnableChannel(m->negDriver, m->negChannel, PWM_PERCENTAGE_TO_WIDTH(m->negDriver, 0));
      chThdSleepMilliseconds(50);
      m->gate->clear(m->gate);
      m->en->clear(m->en);
      m->running = false;
      m->speed = 0;
    }
  }
}


#define CYCLE_TIME      100 //ms
//#define ERR_LOCKED      EVENT_MASK(0)
//#define ERR_OVERCURRENT EVENT_MASK(1)
//#define ERR_POS_LOCKED  EVENT_MASK(2)
//#define ERR_CUR_LOCKED  EVENT_MASK(3)

void set_beep(uint16_t on,uint16_t off, uint16_t times)
{
  dio_map_t *beep = &do_map[BUZZER];
  beep->clear(beep);
  for(uint8_t i=0;i<times;i++){
    beep->set(beep);
    chThdSleepMilliseconds(on);
    beep->clear(beep);
    chThdSleepMilliseconds(off);
  }
}

void BEEP(uint8_t mask)
{
  dio_map_t *beep = &do_map[BUZZER];
  beep->clear(beep);
  uint8_t b = 0;
  if(mask & ERR_M1_MASK){
    b = (mask & 0xf);
  }
  else if(mask & ERR_M2_MASK){
    b = (mask & 0xf)+5; 
  }
  else{
    b = mask+10;
  }
  b--;
  if((mask & 0xf) == 0x5){
    beep->set(beep);
    chThdSleepMilliseconds(900);
    beep->clear(beep);
    chThdSleepMilliseconds(300);
  }
  else{
    for(uint8_t i=0;i<4;i++){
      if((b & (1<<(3-i))) == 0){
        beep->set(beep);
        chThdSleepMilliseconds(300);
        beep->clear(beep);
        chThdSleepMilliseconds(900);
      }
      else{
        beep->set(beep);
        chThdSleepMilliseconds(600);
        beep->clear(beep);
        chThdSleepMilliseconds(600);
      }
    }
  }
}

//static THD_WORKING_AREA(waMotorControl,512);
thread_reference_t m2_trp = NULL;
#define ERR_01  0x1
#define ERR_02  0x2
#define ERR_03  0x4
#define ERR_04  0x8

// events between threads
#define EV_ABORT_LOCK   EVENT_MASK(0)
#define EV_ABORT_OC     EVENT_MASK(1)
#define EV_TERMINATE    EVENT_MASK(31)

enum run_type{
  RT_POS,       // position run
  RT_TIMED,     // timed run
  RT_FOLLOW     // follow run
};

#define ST_POSITION     0x1
#define ST_HOME         0x2
#define ST_PEL          0x4
#define ST_MEL          0x8
//enum stop_type{
//  ST_POSITION=1,
//  ST_HOME,
//  ST_PEL,
//  ST_MEL,
//};

enum act_type{
  ACT_OPEN,
  ACT_CLOSE
};



void setupIWDG(void)
{
//  //const uint32_t LsiFreq = 32000;
//  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
//
//  IWDG_SetPrescaler(IWDG_Prescaler_32);
//
//  IWDG_SetReload(2000);
//
//  IWDG_Enable();
}

#define HIST_SIZE 30
static THD_FUNCTION(procMotorPosRun,p)
{
  motor_run_t *mr = (motor_run_t*)p;
  bool run = true;
  uint16_t speed = 0;
  uint16_t cspeed = 0;
  uint16_t newspeed = 0;
  int16_t lastPos = mr->m->angle;
  uint8_t checkCycle = moduleParam.doorConfig.angleValidCycles;
  uint8_t lockedCount = 0;
  uint8_t lockedTime = 0;
  uint32_t errState = 0;
  uint8_t currentValidDelay = moduleParam.doorConfig.adSampleIgnore;
  mr->m->inPos = false;
  uint8_t inpValidCycle = moduleParam.doorConfig.actionDelay<<1;
  int16_t validAngle = 0;
  
  int16_t angleHist[HIST_SIZE];
  uint8_t angleRecord = 0;
  uint8_t check_cycle = 20; 
  uint8_t isDoor = (mr->m == (&appParam.motorConfig[2]))?0:1;
  uint16_t runCycles = 0;
  memset(angleHist,0,HIST_SIZE*2);
  while(run){
    //wdgReset(&WDGD1);
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS,TIME_IMMEDIATE);
    if(evt & EV_ABORT_OC){
      run = false;
      mr->actForce = 0;
    }
    if(evt & EV_ABORT_LOCK){
      run = false;
      mr->actForce = 0;
    }
    if(evt & EV_TERMINATE){
      run = false;
    }
        
    // valid for speed
    if(mr->actType == ACT_OPEN){
      if(mr->m->angle > mr->d->sldOpenAngle){
        speed = mr->d->slowSpeed;
      }
      else{
        speed = mr->d->normalSpeed;  
      }

      if(mr->stopType & ST_POSITION){
        if(mr->m->angle >= mr->d->openAngle){
          speed = 0;
          mr->m->inPos = true;
          run = false;
        }
      }
      
      if(mr->stopType == ST_PEL){
        if(mr->m->pel->read(mr->m->pel)){
          speed = 0;
          run = false;
        }
      }      
    }
    else{ // door close
      if(mr->m->angle < mr->d->sldCloseAngle){
        speed = mr->d->slowSpeed;
      }
      else{
        speed = mr->d->normalSpeed;
      }

      if(mr->stopType & ST_HOME){
        if(mr->m->orgFound){
          if(mr->m->inhome){
//          if(mr->m->inhome  || (mr->m->angle < mr->d->zero_angle_error)){
            if(!mr->m->inhome){
              // add short delay here
              if(inpValidCycle) inpValidCycle--; //?
              if(inpValidCycle == 0){
                speed = 0;
                mr->m->inPos = true;
                if(!mr->m->inhome){
                  SET_ERR(errState,ERR_MOTOR_POSS);
                }
                run = false;
              }
            }else{
              speed = 0;
              mr->m->inPos = true;
              run = false;
            }
          }
        }else{
          if(mr->m->inhome){
            speed = 0;
            mr->m->inPos = true;
            run = false;
          }
        }
      }
      
      if(mr->stopType & ST_MEL){
        if(mr->m->mel->read(mr->m->mel)){
          speed = 0;
          mr->m->inPos = true;
          run = false;
        }
      }
      

    }
        
    // valid for current
//    if(currentValidDelay) currentValidDelay--;
//    if(currentValidDelay == 0){
//      if(mr->m->dc_current > mr->d->normalMaxCurrent){
//        SET_ERR(errState,ERR_OVERCURRENT);
//        speed = 0;
//      }
//    }
    
//    mr->d->max_working_current = (mr->d->max_working_current>mr->m->dc_current)?mr->d->max_working_current:mr->m->dc_current;
    
    
    // update speed
    if((errState == 0) && (run==true)){
      if(speed > mr->maxSpeed) speed = mr->maxSpeed;
      
      if(speed >= cspeed){
        cspeed += moduleParam.doorConfig.ramp;
        if(cspeed > speed) cspeed = speed;
      }else{
        if(speed == 0){
          cspeed = 0;
        }else{
          cspeed -= moduleParam.doorConfig.ramp;
          if(cspeed < speed) cspeed = speed;
        }
      }
      
      if(cspeed == 0 && run){
        motorControlP(true,mr->dir,mr->m,cspeed);
        validAngle = 0;
      }
      else{
        motorControlP(true,mr->actType==ACT_OPEN?mr->m->posDir:mr->m->negDir,mr->m,cspeed);
        validAngle = (cspeed * moduleParam.doorConfig.angleValidDeg)/mr->maxSpeed;
      }
    }
    else{
      motorControlP(false,mr->actType==ACT_OPEN?mr->m->posDir:mr->m->negDir,mr->m,0);
      run = false;
    }

    if(isDoor){
      if(runCycles == 20){ // check only @ 20 cycles
        if(mr->actType == ACT_OPEN){
          if(angleHist[0] < angleHist[10]){
            SET_ERR(errState,ERR_MOTOR_POL);
            run = false;
          }
        }
        else{
          if(angleHist[0] > angleHist[10]){
            SET_ERR(errState,ERR_MOTOR_POL);
            run = false;
          }
        }
      }
      // valid for door lock
      if(mr->m->orgFound){
        if(mr->m->running){
          if(speed == 0){
            //checkCycle = moduleParam.angle_check_time;
            checkCycle = moduleParam.doorConfig.angleValidCycles;
            currentValidDelay = moduleParam.doorConfig.adSampleIgnore;
            angleRecord = 0;
          }else{
            for(uint8_t i=HIST_SIZE-1;i>=1;i--){
              angleHist[i] = angleHist[i-1];
            }
            angleHist[0] = mr->m->angle;
            angleRecord++;
            // valid for current
            if(currentValidDelay) currentValidDelay--;
            if(currentValidDelay == 0){
              if(mr->m->dc_current > mr->d->normalMaxCurrent){
                SET_ERR(errState,ERR_MOTOR_OC);
                speed = 0;
                run = false;
              }
            }
            mr->d->max_working_current = (mr->d->max_working_current>mr->m->dc_current)?mr->d->max_working_current:mr->m->dc_current;
            // valid for door lock or not
            if(angleRecord > moduleParam.doorConfig.angleValidCycles){
              int16_t diff = angleHist[0] - angleHist[moduleParam.doorConfig.angleValidCycles];
              
              if(diff < 0) diff *= -1;
                
                if(diff < validAngle){
                  lockedCount++;
                  if(lockedCount > moduleParam.doorConfig.lockDetectCycle){
                    speed = 0;
                    SET_ERR(errState,ERR_MOTOR_LC);
                  }
                }
                else{
                  lockedCount = 0;
                }
              lastPos = mr->m->angle;
            }
          }       
        }
      }
    }

    if(!run){
      if(errState == 0){
        if(mr->actForce == 0){
          motorControlP(false,mr->dir,mr->m,0);
        }
        else{
          motorControlP(true,mr->actType==ACT_OPEN?mr->m->posDir:mr->m->negDir,mr->m,mr->actForce);
          chThdSleepMilliseconds(50);
        }
      }
      else{
          chThdSleepMilliseconds(50);
      }
    }
    chThdSleepMilliseconds((CYCLE_TIME));
    runCycles++;
  }
  
  chThdExit((msg_t)errState);
}

static THD_FUNCTION(procMotorTimeRun,p)
{
  motor_run_t *mr = (motor_run_t*)p;
  bool run = true;
  uint16_t speed = 0;
  uint16_t newspeed = 0;
  int16_t lastPos = mr->m->angle;
  uint8_t checkCycle = moduleParam.doorConfig.angleValidCycles;
  uint8_t lockedCount = 0;
  uint8_t lockedTime = 0;
  uint32_t errState = 0;
  uint32_t runTime = 0;
  mr->m->inPos = false;
  while(run){
    //wdgReset(&WDGD1);
    if(chThdShouldTerminateX()){
      run = false;
    }
    
    // valid for current
    if(mr->m->dc_current > mr->d->normalMaxCurrent){
      SET_ERR(errState,ERR_MOTOR_OC);
      speed = 0;
    }else{
//      mr->d->max_working_current = mr->m->dc_current;
      speed = mr->d->normalSpeed;
    }
    mr->d->max_working_current = (mr->d->max_working_current>mr->m->dc_current)?mr->d->max_working_current:mr->m->dc_current;
    
    if(mr->stopType == ST_PEL){
      if(mr->m->pel->read(mr->m->pel)){
        mr->m->inPos = true;
        run = false;
      }
    }
        
    if(mr->stopType == ST_MEL){
      if(mr->m->mel->read(mr->m->mel)){
        mr->m->inPos = true;
        run = false;
      }
    }
    // update speed
    if((errState == 0) && run){
      if(speed > mr->maxSpeed) speed = mr->maxSpeed;
      if(speed == 0){
        if(mr->actForce == 0)
          motorControlP(false,mr->dir,mr->m,mr->actForce);
        else
          motorControlP(true,mr->dir,mr->m,mr->actForce);
      }
      else
        motorControlP(true,mr->actType==ACT_OPEN?mr->m->posDir:mr->m->negDir,mr->m,speed);
    }else{
      run = false;
    }
    
    chThdSleepMilliseconds((CYCLE_TIME));
    runTime += 100;
    if(runTime >= mr->runTime){
      run = false;
    }
    if(!run){
      if(mr->actForce == 0)
        motorControlP(false,mr->dir,mr->m,0);
      else
        motorControlP(true,mr->dir,mr->m,mr->actForce);
    }
  }
  
  //motorControlP(false,mr->dir,mr->m,0);
  chThdExit((msg_t)errState);
}
static THD_FUNCTION(procMotorFollowRun,p)
{
  motor_run_t *mr = (motor_run_t*)p;
  bool run = true;
  uint16_t speed = 0;
  uint16_t cspeed = 0;
  uint16_t newspeed = 0;
  int16_t lastPos = mr->m->angle;
  uint8_t checkCycle = moduleParam.doorConfig.angleValidCycles;
  uint8_t lockedCount = 0;
  uint8_t lockedTime = 0;
  uint32_t errState = 0;
  uint8_t currentValidDelay = moduleParam.doorConfig.adSampleIgnore;
  mr->m->inPos = false;
  bool emgZero = false;
  bool masterLocked = false;
  uint8_t inpValidCycle = moduleParam.doorConfig.actionDelay<<1;
  int16_t validAngle = 0;

  int16_t angleHist[HIST_SIZE];
  uint16_t angleRecord = 0;
  uint8_t check_cycle = 20; 
  uint8_t cycle_delay = 30;
//  uint8_t isDoor = (mr->m == (&appParam.motorConfig[2]))?0:1;
  uint16_t runCycles = 0;
  uint8_t followDelay = 10;
  memset(angleHist,0,HIST_SIZE*2);
  while(run){
    //wdgReset(&WDGD1);
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS,TIME_IMMEDIATE);
    if(evt & EV_ABORT_OC){
      run = false;
      mr->actForce = 0;
    }
    if(evt & EV_ABORT_LOCK){
      run = false;
      mr->actForce = 0;
    }
    if(evt & EV_TERMINATE){
      run = false;
    }
    
    // valid for speed
    int16_t diff_fow;
    if(mr->actType == ACT_OPEN){
      if(mr->m->angle > mr->d->sldOpenAngle){
        speed = mr->d->slowSpeed;
        //validAngle = moduleParam.doorConfig.angleValidDeg>>2;
      }
      else{
        speed = mr->d->normalSpeed;
        //validAngle = moduleParam.doorConfig.angleValidDeg;
      }
      // valid track target
      if(!mr->mf->inPos){
        if(mr->mf->angle < moduleParam.doorConfig.doorFreeAngle){
          diff_fow = mr->mf->angle - mr->m->angle;
          if(diff_fow < (moduleParam.doorConfig.angleDiff>>1)){
            speed = 0;
          }
          else if(diff_fow < moduleParam.doorConfig.angleDiff){
            speed /= 2;
          }
        }
      }
      if(mr->stopType & ST_POSITION){
        if(mr->m->angle >= mr->d->openAngle){
          speed = 0;
          mr->m->inPos = true;
          run = false;
        }
      }
      if(mr->stopType & ST_PEL){
        if(mr->m->pel->read(mr->m->pel)){
          speed = 0;
          mr->m->inPos = true;
          run = false;
        }
      }
      
    }else{ // close
      if(mr->m->angle < mr->d->sldCloseAngle){
        speed = mr->d->slowSpeed;
        //validAngle = moduleParam.doorConfig.angleValidDeg >> 2;
      }
      else{
        speed = mr->d->normalSpeed;
        //validAngle = moduleParam.doorConfig.angleValidDeg;
      }
      // valid track target
      if(!mr->mf->inPos && mr->mf->angle > moduleParam.doorConfig.doorFreeAngle){
          diff_fow = mr->m->angle - mr->mf->angle;
          if(diff_fow < 0){
            //masterLocked = true;
            speed = 1;
          }
          else
          {
            if(diff_fow < moduleParam.doorConfig.angleDiff){
              speed >>= 1;
            }
            else if(diff_fow < (moduleParam.doorConfig.angleDiff>>1)){
              speed = 0;
              emgZero = true;
            }
          }
      }
      if(mr->stopType & ST_POSITION){
        if(mr->m->inhome){
          speed = 0;
          emgZero = true;
          mr->m->inPos = true;
          run = false;
        }
      }
      if(mr->stopType & ST_MEL){
        if(mr->m->mel->read(mr->m->mel)){
          speed = 0;
          emgZero = true;
          mr->m->inPos = true;
          run = false;
        }
      }
        
      if(mr->stopType & ST_HOME){
        if(mr->m->orgFound){
          if(mr->m->inhome){
//          if(mr->m->inhome || (mr->m->angle < mr->d->zero_angle_error)){
            if(mr->m->inhome){
              speed = 0;
              emgZero = true;
              mr->m->inPos = true;
              run = false;
            }else{
              // add short delay here
              if(inpValidCycle) inpValidCycle--;
              if(inpValidCycle == 0){
                speed = 0;
                emgZero = true;
                mr->m->inPos = true;
                if(!mr->m->inhome){
                  SET_ERR(errState,ERR_MOTOR_POSS);
                }
                run = false;
              }
            }
          }
        }else{
          if(mr->m->inhome){
            speed = 0;
            emgZero = true;
            mr->m->inPos = true;
            run = false;
          }
        }
//        else if(mr->m->orgFound && (mr->m->angle < mr->d->zero_angle_error)){
//          if(inpValidCycle) inpValidCycle--;
//          if(inpValidCycle == 0){
//            SET_ERR(errState , ERR_INPOSITION);
//            run = false;
//          }
//        }
      }
    }
    
    
//    if(runCycles == 20){ // check only @ 20 cycles
//      if(mr->actType == ACT_OPEN){
//        if(angleHist[((angleRecord-1) & 0x7)] < angleHist[(angleRecord)&0x7]){
//          SET_ERR(errState,ERR_MOTOR_POL);
//          run = false;
//        }
//      }
//      else{
//        if(angleHist[(angleRecord-1) & 0x7] > angleHist[(angleRecord)&0x7]){
//          SET_ERR(errState,ERR_MOTOR_POL);
//          run = false;
//        }
//      }
//    }
    if(mr->m->orgFound){
      if(mr->m->running){
        if(speed == 0){
          checkCycle = moduleParam.doorConfig.angleValidCycles;
          angleRecord = 0;
          followDelay = 10;
        }else{        
          // update angle history
          for(uint8_t i=HIST_SIZE-1;i>=1;i--){
            angleHist[i] = angleHist[i-1];
          }
          angleHist[0] = mr->m->angle;
          if(followDelay == 0)
            angleRecord++;
          else
            followDelay--;
          // valid for current
          if(currentValidDelay) currentValidDelay--;
          
          if(currentValidDelay == 0){
            if(mr->m->dc_current > mr->d->normalMaxCurrent){
              SET_ERR(errState , ERR_MOTOR_OC);
              speed = 0;
              run = false;
            }
            mr->d->max_working_current = (mr->d->max_working_current>mr->m->dc_current)?mr->d->max_working_current:mr->m->dc_current;

          }
          // valid for door locked
          if(angleRecord > moduleParam.doorConfig.angleValidCycles){
            int16_t diff = angleHist[moduleParam.doorConfig.angleValidCycles] - angleHist[0];
            if(angleHist[0] != 0){
              if(diff < 0) diff *= -1;
              
              if(diff < validAngle){
                lockedCount++;
                if(lockedCount > moduleParam.doorConfig.lockDetectCycle){
                  speed = 0;
                  SET_ERR(errState,ERR_MOTOR_LC);
                }
              }
              else{
                lockedCount = 0;
              }
            }
            
//            checkCycle--;
//            if(checkCycle == 0){
//              checkCycle = moduleParam.doorConfig.angleValidCycles;
//              int16_t diff = mr->m->angle - lastPos;
//              if(diff < 0) diff *= -1;
//              if(speed != 0 && mr->m->angle > validAngle){
//                if(diff < validAngle){
//                  lockedCount++;
//                  if(lockedCount > 3){
//                    speed = 0;
//                    SET_ERR(errState , ERR_MOTOR_LC);
//                  }
//                  
//                }
//                else{
//                  lockedCount = 0;
//                }
//              }
//              lastPos = mr->m->angle;
//            }
          }
        }
      }
    }    
    
    
    // update speed
    if(errState == 0){
      if(speed > mr->maxSpeed) speed = mr->maxSpeed;

      if(speed >= cspeed){
        cspeed += moduleParam.doorConfig.ramp;
        if(cspeed > speed) cspeed = speed;
      }else{
        if(speed == 0) cspeed = 0;
        else{
          cspeed -= moduleParam.doorConfig.ramp;
          if(cspeed < speed) cspeed = speed;
        }
      }
      cspeed = speed;
      if(cspeed == 0 && run){
        motorControlP(true,mr->actType==ACT_OPEN?mr->m->posDir:mr->m->negDir,mr->m,0);
        validAngle = 0;
      }
      else
        motorControlP(true,mr->actType==ACT_OPEN?mr->m->posDir:mr->m->negDir,mr->m,cspeed);
        validAngle = (cspeed * moduleParam.doorConfig.angleValidDeg)/mr->maxSpeed;
    }else{
      motorControlP(false,mr->actType==ACT_OPEN?mr->m->posDir:mr->m->negDir,mr->m,0);
      run = false;
    }
    chThdSleepMilliseconds((CYCLE_TIME));
    runCycles++;
    if(!run){
      if(errState == 0){
        if(mr->actForce == 0)
          motorControlP(false,mr->dir,mr->m,0);
        else
          motorControlP(true,mr->actType==ACT_OPEN?mr->m->posDir:mr->m->negDir,mr->m,mr->actForce);
      }
      }
  }
  chThdExit((msg_t)errState);
}

struct valid_loop{
  uint8_t state;
  uint8_t enMask;
  uint8_t cntr;
  uint8_t hasError;
  bool enabled;
};

static THD_FUNCTION(procDoorTest ,p)
{
  struct valid_loop validLoop;
  motor_run_t mr1,mr2,mr3;
  thread_t *t1,*t2;
  uint8_t stage = 0;
  bool bRun = true;
  int16_t angle, angdif;
  msg_t r1,r2,r3;
  while(bRun){
    switch(stage){
    case 0: // main door direction test
      mr1.m = &appParam.motorConfig[0];
      mr1.d = &moduleParam.door[0];
      mr1.mf = NULL;
      mr1.df = NULL;
      mr1.runType = RT_POS;
      mr1.runTime = 2000; // run 2seconds 
      mr1.actType = ACT_OPEN;
      mr1.stopType = ST_POSITION;
      mr1.actForce = 0;
      mr1.maxSpeed = moduleParam.door[0].slowSpeed;
      angle = mr1.m->angle;
      t1 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(512),"M1",NORMALPRIO,procMotorTimeRun,&mr1);
      r1 = chThdWait(t1);
      angdif = mr1.m->angle - angle;
      if(angdif < -5){ // wrong direction
        
      }
      else if(angdif < 5){ // door not move
        
      }
      chThdSleepMilliseconds(100);
      
    }
    if(chThdShouldTerminateX()){
      bRun = false;
    }
  }
  
}
static THD_FUNCTION(procDoorOpen ,p)
{
  struct valid_loop validLoop;
  motor_run_t mr1,mr2,mr3;
  
  mr1.m = &appParam.motorConfig[0];
  mr1.d = &moduleParam.door[0];
  mr1.mf = NULL;
  mr1.df = NULL;
  mr1.runType = RT_POS;

  mr2.m = &appParam.motorConfig[1];
  mr2.d = &moduleParam.door[1];
  mr2.mf = &appParam.motorConfig[0];
  mr2.df = &moduleParam.door[0];
  mr2.runType = RT_FOLLOW;
  
  mr3.m = &appParam.motorConfig[2];
  mr3.d = &moduleParam.door[2];
  mr3.mf = NULL;
  mr3.df = NULL;
  mr3.runType = RT_POS;
    
  // todo: speed control
  motorControlP(false,DIR_POS,mr1.m,0);
  motorControlP(false,DIR_POS,mr2.m,0);
  motorControlP(false,DIR_POS,mr3.m,0);
  
  
  bool run = true;
  uint32_t errorState = 0;
  
  thread_t *t1,*t2,*t3;
  t1 = t2 = t3 = NULL;
  uint8_t stage = 0;
  uint8_t retry = 0;
  uint32_t retry_delay = 0;
  msg_t r1,r2,r3;
  uint8_t loopCount;
  bool firstOpenM1 = true;
  if(appParam.boardID & LOCK_ENABLED){
    stage = 0;
  }else{
    stage = 1;
    firstOpenM1 = false;
  }
  
  uint8_t enMask = 0x3;
  t1 = t2 = t3 = NULL;
  if(!mr1.m->inhome){
    if(mr1.m->angle > mr1.d->zero_angle_error){
      stage = 1;
    }
  }

  uint8_t openRetry = 0;
  uint16_t speed;
  uint8_t openRevTime = mr1.d->openRevTime;
  if(openRevTime < 2) openRevTime = 2;
  while(run){
    //wdgReset(&WDGD1);
    if(chThdShouldTerminateX()){
      run = false;
    }
    switch(stage){
    case 0:
      //開門壓門
      if(t1 == NULL){
        mr1.runTime = openRevTime*100;
        mr1.actType = ACT_CLOSE;
        mr1.stopType = ST_POSITION;
        mr1.actForce = mr1.d->openRevSpeed;
        speed = moduleParam.door[0].openRevSpeed*(1 + openRetry);
        if(speed > moduleParam.door[0].normalSpeed)
        {
          speed = moduleParam.door[0].normalSpeed;
        }
        mr1.maxSpeed = speed;
        t1 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(512),"M1",NORMALPRIO,procMotorTimeRun,&mr1);
//        chThdSleepMilliseconds((openRevTime-1)*100);
      }
      if(t3 == NULL){ // create working thread if not running
        // 釋放電鎖
        mr3.runTime = moduleParam.doorConfig.lockActiveTime*100;
        mr3.actType = ACT_OPEN;
        mr3.stopType = ST_PEL;
        mr3.actForce = 0;
        mr3.maxSpeed = mr3.d->normalSpeed*(1 + openRetry);
        if(mr3.maxSpeed > 50) mr3.maxSpeed = 50;
        t3 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(512),"M3",NORMALPRIO,procMotorTimeRun,&mr3);
        loopCount = 0;
      }
      else{ // valid thread if started
        loopCount++;
        uint8_t state = 0x0;
        if(t1->state == 0xf) state |= 0x1;
        if(t3->state == 0xf) state |= 0x2;
        
        // todo: check if set timeout as parameter
        if(loopCount > 200){
          SET_ERR(errorState,ERR_TIMEOUT);
          run = false;
        }
        if(state == 0x03){ // thread stopped
          // release memory
          chThdWait(t1); t1 = NULL;
          chThdWait(t3); t3 = NULL;         
          // check lock state
          uint8_t mel = mr3.m->mel->read(mr3.m->mel);
          uint8_t pel = mr3.m->pel->read(mr3.m->pel);
          if((pel ==1) && (mel == 1)){
            SET_ERR(errorState,ERR_LOCK_FB_DUAL);
          }
          else if((pel==0) && (mel == 0)){
            SET_ERR(errorState,ERR_LOCK_FB_NONE);
          }
          else if((pel == 0) && (mel == 1)){ // wrong direction
            SET_ERR(errorState,ERR_LOCK_REVERSED);
          }
          
          if(errorState){  
            // stop this proc
            BEEP(ERR_LOCK_DIR);
            run = false;
          }else{
            t1 = NULL;
            t2 = NULL;
            validLoop.enMask = 0x3;
            retry_delay = moduleParam.doorConfig.actionDelay;
            stage = 1;
          }          
        }
      }
      break;
    case 1:
      if(retry_delay){
        retry_delay--;
      }else{
                
        uint8_t state = 0;
        
        if(appParam.boardID & DUAL_DOOR){
          if(t1 == NULL){
            mr1.actType = ACT_OPEN; 
            mr1.actForce = 0;
            mr1.actTime = 0;
            mr1.stopType = ST_POSITION;
            mr1.maxSpeed = mr1.d->normalSpeed;
            t1 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(512),"M1",NORMALPRIO,procMotorPosRun,&mr1);
          }
          else{
            if(t1->state == 0xf) state |= 0x1;
          }
          
          if(t2 == NULL){
            mr2.actType = ACT_OPEN;
            mr2.stopType = ST_POSITION;
            mr2.actForce = 0;
            mr2.actTime = 0;
            mr2.maxSpeed = mr2.d->normalSpeed;
            // stop m3 if running
            if(mr3.m->running){
              motorControlP(false,0,mr3.m,0);
            }
            t2 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(512),"M2",NORMALPRIO,procMotorFollowRun,&mr2);
          }
          else{
            if(t2->state == 0xf) state |= 0x2;
          }
          
          switch(state){
          case 1: // only t1 stopped
            r1 = t1->u.exitcode;
            if(r1 != MSG_OK){
              // terminate r2
              if(t2 != NULL){
                if(IS_ERR(r1 , ERR_MOTOR_LC))
                  chEvtSignal(t2,EV_ABORT_LOCK);
                else if(IS_ERR(r1 , ERR_MOTOR_OC))
                  chEvtSignal(t2,EV_ABORT_OC);
                // wait t2
                chThdWait(t2);
              }
              chThdWait(t1);
              errorState=r1;
              BEEP(ERR_M1_MASK | (r1-1));
              run = false;
            }
            break;
          case 2: // only t2 stopped
            r2 = t2->u.exitcode;
            if(r2 != MSG_OK){
              // terminate t1
              if(t1 != NULL){
                if(IS_ERR(r2 , ERR_MOTOR_LC))
                  chEvtSignal(t1,EV_ABORT_LOCK);
                else if(IS_ERR(r1 , ERR_MOTOR_OC))
                  chEvtSignal(t1,EV_ABORT_OC);
                // wait t2
                chThdWait(t1);
              }
              chThdWait(t2);
              //alarm(r2);
              errorState=r2;
              BEEP(ERR_M2_MASK | (r2-1));
              run = false;
            }
            break;
          case 3: // both t1 and t2 stopped
            r1 = t1->u.exitcode;
            r2 = t2->u.exitcode;
            if((r1 == MSG_OK) && (r2 == MSG_OK)){
              run = false;
            }else{
              errorState = r1 | r2;
              if(IS_ERR(r1,ERR_MOTOR_LC)){
                moduleParam.door[0].lock_times++;
              }
              if(IS_ERR(r2,ERR_MOTOR_LC)){
                moduleParam.door[1].lock_times++;
              }
              if(r1 != MSG_OK){
                BEEP(ERR_M1_MASK | (r1-1));
              }
              if(r2 != MSG_OK){
                BEEP(ERR_M2_MASK | (r2-1));
              }
            }
            chThdWait(t1);
            chThdWait(t2);
            
            run = false;
            break;
          default:break;
          }        
        }
        else{
          if(t1 == NULL){
            mr1.actType = ACT_OPEN; 
            mr1.actForce = 0;
            mr1.actTime = 0;
            mr1.stopType = ST_POSITION;
            mr1.maxSpeed = mr1.d->normalSpeed;
            t1 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(512),"M1",NORMALPRIO,procMotorPosRun,&mr1);
          }
          else{
            if(t1->state == 0xf) state |= 0x1;
          }
          if(state == 0x1){
            r1 = t1->u.exitcode;
            if(r1 == MSG_OK){
              run = false;
            }
            else{
              errorState=r1;
              if(IS_ERR(r1,ERR_MOTOR_LC)){
                moduleParam.door[0].lock_times++;
              }
              BEEP(ERR_M1_MASK | (r1-1));
            }
            chThdWait(t1);
            run = false;
          }
        }
      }
      break;
    }
    chThdSleepMilliseconds(100);
  }
  chThdExit(errorState);
}

static void RaiseAlarm(uint8_t m, uint8_t mask)
{
  uint8_t err;
  if(m == 1) err = 0x0;
  if(m == 2) err = 0x04;
  err |= mask;
  BEEP(err);
  
}

static THD_FUNCTION(procDoorClose ,p)
{
  struct valid_loop validLoop;
  motor_run_t mr1,mr2,mr3;
  
  mr1.m = &appParam.motorConfig[0];
  mr1.d = &moduleParam.door[0];
  mr1.mf = &appParam.motorConfig[1];
  mr1.df = &moduleParam.door[1];
  mr1.runType = RT_POS;

  mr2.m = &appParam.motorConfig[1];
  mr2.d = &moduleParam.door[1];
  mr2.mf = &appParam.motorConfig[0];
  mr2.df = &moduleParam.door[0];
  mr2.runType = RT_FOLLOW;
  
  mr3.m = &appParam.motorConfig[2];
  mr3.d = &moduleParam.door[2];
  mr3.mf = NULL;
  mr3.df = NULL;
  mr3.runType = RT_POS;
    
  // todo: speed control
  motorControlP(false,DIR_POS,mr1.m,0);
  motorControlP(false,DIR_POS,mr2.m,0);
  motorControlP(false,DIR_POS,mr3.m,0);
  
  
  bool run = true;
  uint32_t errorState = 0;
  
  thread_t *t1,*t2,*t3;
  t1 = t2 = t3 = NULL;
  uint8_t stage = 0;
  uint8_t retry = 0;
  uint32_t retry_delay = 0;
  uint32_t last_delay = 0;
  msg_t r1,r2,r3;
  r1 = r2 = r3 = MSG_OK;
  validLoop.state = 0x0;
  validLoop.cntr = 100;
  validLoop.hasError = 0;
  validLoop.enabled = true;
  validLoop.enMask = 0x3;
  t1 = NULL;
  t2 = NULL;
  t3 = NULL;
  bool emgStop = false;
  while(run){
    //wdgReset(&WDGD1);
    if(chThdShouldTerminateX()){
      run = false;
    }
    eventmask_t flag = chEvtWaitAnyTimeout(ALL_EVENTS,TIME_IMMEDIATE);
    if(flag & EV_TG1_TRIGGER){
      chThdSleepMilliseconds(100);
      if(di_map[TG1_IN].read(&di_map[TG1_IN]) == 0){
        emgStop = true;
      }
    }
    switch(stage){
    case 0:
      r1 = r2 = r3 = MSG_OK;
      if(retry_delay){
        retry_delay--;
      }else{
        uint8_t state = 0x0;
        if(appParam.boardID & DUAL_DOOR){
          // create / monitor main door thread
          if(t2 == NULL){
            mr2.actType = ACT_CLOSE;
            mr2.stopType = ST_HOME;
            mr2.actForce = moduleParam.door[1].closeFwdSpeed;
            mr2.maxSpeed = mr2.d->normalSpeed;
            t2 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(512),"M2",NORMALPRIO,procMotorPosRun,&mr2);
            chThdSleepMilliseconds(100);
          }
          else{
            if(t2->state == 0xf) state |= 0x2;
          }
          if(t1 == NULL){
            mr1.actType = ACT_CLOSE;
            mr1.stopType = ST_HOME;
            mr1.actForce = moduleParam.door[0].closeFwdSpeed; // 關門壓門力量
            mr1.actTime = moduleParam.door[0].closeFwdTime;     // 關門壓門時間
            mr1.maxSpeed = mr1.d->normalSpeed;
            t1 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(512),"M1",NORMALPRIO,procMotorFollowRun,&mr1);
          }
          else{
            if(t1->state == 0xf) state |= 0x1;
          }
          switch(state){
          case 0x1: // t1 terminated only, check if normal or not
            r1 = t1->u.exitcode;
            // if M1 terminate abnormally, terminate M2
//            if(r1 != MSG_OK && r1 != ERR_MOTOR_POSS){
            if(r1 != MSG_OK){
              if(t2 != NULL){
                if(IS_ERR(r1, ERR_LOCKED)){
                  opState.lock_times[0]++;
                  chEvtSignal(t2,ERR_MOTOR_LC);
                }
                else if(IS_ERR(r1 , ERR_MOTOR_OC)){
                  chEvtSignal(t2,EV_ABORT_OC);
                }
                // wait t2
                chThdWait(t2);
              }
              chThdWait(t1);
              errorState=r1;
              run = false;
              BEEP(ERR_M1_MASK | (r1-1));
            }
            break;
          case 0x2: // t2 terminated only
            r2 = t2->u.exitcode;
//            if(r2 != MSG_OK && r2 != ERR_MOTOR_POSS){
            if(r2 != MSG_OK){
              // terminate t1
              if(t1 != NULL){
                if(IS_ERR(r2 , ERR_MOTOR_LC)){
                  opState.lock_times[1]++;
                  chEvtSignal(t1,EV_ABORT_LOCK);
                }
                else if(IS_ERR(r1 , ERR_MOTOR_OC)){
                  chEvtSignal(t1,EV_ABORT_OC);
                }
                // wait t1
                chThdWait(t1);
              }
              chThdWait(t2);
              errorState = r2;
              run = false;
              BEEP(ERR_M2_MASK | (r2-1));
            }
            break;
          case 0x3: // both t1 and t2 terminated
            r1 = t1->u.exitcode;
            r2 = t2->u.exitcode;
            errorState = r1 | r2;
            if(r1 != MSG_OK){
              if(IS_ERR(r1 , ERR_MOTOR_LC) || IS_ERR(r1 , ERR_MOTOR_OC)){
                moduleParam.door[0].lock_times++;
                opState.lock_times[0]++;
                run = false;
                BEEP(ERR_M1_MASK | (r1-1));
              }
            }
            if(r2 != MSG_OK){
  //            SET_ERR(errorState,r2);
              if(IS_ERR(r2 , ERR_MOTOR_LC) || IS_ERR(r2 , ERR_MOTOR_OC)){
                moduleParam.door[0].lock_times++;
                opState.lock_times[1]++;
                run = false;
                BEEP(ERR_M2_MASK | (r2-1));
              }
            }
            chThdWait(t1);
            chThdWait(t2);
            stage = 1;
            retry_delay = moduleParam.doorConfig.actionDelay+moduleParam.door[0].closeFwdTime;
            break;
          default:break;
          }
        }
        else{ // single door
          if(t1 == NULL){
            mr1.mf = NULL;
            mr1.df = NULL;
            mr1.actType = ACT_CLOSE;
            mr1.stopType = ST_HOME;
            mr1.actForce = moduleParam.door[0].closeFwdSpeed;
            mr1.actTime = 0;
            mr1.maxSpeed = mr1.d->normalSpeed;
            t1 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(512),"M1",NORMALPRIO,procMotorPosRun,&mr1);
          }
          else{
            if(t1->state == 0xf) state |= 0x1;
          }
          if(state == 0x1){
            chThdWait(t1);
            r1 = t1->u.exitcode;
            if(r1 != MSG_OK){
              if(IS_ERR(r1 , ERR_MOTOR_LC) || IS_ERR(r1 , ERR_MOTOR_OC)){
                moduleParam.door[0].lock_times++;
                run = false;
              }
              errorState=r1;
              run = false;
              BEEP(ERR_M1_MASK | (r1-1));
            }
            else{
              stage = 1;
              retry_delay = moduleParam.doorConfig.actionDelay+moduleParam.door[0].closeFwdTime;
            }
          }
        }
        if(emgStop){
          if(t1){
            chEvtSignal(t1,EV_ABORT_LOCK);
            chThdWait(t1);
          }
          if(t2){
            chEvtSignal(t2,EV_ABORT_LOCK);
            chThdWait(t2);
          }
          run = false;
          errorState = 0xff;
        }
      }
      break;
    case 1:
      if(retry_delay){
        retry_delay--;
      }else{
        if(appParam.boardID & LOCK_ENABLED){
          if(appParam.boardID & DUAL_DOOR)
            motorControlP(false,0,mr2.m,0);
          chThdSleepMilliseconds(100);
          mr3.actType = ACT_CLOSE;
          mr3.stopType = ST_MEL;
          mr3.maxSpeed = mr3.d->normalSpeed;
          mr3.actForce = 0;
          mr3.runTime = moduleParam.doorConfig.lockActiveTime;
//          t3 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(512),"M3",NORMALPRIO,procMotorPosRun,&mr3);
          t3 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(512),"M3",NORMALPRIO,procMotorTimeRun,&mr3);
          // wait thread exit
          chThdWait(t3);
          // check lock state
          uint8_t mel = mr3.m->mel->read(mr3.m->mel);
          uint8_t pel = mr3.m->pel->read(mr3.m->pel);
          if((pel ==1) && (mel == 1)){
            SET_ERR(errorState,ERR_LOCK_DIR);
          }
          else if((pel==0) && (mel == 0)){
            SET_ERR(errorState,ERR_LOCK_LOST);
          }
          else if((pel == 1) && (mel == 0)){ // wrong direction
            SET_ERR(errorState,ERR_LOCK_DIR);
          }   
          if(errorState){
              BEEP(errorState-1);
          }
          
        }
        if(moduleParam.door[0].closeFwdTime==0xff){
          motorControlP(false,0,mr3.m,0);
          run = false;
        }
        else if(moduleParam.door[0].closeFwdTime){
          last_delay = moduleParam.door[0].closeFwdTime;
          stage = 3;
        }
        else{
          run = false;
          motorControlP(false,0,mr1.m,0);
          if(appParam.boardID & DUAL_DOOR)
            motorControlP(false,0,mr2.m,0);
        }
      }
      break;
    case 3:
      if(last_delay){
        last_delay--;
      }
      if(last_delay == 0){
        run = false;
        motorControlP(false,0,mr3.m,0);
        motorControlP(false,0,mr1.m,0);
//        if(appParam.boardID & DUAL_DOOR)
//          motorControlP(false,0,mr2.m,0);
        if(appParam.boardID & DUAL_DOOR){
          if(!mr1.m->inhome){
            SET_ERR(errorState,ERR_MOTOR_POSS);
            BEEP(ERR_M1_MASK | (ERR_MOTOR_POSS-1));
          }
          if(!mr2.m->inhome){
            SET_ERR(errorState,ERR_MOTOR_POSS);
            BEEP(ERR_M2_MASK | (ERR_MOTOR_POSS-1));
          }
        }
        else{
          if(!mr1.m->inhome){
            SET_ERR(errorState,ERR_MOTOR_POSS);
            BEEP(ERR_M1_MASK | (ERR_MOTOR_POSS-1));
          }
        }
      }
      break;
    default: // error handler case
      if(retry > moduleParam.doorConfig.lockRetry){
        run = false;
      }
      
      break;
    }
    chThdSleepMilliseconds(100);
    
  }
  // check for angle sensor error if normally closed, 2020/11/19
  if(errorState == 0){
    int16_t ang;
    if(appParam.boardID & DUAL_DOOR){
      if(appParam.motorConfig[1].inhome){
        ang = appParam.motorConfig[1].angle<0?-appParam.motorConfig[1].angle:appParam.motorConfig[1].angle;
        if(ang > 20){
          // angle sensor error
          SET_ERR(errorState,ERR_M2_MASK | ERR_MOTOR_ANGS);
          BEEP(errorState);
        }
      }
    }
    if(appParam.motorConfig[0].inhome){
      ang = appParam.motorConfig[0].angle<0?-appParam.motorConfig[0].angle:appParam.motorConfig[1].angle;
      if(ang > 20){
        // angle sensor error
        SET_ERR(errorState,ERR_M1_MASK | ERR_MOTOR_ANGS);
        BEEP(errorState);
      }
    }
  }

  
  chThdExit(errorState);
}

static THD_FUNCTION(procDoorHome ,p)
{
  motor_run_t mr1,mr2,mr3;
  
  mr1.m = &appParam.motorConfig[0];
  mr1.d = &moduleParam.door[0];
  mr1.mf = &appParam.motorConfig[1];
  mr1.df = &moduleParam.door[1];
  mr1.runType = RT_POS;

  mr2.m = &appParam.motorConfig[1];
  mr2.d = &moduleParam.door[1];
  mr2.mf = &appParam.motorConfig[0];
  mr2.df = &moduleParam.door[0];
  mr2.runType = RT_FOLLOW;
  
  mr3.m = &appParam.motorConfig[2];
  mr3.d = &moduleParam.door[2];
  mr3.mf = NULL;
  mr3.df = NULL;
  mr3.runType = RT_POS;
    
  // todo: speed control
  motorControlP(false,DIR_POS,mr1.m,0);
  motorControlP(false,DIR_POS,mr2.m,0);
  motorControlP(false,DIR_POS,mr3.m,0);
  
  
  bool run = true;
  uint32_t errorState = 0;
  
  thread_t *t1,*t2,*t3;
  t1 = t2 = t3 = NULL;
  uint8_t stage = 0;
  uint8_t retry = 0;
  uint32_t retry_delay = 0;
  uint32_t last_delay = 0;
  msg_t r1,r2,r3;
  uint8_t speed;
  uint8_t openRevTime = mr1.d->openRevTime;
  if(openRevTime < 2) openRevTime = 2;
  
  while(run){
    //wdgReset(&WDGD1);
    if(chThdShouldTerminateX()){
      run = false;
    }
    switch(stage){
    case 0:
      if(appParam.boardID & LOCK_ENABLED){ // has lock
        // check if door at home
        if(mr1.m->inhome){
          mr1.runTime = openRevTime*100;
          mr1.actType = ACT_CLOSE;
          mr1.stopType = ST_POSITION;
          mr1.actForce = mr1.d->openRevSpeed;
          speed = moduleParam.door[0].openRevSpeed;
          if(speed > moduleParam.door[0].normalSpeed)
          {
            speed = moduleParam.door[0].normalSpeed;
          }
          mr1.maxSpeed = speed;
          t1 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(512),"M1",NORMALPRIO,procMotorTimeRun,&mr1);
          //chThdSleepMilliseconds((openRevTime-1)*100);
        }
        
        mr3.actType = ACT_OPEN;
        mr3.stopType = ST_PEL;
        mr3.actForce = 0;
        mr3.runTime = moduleParam.doorConfig.lockActiveTime;
        t3 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(512),"M3",NORMALPRIO,procMotorTimeRun,&mr3);
        // wait thread exit
        chThdWait(t3);
        if(t1){
          chThdWait(t1);
          t1 = NULL;
        }
        uint8_t mel = mr3.m->mel->read(mr3.m->mel);
        uint8_t pel = mr3.m->pel->read(mr3.m->pel);
        
        if((pel ==1) && (mel == 1)){
          SET_ERR(errorState , ERR_LOCK_DIR);
        }
        else if((pel==0) && (mel == 0)){
          SET_ERR(errorState , ERR_LOCK_LOST);
        }
        else if((pel == 0) && (mel == 1)){ // wrong direction
          SET_ERR(errorState , ERR_LOCK_DIR);
        }    
        motorControlP(false,DIR_POS,mr3.m,0);
        // todo : add alarm 
        if(errorState){
          motorControlP(false,DIR_POS,mr1.m,0);
          run = false;
          BEEP(errorState-1);
          continue;
        }
        chThdSleepMilliseconds(100);
      }
//      doorCloseCtrl();
      if(appParam.boardID & DUAL_DOOR){
        chThdSleepMilliseconds(moduleParam.doorConfig.actionDelay*100);
        // move m2 home
        mr2.actType = ACT_CLOSE;
        mr2.stopType = ST_HOME;
        mr2.actForce = mr2.d->slowSpeed;
        mr2.maxSpeed = mr2.d->slowSpeed;
        mr2.actTime = 0;
        //chThdSleepMilliseconds(500);
        t2 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(512),"M2",NORMALPRIO,procMotorPosRun,&mr2);
        r2 = chThdWait(t2);
        if(r2 != MSG_OK){
          run = false;
          errorState = r2;
          BEEP(ERR_M2_MASK | (errorState-1));
          continue;
        }
      }
      // move m1 home
      chThdSleepMilliseconds(moduleParam.doorConfig.actionDelay*100);
      mr1.actType = ACT_CLOSE;
      mr1.stopType = ST_HOME;
      mr1.actForce = mr1.d->closeFwdSpeed;
      mr1.maxSpeed = mr1.d->slowSpeed;
      mr1.actTime = 0;
      t1 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(512),"M1",NORMALPRIO,procMotorPosRun,&mr1);
      r1 = chThdWait(t1);
      if(r1 != MSG_OK){
        run = false;
        errorState = r1;
        BEEP(ERR_M1_MASK | (errorState-1));
        continue;
      }
      stage = 1;
      last_delay = moduleParam.doorConfig.actionDelay; // delay 500 ms
      //run = false;
      break;
    case 1:
      if(last_delay){
        last_delay--;
      }else{
        if(appParam.boardID & LOCK_ENABLED){
          motorControlP(false,DIR_POS,mr2.m,0);
          mr3.actType = ACT_CLOSE;
          mr3.stopType = ST_MEL;
          mr3.actForce = 0;
          chThdSleepMilliseconds(100);
          t3 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(512),"M3",NORMALPRIO,procMotorPosRun,&mr3);
          // wait thread exit
          chThdWait(t3);
          chThdSleepMilliseconds(100);
          motorControlP(false,DIR_POS,mr1.m,0);
          uint8_t mel = mr3.m->mel->read(mr3.m->mel);
          uint8_t pel = mr3.m->pel->read(mr3.m->pel);
          
          if((pel ==1) && (mel == 1)){
            SET_ERR(errorState , ERR_LOCK_FB_DUAL);
          }
          else if((pel==0) && (mel == 0)){
            SET_ERR(errorState , ERR_LOCK_FB_NONE);
          }
          else if((pel == 1) && (mel == 0)){ // wrong direction
            SET_ERR(errorState , ERR_LOCK_REVERSED);
          }    
          // todo : add alarm for wrong lock

          if(errorState){
            BEEP(errorState-1);
            run = false;
            continue;
            }
          }else{
            motorControlP(false,DIR_POS,mr2.m,0);
            chThdSleepMilliseconds(200);
            motorControlP(false,DIR_POS,mr1.m,0);
          }
          run = false;
        // set zero angle
        //moduleParam.door[0].zeroAngle = //
        }
      break;
    }
    chThdSleepMilliseconds(100);
  }
  //alarm(ALM_LOCK);
  chThdExit(errorState);
}

#define EVM_TIMERUN_POS     EVENT_MASK(21)
#define EVM_TIMERUN_NEG     EVENT_MASK(22)
#define EVM_FOLLOWRUN_POS   EVENT_MASK(23)
#define EVM_FOLLOWRUN_NEG   EVENT_MASK(24)

enum motor_stage{
  MS_IDLE,
  MS_TIMERUN,
  MS_POSRUN,
};


void app_loadDefault(void)
{
  
}

static void door_close_cb(void *arg)
{
  chSysLockFromISR();
  if(appParam.mainThread){
    appParam.closeByTimeout = 1;
    chEvtSignalI(appParam.mainThread,EV_TG2_TRIGGER);
  }
  chSysUnlockFromISR();
}


static msg_t doorOpenCtrl(void)
{
  thread_t *t1,*t2;
  bool run = true;
  uint8_t err = 0;
  uint8_t retry = 0;
  uint8_t dir;
  t1 = t2 = NULL;
  msg_t r1 = MSG_OK, r2 = MSG_OK;
  uint8_t stage = 0;
  do{
    switch(stage){
    case 0:
      dir = DIR_POS;
      do_map[DOOR_CLOSED].clear(&do_map[DOOR_CLOSED]);
      t1 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(512),"DoorOpenProc",NORMALPRIO,procDoorOpen,&dir);
      r1 = chThdWait(t1);
      t1 = NULL;
      appParam.errState = r1;
      if(r1 == MSG_OK){
        run = false;
      }
      else{
//        alarm(r1);
//        if(IS_ERR(r1,ERR_MOTOR_LC))
//          alarm(ALM_DOOR_BLOCKING);
//        if(IS_ERR(r1,ERR_MOTOR_OC))
//          alarm(ALM_DOOR_OC);
//        if(IS_ERR(r1,ERR_MOTOR_POSS))
//          alarm(ALM_INP);
//        if(IS_ERR(r1,ERR_LOCK_DIR) || IS_ERR(r1,ERR_LOCK_LOST) || IS_ERR(r1,ERR_LOCK_FB_DUAL))
//          alarm(ALM_LOCK);
        for(uint8_t i=0;i<moduleParam.doorConfig.lockRetryIdleCycles/100;i++){
          //wdgReset(&WDGD1);
          chThdSleepMilliseconds(100);
        }
        stage = 1;
      }
      break;
    case 1:
      dir = DIR_NEG;
      // close door and reopen
      t2 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(512),"DoorCloseProc",NORMALPRIO,procDoorClose,&dir);
      r2 = chThdWait(t2);
      appParam.errState = r2;
      t2 = NULL;
      retry++;
      if(retry > moduleParam.doorConfig.lockRetry){
        run = false;
      }
      if(r2 == MSG_OK){
        stage = 0;
      }else{
//        alarm(r2);
//        if(IS_ERR(r2,ERR_LOCKED))
//          alarm(ALM_DOOR_BLOCKING);
//        if(IS_ERR(r2,ERR_OVERCURRENT))
//          alarm(ALM_DOOR_OC);
//        if(IS_ERR(r2,ERR_INPOSITION))
//          alarm(ALM_INP);
//        if(IS_ERR(r2,ERR_LOCK_FAIL) || IS_ERR(r2,ERR_LOCK_FB_NONE) || IS_ERR(r2,ERR_LOCK_FB_DUAL))
//          alarm(ALM_LOCK);
      }
      for(uint8_t i=0;i<moduleParam.doorConfig.lockRetryIdleCycles/100;i++){
        //wdgReset(&WDGD1);
        chThdSleepMilliseconds(100);
      }
      break;
    }
  }while(run);
  
  t1 = NULL;
  if(r1 == MSG_OK && r2 == MSG_OK){
    do_map[DOOR_OPEN_DONE].set(&do_map[DOOR_OPEN_DONE]);    
    return MSG_OK;
  }else{
    return MSG_RESET;
  }
}

static msg_t doorCloseCtrl(void)
{
  thread_t *t1,*t2;
  bool run = true;
  uint8_t err = 0;
  uint8_t retry = 0;
  uint8_t dir;
  msg_t r1 = MSG_OK, r2 = MSG_OK;
  t1 = t2 = NULL;
  uint8_t stage = 0;
  appParam.doorClosing = 1; // closing
  bool reOpen = false;
  do_map[DOOR_OPEN_DONE].clear(&do_map[DOOR_OPEN_DONE]);
  do{
    switch(stage){
    case 0:
      dir = DIR_NEG;
      do_map[DOOR_CLOSED].clear(&do_map[DOOR_CLOSED]);
      t1 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(512),"DoorCloseProc",NORMALPRIO,procDoorClose,&dir);
      appParam.closingThread = t1;
      r1 = chThdWait(t1);
      t1 = NULL;
      if(r1 == MSG_OK){
        run = false;
//        if(IS_ERR(r1,ERR_INPOSITION))
//          alarm(ALM_INP);
      }
      else if(r1 == 0xff){ // user close
        reOpen = true;
        run = false;
      }
      else if(IS_ERR(r1,ERR_MOTOR_POSS)){
        run = false;
        //BEEP(r1);
      }
      else{
        ///BEEP(r1);
//        if(IS_ERR(r1,ERR_LOCKED))
//          alarm(ALM_DOOR_BLOCKING);
//        if(IS_ERR(r1,ERR_OVERCURRENT))
//          alarm(ALM_DOOR_OC);
//        if(IS_ERR(r1,ERR_LOCK_FAIL) || IS_ERR(r1,ERR_LOCK_FB_NONE) || IS_ERR(r1,ERR_LOCK_FB_DUAL))
//          alarm(ALM_LOCK);
        for(uint8_t i=0;i<moduleParam.doorConfig.lockRetryIdleCycles/100;i++){
          //wdgReset(&WDGD1);
          chThdSleepMilliseconds(100);
        }
        stage = 1;
      }
      break;
    case 1:
      dir = DIR_POS;
      // close door and reopen
      t2 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(512),"DoorCloseProc",NORMALPRIO,procDoorOpen,&dir);
      r2 = chThdWait(t2);
      t2 = NULL;
//      chThdSleepMilliseconds(moduleParam.doorConfig.lockRetryIdleCycles);
      retry++;
      if(retry > moduleParam.doorConfig.lockRetry){
        run = false;
      }
      if(r2 == MSG_OK){
        if(reOpen){
          run = false;
        }
        else{
          stage = 0;
        }
        //alarm(ALM_DOOR_BLOCKING);
      }else{
//        if(IS_ERR(r2,ERR_LOCKED))
//          alarm(ALM_DOOR_BLOCKING);
//        if(IS_ERR(r2,ERR_OVERCURRENT))
//          alarm(ALM_DOOR_OC);
//        if(IS_ERR(r2,ERR_INPOSITION))
//          alarm(ALM_INP);
//        if(IS_ERR(r2,ERR_LOCK_FAIL) || IS_ERR(r2,ERR_LOCK_FB_NONE) || IS_ERR(r2,ERR_LOCK_FB_DUAL))
//          alarm(ALM_LOCK);
      }
        for(uint8_t i=0;i<moduleParam.doorConfig.lockRetryIdleCycles/100;i++){
          //wdgReset(&WDGD1);
          chThdSleepMilliseconds(100);
        }
      break;
    }
  }while(run);
  
  appParam.doorClosing = 0;
  t1 = NULL;
  if(reOpen){
    return 0xff;
  }else{
    if(r1 == MSG_OK && r2 == MSG_OK){
      return MSG_OK;
    }else{
      return MSG_RESET;
    }
  }
}
static WDGConfig wdgcfg = {
  STM32_IWDG_PR_32,
  500
};
static THD_WORKING_AREA(waValidDoorState,512);
static THD_FUNCTION(procValidDoorState ,p)
//static void ValidDoorState(void *arg)
{
  (void)p;
  wdgStart(&WDGD1,&wdgcfg);
  while(1){
    if(appParam.doorClosing == 0){ // not judge if closing
      bool c1 = true;
      
      if(appParam.motorConfig[0].inhome == 0){
        c1 = false;
      }
     
      if(appParam.motorConfig[0].angle > moduleParam.door[0].zero_angle_error){
        c1 = false;
      }
      
      
      if(appParam.boardID & DUAL_DOOR){
        if(appParam.motorConfig[1].inhome == 0){
          c1 = false;
        }
        if(appParam.motorConfig[1].angle > moduleParam.door[1].zero_angle_error){
          c1 = false;
        }
      }
      
      if(appParam.boardID & LOCK_ENABLED){ // has lock
        if(appParam.motorConfig[2].mel->read(appParam.motorConfig[2].mel) == 0){
          c1 = false;
        }
      }    
      // check for lock state
      
      if(c1){
        do_map[DOOR_CLOSED].set(&do_map[DOOR_CLOSED]);
      }
      else{
        do_map[DOOR_CLOSED].clear(&do_map[DOOR_CLOSED]);
      }
    }
    wdgReset(&WDGD1);
    handleADCCalculation();  
    chThdSleepMilliseconds(200);
  }
  
  //wdgReset(&WDGD1);
//  chSysLockFromISR();
//  handleADCCalculation();  
//  chVTSetI(&appParam.vtDoorCheck,MS2ST(500),ValidDoorState,NULL);
//  chSysUnlockFromISR();
  
}

static THD_WORKING_AREA(waDoorControl,2048);
static THD_FUNCTION(procDoorControl ,p)
{
  virtual_timer_t vt_door_close;
  //dio_map_t *TG1_IN = &di_map[TG1_IN];
  //dio_map_t *TG2_IN = &di_map[TG2_IN];
  _motor_config_t *m1 = &motors[MOTOR_MAIN];
  _motor_config_t *m2 = &motors[MOTOR_SUB];
  _motor_config_t *m3 = &motors[MOTOR_LOCK];
  _door_config_t *d1 = &moduleParam.door[0];
  _door_config_t *d2 = &moduleParam.door[1];
  _door_config_t *d3 = &moduleParam.door[2];
  uint32_t dir;
  msg_t ret;
  bool ready = false;

  extChannelEnable(&EXTD1,0);
  extChannelEnable(&EXTD1,4);

  motorControlP(false,DIR_POS,m1,0);
  motorControlP(false,DIR_POS,m2,0);
  motorControlP(false,DIR_POS,m3,0);

  // start ADC
  adcStart(&ADCD1,NULL);
  adcStartConversion(&ADCD1,&adcgrpcfg,samples,ADC_GRP1_BUF_DEPTH);

  chVTObjectInit(&vt_door_close);
//  chVTObjectInit(&appParam.vtDoorCheck);
//  chVTSet(&appParam.vtDoorCheck,MS2ST(200),ValidDoorState,NULL);
  chThdSleepMilliseconds(100);
  if((appParam.boardID & MODEL_DOOR_LEFT) == MODEL_DOOR_LEFT){ // 0:OFF, 右型機, 1:ON:左型機
    m1->posDir = DIR_NEG;
    m1->negDir = DIR_POS;
    m2->posDir = DIR_POS;
    m2->negDir = DIR_NEG;
    m3->posDir = DIR_NEG;
    m3->negDir = DIR_POS;
    
  }else{
    m1->posDir = DIR_POS;
    m1->negDir = DIR_NEG;
    m2->posDir = DIR_NEG;
    m2->negDir = DIR_POS;
    m3->posDir = DIR_NEG;
    m3->negDir = DIR_POS;
  }

  appParam.motorConfig[0].inhome = appParam.motorConfig[0].mel->read(appParam.motorConfig[0].mel)==0?1:0;
  appParam.motorConfig[1].inhome = appParam.motorConfig[1].mel->read(appParam.motorConfig[1].mel)==0?1:0;
  thread_t *thread_m1 = NULL, *thread_m2 = NULL;
  thread_m1 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(1024),"PROC_HOME",NORMALPRIO,procDoorHome,NULL);
  //chThdSleepMilliseconds(500);
  ret = chThdWait(thread_m1);
  thread_m1 = NULL;
  appParam.errState = ret;
//  if(ret != MSG_OK){
//    if(IS_ERR(ret,ERR_LOCK_FAIL) || IS_ERR(ret,ERR_LOCK_FB_NONE) || IS_ERR(ret,ERR_LOCK_FB_DUAL))
//      alarm(ALM_LOCK);
//    chThdExit((msg_t)-1);
//  }

  appParam.doorState[0].idle_current = appParam.doorState[0].op_current;
  appParam.doorState[1].idle_current = appParam.doorState[1].op_current;
  
  do_map[DOOR_OPEN_DONE].clear(&do_map[DOOR_OPEN_DONE]);
//  thread_m1 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(512),"MOTOR_M3",NORMALPRIO,procDoorClose,&dir);
//  chThdWait(thread_m1);
//  thread_m1 = NULL;
  do_map[DOOR_CLOSED].set(&do_map[DOOR_CLOSED]);

  msg_t err = MSG_OK;
  if(appParam.boardID & DUAL_DOOR){
    if(appParam.motorConfig[1].inhome){
      moduleParam.door[1].zeroAngle = appParam.icu[1].degree;
      appParam.motorConfig[1].orgFound = 1;
    }else{
      BEEP(RPT_SPOS_SENSOR);
    }
  }

  if(appParam.motorConfig[0].inhome){
    moduleParam.door[0].zeroAngle = appParam.icu[0].degree;
    appParam.motorConfig[0].orgFound = 1;
  }else{
      BEEP(RPT_MPOS_SENSOR);
  }
  
  if(err == MSG_OK){
    ready = true;
  }
    
  appParam.closeByTimeout = 0;
  appParam.openTimes = 0;
  appParam.doorReopen = 0;
  //appParam.doorState = 0;
  extChannelEnable(&EXTD1,5);
  extChannelEnable(&EXTD1,6);
  extChannelEnable(&EXTD1,13);
  while(1){
    //wdgReset(&WDGD1);
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS,MS2ST(100));
    //handleADCCalculation();
    // check if door angle > allowable error
    //ValidDoorState();
    
    if(appParam.userInPress == 1){
      if(appParam.userPressedTime == 0) continue;
      appParam.userPressedTime--;
      if(appParam.userPressedTime == 0){ // 3-seconds
        // check dip switch state
        uint8_t sta = 0xff;//read_board_id();
        uint8_t var1 = (uint8_t)appParam.icu[0].degree;
        uint8_t var2 = (uint8_t)appParam.icu[1].degree;
        if(sta & 0x8){
          switch((sta & 0xf0) >> 4){
          case 0:
            moduleParam.door[0].openAngle = var1;
            moduleParam.door[1].openAngle = var2;
            break;
          case 1:
            moduleParam.door[0].sldOpenAngle = var1;
            moduleParam.door[1].sldOpenAngle = var2;
            break;
          case 2:
            moduleParam.door[0].sldCloseAngle = var1;
            moduleParam.door[1].sldCloseAngle = var2;
            break;
          case 3:
            var1 += 10;
            if(var1 > 100) var1 = 100;
            if(var1 < 10) var1 = 10;
            moduleParam.door[0].normalSpeed = var1;
            var2 += 10;
            if(var2 > 100) var2 = 100;
            if(var2 < 10) var2 = 10;
            moduleParam.door[1].normalSpeed = var1;
            break;
          case 4:
            var1 += 10;
            if(var1 > 100) var1 = 100;
            if(var1 < 10) var1 = 10;
            moduleParam.door[0].slowSpeed = var1;
            var2 += 10;
            if(var2 > 100) var2 = 100;
            if(var2 < 10) var2 = 10;
            moduleParam.door[1].slowSpeed = var1;
            break;
          case 5:
            var1 += 10;
            if(var1 > 50) var1 = 50;
            if(var1 < 10) var1 = 10;
            moduleParam.door[2].normalSpeed = var1;
            break;
          case 6:
            break;
          case 7:
            break;
          }
        }
        else{
          moduleParam.door[0].zeroAngle = appParam.icu[0].degree;
          moduleParam.door[1].zeroAngle = appParam.icu[1].degree;
        }
        sysSaveParams();
        do_map[BUZZER].clear(&do_map[BUZZER]);
        appParam.buzzer.ms_on = 250;
        appParam.buzzer.ms_off = 250;
        appParam.buzzer.times = 3;
        //chVTReset(&appParam.vtBeep);
        //chVTSet(&appParam.vtBeep,MS2ST(100),beep_timeout,NULL);
        appParam.userInPress = 0;
      }
    }
    
    if(evt & EV_USR_TRIGGER){
      uint16_t tmp;
      chThdSleepMilliseconds(100);
      if(di_map[USR_BTN].read(&di_map[USR_BTN]) ==0){
        uint16_t sw = read_board_id();
        sw >>=4;
        switch(sw){
        case 0:
          if(ready){
            appParam.openTimes++;
            //do_map[DOOR_CLOSED].clear(&do_map[DOOR_CLOSED]);
            err = doorOpenCtrl();
            appParam.errState = err;
            if(err == MSG_OK){
              //do_map[DOOR_OPEN_DONE].set(&do_map[DOOR_OPEN_DONE]);
              if(moduleParam.doorConfig.waitTimeToClose == 0){
                chEvtSignal(appParam.mainThread,EV_TG2_TRIGGER);
  //              evt |= EV_TG2_TRIGGER;
              }else{
                chVTSet(&vt_door_close,S2ST(moduleParam.doorConfig.waitTimeToClose),door_close_cb,NULL);
              }
            }
            else{
//              if(err & ERR_LOCKED)
//                alarm(ALM_DOOR_BLOCKING);
//              if(err & ERR_OVERCURRENT)
//                alarm(ALM_DOOR_OC);
            }
          }
//          dir = DIR_POS;
//          do_map[DOOR_CLOSED].clear(&do_map[DOOR_CLOSED]);
//          thread_m1 = chThdCreateFromHeap(NULL,THD_WORKING_AREA_SIZE(1024),"MOTOR_M3",NORMALPRIO,procDoorOpen,&dir);
//          chThdWait(thread_m1);
//          thread_m1 = NULL;
          break;
        case 1://主門主速
          tmp = appParam.vr6 /58 + 30; //
          if(tmp > 100) tmp = 100;
          d1->normalSpeed = tmp;
          sysSaveParams();
          set_beep(250,250,tmp/10+1);
          break;
        case 2:// 主門緩速
          tmp = appParam.vr6/63 + 5;
          if(tmp > 70) tmp = 70;
          d1->slowSpeed = tmp;
          sysSaveParams();
          set_beep(250,250,tmp/10+1);
          break;
        case 3: //主門開門位置
          d1->openAngle = m1->angle;
          //d2->openAngle = m2->angle;
          sysSaveParams();
          set_beep(250,250,1);
          break;
        case 4: // 主門開/關門減速位置
          d1->sldOpenAngle = m1->angle;
          d1->sldCloseAngle = d1->openAngle - d1->sldOpenAngle;
          //d2->sldOpenAngle = m2->angle;
          sysSaveParams();
          set_beep(250,250,1);
          break;
        case 5://副門主速
          tmp = appParam.vr6 /58 + 30; //
          if(tmp > 100) tmp = 100;
          d2->normalSpeed = tmp;
          sysSaveParams();
          set_beep(250,250,tmp/10+1);
          break;
        case 6:// 副門緩速
          tmp = appParam.vr6/63 + 5;
          if(tmp > 70) tmp = 70;
          d2->slowSpeed = tmp;
          sysSaveParams();
          set_beep(250,250,tmp/10+1);
          break;
        case 7: //副門開門位置
          d2->openAngle = m2->angle;
          //d2->openAngle = m2->angle;
          sysSaveParams();
          set_beep(250,250,1);
          break;
        case 8: // 副門開/關門減速位置
          d2->sldOpenAngle = m2->angle;
          d2->sldCloseAngle = d2->openAngle - d2->sldOpenAngle;
          //d2->sldOpenAngle = m2->angle;
          sysSaveParams();
          set_beep(250,250,1);
          break;
        case 9: // 開/關門角度差
          moduleParam.doorConfig.angleDiff = m1->angle - m2->angle;
          sysSaveParams();
          set_beep(250,250,1);
          break;
        case 10://電鎖力道
          tmp = appParam.vr6 /102 + 10; //
          if(tmp > 50) tmp = 50;
          d3->normalSpeed = tmp;
          sysSaveParams();
          set_beep(250,250,tmp/10+1);
          break;
        case 11://電鎖作動時間
          tmp = appParam.vr6 /204; //
          if(tmp > 20) tmp = 20;
          d3->closeFwdTime = d3->openRevTime = tmp;
          sysSaveParams();
          set_beep(250,250,1);
          break;
        case 12://主門阻擋敏感度
        case 13:
          tmp = appParam.vr6 /409 + 1; //
          if(tmp > 10) tmp = 10;
          //moduleParam.doorConfig.lockActiveTime = tmp;
          moduleParam.doorConfig.angleValidDeg = tmp;
          sysSaveParams();
          set_beep(250,250,1);
          break;
        case 14:
          tmp = appParam.vr6 /136; //
          moduleParam.doorConfig.waitTimeToClose = tmp;
          sysSaveParams();
          set_beep(250,250,tmp/5+1);
          break;
        case 15:
          defaultParams();
          sysSaveParams();
          set_beep(250,250,1);
          break;
        }
      }
    }


    if(evt & EV_TG1_TRIGGER){
      if(ready){
        chThdSleepMilliseconds(100);
        if(di_map[TG1_IN].read(&di_map[TG1_IN]) ==0 || appParam.doorReopen == 1){
          appParam.doorReopen = 0;
          appParam.openTimes++;
          //do_map[DOOR_CLOSED].clear(&do_map[DOOR_CLOSED]);
          err = doorOpenCtrl();
          appParam.errState = err;
          if(err == MSG_OK){
            if(di_map[TG1_IN].read(&di_map[TG1_IN]) == 1){
              if(moduleParam.doorConfig.waitTimeToClose == 0){
                appParam.closeByTimeout = 1;
                chEvtSignal(appParam.mainThread,EV_TG2_TRIGGER);
              }else{
                chVTSet(&vt_door_close,S2ST(moduleParam.doorConfig.waitTimeToClose),door_close_cb,NULL);
              }
            }
          }
          else{
          }
        }
        else if(di_map[TG1_IN].read(&di_map[TG1_IN]) == 1){ // check if door @home, start count down to close door 
          //bool act = false;
          if(appParam.motorConfig[0].inhome == 0){
            if(moduleParam.doorConfig.waitTimeToClose == 0){
              appParam.closeByTimeout = 1;
              chEvtSignal(appParam.mainThread,EV_TG2_TRIGGER);
            }else{
              chVTSet(&vt_door_close,S2ST(moduleParam.doorConfig.waitTimeToClose),door_close_cb,NULL);
            }
//            chVTSet(&vt_door_close,S2ST(moduleParam.doorConfig.waitTimeToClose),door_close_cb,NULL);
          }
        }
      }
    }
    
    if(evt & EV_TG1_MBTRG){
      if(ready){
        chThdSleepMilliseconds(100);
        appParam.openTimes++;
        //do_map[DOOR_CLOSED].clear(&do_map[DOOR_CLOSED]);
        err = doorOpenCtrl();
        appParam.errState = err;
        if(err == MSG_OK){
          //do_map[DOOR_OPEN_DONE].set(&do_map[DOOR_OPEN_DONE]);
          if(moduleParam.doorConfig.waitTimeToClose == 0){
            appParam.closeByTimeout = 1;
            chEvtSignal(appParam.mainThread,EV_TG2_TRIGGER);
          }else{
            chVTSet(&vt_door_close,S2ST(moduleParam.doorConfig.waitTimeToClose),door_close_cb,NULL);
          }
        }
        else{
//          if(err & ERR_LOCKED)
//            alarm(ALM_DOOR_BLOCKING);
//          if(err & ERR_OVERCURRENT)
//            alarm(ALM_DOOR_OC);
        }
      }
    }

    if(evt & EV_TG2_TRIGGER){
      if(di_map[TG1_IN].read(&di_map[TG1_IN]) ==0 || (appParam.closeByTimeout == 1)){
        appParam.closeByTimeout = 0;
        //do_map[DOOR_OPEN_DONE].clear(&do_map[DOOR_OPEN_DONE]);
        err = doorCloseCtrl();
        appParam.errState = err;
        if(err == MSG_OK){
          //do_map[DOOR_CLOSED].set(&do_map[DOOR_CLOSED]);
        }
        else if(err == 0xff){ // reopen
          appParam.doorReopen = 1;
        }
        else{
//          if(err & ERR_LOCKED)
//            alarm(ALM_DOOR_BLOCKING);
//          if(err & ERR_OVERCURRENT)
//            alarm(ALM_DOOR_OC);
        }
      }
    }
    
    if(evt & EV_SYS_CLEAR_ERROR){
      reset_buzzer();
    }

    if(evt & EV_SYS_SAVE_PARAM){
      sysSaveParams();
    }
    
    if(evt & EV_SYS_RESET){
      chSysDisable();
      NVIC_SystemReset();
    }
    
    if(evt & EV_SYS_RESET_NVM){
      opState.openTimes = 0;
      for(uint8_t i=0;i<3;i++){
        opState.lock_times[i] = 0;
        opState.max_working_current[i] = 0;
      }
      sysSaveOpstate();
      appParam.openTimes = 0;
    }
    if(appParam.openTimes >= 10){
      opState.openTimes += appParam.openTimes;
      opState.lock_times[0] = moduleParam.door[0].lock_times;
      opState.lock_times[1] = moduleParam.door[1].lock_times;
      opState.max_working_current[0] = moduleParam.door[0].max_working_current;
      opState.max_working_current[1] = moduleParam.door[0].max_working_current;
      sysSaveOpstate();
      appParam.openTimes = 0;
    }
  }
  
}


void main(void)
{
  halInit();
  chSysInit();
  
  if((RCC->CSR & RCC_CSR_WWDGRSTF) == RCC_CSR_WWDGRSTF){
    opState.openTimes++;
    sysSaveOpstate();
  }
  
  RCC->CSR |= RCC_CSR_RMVF;

  //chThdSleepMilliseconds(100);
  i2cInit();
  i2cStart(&I2CD1,&i2ccfg);
  at24eep_init(&I2CD1,32,1024,0x50,2);

  sysParamInit();
  modbusBindingInit();
  
  moduleParam.door[0].normalMaxCurrent = 3000;
  moduleParam.door[1].normalMaxCurrent = 3000;
  
  motors[0].orgFound = 0;
  motors[1].orgFound = 0;
  appParam.boardID = read_board_id();
  appParam.motorConfig = motors;
  appParam.buzzer.ms_off = 0xffff;
  appParam.buzzer.ms_on = 0x0;
  chVTObjectInit(&appParam.vtBeep);

  extStart(&EXTD1,&extcfg);
// bt module
#ifdef APP_USE_RSI
  spiStart(&SPID3,&spicfg_rsi);
  extChannelEnable(&EXTD1,9);
  rsi_app_init();
#endif
  
//  extChannelEnable(&EXTD1,0);
//  extChannelEnable(&EXTD1,4);
//  extChannelEnable(&EXTD1,5);
//  extChannelEnable(&EXTD1,6);
//  extChannelEnable(&EXTD1,13);
  
  uint16_t period;
  if((moduleParam.doorConfig.freq1 < 820) || (moduleParam.doorConfig.freq1 > 2000))
    period = 820;
  else  
    period = moduleParam.doorConfig.freq1;
  
  period = 400;
  pwmcfg_t4.period = period;
  pwmcfg_t5.period = period;
  pwmStart(&PWMD4, &pwmcfg_t4);
  pwmStart(&PWMD5, &pwmcfg_t5);

  //chThdSleepMilliseconds(500);
//  _motor_config_t *m1 = &appParam.motorConfig[0];
//  _motor_config_t *m2 = &appParam.motorConfig[1];
//  _motor_config_t *m3 = &appParam.motorConfig[2];
//  _door_config_t *d1 = &moduleParam.door[0];
//  _door_config_t *d2 = &moduleParam.door[1];
//  _door_config_t *d3 = &moduleParam.door[2];
  moduleParam.door[0].sldCloseAngle = moduleParam.door[0].openAngle - moduleParam.door[0].sldOpenAngle;
  moduleParam.door[1].sldCloseAngle = moduleParam.door[1].openAngle - moduleParam.door[1].sldOpenAngle;
  if(moduleParam.door[0].sldCloseAngle < 40)
    moduleParam.door[0].sldCloseAngle = 40;
  
  if(moduleParam.door[1].sldCloseAngle < 40)
    moduleParam.door[1].sldCloseAngle = 40;
   
  iirInitF(2,4);
  
  chThdCreateStatic(waValidDoorState,sizeof(waValidDoorState),NORMALPRIO,procValidDoorState,NULL);
  appParam.mainThread = chThdCreateStatic(waDoorControl,sizeof(waDoorControl),NORMALPRIO,procDoorControl,NULL);

  while(1){
    chThdSleepMilliseconds(100);
  }
}

void ang1_int_handler(EXTDriver *extp, expchannel_t channel)
{
  dio_map_t *p = &di_map[ANG1_ICP];
  systime_t now = chVTGetSystemTimeX();
  if(p->read(p)){ // rising
    if(appParam.icu[0].last != 0){
      appParam.icu[0].width = now - appParam.icu[0].last;
      appParam.icu[0].last = now;
      if(appParam.icu[0].width > 400){ // at least 4ms
        appParam.icu[0].percent = (float)appParam.icu[0].period*100./(float)appParam.icu[0].width;
  //      appParam.icu[0].degree = appParam.icu[0].percent * moduleParam.door[0].degree_percent;
        if(appParam.icu[0].percent < 8 || appParam.icu[0].percent > 92){
          // error
        }else{
          appParam.icu[0].degree = (appParam.icu[0].percent-10) * moduleParam.doorConfig.degree_percent;
          if(appParam.motorConfig[0].orgFound){
            if(appParam.boardID & MODEL_DOOR_LEFT){
              appParam.motorConfig[0].angle = iirInsertF(0,-(appParam.icu[0].degree - moduleParam.door[0].zeroAngle));
            }else{
              appParam.motorConfig[0].angle = iirInsertF(0,appParam.icu[0].degree - moduleParam.door[0].zeroAngle);
            }
            if(appParam.motorConfig[0].angle < 0) appParam.motorConfig[0].angle = 0;
          }
        }
      }
    }
    appParam.icu[0].last = now;
  }else{
    if(appParam.icu[0].last != 0){
      appParam.icu[0].period = now - appParam.icu[0].last;
    }
  }
  appParam.motorConfig[0].inhome = appParam.motorConfig[0].mel->read(appParam.motorConfig[0].mel)==0?1:0;
  //ValidDoorState();
}

void ang2_int_handler(EXTDriver *extp, expchannel_t channel)
{
  dio_map_t *p = &di_map[ANG2_ICP];
  systime_t now = chVTGetSystemTimeX();
  if(p->read(p)){ // rising
    if(appParam.icu[1].last != 0){
      appParam.icu[1].width = now - appParam.icu[1].last;
      if(appParam.icu[1].width > 400){ // at least 4 ms
        appParam.icu[1].percent = (float)appParam.icu[1].period*100./(float)appParam.icu[1].width;
        if(appParam.icu[0].percent < 8 || appParam.icu[0].percent > 92){
          // error
        }else{

          appParam.icu[1].degree = (appParam.icu[1].percent - 10) * moduleParam.doorConfig.degree_percent;
    //      appParam.icu[1].degree = (appParam.icu[1].percent-10) * 2.75;
          if(appParam.motorConfig[1].orgFound){
            if(appParam.boardID & MODEL_DOOR_LEFT){
              appParam.motorConfig[1].angle = iirInsertF(1,(appParam.icu[1].degree- moduleParam.door[1].zeroAngle));
            }else{
              appParam.motorConfig[1].angle = iirInsertF(1,-(appParam.icu[1].degree- moduleParam.door[1].zeroAngle));
            }
            if(appParam.motorConfig[1].angle < 0) appParam.motorConfig[1].angle = 0;
          }
        }
      }
    }
    appParam.icu[1].last = now;
  }else{
    if(appParam.icu[1].last != 0){
      appParam.icu[1].period = now - appParam.icu[1].last;
    }
  }
  appParam.motorConfig[1].inhome = appParam.motorConfig[1].mel->read(appParam.motorConfig[1].mel)==0?1:0;
  //ValidDoorState();
}

void tg1_int_handler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  if(appParam.doorClosing == 0){
    if(appParam.mainThread)
      chEvtSignalI(appParam.mainThread,EV_TG1_TRIGGER);
  }
  else if(appParam.doorClosing == 1){
    if(appParam.closingThread){
      chEvtSignalI(appParam.closingThread,EV_TG1_TRIGGER);
      chEvtSignalI(appParam.mainThread,EV_TG1_TRIGGER);
    }
  }
  chSysUnlockFromISR();
}

void tg2_int_handler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  if(appParam.mainThread)
    chEvtSignalI(appParam.mainThread,EV_TG2_TRIGGER);
  chSysUnlockFromISR();
}

void usr_int_handler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  if(appParam.mainThread){
    appParam.userPressedTime = 60; // 3-seconds
    appParam.userInPress = 0;
    chEvtSignalI(appParam.mainThread,EV_USR_TRIGGER);
  }
  chSysUnlockFromISR();
}

void gpioa9_int_handler(EXTDriver *extp, expchannel_t channel)
{
#ifdef APP_USE_RSI
  if(appParam.rsi_int)
    appParam.rsi_int();
#endif
#ifdef APP_USE_LORA
  chSysLockFromISR();
  osi_ext_intr();
  chSysUnlockFromISR();
#endif
}

void reset_buzzer(void)
{
  chVTReset(&appParam.vtBeep);
  do_map[BUZZER].clear(&do_map[BUZZER]);
}

