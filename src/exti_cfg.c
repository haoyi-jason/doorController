
#include "ch.h"
#include "hal.h"
#include "sensorHub.h"
#include "exti_cfg.h"

int_state_t intState;
void user_button_handler(EXTDriver *extp, expchannel_t channel)
{
  
}
void sdmmc_int_handler(EXTDriver *extp, expchannel_t channel)
{
  
}
void p7_3_intHandler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  intState.intPort = 7;
  intState.intBit = 3;
  chEvtSignalI(mainThread, EV_CMD_EXTINT);
  chSysUnlockFromISR();
}
void p7_4_intHandler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  intState.intPort = 7;
  intState.intBit = 4;
  chEvtSignalI(mainThread, EV_CMD_EXTINT);
  chSysUnlockFromISR();
  
}
void p7_5_intHandler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  intState.intPort = 7;
  intState.intBit = 5;
  chEvtSignalI(mainThread, EV_CMD_EXTINT);
  chSysUnlockFromISR();
  
}
void p7_13_intHandler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  intState.intPort = 7;
  intState.intBit = 13;
  chEvtSignalI(mainThread, EV_CMD_EXTINT);
  chSysUnlockFromISR();
  
}

void p8_3_intHandler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  intState.intPort = 8;
  intState.intBit = 3;
  chEvtSignalI(mainThread, EV_CMD_EXTINT);
  chSysUnlockFromISR();
  
}
void p8_4_intHandler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  intState.intPort = 8;
  intState.intBit = 4;
  chEvtSignalI(mainThread, EV_CMD_EXTINT);
  chSysUnlockFromISR();
  
}
void p8_5_intHandler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  intState.intPort = 8;
  intState.intBit = 5;
  chEvtSignalI(mainThread, EV_CMD_EXTINT);
  chSysUnlockFromISR();
  
}
void p8_13_intHandler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  intState.intPort = 8;
  intState.intBit = 13;
  chEvtSignalI(mainThread, EV_CMD_EXTINT);
  chSysUnlockFromISR();
  
}

void exti_config(void)
{
  switch(moduleParam.hub.conn_p7){
    
  }
  
  switch(moduleParam.hub.conn_p8){
    
  }
  
}