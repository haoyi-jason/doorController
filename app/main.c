#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "evtimer.h"
#include "chprintf.h"
#include "stdlib.h"
#include "sensorhub.h"



/*
 * Application entry point.
 */
int main(void) {
   // initial chibios 
  halInit();
  chSysInit();
  //modbusGateway_init();
  //autoDrop_sysInit();
  sensorHubInit();
  while(1){
    chThdSleepMilliseconds(10);
  }
   
}
