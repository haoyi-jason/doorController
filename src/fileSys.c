#include "ch.h"
#include "hal.h"
#include "string.h"
#include "ff.h"

FATFS SDC_FS;
FRESULT fres;
FIL sdFile;
DIR dir;
char fileName[32];

uint8_t fileSYsInit()
{
  FRESULT err;

  sdcStart(&SDCD1,NULL);
  sdcConnect(&SDCD1);
  err = f_mount(&SDC_FS,"0",1);
  if(err != FR_OK){
    sdcDisconnect(&SDCD1);
    return 0;
  }
  
  return 1;
}