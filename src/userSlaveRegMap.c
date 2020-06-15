#include "ch.h"
#include "hal.h"
#include "modbusSlaveRegMap.h"
#include "modbusregmap.h"

#define NOF_MBCALLBACK 8
mb_map_callback_t mbcallbacks[] =
{
  {0,10,mbread_0_10,mbwrite_0_10},
  {101,109,NULL,doorCtrl},
  {201,209,NULL,doorCtrl},
  {301,309,NULL,doorCtrl},
  {401,409,NULL,doorCtrl},
  {501,509,NULL,doorCtrl},
  {601,609,NULL,doorCtrl},
  {701,709,NULL,doorCtrl},
};



int8_t mbread(uint16_t usAddr, uint8_t *dptr){
  uint8_t i;
  uint8_t szRet = 0;
  for(i=0;i<NOF_MBCALLBACK;i++){
    if(RANGE_VALID(mbcallbacks[i].regBegin,usAddr,mbcallbacks[i].regEnd)){
      if(mbcallbacks[i].readFcn)
        szRet = mbcallbacks[i].readFcn(usAddr,dptr);
    }
  }
  return szRet;
}

int8_t mbwrite(uint16_t usAddr, uint8_t *dptr)
{
  uint8_t i;
  uint8_t szRet = 0;
  for(i=0;i<NOF_MBCALLBACK;i++){
    if(RANGE_VALID(mbcallbacks[i].regBegin,usAddr,mbcallbacks[i].regEnd)){
      if(mbcallbacks[i].writeFcn)
        szRet = mbcallbacks[i].writeFcn(usAddr,dptr);
    }
  }
  return szRet;
}

int8_t doorCtrl(uint16_t usAddr,uint8_t *dptr)
{
  int8_t szRet = 0;
  
  return szRet;
}
