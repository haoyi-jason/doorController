#ifndef _EXTI_CFG_
#define _EXTI_CFG_
#include "hal.h"

void ang1_int_handler(EXTDriver *extp, expchannel_t channel);
void ang2_int_handler(EXTDriver *extp, expchannel_t channel);
void tg1_int_handler(EXTDriver *extp, expchannel_t channel);
void tg2_int_handler(EXTDriver *extp, expchannel_t channel);
void usr_int_handler(EXTDriver *extp, expchannel_t channel);
void gpioa9_int_handler(EXTDriver *extp, expchannel_t channel);


static const EXTConfig extcfg = {
{
    {EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOB, ang1_int_handler}, 
    {EXT_CH_MODE_DISABLED, NULL}, 
    {EXT_CH_MODE_DISABLED, NULL}, 
    {EXT_CH_MODE_DISABLED, NULL}, 
    {EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOB, ang2_int_handler}, 
    {EXT_CH_MODE_FALLING_EDGE | EXT_MODE_GPIOC, tg1_int_handler}, 
    {EXT_CH_MODE_FALLING_EDGE | EXT_MODE_GPIOC, tg2_int_handler}, 
    {EXT_CH_MODE_DISABLED, NULL}, 
    {EXT_CH_MODE_DISABLED, NULL}, 
    {EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOA, gpioa9_int_handler}, 
    {EXT_CH_MODE_DISABLED, NULL}, 
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOC, usr_int_handler}, 
    {EXT_CH_MODE_DISABLED, NULL}, 
    {EXT_CH_MODE_DISABLED, NULL},
  }
};


//static const EXTConfig extcfg = {
//  {
//    {EXT_CH_MODE_DISABLED, NULL}, 
//    {EXT_CH_MODE_DISABLED, NULL}, 
//    {EXT_CH_MODE_DISABLED, NULL}, 
//    {EXT_CH_MODE_FALLING_EDGE | EXT_MODE_GPIOC, adxl_int_handler},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_FALLING_EDGE | EXT_MODE_GPIOC, adxl_int_handler},
//    {EXT_CH_MODE_FALLING_EDGE | EXT_MODE_GPIOC, bmi160_int1_handler},
//    {EXT_CH_MODE_FALLING_EDGE | EXT_MODE_GPIOC, bmi160_int2_handler},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//    {EXT_CH_MODE_DISABLED, NULL},
//  }
//};


#endif