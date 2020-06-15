#include "hal.h"

#define BEEP_ON()       palSetPad(GPIOA,8)
#define BEEP_OFF()      palClearPad(GPIOA,8)





typedef struct dio_map_s;




typedef struct{
  uint32_t runSecs;
  dio_map_t *doMap;
  dio_map_t *diMap;
}app_param_t;