#ifndef _MODBUSREGMAP_
#define _MODBUSREGMAP_

#define RANGE_VALID(x,y,z) ((x<=y) && (y<=z))


typedef int8_t (*mbcallback)(uint16_t,uint8_t*);

typedef struct mb_map_callback_s{
  uint16_t regBegin;
  uint16_t regEnd;
  mbcallback readFcn;
  mbcallback writeFcn;
}mb_map_callback_t;


#endif