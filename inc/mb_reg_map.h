#ifndef _MB_REG_MAP_
#define _MB_REG_MAP_

typedef int8_t (*reg_map_func)(uint16_t, uint8_t*);

typedef struct{
  uint16_t regBegin;
  uint16_t regEnd;
  reg_map_func read;
  reg_map_func write;
}mb_reg_map_t;
  
extern mb_reg_map_t mb_reg_map_func[];
//#define NOF_REG_FUNC    sizeof(mb_reg_map_func)/sizeof(mb_reg_map_t)
#define NOF_REG_FUNC    2
void mapMBWord(uint8_t *dptr, uint8_t *val);
void mapMBFloat(uint8_t *dptr, uint8_t *val);

int8_t read40000(uint16_t offset, uint8_t *dptr);


#endif