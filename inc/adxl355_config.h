#ifndef _ADXL355_CONFIG_
#define _ADXL355_CONFIG_
/**
  ADXL355 driver config file

*/
#include "adxl355_dev.h"
#include "adxl355_defs.h"
#include "peripheral_if.h"
#include "sysparam.h"

#define ADXL_BUF_SIZE   3*3*100
uint8_t adxlbuf[ADXL_BUF_SIZE];
int32_t result[3];
int8_t adxl355_conversion_done(struct ADXL355_dev *p);
int8_t adxl355_buffer_cb(struct ADXL355_dev *p);
int8_t adxl355_select(uint8_t st);
int8_t adxl355_intctrl(uint8_t st);


adxl355_config_t default_cfg = {
  {0.039,0.039,0.039},
  {0.,0.,0.},
  ADXL355_FS_2G,
  ADXL355_ODR_3_906,
  ADXL355_HPF_OFF,
  0x0,
  {0,0,0},
};

adxl355_dev_t adxl355 = {
  0x11,
  0x12,
  0x13,
  0x14,
  adxlbuf,
  result,
  false,
  ADXL355_STOP,
  &moduleParam.adxl355cfg,
  adxl355_conversion_done,
  adxl355_buffer_cb,
  adxl355_select,
  adxl355_intctrl,
  adxl355_spi_read,
  adxl355_spi_write,
  adxl355_delay
};

#endif