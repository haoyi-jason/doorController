#include "ch.h"
#include "hal.h"
#include "ad7124.h"
#include "ad7124_defs.h"
#include "peripheral_if.h"
#include "sysParam.h"


int32_t adResult[16];
int8_t ad7124_conversion_done(void *p);
int8_t ad7124_select(uint8_t st);
int8_t ad7124_intctrl(uint8_t st);

static ad7124_channel_t channels[] = {
  {.u={5,4,0,0,ADC_BIT_ENABLE}},
  {.u={1,0,0,0,ADC_BIT_DISABLE}},
  {.u={11,10,0,0,ADC_BIT_DISABLE}},
  {.u={15,14,0,0,ADC_BIT_DISABLE}},
  {.u={9,8,0,0,ADC_BIT_DISABLE}},
  {.u={11,10,0,0,ADC_BIT_DISABLE}},
  {.u={13,12,0,0,ADC_BIT_DISABLE}},
  {.u={15,14,0,0,ADC_BIT_DISABLE}},
  {.u={15,14,0,0,ADC_BIT_DISABLE}},
  {.u={15,14,0,0,ADC_BIT_DISABLE}},
  {.u={15,14,0,0,ADC_BIT_DISABLE}},
  {.u={15,14,0,0,ADC_BIT_DISABLE}},
  {.u={15,14,0,0,ADC_BIT_DISABLE}},
  {.u={15,14,0,0,ADC_BIT_DISABLE}},
  {.u={15,14,0,0,ADC_BIT_DISABLE}},
  {.u={15,14,0,0,ADC_BIT_DISABLE}},
};


static ad7124_setup_t setups[] = {
  {.u={ADC_PGA_X128,ADC_REF_REFINT,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_ENABLE}},
  {.u={ADC_PGA_X8,ADC_REF_REFINT,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_DISABLE}},
  {.u={ADC_PGA_X1,ADC_REF_REFINT,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_DISABLE}},
  {.u={ADC_PGA_X1,ADC_REF_REFINT,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_DISABLE}},
  {.u={ADC_PGA_X1,ADC_REF_REFINT,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_DISABLE}},
  {.u={ADC_PGA_X1,ADC_REF_REFINT,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_DISABLE}},
  {.u={ADC_PGA_X1,ADC_REF_REFINT,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_DISABLE}},
  {.u={ADC_PGA_X1,ADC_REF_REFINT,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_DISABLE}},
  {.u={ADC_PGA_X1,ADC_REF_REFINT,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_DISABLE}},
};

static ad7124_filter_t filters[] = {
  {.u={19,ADC_BIT_DISABLE,0,ADC_BIT_DISABLE,ADC_FILER_SINC4}},
  {.u={24,ADC_BIT_DISABLE,0,ADC_BIT_DISABLE,ADC_FILER_SINC4}},
  {.u={24,ADC_BIT_DISABLE,0,ADC_BIT_DISABLE,ADC_FILER_SINC4}},
  {.u={24,ADC_BIT_DISABLE,0,ADC_BIT_DISABLE,ADC_FILER_SINC4}},
  {.u={24,ADC_BIT_DISABLE,0,ADC_BIT_DISABLE,ADC_FILER_SINC4}},
  {.u={24,ADC_BIT_DISABLE,0,ADC_BIT_DISABLE,ADC_FILER_SINC4}},
  {.u={24,ADC_BIT_DISABLE,0,ADC_BIT_DISABLE,ADC_FILER_SINC4}},
  {.u={24,ADC_BIT_DISABLE,0,ADC_BIT_DISABLE,ADC_FILER_SINC4}},
};  
current_drive_t ad7124_iDrv[] = {
  {7,ADC_IOUT_500UA},
  {0,ADC_IOUT_OFF},
};


ad7124_dev_t ad7124 = {
  .chip_id = 0x12,
  .results = adResult,
  .sample = {0,8,2,2,2,2},
  .goStop = false,
  .state = AD7124_STOP,
  .currentChannel = 0,
  .control = {
    .u={
      .ref_en = ADC_BIT_ENABLE,
      .csb_en = ADC_BIT_DISABLE,
      .data_status = ADC_BIT_ENABLE,
      .cont_read = ADC_BIT_DISABLE,
      .drdy_del = ADC_BIT_ENABLE,
      .clk_sel = ADC_CLKSEL_CLK_DIS,
      .mode = AD7124_MODE_CONTINUE,
      .power_mode = AD7124_FULL_POWER
    }
  },
  ad7124_conversion_done,
  &moduleParam.ad7124cfg,
  ad7124_select,
  ad7124_intctrl,
  ad7124_spi_read,
  ad7124_spi_write,
  ad7124_delay,
};

adc_channel_t default_channel[]={
  {1,0,1,0,0.0,1.0,-1,0,-1,0,0},
  {0,0,1,0,0.0,1.0,-1,0,-1,0,0},
  {0,0,1,0,0.0,1.0,-1,0,-1,0,0},
  {0,0,1,0,0.0,1.0,-1,0,-1,0,0},
  {0,0,1,0,0.0,1.0,-1,0,-1,0,0},
  {0,0,1,0,0.0,1.0,-1,0,-1,0,0},
  {0,0,1,0,0.0,1.0,-1,0,-1,0,0},
  {0,0,1,0,0.0,1.0,-1,0,-1,0,0},
};

adc_channel_config_t default_config[]={
  {7,0,0,1000,0},
  {0,0,0,10,0},
  {0,0,0,10,0},
  {0,0,0,10,0},
  {0,0,0,10,0},
  {0,0,0,10,0},
  {0,0,0,10,0},
  {0,0,0,10,0},
};

