#ifndef _PERIPHERAL_IF_H
#define _PERIPHERAL_IF_H

#include "hal.h"

enum dio_names{
  AD7124_CHIP_SEL,
  RELAY_0,
  RELAY_1
};

typedef enum {
  DIO_DIR_INPUT,
  DIO_DIR_OUTPUT,
  DIO_DIR_BYDIREC
}dio_dir_t;

typedef enum {
  DIO_TYPE_GPIO,
  DIO_TYPE_FREQ,    // output: frequency, input: counter
  DIO_TYPE_PWM,     // output: pwm, input: capture
}dio_type_t;

typedef struct{
  dio_dir_t direction;
  dio_type_t type;
  uint32_t val;
  uint32_t fraction;
  uint32_t base;
  ioportid_t port;
  uint16_t pad;
}digital_io_type_t;

extern SPIDriver *bmispi;
extern SPIDriver *adspi;
extern SPIDriver *adxlspi;

int8_t spi_read(SPIDriver *spip,uint8_t reg_adr, uint8_t *b, uint16_t n);
int8_t spi_write(SPIDriver *spip,uint8_t reg_adr, uint8_t *b, uint16_t n);

int8_t ad7124_spi_read(uint8_t reg_adr, uint8_t *b, uint16_t n);
int8_t ad7124_spi_write(uint8_t reg_adr, uint8_t *b, uint16_t n);
int8_t ad7124_select(uint8_t st);
void ad7124_delay(uint32_t period);

int8_t adxl355_spi_read(uint8_t reg_adr, uint8_t *b, uint16_t n);
int8_t adxl355_spi_write(uint8_t reg_adr, uint8_t *b, uint16_t n);
int8_t adxl355_select(uint8_t st);
int8_t adxl355_intctrl(uint8_t st);
void adxl355_delay(uint32_t period);

void bmi160_delay(uint32_t period);
int8_t bmi160_write(uint8_t dev_adr, uint8_t reg_adr, uint8_t *b, uint16_t n);
int8_t bmi160_read(uint8_t dev_adr, uint8_t reg_adr, uint8_t *b, uint16_t n);

int8_t spi2_cs_0(uint8_t st);
int8_t spi3_cs_0(uint8_t st);

#endif