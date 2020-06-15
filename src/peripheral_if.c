/**
file: peripheral_if.c
description: peripheral interface

***/

#include "ch.h"
#include "hal.h"
#include "sensorhub.h"
#include "peripheral_if.h"
#include "bmi160.h"

#define AD7124_REG_READ(x,y,z) spi_read(&SPID1,x,y,z)
#define AD7124_REG_WRITE(x,y,z) spi_write(&SPID1,x,y,z)
#define AD7124_SPI_SEL(x) set_dig_out(AD7124_CHIP_SEL,x)

#define P7_SPI_SELECT() (palClearPad(GPIOB,12))
#define P7_SPI_UNSELECT() (palSetPad(GPIOB,12))
#define P8_SPI_SELECT() (palClearPad(GPIOB,2))
#define P8_SPI_UNSELECT() (palSetPad(GPIOB,2))

SPIDriver *adspi;
SPIDriver *adxlspi;
SPIDriver *bmispi;

static digital_io_type_t dio[] = {
  {DIO_DIR_OUTPUT,DIO_TYPE_GPIO,1,0,0,GPIOA,3,},
  
};

int8_t spi_read(SPIDriver *spip,uint8_t reg_adr, uint8_t *b, uint16_t n)
{
  spiAcquireBus(spip);
  spiSend(spip,1,&reg_adr);
  spiReceive(spip,n,b);
  spiReleaseBus(spip);
  return 0;
}

int8_t spi_write(SPIDriver *spip,uint8_t reg_adr, uint8_t *b, uint16_t n)
{
  spiSend(spip,1,&reg_adr);
  spiSend(spip,n,b);
  return 0;
}


int8_t ad7124_spi_read(uint8_t reg_adr, uint8_t *b, uint16_t n)
{
    return spi_read(adspi,reg_adr,b,n);
}

int8_t ad7124_spi_write(uint8_t reg_adr, uint8_t *b, uint16_t n)
{
    return spi_write(adspi,reg_adr,b,n);
}

int8_t adxl355_spi_read(uint8_t reg_adr, uint8_t *b, uint16_t n)
{
    return spi_read(adxlspi,reg_adr,b,n);
}

int8_t adxl355_spi_write(uint8_t reg_adr, uint8_t *b, uint16_t n)
{
    return spi_write(adxlspi,reg_adr,b,n);
}


int8_t set_dig_out(uint8_t ch, uint32_t v)
{
  dio[ch].val = v;
  if(v){
    palSetPad(dio[ch].port,dio[ch].pad);
  }
  else{
    palClearPad(dio[ch].port,dio[ch].pad);
  }
  return 0;
}

int8_t bmi160_read(uint8_t dev_adr, uint8_t reg_adr, uint8_t *b, uint16_t n)
{
  spiAcquireBus(bmispi);
  spiSelect(bmispi);
  spiSend(bmispi,1,&reg_adr);
  spiReceive(bmispi,n,b);
  spiUnselect(bmispi);  
  spiReleaseBus(bmispi);
  return BMI160_OK;
}

int8_t bmi160_write(uint8_t dev_adr, uint8_t reg_adr, uint8_t *b, uint16_t n)
{
  spiAcquireBus(bmispi);
  spiSelect(bmispi);
  spiSend(bmispi,1,&reg_adr);
  spiSend(bmispi,n,b);
  spiUnselect(bmispi);
  spiReleaseBus(bmispi);
  return BMI160_OK;
}

void bmi160_delay(uint32_t period)
{
    chThdSleepMilliseconds(period);
//  if(appParam.sensorSta == SEN_RUNNING){
//    chThdSleepMicroseconds(period);
//  }else{
//    chThdSleepMilliseconds(period);
//  }
}

void ad7124_delay(uint32_t period)
{
    chThdSleepMilliseconds(period);
}

int8_t ad7124_select(uint8_t st)
{
  int8_t ret = AD7124_OK;
  if(st)
    palSetPad(GPIOB,12);
  else
    palClearPad(GPIOB,12);
  return ret;
}

int8_t ad7124_intctrl(uint8_t st)
{
  int8_t ret = AD7124_OK;
  
  if(st)
    extChannelEnable(&EXTD1,14);
  else
    extChannelDisable(&EXTD1,14);
  
  return ret;
}

int8_t adxl355_select(uint8_t st)
{
  int8_t ret = AD7124_OK;
  if(st)
    palSetPad(GPIOB,2);
  else
    palClearPad(GPIOB,2);
  return ret;
}

int8_t adxl355_intctrl(uint8_t st)
{
  int8_t ret = ADXL355_OK;
  
  if(st)
    extChannelEnable(&EXTD1,3);
  else
    extChannelDisable(&EXTD1,3);
  
  return ret;
}

void adxl355_delay(uint32_t period)
{
    chThdSleepMilliseconds(period);
}

int8_t spi2_cs_0(uint8_t st)
{
  int8_t ret = 0;
  if(st)
    palSetPad(GPIOB,12);
  else
    palClearPad(GPIOB,12);
  return ret;
}

int8_t spi3_cs_0(uint8_t st)
{
  int8_t ret = 0;
  if(st)
    palSetPad(GPIOA,15);
  else
    palClearPad(GPIOA,15);
  return ret;
}
