#include "ch.h"
#include "hal.h"
#include "user.h"
#include "simplelink_porting.h"
#include "chprintf.h"
#include "exti_cfg.h"
#include "osi_chibios.h"

SPIDriver *spiDev = &SPID1;
P_EVENT_HANDLER     pIrqEventHandler = 0;
static bool intMasked = false;
//void cc3100_int_handler(EXTDriver *extp, expchannel_t channel);

void spi_select()
{
  palClearPad(GPIOA,GPIOA_SPI1_CS);
}

void spi_deselect()
{
  palSetPad(GPIOA,GPIOA_SPI1_CS);
}

static SPIConfig spicfg = {
  NULL,
  GPIOA,
  GPIOA_SPI1_CS,
  SPI_CR1_BR_0 |SPI_CR1_BR_2 | SPI_CR1_BR_1
};
static SPIConfig spicfg2 = {
  NULL,
  NULL,
  NULL,
  SPI_CR1_BR_2 | SPI_CR1_BR_1
};

Fd_t spi_Open(SPIDriver *spip, SPIConfig *cfg)
{
    spiStart(spiDev,&spicfg);
//    CC3100_Disable();
    chThdSleepMilliseconds(10);
    //CC3100_InterruptEnable();
    return OSI_OK;
}

int spi_Close(Fd_t fd)
{
  if(spiDev){
    spiStop(spiDev);
    //spiDev = 0;
    //CC3100_InterruptDisable();
    return OSI_OK;
  }
  
  return -1;
}

int spi_Write(Fd_t fd, unsigned char *pBuff, int len)
{
  int ret = len;
  chThdSleepMilliseconds(2);
  if(spiDev){
    spiAcquireBus(spiDev);
    spiSelect(spiDev);
    chThdSleepMicroseconds(500);
    spiSend(spiDev,len, pBuff);
    chThdSleepMicroseconds(500);
    spiUnselect(spiDev);
    spiReleaseBus(spiDev);
    //chThdSleepMilliseconds(5);
    return ret;
  }
    //chThdSleepMilliseconds(5);
  return -1;
}

int spi_Read(Fd_t fd, unsigned char *pBuff, int len)
{
  int ret = len;
  if(len == 0) return 0;
  chThdSleepMilliseconds(2);
  if(spiDev){
    spiAcquireBus(spiDev);
    spiSelect(spiDev);
    chThdSleepMicroseconds(500);
    spiReceive(spiDev,len,pBuff);
    chThdSleepMicroseconds(500);
    spiUnselect(spiDev);
    spiReleaseBus(spiDev);
    //chThdSleepMilliseconds(5);
    return ret;
  }
    //chThdSleepMilliseconds(5);
  return -1;
}


int CC3100_Disable(void)
{
  palClearPad(CC3100_HIB_PORT,CC3100_HIB_PAD);
  CC3100_InterruptDisable();
  return 0;
}

int CC3100_Enable(void)
{
  
  //chThdSleepMilliseconds(2000);
  palSetPad(CC3100_HIB_PORT,CC3100_HIB_PAD); 
  CC3100_InterruptEnable();
  return 0;
}

int registerInterruptHandler(P_EVENT_HANDLER InterruptHdl , void* pValue)
{
  pIrqEventHandler = InterruptHdl;
  return 0;
}

void CC3100_InterruptEnable()
{
  extChannelEnable(&EXTD1,CC3100_IRQ_PAD);
}

void CC3100_InterruptDisable()
{
  extChannelDisable(&EXTD1,CC3100_IRQ_PAD);
}

void simplink_int_handler(EXTDriver *extp, expchannel_t channel)
{
  chSysLockFromISR();
  if(NULL != pIrqEventHandler){
    pIrqEventHandler(0);
  }
  chSysUnlockFromISR();
}

void maskIntHandler()
{
  intMasked = true;
}

void unmaskIntHandler()
{
  intMasked = false;
}

_u32 port_getTimeStamp(void)
{
  return (_u32)chVTGetSystemTimeX();
}
