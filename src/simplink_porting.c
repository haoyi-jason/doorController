#include "ch.h"
#include "hal.h"
#include "user.h"
#include "simplelink_porting.h"
#include "chprintf.h"

SPIDriver *spiDev;

Fd_t spi_Open(SPIDriver *spip, SPIConfig *cfg)
{
  spiDev = spip;
  spiStart(spiDev,cfg);

  return 0;
}

Fd_t spi_Close(Fd_t fd)
{
  // disable interrupt
  
  spiStop(spiDev);
  
  return 0;
}

Fd_t spi_Write(Fd_t fd, unsigned char *pBuff, int len)
{
  spiSend(spiDev,len, pBuff);
}

int spi_Read(Fd_t fd, unsigned char *pBuff, int len)
{
  spiReceive(spiDev,len,pBuff);
  return 0;
}

void CLI_Write(...)
{
  chprintf((BaseSequentialStream*)&SD1,...);
}


