#ifndef _SIMPLINK_PORTING_H_
#define _SIMPLINK_PORTING_H_

#include "hal.h"
#include "chprintf.h"
#include "simplelink.h"

#define CC3100_IRQ_PORT GPIOA
#define CC3100_IRQ_PAD  0
#define CC3100_HIB_PORT GPIOA
#define CC3100_HIB_PAD 1
#define CLI_Write(...) (chprintf((BaseSequentialStream*)&SD1,__VA_ARGS__))

/**
  Sensor hub P6 connection
IRQ: PA0
nHIB: PA1
I/F :SPI1
CONSOLE: N/A
*/
    
extern uint32_t  g_Status;
extern uint32_t  g_PingPacketsRecv;
extern uint32_t  g_GatewayIP;
extern uint32_t  g_StationIP;

#define SL_STOP_TIMEOUT        0xFF
#define STATUS_BIT_PING_DONE  31
#define PING_INTERVAL   1000    /* In msecs */
#define PING_TIMEOUT    3000    /* In msecs */
#define PING_PKT_SIZE   20      /* In bytes */
#define NO_OF_ATTEMPTS  3
#define HOST_NAME       "www.ti.com"

typedef enum{
    LAN_CONNECTION_FAILED = -0x7D0,        /* Choosing this number to avoid overlap with host-driver's error codes */
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

typedef void (*P_EVENT_HANDLER)(void* pValue);
typedef int16_t* Fd_t;

Fd_t spi_Open(SPIDriver *spip, SPIConfig *cfg);
int spi_Close(Fd_t fd);
int spi_Write(Fd_t fd, unsigned char *pBuff, int len);
int spi_Read(Fd_t fd, unsigned char *pBuff, int len);

void CC3100_nHIB_init();
void CC3100_enable(void);
void CC3100_disable(void);
void CC3100_InterruptEnable(void);
void CC3100_InterruptDisable(void);
int registerInterruptHandler(P_EVENT_HANDLER InterruptHdl , void* pValue);
void Delay(unsigned long delay);

int CC3100_Disable(void);
int CC3100_Enable(void);

void CC3100_InterruptEnable();
void CC3100_InterruptDisable();

void maskIntHandler();
void unmaskIntHandler();

_u32 sl_GetTimestamp(void);

#endif