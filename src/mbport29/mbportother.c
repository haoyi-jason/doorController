/* 
 * MODBUS Library: STM32 CubeMX, FreeRTOS port
 * Copyright (c) 2016 Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * $Id: mbportother.c 1068 2016-08-26 16:05:09Z cwalter $
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ch.h>
#include <hal.h>
//#include <semphr.h>

/* ----------------------- Platform includes --------------------------------*/

#include "mbport.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "common/mbtypes.h"
#include "common/mbportlayer.h"
#include "common/mbframe.h"
#include "common/mbutils.h"

/* ----------------------- Defines ------------------------------------------*/
#define portMAX_DELAY   100
/* ----------------------- Type definitions ---------------------------------*/

/* ----------------------- Static variables ---------------------------------*/
//STATIC xSemaphoreHandle xCritSection;
static semaphore_t xCritSection;

/* ----------------------- Static functions ---------------------------------*/

/* ----------------------- Function prototypes ------------------------------*/

/* ----------------------- Start implementation -----------------------------*/

void
vMBPInit( void )
{
//    xCritSection = xSemaphoreCreateRecursiveMutex(  );
    chSemObjectInit(&xCritSection,4);
    MBP_ASSERT( NULL != &xCritSection );
}

void
vMBPEnterCritical( void )
{
//    signed portBASE_TYPE xResult;
//    xResult = xSemaphoreTakeRecursive( xCritSection, portMAX_DELAY );
    msg_t msg = chSemWaitTimeout(&xCritSection,portMAX_DELAY);
    MBP_ASSERT( MSG_OK == msg );
}

void
vMBPExitCritical( void )
{
//    signed portBASE_TYPE xResult;
//    xResult = xSemaphoreGiveRecursive( xCritSection );
//    MBP_ASSERT( pdTRUE == xResult );
    chSemSignal(&xCritSection);
    //MBP_ASSERT( MSG_OK == msg );
}

void
vMBPAssert( const char *pszFile, int iLineNo )
{
    __disable_irq(  );
    /* Let the watchdog trigger a reset here. */
    for( ;; );
}
