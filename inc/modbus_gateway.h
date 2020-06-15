#ifndef _MODBUS_GATEWAY_
#define _MODBUS_GATEWAY_

#include "ch.h"
#include "hal.h"

#define EV_MBGW_LDISP_CMDRCVD    EVENT_MASK(1)
#define EV_MBGW_PLAYWAVE        EVENT_MASK(2)

extern thread_t *thdGateway;

void modbusGateway_init(void);

#endif