#ifndef _MODBUS_BINDING_
#define _MODBUS_BINDING_

#define MBS_SERIAL_PORT	            ( MB_UART_1 )
#define MBS_SERIAL_BAUDRATE         ( 19200 )
#define MBS_PARITY                  ( MB_PAR_NONE )
#define MBS_SLAVE_ADDRESS           ( 1 )


void modbusBindingInit();
void modbusBindingStop();
#endif