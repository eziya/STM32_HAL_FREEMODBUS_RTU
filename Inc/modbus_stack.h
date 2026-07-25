#ifndef __MODBUS_STACK_H
#define __MODBUS_STACK_H

#include <stdint.h>
#include "main.h"

uint8_t ModbusRTUStackInit(void);
void ModbusRTUStackPoll(void);

#if APP_USE_FREERTOS
void ModbusRTUTask(void const * argument);
#endif

#endif
