#ifndef __MODBUS_DIAG_H
#define __MODBUS_DIAG_H

#include <stdint.h>

extern volatile uint32_t g_uartOverrunErrorCount;
extern volatile uint32_t g_uartNoiseErrorCount;
extern volatile uint32_t g_uartFrameErrorCount;
extern volatile uint32_t g_uartParityErrorCount;

extern volatile uint32_t g_mbInitErrorCount;
extern volatile uint32_t g_mbEnableErrorCount;
extern volatile uint32_t g_mbPollErrorCount;
extern volatile uint32_t g_mbRegNoregErrorCount;
extern volatile uint32_t g_mbPortEventDropCount;

#endif
