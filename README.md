# STM32_HAL_FREEMODBUS_RTU
FreeMODBUS RTU port for STM32 HAL library

http://blog.naver.com/eziya76/220970378890

## Runtime configuration (current example)
- MCU clock: SYSCLK 168 MHz (APB1 = HCLK/4)
- UART: USART2, 19200 baud, 8N1
- Modbus mode: RTU Slave
- Slave ID: `1`
- Port number in `eMBInit`: `3` (logical port ID used by this demo)
- Register map:
  - Input Register start: `1000`
  - Count: `8`
  - Values: `11,22,33,44,55,66,77,88`

## Enabled Modbus functions
- `0x04 Read Input Registers` only
- Other function codes are disabled in `Middlewares/Third_Party/modbus/include/mbconfig.h`

## RTOS / Bare-metal build mode
- Build-time switch: `APP_USE_FREERTOS` in `/home/runner/work/STM32_HAL_FREEMODBUS_RTU/STM32_HAL_FREEMODBUS_RTU/Inc/main.h`
  - `0`: bare-metal loop (`ModbusRTUStackInit` + `ModbusRTUStackPoll`)
  - `1`: FreeRTOS task mode (`ModbusRTUTask`)

## Diagnostics counters
The demo exposes simple counters for troubleshooting:
- UART error counters (ORE/NE/FE/PE): `/home/runner/work/STM32_HAL_FREEMODBUS_RTU/STM32_HAL_FREEMODBUS_RTU/Src/stm32f4xx_it.c`
- Event queue drop counter: `/home/runner/work/STM32_HAL_FREEMODBUS_RTU/STM32_HAL_FREEMODBUS_RTU/Middlewares/Third_Party/modbus/port/portevent.c`
- Modbus init/poll/unsupported-register counters: `/home/runner/work/STM32_HAL_FREEMODBUS_RTU/STM32_HAL_FREEMODBUS_RTU/Src/mdtask.c`

## Quick test (Modbus Poll)
1. Connection: Serial, COMx, 19200, 8 data bits, no parity, 1 stop bit.
2. Slave ID: `1`
3. Function: `04 Read Input Registers`
4. Start address: `1000`, Quantity: `8`
5. Expected response values: `11,22,33,44,55,66,77,88`

Reference
-  https://habrahabr.ru/post/279747/
-  http://ctrl-v.biz/blog/6
-  http://factory.ergoinventus.net/2016/05/stm32-hal-uart-isq-handler.html
