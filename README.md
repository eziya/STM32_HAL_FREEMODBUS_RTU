# STM32_HAL_FREEMODBUS_RTU
FreeMODBUS RTU port for STM32 HAL library

http://blog.naver.com/eziya76/220970378890

## Recent Updates (Bare-metal Optimization & Bug Fixes)
- **Bare-metal Architecture**: Completely removed FreeRTOS dependencies and threads. The application now runs on a highly efficient `while(1)` polling loop.
- **Timing Accuracy**: Fixed a critical TIM7 prescaler calculation bug (APB1 clock x2) and added a counter reset upon start to ensure precise T3.5 timeout detection.
- **Robust UART Error Handling**: Prevented infinite interrupt lockups caused by UART Overrun (ORE) or Framing Errors by directly clearing the SR/DR registers.
- **Lightweight Event Queue**: Replaced the previous ring buffer with an ultra-lightweight 4-depth fixed queue to minimize interrupt disable time and prevent event loss.
- **Hardware Integration**: Dynamically applied the `ulBaudRate` and `eParity` parameters to the UART HAL initialization.
- **C++ Compatibility**: Restored missing `extern "C"` macros in the port headers.

> **Note on Modbus Addressing (Off-by-one):**
> The FreeModbus stack passes a 1-based address to callbacks. Since the example validates `usAddress` starting from 1000, the master device must request PDU address `999 (0x03E7)` to read this register.

## Reference
-  https://habrahabr.ru/post/279747/
-  http://ctrl-v.biz/blog/6
-  http://factory.ergoinventus.net/2016/05/stm32-hal-uart-isq-handler.html
