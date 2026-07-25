# STM32_HAL_FREEMODBUS_RTU
FreeMODBUS RTU port for STM32 HAL library

## 🚀 2nd Update (Bare-metal & Robustness Optimization)
- **FreeRTOS 완전 제거**: 순수 Bare-metal `while(1)` 폴링 방식으로 동작하도록 구조를 최적화했습니다. (불필요한 OS 오버헤드 0%)
- **안정성 강화**: TIM7 타이머 2배수 클럭 및 카운터 리셋 버그 수정, UART 에러(ORE/FE) 발생 시 무한 락 방지.
- **초경량 큐 도입**: 이벤트 링버퍼를 4-Depth 사이즈의 가장 가벼운 큐로 대체하여 인터럽트 랙을 방지했습니다.

## 📝 중요: Modbus 주소 매핑 규칙 (Off-by-one)
- 본 예제의 Input Register 콜백(`eMBRegInputCB`)에서 검사하는 `usAddress` 기준점은 1000입니다.
- FreeModbus 스택은 마스터가 요청한 PDU 주소값에 **+1**을 더해서 전달합니다. (1-based 주소 반환)
- **따라서, 마스터 장치(예: Modbus Poll)에서 1000번지를 응답받으려면 통신 패킷(PDU) 상으로는 주소 999 (0x03E7) 번지를 요청해야 합니다.**

## Reference
-  https://habrahabr.ru/post/279747/
-  http://ctrl-v.biz/blog/6
-  http://factory.ergoinventus.net/2016/05/stm32-hal-uart-isq-handler.html
