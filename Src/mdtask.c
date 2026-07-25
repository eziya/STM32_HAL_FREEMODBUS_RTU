#include "stm32f4xx_hal.h"
// cmsis_os.h 제거

#include "mb.h"
#include "mbport.h"

#define REG_INPUT_START 1000
#define REG_INPUT_NREGS 8

static USHORT usRegInputStart = REG_INPUT_START;
static USHORT usRegInputBuf[REG_INPUT_NREGS];

// App_ModbusInitData: 메인 루프 진입 전 호출되는 초기화 함수
void App_ModbusInitData(void)
{ 
  /* 초기화 할 레지스터 값 세팅 */
  usRegInputBuf[0] = 11;
  usRegInputBuf[1] = 22;
  usRegInputBuf[2] = 33;
  usRegInputBuf[3] = 44;
  usRegInputBuf[4] = 55;
  usRegInputBuf[5] = 66;
  usRegInputBuf[6] = 77;
  usRegInputBuf[7] = 88;  
  
  // (참고) eMBInit(), eMBEnable(), eMBPoll()은 main.c의 Bare-metal 루프에서 호출됩니다.
}

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    /*
     * [IMPORTANT] Modbus 주소 매핑 (Off-by-One 주의)
     * FreeModbus 스택은 마스터가 전송한 PDU 주소에 +1을 한 1-based 주소를 usAddress 인자로 넘깁니다.
     * 즉, 마스터가 PDU 주소 999 (0x03E7)를 요청하면 usAddress는 1000이 됩니다.
     * 이 코드는 usAddress(1000 ~ 1007)를 체크하므로, 마스터 측에서는 999번지를 요청해야 합니다.
     */
    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
				
				HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
    }
    else
    {
			  HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
        eStatus = MB_ENOREG;			
    }

    return eStatus;
}

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                 eMBRegisterMode eMode )
{
    return MB_ENOREG;
}


eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
    return MB_ENOREG;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}
