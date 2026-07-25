#include "stm32f4xx_hal.h"
#include "main.h"
#if APP_USE_FREERTOS
#include "cmsis_os.h"
#endif

#include "mb.h"
#include "mbport.h"

#define REG_INPUT_START 1000
#define REG_INPUT_NREGS 8

static USHORT usRegInputStart = REG_INPUT_START;
static USHORT usRegInputBuf[REG_INPUT_NREGS];

volatile uint32_t g_mbInitErrorCount = 0;
volatile uint32_t g_mbEnableErrorCount = 0;
volatile uint32_t g_mbPollErrorCount = 0;
volatile uint32_t g_mbRegNoregErrorCount = 0;

uint8_t ModbusRTUStackInit(void)
{
  eMBErrorCode eStatus;

  usRegInputBuf[0] = 11;
  usRegInputBuf[1] = 22;
  usRegInputBuf[2] = 33;
  usRegInputBuf[3] = 44;
  usRegInputBuf[4] = 55;
  usRegInputBuf[5] = 66;
  usRegInputBuf[6] = 77;
  usRegInputBuf[7] = 88;

  eStatus = eMBInit( MB_RTU, 1, 3, 19200, MB_PAR_NONE );
  if( eStatus != MB_ENOERR )
  {
    g_mbInitErrorCount++;
    return 0;
  }

  eStatus = eMBEnable();
  if( eStatus != MB_ENOERR )
  {
    g_mbEnableErrorCount++;
    return 0;
  }

  return 1;
}

void ModbusRTUStackPoll(void)
{
  eMBErrorCode eStatus = eMBPoll();
  if( eStatus != MB_ENOERR )
  {
    g_mbPollErrorCount++;
  }
}

#if APP_USE_FREERTOS
void ModbusRTUTask(void const * argument)
{ 
  (void)argument;
  if(!ModbusRTUStackInit())
  {
    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
    for(;;)
    {
      osDelay(1000);
    }
  }
  
  while(1) {
    ModbusRTUStackPoll();
  }
}
#endif

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

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
        g_mbRegNoregErrorCount++;
        eStatus = MB_ENOREG;			
    }

    return eStatus;
}

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                 eMBRegisterMode eMode )
{
    (void)pucRegBuffer;
    (void)usAddress;
    (void)usNRegs;
    (void)eMode;
    g_mbRegNoregErrorCount++;
    return MB_ENOREG;
}


eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
    (void)pucRegBuffer;
    (void)usAddress;
    (void)usNCoils;
    (void)eMode;
    g_mbRegNoregErrorCount++;
    return MB_ENOREG;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    (void)pucRegBuffer;
    (void)usAddress;
    (void)usNDiscrete;
    g_mbRegNoregErrorCount++;
    return MB_ENOREG;
}
