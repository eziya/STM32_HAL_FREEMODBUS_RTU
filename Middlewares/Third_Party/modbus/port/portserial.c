/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

#include "port.h"
 
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
 
/* ----------------------- static functions ---------------------------------*/
//static void prvvUARTTxReadyISR( void );
//static void prvvUARTRxISR( void );
 
/* -----------------------    variables     ---------------------------------*/
extern UART_HandleTypeDef huart2;
 
/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
  /* If xRXEnable enable serial receive interrupts. If xTxENable enable
  * transmitter empty interrupts.
  */
  
  if (xRxEnable) {        
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  } else {    
    __HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
  }
  
  if (xTxEnable) {    
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_TXE);
  } else {
    __HAL_UART_DISABLE_IT(&huart2, UART_IT_TXE);
  }  
}
 
BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
  /* 
   * [BUG FIX] 매개변수로 넘어온 Baudrate와 Parity를 무시하지 않고 실제 하드웨어에 적용합니다.
   * 이렇게 해야 eMBInit() 호출 시 원하는 보레이트로 정상 변경됩니다.
   */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = ulBaudRate;
  
  if (ucDataBits == 8) {
      huart2.Init.WordLength = UART_WORDLENGTH_8B;
  } else {
      huart2.Init.WordLength = UART_WORDLENGTH_9B;
  }
  
  huart2.Init.StopBits = UART_STOPBITS_1;
  
  if( eParity == MB_PAR_NONE ) {
      huart2.Init.Parity = UART_PARITY_NONE;
  } else if( eParity == MB_PAR_ODD ) {
      huart2.Init.Parity = UART_PARITY_ODD;
  } else if( eParity == MB_PAR_EVEN ) {
      huart2.Init.Parity = UART_PARITY_EVEN;
  }
  
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
      return FALSE;
  }
  
  return TRUE;
}
 
BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
  /* Put a byte in the UART transmit register without blocking.
  * This function is called from the TXE interrupt path, so HAL_UART_Transmit
  * must not be used here because it can block.
  * This path intentionally bypasses HAL TX state tracking and relies on the
  * dedicated Modbus IRQ-driven TX flow only. */
  huart2.Instance->DR = (uint8_t)ucByte;
  return TRUE;
}
 
BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
  /* Return the byte in the UARTs receive buffer. This function is called
  * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
  */
  *pucByte = (uint8_t)(huart2.Instance->DR & (uint8_t)0x00FF);  
  return TRUE;
}
 
/* Create an interrupt handler for the transmit buffer empty interrupt
* (or an equivalent) for your target processor. This function should then
* call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
* a new character can be sent. The protocol stack will then call 
* xMBPortSerialPutByte( ) to send the character.
 
static void prvvUARTTxReadyISR( void )
{
pxMBFrameCBTransmitterEmpty(  );
}
*/
 
/* Create an interrupt handler for the receive interrupt for your target
* processor. This function should then call pxMBFrameCBByteReceived( ). The
* protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
* character.
 
static void prvvUARTRxISR( void )
{
pxMBFrameCBByteReceived(  );
}
*/
