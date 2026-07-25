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
  * Policy:
  * - RX path enables RXNE + ERR interrupts to detect and recover from ORE/FE/NE/PE.
  * - TX path enables TXE interrupt only while actively transmitting.
  */
  
  if (xRxEnable) {        
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_PE);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  } else {    
    __HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(&huart2, UART_IT_PE);
    __HAL_UART_DISABLE_IT(&huart2, UART_IT_ERR);
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
  Do nothing, initialization is handled by MX_USART2_UART_Init().
  This STM32F4 demo intentionally uses fixed hardware settings:
  USART2, 19200 baud, 8 data bits, no parity.
  Parameters are accepted for FreeModbus API compatibility only.
  */
  (void)ucPORT;
  (void)ulBaudRate;
  (void)ucDataBits;
  (void)eParity;
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
