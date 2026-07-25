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
 * File: $Id: porttimer.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
 
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
 
/* ----------------------- static functions ---------------------------------*/
//static void prvvTIMERExpiredISR( void );
 
/* -----------------------    variables     ---------------------------------*/
extern TIM_HandleTypeDef htim7;
uint16_t timeout = 0;
volatile uint16_t downcounter = 0;
 
/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
  TIM_MasterConfigTypeDef sMasterConfig;
  
  htim7.Instance = TIM7;
  /* 
   * [BUG FIX] STM32F4 클럭 트리에서 APB1 분주비가 1이 아닐 경우(DIV4), 
   * 타이머 클럭은 PCLK1의 2배(x2)로 동작합니다.
   * 따라서 정확히 1MHz 카운터(1us 단위)를 만들기 위해 *2 를 곱해주어야 합니다.
   */
  htim7.Init.Prescaler = ((HAL_RCC_GetPCLK1Freq() * 2) / 1000000) - 1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 50 - 1; // 50us 마다 인터럽트 발생
  
  timeout = usTim1Timerout50us;
  
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    return FALSE;
  }
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    return FALSE;
  }
  
  return TRUE;
}
 
 
void
vMBPortTimersEnable(  )
{
  /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
  downcounter = timeout;
  
  /* [BUG FIX] 타이머를 켤 때 카운터 레지스터(CNT)를 0으로 강제 초기화하여 
   * 이전 카운트 값이 남아 첫 인터럽트가 조기에 발생하는(T3.5 타임아웃 오류) 현상 방지 
   */
  __HAL_TIM_SET_COUNTER(&htim7, 0);
  HAL_TIM_Base_Start_IT(&htim7);
}
 
void
vMBPortTimersDisable(  )
{
  /* Disable any pending timers. */
  HAL_TIM_Base_Stop_IT(&htim7);
}
 
/* Create an ISR which is called whenever the timer has expired. This function
* must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
* the timer has expired.
 
static void prvvTIMERExpiredISR( void )
{
( void )pxMBPortCBTimerExpired(  );
}
*/
