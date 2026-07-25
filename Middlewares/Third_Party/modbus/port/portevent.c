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
 * File: $Id: portevent.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Modbus includes ----------------------------------*/
/* port.h provides IRQ critical-section macros used by this queue. */
#include "port.h"
#include "mb.h"
#include "mbport.h"

/* ----------------------- Variables ----------------------------------------*/
/* 
 * [OPTIMIZATION] RTOS 없는 메인 루프 폴링 방식에서는 복잡한 링버퍼 대신 
 * 단일 이벤트 플래그를 사용하여 인터럽트 차단(Disable) 시간을 극적으로 줄입니다.
 * Modbus RTU는 특성상 한 번에 하나의 트랜잭션만 발생하므로 단일 플래그로 충분합니다.
 */
static volatile eMBEventType eQueuedEvent;
static volatile BOOL xEventInQueue;

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortEventInit( void )
{
    xEventInQueue = FALSE;
    return TRUE;
}

BOOL
xMBPortEventPost( eMBEventType eEvent )
{
    ENTER_CRITICAL_SECTION(  );
    eQueuedEvent = eEvent;
    xEventInQueue = TRUE;
    EXIT_CRITICAL_SECTION(  );
    return TRUE;
}

BOOL
xMBPortEventGet( eMBEventType * eEvent )
{
    BOOL xEventHappened = FALSE;

    ENTER_CRITICAL_SECTION(  );
    if( xEventInQueue )
    {
        *eEvent = eQueuedEvent;
        xEventInQueue = FALSE;
        xEventHappened = TRUE;
    }
    EXIT_CRITICAL_SECTION(  );

    return xEventHappened;
}

/* ----------------------- Critical Section Wrapper -------------------------*/
/* [BUG FIX] 중첩된 크리티컬 섹션(Nesting) 시 안전한 복원을 위한 래퍼 함수 */
static uint32_t ulCriticalNesting = 0;

void vMBPortEnterCritical(void)
{
    __disable_irq();
    ulCriticalNesting++;
}

void vMBPortExitCritical(void)
{
    if( ulCriticalNesting > 0 )
    {
        ulCriticalNesting--;
        if( ulCriticalNesting == 0 )
        {
            __enable_irq();
        }
    }
}
