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
 * [OPTIMIZATION] RTOS 없는 Bare-metal 환경에서, 단일 플래그의 이벤트 유실 위험을 
 * 방지하기 위해 가장 가벼운 4-depth 고정 큐(Ring Buffer)를 사용합니다.
 * 크기가 작아 처리가 빠르고, 버스트(Burst) 노이즈에도 강건하게 대응합니다.
 */
#define MB_EVENT_QUEUE_SIZE 4

static volatile UCHAR ucQueueHead;
static volatile UCHAR ucQueueTail;
static volatile eMBEventType xQueuedEvents[MB_EVENT_QUEUE_SIZE];

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortEventInit( void )
{
    ucQueueHead = 0;
    ucQueueTail = 0;
    return TRUE;
}

BOOL
xMBPortEventPost( eMBEventType eEvent )
{
    BOOL xStatus = FALSE;
    UCHAR ucNextHead;

    ENTER_CRITICAL_SECTION(  );
    ucNextHead = ( UCHAR )( ( ucQueueHead + 1 ) & ( MB_EVENT_QUEUE_SIZE - 1 ) ); // % 연산 대신 비트마스킹(&) 최적화 (사이즈가 2의 승수일 때 가능)

    if( ucNextHead != ucQueueTail )
    {
        xQueuedEvents[ucQueueHead] = eEvent;
        ucQueueHead = ucNextHead;
        xStatus = TRUE;
    }
    EXIT_CRITICAL_SECTION(  );

    return xStatus;
}

BOOL
xMBPortEventGet( eMBEventType * eEvent )
{
    BOOL xEventHappened = FALSE;

    ENTER_CRITICAL_SECTION(  );
    if( ucQueueHead != ucQueueTail )
    {
        *eEvent = xQueuedEvents[ucQueueTail];
        ucQueueTail = ( UCHAR )( ( ucQueueTail + 1 ) & ( MB_EVENT_QUEUE_SIZE - 1 ) );
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
