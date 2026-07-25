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
/* Small bounded queue (8 events) to absorb short ISR bursts with low RAM cost. */
#define MB_EVENT_QUEUE_SIZE 8

static volatile UCHAR ucQueueHead;
static volatile UCHAR ucQueueTail;
static eMBEventType xQueuedEvents[MB_EVENT_QUEUE_SIZE];

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortEventInit( void )
{
    ENTER_CRITICAL_SECTION(  );
    ucQueueHead = 0;
    ucQueueTail = 0;
    EXIT_CRITICAL_SECTION(  );
    return TRUE;
}

BOOL
xMBPortEventPost( eMBEventType eEvent )
{
    BOOL xStatus = FALSE;
    UCHAR ucNextHead;

    ENTER_CRITICAL_SECTION(  );
    ucNextHead = ( UCHAR )( ( ucQueueHead + 1 ) % MB_EVENT_QUEUE_SIZE );

    if( ucNextHead != ucQueueTail )
    {
        xQueuedEvents[ucQueueHead] = eEvent;
        ucQueueHead = ucNextHead;
        xStatus = TRUE;
    }
    /* ucNextHead == ucQueueTail means queue is full; drop newest event. */
    EXIT_CRITICAL_SECTION(  );

    return xStatus;
}

BOOL
xMBPortEventGet( eMBEventType * eEvent )
{
    BOOL            xEventHappened = FALSE;

    ENTER_CRITICAL_SECTION(  );
    if( ucQueueHead != ucQueueTail )
    {
        *eEvent = xQueuedEvents[ucQueueTail];
        ucQueueTail = ( UCHAR )( ( ucQueueTail + 1 ) % MB_EVENT_QUEUE_SIZE );
        xEventHappened = TRUE;
    }
    EXIT_CRITICAL_SECTION(  );

    return xEventHappened;
}
