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
 * File: $Id: port.h,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

#ifndef _PORT_H
#define _PORT_H

#include <assert.h>
#include "stm32f4xx_hal.h"

#define INLINE inline

#ifdef __cplusplus
#define PR_BEGIN_EXTERN_C           extern "C" {
#define PR_END_EXTERN_C             }
#else
#define PR_BEGIN_EXTERN_C
#define PR_END_EXTERN_C
#endif

/* [BUG FIX] 안전한 크리티컬 섹션 중첩을 위한 함수 래핑 */
extern void vMBPortEnterCritical(void);
extern void vMBPortExitCritical(void);

#define ENTER_CRITICAL_SECTION() vMBPortEnterCritical()
#define EXIT_CRITICAL_SECTION() vMBPortExitCritical()

typedef uint8_t BOOL;

typedef unsigned char UCHAR;
typedef char CHAR;

typedef uint16_t USHORT;
typedef int16_t SHORT;

typedef uint32_t ULONG;
typedef int32_t LONG;

#ifndef TRUE
#define TRUE            1
#endif

#ifndef FALSE
#define FALSE           0
#endif

#endif
