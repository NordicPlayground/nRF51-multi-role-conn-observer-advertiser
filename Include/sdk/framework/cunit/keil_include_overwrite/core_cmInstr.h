/**************************************************************************//**
 * @file     core_cmInstr.h
 * @brief    CMSIS Cortex-M Core Instruction Access Header File
 * @version  V2.01
 * @date     06. December 2010
 *
 * @note
 * Copyright (C) 2009-2010 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M 
 * processor based microcontrollers.  This file can be freely distributed 
 * within development tools that are supporting such ARM based processors. 
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/

#if defined ( __CC_ARM   ) || defined (__GNUC__)

#ifndef __CORE_CMINSTR_H__
#define __CORE_CMINSTR_H__


// Overriding the Cortex-M0 assembly calls.
void __NOP(void);

void __WFI(void);

void __WFE(void);

void __SEV(void);

void __ISB(void);

void __DSB(void);

void __DMB(void);

uint32_t __REV(uint32_t value);

uint32_t __REV16(uint32_t value);

int32_t __REVSH(int32_t value);

uint32_t __RBIT(uint32_t value);

uint8_t __LDREXB(volatile uint8_t *addr);

uint16_t __LDREXH(volatile uint16_t *addr);

uint32_t __LDREXW(volatile uint32_t *addr);

uint32_t __STREXB(uint8_t value, volatile uint8_t *addr);

uint32_t __STREXH(uint16_t value, volatile uint16_t *addr);

uint32_t __STREXW(uint32_t value, volatile uint32_t *addr);

void __CLREX(void);

uint8_t __CLZ(uint32_t value);

#endif /* __CORE_CMINSTR_H__ */

#endif /* defined ( __CC_ARM   ) || defined (__GNUC__) */

