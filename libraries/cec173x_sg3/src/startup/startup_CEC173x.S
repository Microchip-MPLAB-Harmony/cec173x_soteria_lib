/* File: startup_CEC173x.S
 * Derived from:
 * Purpose: startup file for Cortex-M4 devices. Should use with
 *   GCC for ARM Embedded Processors
 * Version: V2.0
 * Date: 16 August 2013
 */
/* Copyright (c) 2011 - 2013 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/
/*
 * Additional Copyright (c) 2020 Microchip Technologies Inc.
 */

#include "startup.h"

	.syntax unified
	.arch   armv7e-m

	.section .stack
	.align  3
#ifdef __STACK_SIZE
	.equ    Stack_Size, __STACK_SIZE
#else
	.equ    Stack_Size, 0x1000
#endif
	.globl  __StackTop
	.globl  __StackLimit
__StackLimit:
	.space  Stack_Size
	.size   __StackLimit, . - __StackLimit
__StackTop:
	.size   __StackTop, . - __StackTop

	.section .heap
	.align  3
#ifdef __HEAP_SIZE
	.equ    Heap_Size, __HEAP_SIZE
#else
	.equ    Heap_Size, 0
#endif
	.globl  __HeapBase
	.globl  __HeapLimit
__HeapBase:
	.if Heap_Size
	.space  Heap_Size
	.endif
	.size   __HeapBase, . - __HeapBase
__HeapLimit:
	.size   __HeapLimit, . - __HeapLimit

	.section .startup,code
	.thumb
	.thumb_func
	.align 2
	.weak  reset_hook_1
	.type  reset_hook_1, %function
reset_hook_1:
	nop
	nop
	bx	lr

	.pool
	.size	reset_hook_1, . - reset_hook_1

	.section .startup, code
	.thumb
	.thumb_func
	.align  2
	.globl  Reset_Handler
	.type   Reset_Handler, %function
Reset_Handler:
/*
 * On Vector or chip reset vector table is 0 (Boot-ROM)
 * We load our application vector table into Cortex-M3/M4 VTOR
 * register at 0xE000ED08.
 * We load SP with stack pointer from out vector table.
 */
	ldr	r2, =exception_table
	ldr	r3, =0xE000ED08
	str	r2, [r3]
	dsb
	isb
	ldr	r3, [r2]
	msr	msp, r3
	isb	0xf
	msr	psp, r3
	isb	0xf

	bl	reset_hook_1

#ifdef STACK_SENTINEL
/*
 *  Help diagnose stack underflow by filling the stack with
 *  a sentinel value.
 */
	ldr	r0, =STACK_SENTINEL_VAL
	ldr	r1, =__StackLimit
	ldr	r2, =__StackTop
.L_loop_ss0:
	str	r0, [r1], #4
	cmp	r1, r2
	bne	.L_loop_ss0
#endif

/*
 *  Firstly it copies data from read only memory to RAM. There are two schemes
 *  to copy. One can copy more than one sections. Another can only copy
 *  one section.  The former scheme needs more instructions and read-only
 *  data to implement than the latter.
 *  Macro __STARTUP_COPY_MULTIPLE is used to choose between two schemes.  */

#ifdef __STARTUP_COPY_MULTIPLE
/*
 *  Multiple sections scheme.
 *
 *  Between symbol address __copy_table_start__ and __copy_table_end__,
 *  there are array of triplets, each of which specify:
 *    offset 0: LMA of start of a section to copy from
 *    offset 4: VMA of start of a section to copy to
 *    offset 8: size of the section to copy. Must be multiply of 4
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
	ldr	r4, =__copy_table_start__
	ldr	r5, =__copy_table_end__

.L_loop0:
	cmp	r4, r5
	bge	.L_loop0_done
	ldr	r1, [r4]
	ldr	r2, [r4, #4]
	ldr	r3, [r4, #8]

.L_loop0_0:
	subs	r3, #4
	ittt	ge
	ldrge	r0, [r1, r3]
	strge	r0, [r2, r3]
	bge	.L_loop0_0

	adds	r4, #12
	b	.L_loop0

.L_loop0_done:
#else
/*
 * Single section scheme.
 *
 *  The ranges of copy from/to are specified by following symbols
 *    __etext: LMA of start of the section to copy from. Usually end of text
 *    __data_start__: VMA of start of the section to copy to
 *    __data_end__: VMA of end of the section to copy to
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
	ldr	r1, =__etext
	ldr	r2, =__data_start__
	ldr	r3, =__data_end__

.L_loop1:
	cmp	r2, r3
	ittt	lt
	ldrlt	r0, [r1], #4
	strlt	r0, [r2], #4
	blt	.L_loop1
#endif /*__STARTUP_COPY_MULTIPLE */

/*
 *  This part of work usually is done in C library startup code.
 *  Otherwise, define this macro to enable it in this startup.
 *
 *  There are two schemes too. One can clear multiple BSS sections.
 *  Another can only clear one section. The former is more size expensive
 *  than the latter.
 *
 *  Define macro __STARTUP_CLEAR_BSS_MULTIPLE to choose the former.
 *  Otherwise efine macro __STARTUP_CLEAR_BSS to choose the later.
 */
#ifdef __STARTUP_CLEAR_BSS_MULTIPLE
/*
 * Multiple sections scheme.
 *
 *  Between symbol address __copy_table_start__ and __copy_table_end__,
 *  there are array of tuples specifying:
 *    offset 0: Start of a BSS section
 *    offset 4: Size of this BSS section. Must be multiply of 4
 */
	ldr	r3, =__zero_table_start__
	ldr	r4, =__zero_table_end__

.L_loop2:
	cmp	r3, r4
	bge	.L_loop2_done
	ldr	r1, [r3]
	ldr	r2, [r3, #4]
	movs	r0, 0

.L_loop2_0:
	subs	r2, #4
	itt	ge
	strge	r0, [r1, r2]
	bge	.L_loop2_0

	adds	r3, #8
	b	.L_loop2
.L_loop2_done:
#elif defined (__STARTUP_CLEAR_BSS)
/*
 * Single BSS section scheme.
 *
 *  The BSS section is specified by following symbols
 *    __bss_start__: start of the BSS section.
 *    __bss_end__: end of the BSS section.
 *
 *  Both addresses must be aligned to 4 bytes boundary.
 */
	ldr	r1, =__bss_start__
	ldr	r2, =__bss_end__

	movs	r0, 0
.L_loop3:
	cmp	r1, r2
	itt	lt
	strlt	r0, [r1], #4
	blt	.L_loop3
#endif /* __STARTUP_CLEAR_BSS_MULTIPLE || __STARTUP_CLEAR_BSS */

#ifndef __NO_SYSTEM_INIT
	bl	SystemInit
#endif
	bl	__libc_init_array
	
#ifndef __START
#define __START _start
#endif
	bl	main

	.pool
	.size	Reset_Handler, . - Reset_Handler

	.section .startup,code
	.balign 8
	.thumb_func
	.weak	Unimplemented_Handler
	.type	Unimplemented, %function
Unimplemented_Handler:
#ifdef __UNIMPLEMENTED_ISR
	b 	__UNIMPLEMENTED_ISR
#else
	mov	r0, #13
	b	.
#endif
	.size	Unimplemented_Handler, . - Unimplemented_Handler