/*
 * Copyright 2020 Microchip Technologies Inc.
 */

/*
 * This file is include by startup_CEC173x.S and must not contain
 * complex preprocessor directives.
 * We define various startup flags.
 */

/* Default stack size is 4KB */
#define __STACK_SIZE 0x1000

/* Default heap size is 0 */
#define __HEAP_SIZE 0x0000

/*
 * Stack sentinel
 * If defined startup code will fill the default stack with the
 * define 32-bit value.
 */
/* #define STACK_SENTINEL */
#define STACK_SENTINEL_VAL 0x57AC5E91

/*
 * If mutliple initialized data sections are implemented then define
 * this.
 */
/* #define __STARTUP_COPY_MULTIPLE */

/*
 * If multiple BSS sections are implemented and startup code should
 * zero fill them Undefine __STARTUP_CLEAR_BSS and define this.
 */
/* #define __STARTUP_CLEAR_BSS_MULTIPLE */

/*
 * If a single BSS section is implemented and startup code should zero
 * fill it undefine __STARTUP_CLEAR_BSS_MULTIPLE and defint this.
 */
#define __STARTUP_CLEAR_BSS

/*
 * Skip calling CMSIS SystemInit before C startup code.
 */
/* #define __NO_SYSTEM_INIT */

/*
 * The last action in startup is to call C library init which then
 * calls C main. Override the call to C library init by defining this
 * to the alternate function.
 * For example, if no C library is used one can define this to be main.
 */
/* #define __START main */

/*
 * Application implements a default interrupt handler
 */
/* #define __DEFAULT_ISR Default_ISR */
