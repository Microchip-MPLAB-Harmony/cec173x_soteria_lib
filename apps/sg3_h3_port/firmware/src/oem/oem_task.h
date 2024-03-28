/*****************************************************************************
* Copyright 2020 Microchip Technology Inc. and its subsidiaries.
* You may use this software and any derivatives exclusively with
* Microchip products.
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP 'AS IS'.
* NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
* INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
* AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.
* TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
* CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF
* FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
* MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE
* OF THESE TERMS.
*****************************************************************************/

/** @file oem_task.h
 */

#ifndef OEM_TASK_H
#define OEM_TASK_H

#include <stddef.h>
#include <stdint.h>

#include <FreeRTOS.h>
#include "oem_task.h"

#include "common.h"
extern uintptr_t shared_data_base(void);
extern size_t shared_data_size(void);
extern uintptr_t shared_bss_base(void);
extern size_t shared_bss_size(void);
extern uint32_t shared_data_mpu_attr(void);
#define OEM_STACK_ATTR                          __attribute__((section("oem_stack")))
#define SHARED_BSS_ATTR                         __attribute__((section("shared_bss")))
#ifdef __cplusplus
extern "C" {
#endif

#define USE_OEM_TASK_BUFFER 0
#define configOEM_TASK_PRIORITY 1U
#define OEM_TASK_PRIORITY ((tskIDLE_PRIORITY + configOEM_TASK_PRIORITY) % configMAX_PRIORITIES)
/* Stack size must be a power of 2 if the task is restricted */
#define OEM_TASK_STACK_SIZE 256U
#define OEM_TASK_STACK_WORD_SIZE ((OEM_TASK_STACK_SIZE) / 4U)

#if !IS_PWR2(OEM_TASK_STACK_SIZE)
#error "FORCED BUILD ERROR TASK1 Stack size is not a power of 2!"
#endif
#define OEM_TASK_STACK_ALIGN __attribute__ ((aligned(OEM_TASK_STACK_SIZE)))

/* OEM_TASK: First MPU region */
#define OEM_TASK_BUF_SIZE 256U
#define OEM_TASK_BUF_MPU_ATTR 0U
#define OEM_TASK_BUF_ALIGN

#if !IS_PWR2(OEM_TASK_BUF_SIZE)
#error "FORCED BUILD ERROR sb_core task Buffer size is not a power of 2!"
#endif

#undef OEM_TASK_BUF_MPU_ATTR
/* This Task R/W.  Privileged R/W. */
#define OEM_TASK_BUF_MPU_ATTR \
    (portMPU_REGION_READ_WRITE | portMPU_REGION_CACHEABLE_BUFFERABLE \
     | portMPU_REGION_EXECUTE_NEVER)

#undef OEM_TASK_BUF_ALIGN
#define OEM_TASK_BUF_ALIGN __attribute__((aligned(OEM_TASK_BUF_SIZE)))

#define OEM_TASK_DELAY_MS 10000U
#define OEM_TASK_DELAY_TICKS OEM_TASK_DELAY_MS / portTICK_PERIOD_MS
#ifdef config_CEC_AHB_PROTECTION_ENABLE
int oem_task_create(void *pvParams, CEC_AHB_PRIV_REGS_VALUES *pTaskPrivRegValues);
#else
int oem_task_create(void *pvParams);
#endif

#endif
/* end OEM_TASK_H */
/**   @}
 */