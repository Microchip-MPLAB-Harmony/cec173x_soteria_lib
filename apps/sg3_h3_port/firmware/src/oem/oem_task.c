/*****************************************************************************
 * Copyright (c) 2022 Microchip Technology Inc. and its subsidiaries.
 * You may use this software and any derivatives exclusively with
 * Microchip products.
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".
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

#include "app.h"

#include "oem_task.h"

static void oem_task(void *pvParameters);
static StaticTask_t oem_task_tcb;
static uint32_t oem_task_stack[OEM_TASK_STACK_WORD_SIZE] OEM_STACK_ATTR OEM_TASK_STACK_ALIGN;

SHARED_BSS_ATTR TaskHandle_t oem_task_handle = NULL;

#if USE_OEM_TASK_BUFFER
union
{
    uint32_t w[OEM_TASK_BUF_SIZE / 4];
    uint8_t  b[OEM_TASK_BUF_SIZE];
} oem_task_buf SHARED_BSS_ATTR OEM_TASK_BUF_ALIGN;

#define OEM_TASK_BUF_ADDR &oem_task_buf.w[0]
#endif
/*
 * FreeRTOS restricted task creation requires TaskParameter_t
 * ISSUE: FreeRTOS defined member .pcName as const signed char * const
 * We get a compiler error assigning pcName at runtime.
 * If we assign statically then we can't use pvParams in the task
 * create function.
 * FreeRTOS MPU region attribute defines
 * portMPU_REGION_READ_WRITE
 * portMPU_REGION_PRIVILEGED_READ_ONLY
 * portMPU_REGION_READ_ONLY
 * portMPU_REGION_PRIVILEGED_READ_WRITE
 * portMPU_REGION_CACHEABLE_BUFFERABLE
 * portMPU_REGION_EXECUTE_NEVER
 */
/*
 * FreeRTOS TaskParameters_t structure contains two constant items:
 * type * const objname.
 * The C standard states these items are immutable and can only be
 * set at initialization not run time assignment.
 * Therefore we must create a TaskParameters_t as a static global to
 * initialize the pcName and pxTaskBuffer objects.
 * We can intialize all other items in the task creation function.
 */
static const TaskParameters_t oem_task_def =
{
    .pcName = "oem_task",
    .pxTaskBuffer = &oem_task_tcb
};

TaskHandle_t oem_task_get_handle(void)
{
    return oem_task_handle;
}

/*
 * Called from C main before FreeRTOS scheduler is started.
 * Cortex-M4 is in default reset state: privileged thread mode.
 * Program any Cortex-M4 privileged registers here:
 * Examples: NVIC priority for an interrupts used by this task.
 * Chip init has set all interrupt priorities to the lowest.
 */
#ifdef config_CEC_AHB_PROTECTION_ENABLE
int oem_task_create(void *pvParams, CEC_AHB_PRIV_REGS_VALUES *pTaskPrivRegValues)
#else
int oem_task_create(void *pvParams)
#endif
{
    BaseType_t frc = pdFAIL;
    uintptr_t shared_data_addr = shared_data_base();
    size_t sharedsz = shared_data_size();
    uintptr_t shared_bss_addr = shared_bss_base();
    size_t shared_bsssz = shared_bss_size();

    /* Check if MPU base addresses are valid */
    configASSERT(shared_data_addr);
    configASSERT(shared_bss_addr);

    TaskParameters_t td = oem_task_def;
    config_task_parameters(&td, oem_task, OEM_TASK_STACK_WORD_SIZE, NULL,
                           OEM_TASK_PRIORITY, oem_task_stack, pTaskPrivRegValues);
    config_task_memory_regions(&td, 0, shared_data_addr, sharedsz, shared_data_mpu_attr());
    config_task_memory_regions(&td, 1, shared_bss_addr, shared_bsssz, shared_data_mpu_attr());

    frc = xTaskCreateRestrictedStatic(&td, &oem_task_handle);

    if (frc != pdPASS)
    {
        return -1;
    }

#if config_CEC_DATA_ISOLATION_CHECKS == 1
    di_checks_register_task(oem_task_handle, DI_APP_ID_OEM);
#endif

    /* Create other FreeRTOS objects required by this task here */

    return 0;
}

/*
 * FreeRTOS task1
 */
static void oem_task(void *pvParameters)
{
    while (1)
    {
        //trace11(OEM_TRACE, task1, 1, "OEMST %u ", OEM_TASK_DELAY_TICKS);  // OEM Task sleep ticks
        vTaskDelay(OEM_TASK_DELAY_TICKS);
        //trace0(OEM_TRACE, task1, 1, "OEMW" ); //  OEM task wake Do not uncomment, commented print to reduce stack occupancy
    }
}




/**   @}
 */

