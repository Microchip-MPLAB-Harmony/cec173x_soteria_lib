/*****************************************************************************
 * Copyright (c) 2020 Microchip Technology Inc. and its subsidiaries.
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

#include "definitions.h"

#include "spt_task.h"
#include "spt_app.h"
#include "pmci.h"
#include "../spdm/spdm_common.h"
#include "app.h"

/* MPU */
static void spt_main(void *pvParameters);
SPT_BSS_ATTR static StaticTask_t spt_task1_tcb;
static uint32_t spt_task1_stack[SPT_TASK1_STACK_WORD_SIZE] SPT_STACK_ATTR SPT_TASK1_STACK_ALIGN;
SPT_BSS_ATTR static TaskHandle_t spt_task1_handle = NULL;
SPT_BSS_ATTR static SPT_CONTEXT *sptContext = NULL;
extern SPT_BSS_ATTR DI_CONTEXT_SPT *spt_di_context;

static union
{
    uint32_t w[SPT_TASK1_BUF_SIZE / 4];
    uint8_t  b[SPT_TASK1_BUF_SIZE];
} spt_task1_buf SPT_BSS_ATTR_256ALIGN SPT_TASK1_BUF_ALIGN;

#define SPT_TASK1_BUF_ADDR &spt_task1_buf.w[0]

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

static const TaskParameters_t spt_task1_def =
{
    .pcName = "spt_main",
    .pxTaskBuffer = &spt_task1_tcb
};

/****************************************************************/
/** spt_task1_get_handle
* Get the SPT task handle
* @param  void
* @return TaskHandle_t - SPT task handle
*****************************************************************/
TaskHandle_t spt_task_get_handle(void)
{
    return spt_task1_handle;
}

/****************************************************************/
/** spt_app_task_create()
* Function to Create the freertos task for spt
* @param  pvParams
* @return none
*****************************************************************/
#ifdef config_CEC_AHB_PROTECTION_ENABLE
int spt_app_task_create(void *pvParams, CEC_AHB_PRIV_REGS_VALUES *pTaskPrivRegValues)
#else
int spt_app_task_create(void *pvParams)
#endif
{
    //trace0(0, MCTP_TSK, 0, "spt_tsk_create MPU");
    BaseType_t frc = pdFAIL;
    uintptr_t spt_bss_addr = spt_bss_base();
    size_t spt_bsssz = spt_bss_size();
    DI_CONTEXT_SPT *di_context;

    if (spt_bss_addr == 0U)
    {
        return -1;
    }

    /* Check if MPU base addresses are valid */
    configASSERT(spt_bss_addr);
    /* Check if context size is greater than maximum buffer size for the task */
    configASSERT(SPT_TASK1_BUF_SIZE > sizeof(SPT_CONTEXT));

    TaskParameters_t td = spt_task1_def;
    config_task_parameters(&td, spt_main, SPT_TASK1_STACK_WORD_SIZE, pvParams,
                           SPT_TASK1_PRIORITY, spt_task1_stack, pTaskPrivRegValues);
    config_task_memory_regions(&td, 0, spt_bss_addr, spt_bsssz, spt_data_mpu_attr());

    frc = xTaskCreateRestrictedStatic(&td, &spt_task1_handle);


    if (frc != pdPASS)
    {
        return -1;
    }

#if config_CEC_DATA_ISOLATION_CHECKS == 1
    di_checks_register_task(spt_task1_handle, DI_APP_ID_SPT);
#endif

    /* Create other FreeRTOS objects required by this task here */
    sptContext = spt_ctxt_get();
    di_context = (DI_CONTEXT_SPT*)pvParams;
    sptContext->xspt_EventGroupHandle = di_context->spt_request.evt_grp_handle;
;
    ////trace0(0, MCTP_TSK, 0, "spt_tsk_create: Done");

    return 0;
}

/****************************************************************/
/** spt_ctxt_get()
* Get the SPT Context
* @param void
* @return SPT_context
*****************************************************************/
SPT_CONTEXT* spt_ctxt_get(void)
{
    SPT_CONTEXT* ret_spt_ctxt;

// coverity[misra_c_2012_rule_11_3_violation:FALSE]
    ret_spt_ctxt = (SPT_CONTEXT*)(SPT_TASK1_BUF_ADDR);

    return ret_spt_ctxt;
}

SPT_CONTEXT_PER_CHANNEL* spt_channel_ctxt_get(uint8_t channel)
{
    SPT_CONTEXT* ret_spt_ctxt = spt_ctxt_get();
    SPT_CONTEXT_PER_CHANNEL* ch_txt = NULL;
    if(NULL != ret_spt_ctxt)
    {
        if(channel)
        {
            ch_txt = &ret_spt_ctxt->sptCtxt1;
        }
        else
        {
            ch_txt = &ret_spt_ctxt->sptCtxt0;
        }
    }

    return ch_txt;
}

/****************************************************************/
/** spt_main()
* main process of SPT task
* @param  pvParameters - Pointer that will be used as the parameter for the task
* being created.
* @return none
*****************************************************************/
static void spt_main(void* pvParameters)
{
    EventBits_t uxBits;
    spt_di_init(pvParameters);
    sptContext = spt_ctxt_get();

    if(NULL == sptContext)
    {
        return;
    }

    if(NULL == spt_di_context)
    {
        //trace1(0, SPT, 0, "[%s]:DI setup err", __FUNCTION__);
        return;
    }

#if config_CEC_DATA_ISOLATION_CHECKS == 1
    /* We release the semaphore here instead of inside di_mctp_init() */
    /* since a task has to be registered and the value of pxCurrentTCB */
    /* should point to the current task before data isolation cross checking */
    /* can be performed */
    for(uint8_t i=0; i<DI_MAX_SPT_APP; i++)
    {
        xSemaphoreGive(spt_di_context->mutex_handle[i]); /*make semaphore available for aquisition*/
    }
#endif

    while(1)
    {
        uxBits = xEventGroupWaitBits(sptContext->xspt_EventGroupHandle,
                                     (SPT_ISR_EVENT_BIT | SPT_EVENT_BIT |
                                     SPT_DI_EVENT_REQUEST),
                                     pdTRUE,
                                     pdFALSE,
                                     portMAX_DELAY);
        if((SPT_ISR_EVENT_BIT == (uxBits & SPT_ISR_EVENT_BIT)) || (SPT_EVENT_BIT == (uxBits & SPT_EVENT_BIT)))
        {
            spt_event_handle(uxBits);
        }
        if(SPT_DI_EVENT_REQUEST == (uxBits & SPT_DI_EVENT_REQUEST))
        {
            spt_di_get_next_request();
        }
    }
}

void spt_set_event_bits(EventBits_t uxBitsToSet)
{
    sptContext = spt_ctxt_get();
    if(NULL == sptContext)
    {
        return;
    }
    (void)xEventGroupSetBits( sptContext->xspt_EventGroupHandle, uxBitsToSet );
}

void spt_set_event_bits_isr(EventBits_t uxBitsToSet)
{
    sptContext = spt_ctxt_get();
    if(NULL == sptContext)
    {
        return;
    }
    
    BaseType_t xHigherPriorityTaskWoken_spt, xResult;
    
    xHigherPriorityTaskWoken_spt = pdFALSE;
    
    xResult = xEventGroupSetBitsFromISR(sptContext->xspt_EventGroupHandle, uxBitsToSet,
                                        &xHigherPriorityTaskWoken_spt);
    if (xResult != pdFAIL)
    {
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken_spt);
    }
}

#ifdef config_CEC_AHB_PROTECTION_ENABLE
int spt_task_create(void *pvParams, CEC_AHB_PRIV_REGS_VALUES *pTaskPrivRegValues)
#else
int spt_task_create(void *pvParams)
#endif
{
    void *di_ctxt;
    int return_val =0u;
    di_ctxt = di_spt_context_get();

#ifdef config_CEC_AHB_PROTECTION_ENABLE
    return_val = spt_app_task_create(di_ctxt, pTaskPrivRegValues);
#else
    return_val = smbus_app_task_create(di_ctxt);
#endif
    return return_val;

}

/******************************************************************************/
/** spt_task_wait_event_bits();
* Wait for bit set from MCTP, bit is set once MCTP receives and transmits to application
* layer
* @param event_bits - Event bits to be waited
* @return uint32_t event bits waited or Timeout
*******************************************************************************/
uint32_t spt_task_wait_event_bits(uint32_t event_bits)
{
    uint32_t uxBits = 0;
    sptContext = spt_ctxt_get();
    if(NULL == sptContext)
    {
        return uxBits;
    }

    uxBits = xEventGroupWaitBits( sptContext->xspt_EventGroupHandle,
                                event_bits, pdTRUE, pdFALSE, portMAX_DELAY);
    return uxBits;
}