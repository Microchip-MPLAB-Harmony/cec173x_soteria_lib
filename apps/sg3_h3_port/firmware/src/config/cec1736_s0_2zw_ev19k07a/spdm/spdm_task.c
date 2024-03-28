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

#include "app.h"
#include "spdm_task.h"
#include "spdm_pkt_prcs.h"
#include "../../../common/include/common.h"
#include "../pldm/pldm_common.h"
#include "../pldm/pldm_pkt_prcs.h"

/* MPU */
static void spdm_main(void *pvParameters);
static StaticTask_t spdm_tcb;

extern SPDM_BSS1_ATTR DI_CONTEXT_SPDM *spdm_di_context;
static uint32_t spdm_task_stack[SPDM_STACK_WORD_SIZE] SPDM_STACK_ATTR SPDM_STACK_ALIGN;
SPDM_BSS2_ATTR static TaskHandle_t spdm_handle;
SPDM_BSS0_ATTR SPDM_CONTEXT *spdmContext;
PLDM_BSS2_ATTR PLDM_CONTEXT *pldmContext;
extern SPDM_BSS1_ATTR uint8_t spdm_flash_busy;

union
{
    uint32_t w[SPDM_TASK_BUF_SIZE / 4];
    uint8_t  b[SPDM_TASK_BUF_SIZE];
} spdm_task_buf SPDM_BSS0_ATTR_1024ALIGNED SPDM_TASK_BUF_ALIGN;

#define SPDM_TASK_BUF_ADDR &spdm_task_buf.w[0]
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
static const TaskParameters_t spdm_def =
{
    .pcName = "spdm_main",
    .pxTaskBuffer = &spdm_tcb
};

TaskHandle_t spdm_get_handle(void)
{
    return spdm_handle;
}

/****************************************************************/
/** spdm_task_create()
* Function to Create the freertos task for spdm
* @param  pvParams
* @return none
**********************************************************************/
#ifdef config_CEC_AHB_PROTECTION_ENABLE
int spdm_app_task_create(void *pvParams, CEC_AHB_PRIV_REGS_VALUES *pTaskPrivRegValues)
#else
int spdm_app_task_create(void *pvParams)
#endif
{
    //trace1(0, SPDM_TSK, 0, "[%s]: SPDM task create - MPU", __FUNCTION__);
    BaseType_t frc = pdFAIL;
    uintptr_t spdm_bss0_addr = spdm_bss0_base();
    size_t spdm_bss0_sz = spdm_bss0_size();
    uintptr_t spdm_bss1_addr = spdm_bss1_base();
    size_t spdm_bss1_sz = spdm_bss1_size();
    uintptr_t spdm_bss2_addr = spdm_bss2_base();
    size_t spdm_bss2_sz = spdm_bss2_size();

    DI_CONTEXT_SPDM *di_context;

    /* Check if MPU base addresses are valid */
    configASSERT(spdm_bss0_addr);
    configASSERT(spdm_bss1_addr);
    configASSERT(spdm_bss2_addr);

    /* Check if context size is greater than maximum buffer size for the task */
    configASSERT(SPDM_TASK_BUF_SIZE > sizeof(SPDM_CONTEXT));

    TaskParameters_t td = spdm_def;
    config_task_parameters(&td, spdm_main, SPDM_STACK_WORD_SIZE, pvParams,
                           SPDM_PRIORITY, spdm_task_stack, pTaskPrivRegValues);
    config_task_memory_regions(&td, 0, spdm_bss0_addr, spdm_bss0_sz, spdm_data_mpu_attr());
    config_task_memory_regions(&td, 1, spdm_bss1_addr, spdm_bss1_sz, spdm_data_mpu_attr());
    config_task_memory_regions(&td, 2, spdm_bss2_addr, spdm_bss2_sz, spdm_data_mpu_attr());

    frc = xTaskCreateRestrictedStatic(&td, &spdm_handle);


    if (frc != pdPASS)
    {
        return -1;
    }

#if config_CEC_DATA_ISOLATION_CHECKS == 1
    di_checks_register_task(spdm_handle, DI_APP_ID_SPDM);
#endif

    /* Create other FreeRTOS objects required by this task here */
    spdmContext =  spdm_ctxt_get();
//    spdmContext->xSPDMEventGroupHandle = xEventGroupCreateStatic(&spdmContext->xSPDMCreatedEventGroup);
    di_context = (DI_CONTEXT_SPDM*)pvParams;
    spdmContext->xSPDMEventGroupHandle = di_context->spdm_request.evt_grp_handle;

    //trace1(0, SPDM_TSK, 0, "[%s]: SPDM tsk create - Done", __FUNCTION__);

    return 0;
}

/******************************************************************************/
/** spdm_ctxt_get()
* Get the SPDM Context
* @param void
* @return SPDM_context
*******************************************************************************/
SPDM_CONTEXT* spdm_ctxt_get(void)
{
    SPDM_CONTEXT* ret_spdm_ctxt;

    ret_spdm_ctxt = (SPDM_CONTEXT*)(SPDM_TASK_BUF_ADDR);

    return ret_spdm_ctxt;
}

/******************************************************************************/
/** spdm_main()
* main process of SPDM task
* @param  pvParameters - Pointer that will be used as the parameter for the task
* being created.
* @return none
********************************************************************************/
static void spdm_main(void* pvParameters)
{
    EventBits_t uxBits;
    spdm_di_init(pvParameters);

    spdmContext = spdm_ctxt_get();
    if(NULL == spdmContext)
    {
        return;
    }
    pldmContext = pldm_ctxt_get();
    
    if (NULL == spdm_di_context)
    {
        //trace1(0, SPDM_TSK, 0, "[%s]:DI setup err", __FUNCTION__);
        return;
    }

    pldm_init_flags();
    pldmContext->xPLDMRespTimer =
        xTimerCreateStatic("PLDMResp_timer", // Text name for the task.  Helps debugging only.  Not used by FreeRTOS.
                           pdMS_TO_TICKS(FD_T1), // The period of the timer in ticks.
                           pdTRUE, // This is an auto-reload timer.
                           NULL, // A variable incremented by the software timer's callback function.
                           PLDMResp_timer_callback, // The function to execute when the timer expires.
                           &pldmContext->PLDMResp_TimerBuffer); // The buffer that will hold the software timer structure.

    spdmContext->xSPDMHBTimer =
        xTimerCreateStatic("SPDMHB_timer", // Text name for the task.  Helps debugging only.  Not used by FreeRTOS.
                           pdMS_TO_TICKS(HB_TIMEOUT_IN_MS), // The period of the timer in ticks.
                           pdTRUE, // This is an auto-reload timer.
                           NULL, // A variable incremented by the software timer's callback function.
                           SPDMHB_timer_callback, // The function to execute when the timer expires.
                           &spdmContext->SPDMHB_TimerBuffer); // The buffer that will hold the software timer structure.

    spdmContext->spdm_state_info = SPDM_IDLE;
    //trace1(0, SPDM_TSK, 0, "[%s]: SPDM tsk main proc", __FUNCTION__);
    while(1)
    {
        //trace0(0, SPDM_TSK, 0, "spdm_main: Loop");
        uxBits = xEventGroupWaitBits(spdmContext->xSPDMEventGroupHandle,
                                     (SPDM_EVENT_BIT | PLDM_EVENT_BIT | MCTP_DI_EVENT_RESPONSE | SB_CORE_DI_EVENT_RESPONSE | SPDM_INIT_START_BIT |
                                      SPDM_I2C_EVENT_BIT | SB_CORE_DI_EVENT_APPLY_RESPONSE | PLDM_RESP_EVENT_BIT | SPDM_RESP_EVENT_BIT |
                                      SPDM_REAUTH_DONE_BIT),
                                     pdTRUE,
                                     pdFALSE,
                                     portMAX_DELAY );
        if (SPDM_INIT_START_BIT == (uxBits & SPDM_INIT_START_BIT))
        {
            spdm_init_task(spdmContext);
        }
        if (SPDM_REAUTH_DONE_BIT == (uxBits & SPDM_REAUTH_DONE_BIT))
        {
            spdm_update_ap_msr_data(spdmContext);
        }
        if (SPDM_I2C_EVENT_BIT == (uxBits & SPDM_I2C_EVENT_BIT))
        {
            spdm_di_get_i2c_response();
        }
        //This check ensures SPDM requests are not executed when write certificate is being processed
        if ((SPDM_EVENT_BIT == (uxBits & SPDM_EVENT_BIT)) && (SPDM_I2C_EVENT_BIT != (uxBits & SPDM_I2C_EVENT_BIT)))
        {
            switch (spdmContext->spdm_state_info)
            {
            case SPDM_IDLE:
                spdm_event_task(spdmContext);
                //do nothing
                break;
            case SPDM_INIT_CERT_PARAMS:
                spdm_pkt_initialize_cert_params_to_default(spdmContext);
                break;
            case SPDM_GET_CERT_FROM_APCFG:
                spdm_pkt_get_cert_from_apcfg(spdmContext);
                break;
            case SPDM_COPY_CERT_DATA_TO_BUF:
                spdm_pkt_copy_cert_data_to_buf(spdmContext);
                break;
            case SPDM_CALC_HASH_CHAIN:
                spdm_flash_busy = true;
                spdm_pkt_store_hash_of_chain(spdmContext);
                spdm_pkt_store_signature_type();
                spdm_flash_busy = false;
                pldm_pkt_get_config_from_apcfg(pldmContext);
                break;
            case SPDM_CMD_PROCESS_MODE:
                spdm_event_task(spdmContext);
                break;
            default:
                break;
            }
        }
        if (MCTP_DI_EVENT_RESPONSE == (uxBits & MCTP_DI_EVENT_RESPONSE))
        {
            spdm_di_get_mctp_response();
        }
        if (SB_CORE_DI_EVENT_RESPONSE == (uxBits & SB_CORE_DI_EVENT_RESPONSE))
        {
            spdm_di_get_sb_core_response();
        }
        if (PLDM_EVENT_BIT == (uxBits & PLDM_EVENT_BIT))
        {
            //trace0(0, SPDM_TSK, 0, "PLDM:bit set step into event task for recv and trans");
            switch(pldmContext->pldm_state_info)
            {
            case PLDM_IDLE:
                // do nothing
                break;
            case PLDM_CMD_GET_AP_CFG:
                //pldm_pkt_get_config_from_apcfg(pldmContext);
                break;
            case PLDM_CMD_PROCESS_MODE:
                pldm1_event_task();
                break;
            }
        }
        if (SB_CORE_DI_EVENT_APPLY_RESPONSE == (uxBits & SB_CORE_DI_EVENT_APPLY_RESPONSE))
        {
            spdm_di_get_sb_core_response();
        }

        if (PLDM_RESP_EVENT_BIT == (uxBits & PLDM_RESP_EVENT_BIT))
        {
            pldm_pkt_tx_packet();
        }

        if (SPDM_RESP_EVENT_BIT == (uxBits & SPDM_RESP_EVENT_BIT))
        {
            spdm_pkt_tx_packet();
        }
    }
}

/****************************************************************/
/** SET_SPDM_EVENT_FLAG()
* Set event flag to trigger SPDM task to process
* @param  none
* @return none
**********************************************************************/
void SET_SPDM_EVENT_FLAG(void)
{
    spdmContext = spdm_ctxt_get();
    if(NULL == spdmContext)
    {
        return;
    }
    //trace1(0, SPDM_TSK, 0, "[%s]: set SPDM event", __FUNCTION__);
    xEventGroupSetBits( spdmContext->xSPDMEventGroupHandle, SPDM_EVENT_BIT );
}

/****************************************************************/
/** SET_PLDM_EVENT_FLAG()
* Set event flag to trigger PLDM task to process
* @param  none
* @return none
**********************************************************************/
void SET_PLDM_EVENT_FLAG(void)
{
    pldmContext = pldm_ctxt_get();
    if (NULL == pldmContext)
    {
        return;
    }
    pldmContext->pldm_state_info = PLDM_CMD_PROCESS_MODE;
    //spdmContext->pldm_tx_state = PLDM_TX_IDLE;
    //trace0(0, SPDM_TSK, 0, "PLDM:set event");
    xEventGroupSetBits( spdm_ctxt_get()->xSPDMEventGroupHandle, PLDM_EVENT_BIT);
}

/****************************************************************/
/** mctp_task_create()
* Function to Create the MCTP task for smbus
* @param  pvParams
* @return none
**********************************************************************/
#ifdef config_CEC_AHB_PROTECTION_ENABLE
int spdm_task_create(void *pvParams, CEC_AHB_PRIV_REGS_VALUES *pTaskPrivRegValues)
#else
int spdm_task_create(void *pvParams)
#endif
{
    void *di_ctxt;
    int return_val =0u;

    di_ctxt = di_spdm_context_get();

#ifdef config_CEC_AHB_PROTECTION_ENABLE
    return_val = spdm_app_task_create(di_ctxt, pTaskPrivRegValues);
#else
    return_val = spdm_app_task_create(di_ctxt);
#endif

    return 0;
}
/******************************************************************************/
/** pldm_response_timeout_start
* Start the software PLDMResponse timer
* @param void
* @return void
*******************************************************************************/
void pldm_response_timeout_start(void)
{
    pldmContext = pldm_ctxt_get();
    if (NULL != pldmContext)
    {
        if (NULL != pldmContext->xPLDMRespTimer)
        {
            if (xTimerStart(pldmContext->xPLDMRespTimer, 0) != pdPASS)
            {
                //trace0(1, SPDM_TSK, 1, "Err:PLDMResp tmr could not set to active state");
                return;
            }
        }
    }
}

/******************************************************************************/
/** pldm_response_timeout_stop
* Stop the software PLDMResponse timer
* @param void
* @return void
*******************************************************************************/
void pldm_response_timeout_stop(void)
{
    pldmContext = pldm_ctxt_get();
    if (NULL != pldmContext)
    {
        if (NULL != pldmContext->xPLDMRespTimer)
        {
            if (xTimerStop(pldmContext->xPLDMRespTimer, 0) != pdPASS)
            {
              //  trace0(1, SB_MONITOR, 1, "Err:PLDMResp timer stop fail");
                return;
            }
        }
    }
}

/******************************************************************************/
/** spdm_hb_response_timeout_start
* Start the software SPDMHB timer
* @param void
* @return void
*******************************************************************************/
void spdm_hb_response_timeout_start(void)
{
    spdmContext = spdm_ctxt_get();
    if (NULL != spdmContext)
    {
        if (NULL != spdmContext->xSPDMHBTimer)
        {
            if (xTimerStart(spdmContext->xSPDMHBTimer, 0) != pdPASS)
            {
               // trace0(1, SPDM_TSK, 1, "SHRTSR");
                return;
            }
        }
    }
}

/******************************************************************************/
/** spdm_hb_response_timeout_stop
* Stop the software SPDMHB timer
* @param void
* @return void
*******************************************************************************/
void spdm_hb_response_timeout_stop(void)
{
    spdmContext = spdm_ctxt_get();
    if (NULL != spdmContext)
    {
        if (NULL != spdmContext->xSPDMHBTimer)
        {
            if (xTimerStop(spdmContext->xSPDMHBTimer, 0) != pdPASS)
            {
               // trace0(1, SPDM_TSK, 1, "SHRTSO");
                return;
            }
        }
    }
}

/******************************************************************************/
/** spdm_ctxt_get()
* Get the SPDM Context
* @param void
* @return SPDM_context
*******************************************************************************/
PLDM_CONTEXT* pldm_ctxt_get(void)
{
    spdmContext = spdm_ctxt_get();
    if (NULL != spdmContext)
    {
        pldmContext = &spdmContext->pldm_context;
    }
    return pldmContext;
}