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
#include "app.h"
#include "mctp.h"
#include "mctp_common.h"
#include "mctp_task.h"
#include "mctp_smbus.h"
#include "mctp_spt.h"
#include "mctp_config.h"
#include "../../../common/include/common.h"
#include "../../../common/include/FreeRTOS.h"

/* MPU */
extern MCTP_BSS_ATTR DI_CONTEXT_MCTP *mctp_di_context;
static void mctp_main(void *pvParameters);
MCTP_BSS1_ATTR static StaticTask_t mctp_task1_tcb;
static uint32_t mctp_task1_stack[MCTP_TASK1_STACK_WORD_SIZE] MCTP_STACK_ATTR MCTP_TASK1_STACK_ALIGN;
MCTP_BSS1_ATTR static TaskHandle_t mctp_task1_handle = NULL;
MCTP_BSS1_ATTR static MCTP_CONTEXT *mctpContext = NULL;
MCTP_BSS1_ATTR uint8_t is_attest_port_enabled; // Initialize attestation Port only once

static union
{
    uint32_t w[MCTP_TASK1_BUF_SIZE / 4];
    uint8_t  b[MCTP_TASK1_BUF_SIZE];
} mctp_task1_buf MCTP_BSS_ATTR_512ALIGNED MCTP_TASK1_BUF_ALIGN;

#define MCTP_TASK1_BUF_ADDR &mctp_task1_buf.w[0]

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
static const TaskParameters_t mctp_task1_def =
{
    .pcName = "mctp_main",
    .pxTaskBuffer = &mctp_task1_tcb
};

TaskHandle_t mctp_task1_get_handle(void)
{
    return mctp_task1_handle;
}

/****************************************************************/
/** mctp_task_create()
* Function to Create the freertos task for mctp
* @param  pvParams
* @return none
**********************************************************************/
#ifdef config_CEC_AHB_PROTECTION_ENABLE
int mctp_app_task_create(void *pvParams, CEC_AHB_PRIV_REGS_VALUES *pTaskPrivRegValues)
#else
int mctp_app_task_create(void *pvParams)
#endif
{
    //trace0(0, MCTP_TSK, 0, "mctp_tsk_create MPU");
    BaseType_t frc = pdFAIL;
    uintptr_t mctp_bss0_addr = mctp_bss0_base();
    size_t mctp_bss0sz = mctp_bss0_size();
    uintptr_t mctp_bss1_addr = mctp_bss1_base();
    size_t mctp_bss1sz = mctp_bss1_size();
    DI_CONTEXT_MCTP *di_context;

    /* Check if MPU base addresses are valid */
    configASSERT(mctp_bss0_addr);
    configASSERT(mctp_bss1_addr);

    /* Check if context size is greater than maximum buffer size for the task */
    configASSERT(MCTP_TASK1_BUF_SIZE > sizeof(MCTP_CONTEXT));

    TaskParameters_t td = mctp_task1_def;
    config_task_parameters(&td, mctp_main, MCTP_TASK1_STACK_WORD_SIZE, pvParams,
                           MCTP_TASK1_PRIORITY, mctp_task1_stack, pTaskPrivRegValues);
    config_task_memory_regions(&td, 0, mctp_bss0_addr, mctp_bss0sz, mctp_data_mpu_attr());
    config_task_memory_regions(&td, 1, mctp_bss1_addr, mctp_bss1sz, mctp_data_mpu_attr());

    frc = xTaskCreateRestrictedStatic(&td, &mctp_task1_handle);


    if (frc != pdPASS)
    {
        return -1;
    }

#if config_CEC_DATA_ISOLATION_CHECKS == 1
    di_checks_register_task(mctp_task1_handle, DI_APP_ID_MCTP);
#endif

    /* Create other FreeRTOS objects required by this task here */
    mctpContext = mctp_ctxt_get();
//    mctpContext->xmctp_EventGroupHandle = xEventGroupCreateStatic(&mctpContext->xmctp_CreatedEventGroup);
    di_context = (DI_CONTEXT_MCTP*)pvParams;
    mctpContext->xmctp_EventGroupHandle = di_context->mctp_request.evt_grp_handle;

    //trace0(0, MCTP_TSK, 0, "mctp_tsk_create: Done");

    return 0;
}

/******************************************************************************/
/** mctp_ctxt_get()
* Get the MCTP Context
* @param void
* @return MCTP_context
*******************************************************************************/
MCTP_CONTEXT* mctp_ctxt_get(void)
{
    MCTP_CONTEXT* ret_mctp_ctxt;

// coverity[misra_c_2012_rule_11_3_violation:FALSE]
    ret_mctp_ctxt = (MCTP_CONTEXT*)(MCTP_TASK1_BUF_ADDR);

    return ret_mctp_ctxt;
}

/******************************************************************************/
/** mctp_main()
* main process of MCTP task
* @param  pvParameters - Pointer that will be used as the parameter for the task
* being created.
* @return none
********************************************************************************/
static void mctp_main(void* pvParameters)
{
    EventBits_t uxBits;
    uint8_t i=0;
    mctp_di_init(pvParameters);
    mctpContext = mctp_ctxt_get();

    if(NULL == mctpContext)
    {
        return;
    }

    if(NULL == mctp_di_context)
    {
        //trace1(0, MCTP_TSK, 0, "[%s]:DI setup err", __FUNCTION__);
        return;
    }

#if config_CEC_DATA_ISOLATION_CHECKS == 1
    /* We release the semaphore here instead of inside di_mctp_init() */
    /* since a task has to be registered and the value of pxCurrentTCB */
    /* should point to the current task before data isolation cross checking */
    /* can be performed */
    for(i=0; i<DI_MAX_MCTP_APPS; i++)
    {
        xSemaphoreGive(mctp_di_context->mctp_master_context.mutex_handles[i]); /*make semaphore available for aquisition*/
    }
#endif

    mctp_init_task();

    while(true)
    {
        uxBits = xEventGroupWaitBits((mctpContext->xmctp_EventGroupHandle),
                                     (MCTP_EVENT_BIT | MCTP_I2C_ENABLE_BIT | MCTP_DI_EVENT_REQUEST | MCTP_I2C_DISABLE_BIT |
                                      MCTP_SPT_CTRL_BIT | SMB_DI_EVENT_RESPONSE | SPT_DI_EVENT_RESPONSE),
                                     pdTRUE,
                                     pdFALSE,
                                     portMAX_DELAY);

        if(MCTP_DI_EVENT_REQUEST == (uxBits & MCTP_DI_EVENT_REQUEST))
        {
            mctp_di_get_next_request();
        }

        if(MCTP_I2C_ENABLE_BIT == (uxBits & MCTP_I2C_ENABLE_BIT))
        {
            mctp_update_i2c_params(mctpContext);
            mctp_i2c_enable();
        }

        if(MCTP_I2C_DISABLE_BIT == (uxBits & MCTP_I2C_DISABLE_BIT))
        {
            mctp_i2c_disable();
        }

        if(MCTP_SPT_CTRL_BIT == (uxBits & MCTP_SPT_CTRL_BIT))
        {
            mctp_update_spt_params(mctpContext);
            mctp_spt_enable();
        }

        if(MCTP_EVENT_BIT == (uxBits & MCTP_EVENT_BIT))
        {
            mctp_event_task();
        }

        if(SMB_DI_EVENT_RESPONSE == (uxBits & SMB_DI_EVENT_RESPONSE))
        {
            mctp_di_get_smb_response();
        }

        if(SPT_DI_EVENT_RESPONSE == (uxBits & SPT_DI_EVENT_RESPONSE))
        {
            mctp_di_get_spt_response();
        }
    }
}

/****************************************************************/
/** SET_MCTP_EVENT_FLAG()
* Set event flag to trigger MCTP task to process
* @param  none
* @return none
**********************************************************************/
void SET_MCTP_EVENT_FLAG(void)
{
    mctpContext = mctp_ctxt_get();
    if(NULL == mctpContext)
    {
        return;
    }
    (void)xEventGroupSetBits( mctpContext->xmctp_EventGroupHandle, MCTP_EVENT_BIT );
}

void mctp_i2c_update(uint16_t slv_addr, uint8_t freq, uint8_t eid)
{
    mctpContext = mctp_ctxt_get();
    if(NULL == mctpContext)
    {
        return;
    }
    mctpContext->eid = eid;
    mctpContext->i2c_bus_freq = freq;
    mctpContext->i2c_slave_addr = slv_addr;
}

void mctp_spt_update(uint8_t channel, uint8_t io_mode, uint8_t spt_wait_time,
        uint8_t tar_time, uint8_t enable)
{
    mctpContext = mctp_ctxt_get();
    if(NULL == mctpContext)
    {
        return;
    }
    mctpContext->spt_enable = enable;
    mctpContext->spt_io_mode = io_mode;
    mctpContext->spt_wait_time = spt_wait_time;
    mctpContext->spt_channel = channel;
    mctpContext->spt_tar_time = tar_time;
}
void mctp_i2c_enable()
{
    UINT8 smb_status = 0x00;
    UINT16 smb_address = 0x00;
    UINT8 attestation_port_sel = 0x00;
    UINT8 valid_port = true;
    mctpContext = mctp_ctxt_get();
    if((NULL == mctpContext) || (is_attest_port_enabled == 1U)) // Initialize attestation Port only once
    {
        return;
    }

    attestation_port_sel = mctp_otp_get_crisis_mode_smb_port();

    switch(attestation_port_sel)
    {
    case SMB_PORT_0:
        /* configure GPIOs as I2C function */
        AHB_API_gpio_property_set( ATTESTATION_I2C00_SLAVE_CLK_LINE, GPIO_PROP_ALL, SMBus_CLK_LINE_CFG_1 );
        AHB_API_gpio_property_set( ATTESTATION_I2C00_SLAVE_DAT_LINE, GPIO_PROP_ALL, SMBus_DAT_LINE_CFG_1 );
        break;
    case SMB_PORT_4:
        /* configure GPIOs as I2C function */
        AHB_API_gpio_property_set( ATTESTATION_I2C04_SLAVE_CLK_LINE, GPIO_PROP_ALL, SMBus_CLK_LINE_CFG_1 );
        AHB_API_gpio_property_set( ATTESTATION_I2C04_SLAVE_DAT_LINE, GPIO_PROP_ALL, SMBus_DAT_LINE_CFG_1 );
        break;
    case SMB_PORT_6:
        /* configure GPIOs as I2C function */
        AHB_API_gpio_property_set( ATTESTATION_I2C06_SLAVE_CLK_LINE, GPIO_PROP_ALL, SMBus_CLK_LINE_CFG_1 );
        AHB_API_gpio_property_set( ATTESTATION_I2C06_SLAVE_DAT_LINE, GPIO_PROP_ALL, SMBus_DAT_LINE_CFG_1 );
        break;
    case SMB_PORT_9:
        /* configure GPIOs as I2C function */
        AHB_API_gpio_property_set( ATTESTATION_I2C09_SLAVE_CLK_LINE, GPIO_PROP_ALL, SMBus_CLK_LINE_CFG_1 );
        AHB_API_gpio_property_set( ATTESTATION_I2C09_SLAVE_DAT_LINE, GPIO_PROP_ALL, SMBus_DAT_LINE_CFG_1 );
        break;
    case SMB_PORT_10:
        /* configure GPIOs as I2C function */
        AHB_API_gpio_property_set( ATTESTATION_I2C10_SLAVE_CLK_LINE, GPIO_PROP_ALL, SMBus_CLK_LINE_CFG_1 );
        AHB_API_gpio_property_set( ATTESTATION_I2C10_SLAVE_DAT_LINE, GPIO_PROP_ALL, SMBus_DAT_LINE_CFG_1 );
        break;
    case SMB_PORT_15:
        /* configure GPIOs as I2C function */
        AHB_API_gpio_property_set( ATTESTATION_I2C15_SLAVE_CLK_LINE, GPIO_PROP_ALL, SMBus_CLK_LINE_CFG_1 );
        AHB_API_gpio_property_set( ATTESTATION_I2C15_SLAVE_DAT_LINE, GPIO_PROP_ALL, SMBus_DAT_LINE_CFG_1 );
        break;
    default:
        valid_port = false;
        break;
    }
    if(valid_port)
    {
        /* initialization specific to smbus */
        /* For data isolation - Register slave application function with smbus driver not required */
        // smb_status = mctp_smbus_init();

        smb_address = mctpContext->i2c_slave_addr;
        mctp_smbaddress_update(smb_address, attestation_port_sel, 1);
        is_attest_port_enabled = 1U;
    }
}

/****************************************************************/
/** Disable I2C attestation port
* @param  none
* @return none
**********************************************************************/
void mctp_i2c_disable(void)
{
    UINT16 smb_address = 0x00;
    UINT8 attestation_port_sel = 0x00;
    mctpContext = mctp_ctxt_get();

    if(NULL == mctpContext || (is_attest_port_enabled == 0U))
    {
        return;
    }

    smb_address = mctpContext->i2c_slave_addr;
    attestation_port_sel = mctp_otp_get_crisis_mode_smb_port();
    mctp_smbaddress_update(smb_address, attestation_port_sel, 0);
    is_attest_port_enabled = 0;
}

void sb_mctp_i2c_enable()
{
    mctpContext = mctp_ctxt_get();
    if(NULL == mctpContext)
    {
        return;
    }
    xEventGroupSetBits( mctpContext->xmctp_EventGroupHandle, MCTP_I2C_ENABLE_BIT );
}

void sb_mctp_spt_enable()
{
    mctpContext = mctp_ctxt_get();
    if(NULL == mctpContext)
    {
        return;
    }
    xEventGroupSetBits( mctpContext->xmctp_EventGroupHandle, MCTP_SPT_CTRL_BIT);
}

/****************************************************************/
/** mctp_task_create()
* Function to Create the MCTP task for smbus
* @param  pvParams
* @return none
**********************************************************************/
#ifdef config_CEC_AHB_PROTECTION_ENABLE
int mctp_task_create(void *pvParams, CEC_AHB_PRIV_REGS_VALUES *pTaskPrivRegValues)
#else
int mctp_task_create(void *pvParams)
#endif
{
    void *di_ctxt;
    int return_val =0u;

    di_ctxt = di_mctp_context_get();

#ifdef config_CEC_AHB_PROTECTION_ENABLE
    return_val = mctp_app_task_create(di_ctxt, pTaskPrivRegValues);
#else
    return_val = mctp_app_task_create(di_ctxt);
#endif

    return return_val;
}