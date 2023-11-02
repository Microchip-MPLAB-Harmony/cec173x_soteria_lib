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

#ifndef SPT_TASK_H
#define SPT_TASK_H

#include <stddef.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "spt_drv.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MPU */
#define configSPT_TASK1_PRIORITY 6
#define SPT_TASK1_PRIORITY ((tskIDLE_PRIORITY + configSPT_TASK1_PRIORITY) % configMAX_PRIORITIES)
/* Stack size must be a power of 2 if the task is restricted */
#define SPT_TASK1_STACK_SIZE 1024U       // 2 * configMINIMAL_STACK_SIZE (120)
#define SPT_TASK1_STACK_WORD_SIZE ((SPT_TASK1_STACK_SIZE) / 4U)
    
#define SPT_TASK1_STACK_ALIGN __attribute__ ((aligned(SPT_TASK1_STACK_SIZE)))

#define SPT_TASK1_BUF_SIZE		256U
#define SPT_TASK1_BUF_MPU_ATTR 0U
#define SPT_TASK1_BUF_ALIGN __attribute__((aligned(SPT_TASK1_BUF_SIZE)))

#define SPT_EVENT_BIT          (1U << 0)
#define SPT_ISR_EVENT_BIT          (1U << 1)
    
typedef struct SPT_CONTEXT_PER_CHANNEL
{     
    uint8_t io_mode;

    uint8_t wait_time;

    uint8_t tar_time;

    uint8_t* txappbuff_ptr;
    
    uint16_t writecount;
    
    uint8_t pecEnable;
    
    TX_FUNC_PTR txfunc_ptr;
    
    SPT_SLAVE_FUNC_PTR slaveFuncPtr;
    
}SPT_CONTEXT_PER_CHANNEL;

typedef struct SPT_CONTEXT_STRUCT
{
         
    SPT_CONTEXT_PER_CHANNEL sptCtxt0;
    
    SPT_CONTEXT_PER_CHANNEL sptCtxt1;
    /* Event group handle */
    EventGroupHandle_t xspt_EventGroupHandle;

    /* Event group buffer*/
    StaticEventGroup_t xspt_CreatedEventGroup;
}SPT_CONTEXT;

/* Events that smbus driver thread waits on (event_flags_2_smbus) */
#define SPT_EVENT_ISR BIT(0)
#define SPT_TASK_EVENT   BIT(0)
#define SPT_EVENT_TIMER   BIT(1)
#define SPT_EVENT_MSG_QUEUE BIT(3)


/****************************************************************/
/** SET_SPT_EVENT_FLAG
* Set event flag to trigger SPT task to process
* @param  void
* @return void
*****************************************************************/
void SET_SPT_EVENT_FLAG(void);

/****************************************************************/
/** SET_SPT_ENABLE_EVENT
* Enable SPT  module
* @param void
* @return void
*****************************************************************/
void SET_SPT_ENABLE_EVENT(void);

/****************************************************************/
/** spt_app_task_create()
* Function to Create the freertos task for spt
* @param  pvParams
* @return none
*****************************************************************/
#ifdef config_CEC_AHB_PROTECTION_ENABLE
int spt_app_task_create(void *pvParams, CEC_AHB_PRIV_REGS_VALUES *pTaskPrivRegValues);
#else
int spt_app_task_create(void *pvParams);
#endif

/****************************************************************/
/** spt_task_get_handle
* Get the SPT task handle
* @param  void
* @return TaskHandle_t - SPT task handle
*****************************************************************/
TaskHandle_t spt_task_get_handle(void);

/****************************************************************/
/** spt_ctxt_get()
* Get the SPT Context
* @param void
* @return SPT_context
*****************************************************************/
SPT_CONTEXT* spt_ctxt_get(void);

SPT_CONTEXT_PER_CHANNEL* spt_channel_ctxt_get(uint8_t channel);

void spt_set_event_bits(EventBits_t uxBitsToSet);

void spt_set_event_bits_isr(EventBits_t uxBitsToSet);

/******************************************************************************/
/** spt_task_wait_event_bits();
* Wait for bit set from MCTP, bit is set once MCTP receives and transmits to application
* layer
* @param event_bits - Event bits to be waited
* @return uint32_t event bits waited or Timeout
*******************************************************************************/
uint32_t spt_task_wait_event_bits(uint32_t event_bits);

/****************************************************************/
/** smbus_task_create()
* Function to Create the freertos task for smbus
* @param  pvParams
* @return none
**********************************************************************/
#ifdef config_CEC_AHB_PROTECTION_ENABLE
int spt_task_create(void *pvParams, CEC_AHB_PRIV_REGS_VALUES *pTaskPrivRegValues);
#else
int spt_task_create(void *pvParams);
#endif

#ifdef __cplusplus
}
#endif

#endif /* SPT_TASK_H */
