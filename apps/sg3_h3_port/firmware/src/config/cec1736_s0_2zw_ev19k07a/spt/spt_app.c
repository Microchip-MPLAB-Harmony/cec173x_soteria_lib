/****************************************************************************
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

#include "spt_app.h"
#include "FreeRTOS.h"
#include "spt/spt_task.h"
#include "pmci.h"

extern SPT_BSS_ATTR DI_QUEUEITEM_SPT* spt_tx_resp_queueitem;

uint16_t spt_get_current_timestamp(void) {
    return (uint16_t) (xTaskGetTickCount() / 10);
 
}

void spt_event_handle(EventBits_t uxBits)
{
   // if((SPT_ISR_EVENT_BIT == uxBits) || (SPT_EVENT_BIT == uxBits))
    {
        spt_write_event_handler();
        spt_read_event_handler();        
    }
}

uint8_t spt_tx(uint8_t channel, uint8_t* buff_ptr, uint16_t writecount, uint8_t pecEnable, TX_FUNC_PTR func_ptr)
{
    spt_write(channel, buff_ptr, writecount, pecEnable, func_ptr);
}

uint8_t spt_tx_done_callback(uint8_t channel, uint8_t status, TX_FUNC_PTR func_ptr, uint8_t* app_buff)
{
    if(NULL != spt_tx_resp_queueitem)
    {
        spt_tx_resp_queueitem->item_details.res_spt_tx.channel = channel;
        spt_tx_resp_queueitem->item_details.res_spt_tx.status = status;
        spt_di_tx_callback(spt_tx_resp_queueitem);
    }
    
}


void spt_raise_interrrupt_event()
{
    spt_set_event_bits_isr(SPT_ISR_EVENT_BIT);
}

void spt_event_trigger()
{
    spt_set_event_bits(SPT_EVENT_BIT);
}
