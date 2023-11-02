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

#ifndef SPT_APP_H
#define SPT_APP_H

#include <stddef.h>
#include <stdint.h>
#include "spt/spt_drv.h"


//uint8_t spt_init_advanced_single_mode();
//uint8_t spt_init_advanced_quad_mode();

void spt_event_handle(EventBits_t uxBits);

uint8_t spt_register_slave(uint8_t channel, SPT_SLAVE_FUNC_PTR slaveFuncPtr);

uint16_t spt_get_current_timestamp(void);

void spt_channel_enable(uint8_t channel, uint8_t io_mode, uint8_t spt_wait_time,
        uint8_t tar_time);

void spt_channel_disable(uint8_t channel);

uint8_t spt_tx(uint8_t channel, uint8_t* buff_ptr, uint16_t writecount, uint8_t pecEnable, TX_FUNC_PTR func_ptr);

uint8_t spt_tx_done_callback(uint8_t channel, uint8_t status, TX_FUNC_PTR func_ptr, uint8_t* app_buff);

void spt_raise_interrrupt_event();

void spt_event_trigger();
#endif /* SPT_APP_H */