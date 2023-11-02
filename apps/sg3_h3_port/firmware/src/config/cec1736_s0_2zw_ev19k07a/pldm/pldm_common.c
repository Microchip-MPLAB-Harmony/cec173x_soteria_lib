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

#include "pldm_common.h"

/** timer_delay - 
 * @param delay in timer count units
 * @notes Input Capture Compare free run counter is operating 
 * at (3 MHz +/- 2%) [2.94, 3.06] MHz, [0.340, 0.327] us.
 * Assume +2% to insure delays meet the minimum.
 * 1 us = 1/0.327 = 3.06 counts
 * num_us = num_us * 3 = num_us * (2+1) = (num_us << 1) + num_us
 */
void timer_delay(uint32_t counts_to_delay)
{ 
    volatile uint32_t diff, curr_cnt;
    uint32_t  start_cnt;
    
    if (counts_to_delay) {
        
        start_cnt = CCT_REGS->CCT_FREE_RUN;
        
        do {
            curr_cnt = CCT_REGS->CCT_FREE_RUN;
            if (curr_cnt >= start_cnt) {
                diff = curr_cnt - start_cnt;
            } else {
                diff = curr_cnt + (0xfffffffful - start_cnt);
            }
        } while (diff < counts_to_delay);
    }
}


/* Input Capture Compare free run counter is operating at (3 MHz +/- 2%)
 * [2.94, 3.06] MHz, [0.340, 0.327] us
 * Assume +2% to insure delays meet the minimum.
 * 1 ms = 3060 (0x0BF4) counts
 * count = ms * 3060 = ms * (3072 - 12) = ms * (0xC00 - 0x0C)
 * = ms * (0x800 + 0x400 - (0x08 + 0x04))
 * = (ms * 0x800) + (ms * 0x400) - (ms * 0x08) - (ms * 0x04)
 * = (ms << 11) + (ms << 10) - (ms << 3) - (ms << 2)
 */
void timer_delay_ms(uint32_t num_ms)
{
    uint32_t counts = (num_ms << 11ul) + (num_ms << 10ul) - (num_ms << 3ul) - (num_ms << 2ul);
    
    timer_delay(counts);

    return;
}

void timer_delay_us(uint32_t num_us)
{
    uint32_t counts = num_us + (num_us << 1ul) + (num_us >> 4ul);
    
    timer_delay(counts);

    return;
}