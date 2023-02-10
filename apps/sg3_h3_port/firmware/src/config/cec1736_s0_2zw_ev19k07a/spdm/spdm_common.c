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
#include "spdm_common.h"

/******************************************************************************/
/** safe_subraction_16
* Process safe subtraction of two variables of uint16
* @param uint16_t  - operand 1
* @param uint16_t  - operand 2
* @param uint16_t* - resultant pointer
* @return uint8_t  - 0 success
*******************************************************************************/
uint8_t safe_subraction_16(uint16_t minuend, uint16_t subtrahend, uint16_t * rslt)
{
    uint8_t retVal = 1;
    if (minuend >= subtrahend)
    {
        if(rslt != NULL)
        {
            *rslt = minuend - subtrahend;
        } // else ignore the result since it is not required.
        retVal = 0u;
    }
    return retVal;
}