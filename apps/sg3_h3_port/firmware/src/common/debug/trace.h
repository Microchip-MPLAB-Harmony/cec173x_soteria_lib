/*
 **********************************************************************************
* © 2013 Microchip Technology Inc. and its subsidiaries.
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
 **********************************************************************************
 *  DEBUG.H
 *      This is the debug message output definition header
 **********************************************************************************
 *  SMSC version control information (Perforce):
 *
 *  FILE:     $File: //depot_pcs/FWEng/solutions/Glacier_GEN3/maincodeline/Harmony/secureboot_app_library/sg3_h3_port/firmware/src/common/debug/trace.h $
 *  REVISION: $Revision: #3 $
 *  DATETIME: $DateTime: 2023/01/10 04:01:19 $
 *  AUTHOR:   $Author: i64652 $
 *
 *  Revision history (latest first):
 *      #xx
 ***********************************************************************************
 */

/** @defgroup DEBUG DEBUG
 *  @{
 */

/** @file DEBUG.h
* \brief Debugger header file
* \author FW Team
* 
* This file contains all function declarations in DEBUG.c
******************************************************************************/
#ifndef TRACE_H
#define TRACE_H

#include <stdarg.h>

void tracex(const char *fmt, ...);
void tracex_from_ISR(const char *fmt, ...);
void print_buf(uint8_t *buf, uint32_t len);

#endif /* #ifndef TRACE_H */

/**   @}
 */

