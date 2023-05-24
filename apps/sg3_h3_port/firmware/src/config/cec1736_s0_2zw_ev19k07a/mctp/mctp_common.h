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

#ifndef MCTP_COMMON_H    /* Guard against multiple inclusion */
#define MCTP_COMMON_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include "definitions.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

#define MCTP_BSS_ATTR                                     __attribute__((section("mctp_bss")))

/* -------------------- SMB Protocol Type --------------------------------------
   SMB table PROTOCOL TYPE

   Bit 7,6 - 00 = No Operation.
             01 = Write number of bytes in SMB buffer [1].
             10 = Write number of bytes in SMB buffer [1]
                  then read  number of bytes in bits 5 - 0.
             11 = Write number of bytes in SMB buffer [1]
                  then do repeated start condition, then
                  read number of bytes in bits 5 - 0.

   Bit 5 - 0    = Number of read bytes:
                  0 - 3E : read fixed number of bytes from 0 - 3Eh
                  3F     : read number of bytes indicated by first
                           byte of return string from SMB device. */
#define I2C_OPER_NOP                                      0U
#define I2C_OPER_WRITE                                    1U
#define I2C_OPER_READ                                     2U
#define I2C_OPER_REPEAT                                   3U

#define SMB_PROTO_READ_BYTE                               ((I2C_OPER_REPEAT << 6) + 0x01U)
#define SMB_PROTO_READ_WORD                               ((I2C_OPER_REPEAT << 6) + 0x02U)
#define SMB_PROTO_PROCESS_CALL                            ((I2C_OPER_REPEAT << 6) + 0x02U)
#define SMB_PROTO_WRITE_BLOCK                             ((I2C_OPER_WRITE  << 6) + 0x00U)
#define SMB_PROTO_READ_BLOCK                              ((I2C_OPER_REPEAT << 6) + 0x3FU)
#define SMB_PROTO_BLOCK_WRITE_BLOCK_READ_PROCESS_CALL     ((I2C_OPER_REPEAT << 6) + 0x3FU)

#define is_add_safe(sum, aug_or_add)                     ((sum) < (aug_or_add) ? 0 : 1) // Coverity INT30-C Postcondition Test
#define is_sub_safe(ui_a, ui_b)                          ((ui_a) < (ui_b) ? 0 : 1) // Coverity INT30-C Precondition Test

/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* MCTP_COMMON_H */

/* *****************************************************************************
 End of File
 */
