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

#ifndef APP_PLDM_H
#define APP_PLDM_H

#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************/
/** pldm_get_staged_address_for_crisis_recovery();
* PLDMResp timer callback
* @param TimerHandle_t pxTimer
* @return None
*******************************************************************************/
void pldm_get_staged_address_for_crisis_recovery(uint16_t comp_identifier);

/******************************************************************************/
/** This function returns the AP, COMP, HT_NUM based on PLDM component identifier
* @param component_identifier - PLDM Component identifier
* @param ap - to return the AP
* @param comp - to return the Component
* @param ht_num - to return the HT_NUM
* @return void
*******************************************************************************/
void sb_get_apx_compx_htnum_based_on_comp_identifier(uint16_t component_identifier, uint8_t *ap, uint8_t *comp,
        uint8_t *ht_num);

/******************************************************************************/
/** This is called for starting EC_FW/AP_CFG update process
* EC_FW/AP_CFG is copied to staged location before this function is called, and
* reusing EC_FW update and reboot I2C command code.
* @param comp_identifier
* @return void
*******************************************************************************/
void pldm_start_update(uint16_t comp_identifier);        

#ifdef __cplusplus
}
#endif

#endif