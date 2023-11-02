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

#ifndef MCTP_SPT_H
#define MCTP_SPT_H

#include "mctp_common.h"
#include "mctp_base.h"
#include "mctp.h"

#define MCTP_SPT_AQ_TIMEOUT  100000U

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/** Initializes mctp-spt interface. It calls spt_slave _register for
* registration with spt.
* @param void
* @return MCTP_TRUE if success, else MCTP_FALSE.
*******************************************************************************/
uint8_t mctp_spt_init(void);

/******************************************************************************/
/** This is called when MCTP packet is to be transmitted over spt.
* @param *tx_buf Pointer to TX packet buffer
* @return void
*******************************************************************************/
void mctp_transmit_spt(MCTP_PKT_BUF *tx_buf);

/******************************************************************************/
/** This is called when packet is received over spt.
* @param *buffer_info Pointer to MCTP_BUFFER_INFO structure of spt layer
* @param slaveTransmitFlag Slave Transmit Flag
* @return SPT_STATUS_BUFFER_DONE / SPT_STATUS_BUFFER_ERROR to SPT layer
*******************************************************************************/
uint8_t mctp_receive_spt(MCTP_BUFFER_INFO *buffer_info, uint8_t slaveTransmitFlag);

/******************************************************************************/
/** Once the packet trasmission is initiated over spt, this function will be
* called by spt layer to return the status code to mctp layer.
* Based on status code, mctp layer will schedule re-transmission of packet or
* drop the packet / mark buffer available.
* @param status Status code returned by spt layer
* @param *buffer_ptr Pointer to packet buffer
* @param *newTxParams Pointer to structure for new spt Tx - Not Used
* @return Release or Retry code to spt layer.
*******************************************************************************/
uint8_t mctp_spt_tx_done(uint8_t channel, uint8_t status, uint8_t *buffer_ptr, SPT_MAPP_CBK_NEW_TX *newTxParams);

/******************************************************************************/
/** This will be called when spt tx status is success, nack retry exhaust,
* lab retry exhaust, bus error, or busy retry exhaust. This will configure
* buffer parameters and configure events based on packet type (req or response).
* @param *tx_buf Pointer to TX packet buffer
* @param pkt_type Packet type (request or response packet)
* @param status spt transaction status
* @return void
*******************************************************************************/
void mctp_sptdone_handler(MCTP_PKT_BUF *tx_buf);

/******************************************************************************/
/** This will be called when TX packet buffer is dropped or abandoned.
* This will clear buffer parameters to mark that TX buffer available.
* @param *tx_buf Pointer to TX packet buffer
* @return void
*******************************************************************************/
void mctp_sptdone_drop(MCTP_PKT_BUF *pkt_buf);

/******************************************************************************/
/** Once any handler writes valid packet in TX buffer; it will call this
* function to configure/initialize tx buffer parameters and handle tx state
* for scheduling packet transmission over spt.
* @param *tx_buf Pointer to TX packet buffer
* @return void
*******************************************************************************/
void mctp_spt_txpktready_init(MCTP_PKT_BUF *tx_buf);

/******************************************************************************/
/** This is called by MCPT module whenever spt is enabled. 
*******************************************************************************/
extern void mctp_spt_enable();

#ifdef __cplusplus
}
#endif

#endif /*MCTP_SMBUS_H*/
/**   @}
 */
