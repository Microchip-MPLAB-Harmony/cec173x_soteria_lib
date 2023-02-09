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

#ifndef MCTP_SMBUS_H
#define MCTP_SMBUS_H
#include "common.h"
#include "mctp_common.h"
#include "mctp_base.h"
#include "mctp.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Supported SMBus Speed*/
#define SMBUS_AT_400_KHZ    1U
#define SMBUS_AT_100_KHZ    0U

/* Config Which I2C channel*/
#define MCTP_SMBUS_CHANNEL  SMB_CHANNEL_3

/* Config Speed for Smbus either 400 or 100 Khz*/
#define SMBUS_SPEED_CONFIG  SMBUS_AT_400_KHZ

#define MCTP_SMBUS_AQ_TIMEOUT  100000U

/*MCTP Retry Couners */
#define MCTPSMBNKRET  0x08U
#define MCTPSMBLBRET  0x0FU
#define MCTPREQTOUT   0x0CU
#define MCTPREQRET    0x03U

#define INPUT_BUF_MAX_BYTES 1224U

/******************************************************************************/
/** Initializes mctp-smbus interface. It calls smb_slave _register for
* registration with smbus.
* @param void
* @return MCTP_TRUE if success, else MCTP_FALSE.
*******************************************************************************/
uint8_t mctp_smbus_init(void);

/******************************************************************************/
/** This is called when MCTP packet is to be transmitted over smbus.
* @param *tx_buf Pointer to TX packet buffer
* @return void
*******************************************************************************/
void mctp_transmit_smbus(MCTP_PKT_BUF *tx_buf);

/******************************************************************************/
/** This is called when packet is received over smbus.
* @param *buffer_info Pointer to I2C_BUFFER_INFO structure of smbus layer
* @param slaveTransmitFlag Slave Transmit Flag
* @return I2C_STATUS_BUFFER_DONE / I2C_STATUS_BUFFER_ERROR to smbus layer
*******************************************************************************/
uint8_t mctp_receive_smbus(I2C_BUFFER_INFO *buffer_info, uint8_t slaveTransmitFlag);

/******************************************************************************/
/** Once the packet trasmission is initiated over smbus, this function will be
* called by smbus layer to return the status code to mctp layer.
* Based on status code, mctp layer will schedule re-transmission of packet or
* drop the packet / mark buffer available.
* @param status Status code returned by smbus layer
* @param *buffer_ptr Pointer to packet buffer
* @param *newTxParams Pointer to structure for new SMBus Tx - Not Used
* @return Release or Retry code to smbus layer.
*******************************************************************************/
uint8_t mctp_smbmaster_done(uint8_t channel, uint8_t status, uint8_t *buffer_ptr, I2C_MAPP_CBK_NEW_TX *newTxParams);

/******************************************************************************/
/** This will be called when smbus tx status is success, nack retry exhaust,
* lab retry exhaust, bus error, or busy retry exhaust. This will configure
* buffer parameters and configure events based on packet type (req or response).
* @param *tx_buf Pointer to TX packet buffer
* @param pkt_type Packet type (request or response packet)
* @param status Smbus transaction status
* @return void
*******************************************************************************/
void mctp_smbdone_handler(MCTP_PKT_BUF *tx_buf);

/******************************************************************************/
/** This will be called when TX packet buffer is dropped or abandoned.
* This will clear buffer parameters to mark that TX buffer available.
* @param *tx_buf Pointer to TX packet buffer
* @return void
*******************************************************************************/
void mctp_smbdone_drop(MCTP_PKT_BUF *pkt_buf);

/******************************************************************************/
/** Once any handler writes valid packet in TX buffer; it will call this
* function to configure/initialize tx buffer parameters and handle tx state
* for scheduling packet transmission over smbus.
* @param *tx_buf Pointer to TX packet buffer
* @return void
*******************************************************************************/
void mctp_txpktready_init(MCTP_PKT_BUF *tx_buf);

/******************************************************************************/
/** This is called when packet received over smbus is targeted for EC.
* @param *buffer_info Pointer to I2C_BUFFER_INFO structure of smbus layer
* @return void
*******************************************************************************/
uint8_t mctp_copy_rxpkt_for_ec(I2C_BUFFER_INFO *buffer_info);

/******************************************************************************/
/** This is called by smbus module whenever SMBUS address is updated.
* @param smb_address - Bus address
* @return mctp_port - I2C controller port 
*******************************************************************************/
extern void mctp_smbaddress_update(uint8_t smb_address, uint8_t mctp_port);

/******************************************************************************/
/** This is called when packet received over smbus and the packet is 
* meant for SPDM or PLDM modules
* @param rx_packet_len - length of the received packet
* @param buffer_info - pointer to store the packetized data
* @param rx_buf - pointer to the received data
* @return void
*******************************************************************************/
uint8_t packetize_data(uint8_t rx_packet_len, I2C_BUFFER_INFO *buffer_info, MCTP_PKT_BUF *rx_buf);

#define SET_MCTP_EVENT_TASK(mctp)   SET_MCTP_EVENT_FLAG()

void SET_SPDM_EVENT_FLAG(void);
#define SET_EVENT_SPDM_TASK(spdm)   SET_SPDM_EVENT_FLAG()

void SET_PLDM_EVENT_FLAG(void);
#define SET_EVENT_PLDM_TASK(pldm)   SET_PLDM_EVENT_FLAG()

/**********************************************************************************************/
/** This is called when packet received over smbus is targeted for EC and message type is PLDM.
* @param *buffer_info Pointer to BUFFER_INFO structure of smbus layer
* @return void
***********************************************************************************************/
uint8_t mctp_copy_rx_for_pldm_for_ec(I2C_BUFFER_INFO *buffer_info);

/******************************************************************************/
/** mctp_otp_get_crisis_mode_smb_port();
* Get the crisis port SMBUS port from OTP
* @param None
* @return uint8_t -   returns 0xFF for invalid port,
*                      returns enum smb_ports for valid port select bits,
*******************************************************************************/
uint8_t mctp_otp_get_crisis_mode_smb_port(void);

/******************************************************************************/
/** This is called when packet received over smbus is targeted for EC and message type is for spdm.
* @param *buffer_info Pointer to BUFFER_INFO structure of smbus layer
* @return void
*******************************************************************************/
uint8_t mctp_copy_rx_for_spdm_for_ec(I2C_BUFFER_INFO *buffer_info);

#ifdef __cplusplus
}
#endif

#endif /*MCTP_SMBUS_H*/
/**   @}
 */
