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

#include "definitions.h"

#include "mctp.h"
#include "mctp_common.h"
#include "mctp_base.h"
#include "mctp_smbus.h"
#include "mctp_control.h"
#include "trace.h"
#include "mctp_task.h"
#include "mctp_config.h"
#include "../common/include/common.h"

extern MCTP_BSS_ATTR UINT8 mctp_tx_state;
extern MCTP_BSS_ATTR_8ALIGNED MCTP_PKT_BUF mctp_pktbuf[MCTP_PKT_BUF_NUM]__attribute__ ((aligned(8)));

extern MCTP_BSS_ATTR UINT8 mctp_wait_smbus_callback;
extern MCTP_BSS_ATTR UINT8 is_pldm_request_firmware_update;
extern MCTP_BSS_ATTR UINT8 msg_type_tx; // pldm or spdm or mctp - when transmitting multiple/single pkt through smbus

/******************************************************************************/
/** Initializes mctp-smbus interface. It calls smb_slave _register for
* registration with smbus.
* @param void
* @return MCTP_TRUE if success, else MCTP_FALSE.
*******************************************************************************/
uint8_t mctp_smbus_init(void)
{
    uint8_t ret_val;
    uint8_t status_init;

    ret_val = (uint8_t)MCTP_FALSE;

    /* register with smbus */
    status_init = smb_register_slave(MCTP_I2C_CHANNEL,
                                     (SLAVE_FUNC_PTR )mctp_receive_smbus);

    /* smbus slave registration successful */
    if(status_init == (uint8_t)I2C_SLAVE_APP_STATUS_OK)
    {
        ret_val = (uint8_t)MCTP_TRUE;
    }

    return ret_val;

} /* End mctp_smbus_init() */

/******************************************************************************/
/** This is called when packet is received over smbus.
* @param *buffer_info Pointer to MCTP_BUFFER_INFO structure of smbus layer
* @param slaveTransmitFlag Slave Transmit Flag
* @return I2C_STATUS_BUFFER_DONE / I2C_STATUS_BUFFER_ERROR to smbus layer
*******************************************************************************/
#define SMB_DEBUG   1
uint8_t mctp_receive_smbus(MCTP_BUFFER_INFO *buffer_info, uint8_t slaveTransmitFlag)
{
    uint8_t pkt_valid;
    uint8_t pkt_len;
    MCTP_PKT_BUF *pkt_buf;
    MCTP_PKT_BUF *spdm_msg_rx_buf = NULL;
    MCTP_PKT_BUF *pldm_msg_rx_buf = NULL;
    pkt_buf = (MCTP_PKT_BUF *)((void *)&buffer_info->buffer_ptr[0]);

    /* if PEC not valid, drop packet */
    if ((bool)buffer_info->pecFlag == MCTP_FALSE)
    {
        /* inform smbus layer to free it's buffer */
        return (uint8_t)I2C_STATUS_BUFFER_DONE;
    }
    /* Check Total packet received if more than 73 Bytes*/
    if(buffer_info->DataLen > (MCTP_PACKET_MAX))
    {
        return (uint8_t)I2C_STATUS_BUFFER_ERROR;
    }

    /* check if MCTP packet */
    if ((pkt_buf->pkt.field.hdr.cmd_code != MCTP_SMBUS_HDR_CMD_CODE) ||
            (pkt_buf->pkt.field.hdr.rw_dst != 0U) ||
            (pkt_buf->pkt.field.hdr.ipmi_src != 1U) ||
            (pkt_buf->pkt.field.hdr.byte_cnt < MCTP_BYTECNT_MIN_INCLUDING_MSG_TYPE) ||
            (buffer_info->DataLen < MCTP_PACKET_MIN))
    {
        return (uint8_t)I2C_STATUS_BUFFER_ERROR;
    }

    pkt_len = ( buffer_info->buffer_ptr[MCTP_PKT_BYTE_CNT_POS] + \
                MCTP_BYTECNT_OFFSET + \
                MCTP_PEC_BYTE );

    if(buffer_info->DataLen != pkt_len)
    {
        return (uint8_t)I2C_STATUS_BUFFER_ERROR;
    }

    /* MCTP doesn't support slave transmit protocol, drop packet */
    if ((bool)slaveTransmitFlag == I2C_SLAVE_TRANSMIT_TRUE)
    {
        /* inform smbus layer to free it's buffer */
        return (uint8_t)I2C_STATUS_BUFFER_DONE;
    }

    /* check validation of received packet */
    pkt_valid = mctp_packet_validation(buffer_info->buffer_ptr);

    /* if received packet is found valid */
    if(MCTP_TRUE == (bool)pkt_valid)
    {
        /* call mctp packet routing function */
        if(0U != mctp_packet_routing(buffer_info))
        {
            return (uint8_t)I2C_STATUS_BUFFER_ERROR;
        }
    }
    else
    {
        spdm_msg_rx_buf = (MCTP_PKT_BUF *) &mctp_pktbuf[MCTP_BUF3];
        memset(spdm_msg_rx_buf, 0, MCTP_PKT_BUF_DATALEN);
        spdm_msg_rx_buf->buf_full = (uint8_t)MCTP_EMPTY;
        pldm_msg_rx_buf = (MCTP_PKT_BUF *) &mctp_pktbuf[MCTP_BUF4];
        memset(pldm_msg_rx_buf, 0, MCTP_PKT_BUF_DATALEN);
        pldm_msg_rx_buf->buf_full = MCTP_EMPTY;
        return (uint8_t)I2C_STATUS_BUFFER_ERROR;
    }

    /* inform smbus layer to free it's buffer */
    return (uint8_t)I2C_STATUS_BUFFER_DONE;

} /* End mctp_receive_smbus() */


/******************************************************************************/
/** This is called when MCTP packet is to be transmitted over smbus.
* @param *tx_buf Pointer to TX packet buffer
* @return void
*******************************************************************************/
void mctp_transmit_smbus(MCTP_PKT_BUF *tx_buf)
{
    uint8_t status_tx;

    if (tx_buf == NULL)
    {
        return;
    }

    status_tx = mctp_di_smb_protocol_execute(MCTP_SMBUS_CHANNEL,
                (UINT8 *)&tx_buf->pkt,
                SMB_WRITE_BLOCK,
                tx_buf->pkt.data[MCTP_PKT_BYTE_CNT_POS] + 3,
                TRUE);

} /* End mctp_transmit_smbus() */

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
uint8_t mctp_smbmaster_done(uint8_t channel, uint8_t status, uint8_t *buffer_ptr, I2C_MAPP_CBK_NEW_TX *newTxParams)
{
    MCTP_PKT_BUF *tx_buf;
    uint8_t ret_val;
    uint8_t spdm_pend = 0x00;
	uint8_t pldm_pend = 0x00;
    MCTP_TX_CXT *mctp_tx_ctxt = NULL;
    /* get current TX buffer pointer */
    tx_buf = (MCTP_PKT_BUF *)((void *) buffer_ptr);

    mctp_tx_ctxt = mctp_msg_tx_ctxt_lookup(tx_buf->pkt.field.hdr.src_eid,tx_buf->pkt.field.hdr.dst_eid,
                                tx_buf->pkt.field.hdr.msg_tag);
    if (mctp_tx_ctxt != NULL) {
        msg_type_tx = mctp_tx_ctxt->message_type;
    }
    
    if((msg_type_tx == MCTP_IC_MSGTYPE_SPDM) || (msg_type_tx == MCTP_IC_MSGTYPE_SECURED_MESSAGE))
    {
        if((buffer_ptr[MCTP_PKT_TO_MSGTAG_POS] & MCTP_EOM_REF_MSK) != MCTP_EOM_REF)
        {
            spdm_pend = true;
        }
        else
        {
            spdm_pend = false;
        }
    }
    else if (msg_type_tx == MCTP_IC_MSGTYPE_PLDM)
    {
        if((buffer_ptr[MCTP_PKT_TO_MSGTAG_POS] & MCTP_EOM_REF_MSK) != MCTP_EOM_REF)
        {
            pldm_pend = true;
        }
        else
        {
            pldm_pend = false;
        }
    }
    /* based on status code, schedule re-transmission of packet or
    * drop the packet / mark buffer available */
    switch (status)
    {
    /* packet was transmitted successfully over smbus */
    case (uint8_t)I2C_SUCCESS_TX:
        /* update buffer parameters and configure events */
        mctp_smbdone_handler(tx_buf);

        mctp_wait_smbus_callback = 0x0;

        ret_val = (uint8_t)I2C_APP_RETVAL_RELEASE_SMBUS;

        /* change state to search valid tx buffer */
        mctp_tx_state = (uint8_t)MCTP_TX_NEXT;

        /* set mctp event task */
        SET_MCTP_EVENT_TASK(mctp);

        break;

    /* lost arbitration */
    case (uint8_t)I2C_ERROR_LAB:
        /* increment lab retry count */
        tx_buf->smbus_lab_retry_count++;

        /* if lost arbitration retry count is exhausted */
        if(tx_buf->smbus_lab_retry_count > MCTPSMBLBRET)
        {
            /* update buffer parameters and configure events */
            mctp_smbdone_handler(tx_buf);

            mctp_wait_smbus_callback = 0x0;

            /* return release code to smbus layer */
            ret_val = (uint8_t)I2C_APP_RETVAL_RELEASE_SMBUS;

            /* change state to search valid tx buffer */
            mctp_tx_state = (uint8_t)MCTP_TX_NEXT;

            /* set mctp event task */
            SET_MCTP_EVENT_TASK(mctp);
        }
        /* else if lost arbitration retry count is not exhausted */
        else
        {
            ret_val = (uint8_t)I2C_APP_RETVAL_RETRY;
        }

        break;

    /* nack condition */
    case (uint8_t)I2C_ERROR_MADDR_NAKX:
    case (uint8_t)I2C_ERROR_MDATA_NAKX:
        /* increment nack retry count */
        tx_buf->smbus_nack_retry_count++;

        /* reset lost arbitration count at every nack */
        tx_buf->smbus_lab_retry_count = 0;

        /* if nack retry count is exhausted */
        if(tx_buf->smbus_nack_retry_count > MCTPSMBNKRET)
        {
            /* update buffer parameters and configure events */
            mctp_smbdone_handler(tx_buf);

            mctp_wait_smbus_callback = 0x0;

            /* return release code to smbus layer */
            ret_val = (uint8_t)I2C_APP_RETVAL_RELEASE_SMBUS;

            /* change state to search valid tx buffer */
            mctp_tx_state = (uint8_t)MCTP_TX_NEXT;

            /* set mctp event task */
            SET_MCTP_EVENT_TASK(mctp);
        }
        /* else if nack retry count is not exhausted */
        else
        {
            ret_val = (uint8_t)I2C_APP_RETVAL_RETRY;
        }

        break;

    /* bus error or unknown condition */
    case (uint8_t)I2C_ERROR_BER_TIMEOUT:
    case (uint8_t)I2C_ERROR_BER_NON_TIMEOUT:
    default:
        /* update buffer parameters and configure events */
        mctp_smbdone_handler(tx_buf);

        mctp_wait_smbus_callback = 0x0;

        /* return release code to smbus layer */
        ret_val = (uint8_t)I2C_APP_RETVAL_RELEASE_SMBUS;

        /* change state to search valid tx buffer */
        mctp_tx_state = (uint8_t)MCTP_TX_NEXT;

        /* set mctp event task */
        SET_MCTP_EVENT_TASK(mctp);

        break;

    } /* end of switch */

    //trace1(0, MCTP, 0, "mctp_smbmaster_done: mctp_tx_state = %04Xh", mctp_tx_state);
    //trace1(0, MCTP, 0, "mctp_smbmaster_done: ret_val = %04Xh", ret_val);
    //trace0(0, MCTP, 0, "mctp_smbmaster_done: End");

    if(spdm_pend)//if eom is not 1 for the spdm messaging, trigger spdm for further transmission over mctp
    {
        trigger_spdm_res_event();
    }

    if (is_pldm_request_firmware_update)
    {
        trigger_pldm_res_event();
    }

    if (pldm_pend)
    {
        trigger_pldm_res_event();
    }
    return ret_val;

}  /* End mctp_smbmaster_done */

/******************************************************************************/
/** This will be called when smbus tx status is success, nack retry exhaust,
* lab retry exhaust, bus error, or busy retry exhaust. This will configure
* buffer parameters and configure events based on packet type (req or response).
* @param *tx_buf Pointer to TX packet buffer
* @param pkt_type Packet type (request or response packet)
* @param status Smbus transaction status
* @return void
*******************************************************************************/
void mctp_smbdone_handler(MCTP_PKT_BUF *tx_buf)
{
    /* drop packet buffer */
    mctp_smbdone_drop(tx_buf);
} /* End mctp_smbdone_handler */

/******************************************************************************/
/** This will be called when TX packet buffer is dropped or abandoned.
* This will clear buffer parameters to mark that TX buffer available.
* @param *tx_buf Pointer to TX packet buffer
* @return void
*******************************************************************************/
void mctp_smbdone_drop(MCTP_PKT_BUF *pkt_buf)
{
    /* packet is dropped; mark that buffer available */
    pkt_buf->buf_full = (uint8_t)MCTP_EMPTY;
    /* clear buffer parameters */
    pkt_buf->smbus_nack_retry_count = 0;
    pkt_buf->smbus_lab_retry_count = 0;
    pkt_buf->smbus_acquire_retry_count = 0;
    pkt_buf->request_tx_retry_count = 0;
    pkt_buf->request_per_tx_timeout_count = 0;
    pkt_buf->rx_timestamp = 0;
} /* End mctp_smbdone_drop */

/******************************************************************************/
/** Once any handler writes valid packet in TX buffer; it will call this
* function to configure/initialize tx buffer parameters and handle tx state
* for scheduling packet transmission over smbus.
* @param *tx_buf Pointer to TX packet buffer
* @return void
*******************************************************************************/
void mctp_smbus_txpktready_init(MCTP_PKT_BUF *tx_buf)
{
    /* configure/initialize tx packet buffer parameters */
    tx_buf->smbus_nack_retry_count = 0;
    tx_buf->smbus_lab_retry_count = 0;
    tx_buf->smbus_acquire_retry_count = 0;
    tx_buf->request_per_tx_timeout_count = 0;
    tx_buf->request_tx_retry_count = 0;
    tx_buf->buf_full = (uint8_t)MCTP_TX_PENDING;

    /* If tx state is MCTP_IDLE; then switch to MCTP_TX_NEXT. Else if tx state
     * is not MCTP_IDLE; then tx state machine will automatically switch to
     * MCTP_TX_NEXT after completing current tx */
    if(mctp_tx_state == (uint8_t)MCTP_TX_IDLE)
    {
        mctp_tx_state = (uint8_t)MCTP_TX_NEXT;

        /* set mctp event task */
        SET_MCTP_EVENT_TASK(mctp);
    }

} /* End mctp_txpktready_init */

/******************************************************************************/
/** This is called by smbus module whenever MEC1324 SMBUS address is updated.
*******************************************************************************/
void mctp_smbaddress_update(uint16_t smb_address, uint8_t mctp_port, uint8_t enable_flag)
{
    uint8_t smbus_config;

    mctp_rt.ep.ec.field.smb_address     = (uint8_t)(smb_address& UINT8_MAX);;
    mctp_cfg.smbus_enable = enable_flag;

    smbus_config = (mctp_cfg.smbus_fairness << 2 | mctp_cfg.smbus_enable);

    mctp_di_smb_configure_and_enable(MCTP_I2C_CHANNEL, 
                                    smb_address, 
                                    mctp_cfg.smbus_speed, 
                                    mctp_port, 
                                    smbus_config,
                                    mctp_cfg.smbus_enable);

} /* End mctp_smbaddress_update */

/******************************************************************************/
/** mctp_otp_get_crisis_mode_smb_port();
* Get the crisis port SMBUS port from OTP
* @param None
* @return uint8_t -   returns 0xFF for invalid port,
*                      returns enum smb_ports for valid port select bits,
*******************************************************************************/
uint8_t mctp_otp_get_crisis_mode_smb_port(void)
{
    uint8_t i2c_port = 0;
    uint8_t crisis_mode = 0x00; //coverity fix

    if(0 != efuse_read_data(ATTESTATION_PORT_EFUSE_START_ADDR, &crisis_mode, 0x01))
    {
        crisis_mode = 0xFF;
    }

    switch(crisis_mode & CR_MODE_I2C_PORT_BITS)
    {
    case 0:
        i2c_port = SMB_PORT_0;
        break;
    case 1:
        i2c_port = SMB_PORT_4;
        break;
    case 2:
        i2c_port = SMB_PORT_6;
        break;
    case 3:
        i2c_port = SMB_PORT_9;
        break;
    case 4:
        i2c_port = SMB_PORT_10;
        break;
    case 5:
        i2c_port = SMB_PORT_15;
        break;
    default:
        i2c_port = 0xFF;
        break;
    }

    return i2c_port;
}
