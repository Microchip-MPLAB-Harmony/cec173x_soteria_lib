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
#include "mctp_spt.h"
#include "mctp_control.h"
#include "mctp_task.h"
#include "mctp_config.h"
#include "pmci.h"

extern MCTP_BSS_ATTR uint8_t mctp_tx_state;
extern MCTP_BSS_ATTR MCTP_PKT_BUF mctp_pktbuf[MCTP_PKT_BUF_NUM]__attribute__ ((aligned(8)));
extern MCTP_BSS_ATTR uint8_t mctp_wait_spt_callback;
extern MCTP_BSS_ATTR uint8_t is_pldm_request_firmware_update;
extern MCTP_BSS_ATTR uint8_t msg_type_tx; // pldm or spdm or mctp - when transmitting multiple/single pkt through smbus

/******************************************************************************/
/** Initializes mctp-spt interface. It calls spt_slave _register for
* registration with spt.
* @param void
* @return MCTP_TRUE if success, else MCTP_FALSE.
*******************************************************************************/
uint8_t mctp_spt_init(void)
{
    uint8_t ret_val;
    uint8_t status_init;

    ret_val = (uint8_t)MCTP_FALSE;

    /* register with spt */

    mctp_spt_enable();
    status_init = mctp_spt_rx_register(mctp_cfg.spt_channel,
                                     (MCTP_SLAVE_FUNC_PTR )mctp_receive_spt);

    /* spt slave registration successful */
    if(status_init == (uint8_t)SPT_SLAVE_APP_STATUS_OK)
    {
        ret_val = (uint8_t)MCTP_TRUE;
    }

    return ret_val;

} /* End mctp_spt_init() */

/******************************************************************************/
/** This is called when packet is received over spt.
* @param *buffer_info Pointer to MCTP_BUFFER_INFO structure of spt layer
* @param slaveTransmitFlag Slave Transmit Flag
* @return SPT_STATUS_BUFFER_DONE / SPT_STATUS_BUFFER_ERROR to spt layer
*******************************************************************************/
uint8_t mctp_receive_spt(MCTP_BUFFER_INFO *buffer_info, uint8_t slaveTransmitFlag)
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
        /* inform spt layer to free it's buffer */
        return (uint8_t)SPT_STATUS_BUFFER_DONE;
    }
    /* Check Total packet received if more than 73 Bytes*/
    if(buffer_info->DataLen > (MCTP_PACKET_MAX))
    {
        return (uint8_t)SPT_STATUS_BUFFER_ERROR;
    }
    /* check if MCTP packet */
    if ((pkt_buf->pkt.field.hdr.cmd_code != MCTP_SPT_HDR_CMD_CODE) ||
            (pkt_buf->pkt.field.hdr.byte_cnt < MCTP_BYTECNT_MIN) ||
            (buffer_info->DataLen < MCTP_PACKET_MIN))
    {
        return (uint8_t)SPT_STATUS_BUFFER_ERROR;
    }

    pkt_len = ( buffer_info->buffer_ptr[MCTP_PKT_BYTE_CNT_POS] + \
                MCTP_BYTECNT_OFFSET + \
                MCTP_PEC_BYTE );

    if(buffer_info->DataLen != pkt_len)
    {
        return (uint8_t)SPT_STATUS_BUFFER_ERROR;
    }

    /* check validation of received packet */
    pkt_valid = mctp_packet_validation(buffer_info->buffer_ptr);

    /* if received packet is found valid */
    if(MCTP_TRUE == (bool)pkt_valid)
    {
        /* call mctp packet routing function */
        if(0U != mctp_packet_routing(buffer_info))
        {
            return (uint8_t)SPT_STATUS_BUFFER_ERROR;
        }
    }
    else
    {
        spdm_msg_rx_buf = (MCTP_PKT_BUF *) &mctp_pktbuf[MCTP_BUF3];
        memset(spdm_msg_rx_buf, 0, MCTP_PKT_BUF_DATALEN);
        spdm_msg_rx_buf->buf_full = (uint8_t)MCTP_EMPTY;
        pldm_msg_rx_buf = (MCTP_PKT_BUF *) &mctp_pktbuf[MCTP_BUF4];
        memset(pldm_msg_rx_buf, 0, MCTP_PKT_BUF_DATALEN);
        pldm_msg_rx_buf->buf_full = (uint8_t)MCTP_EMPTY;
        return (uint8_t)SPT_STATUS_BUFFER_ERROR;
    }

    /* inform spt layer to free it's buffer */
    return (uint8_t)SPT_STATUS_BUFFER_DONE;

} /* End mctp_receive_sspt() */

/******************************************************************************/
/** This is called when MCTP packet is to be transmitted over smbus.
* @param *tx_buf Pointer to TX packet buffer
* @return void
*******************************************************************************/
void mctp_transmit_spt(MCTP_PKT_BUF *tx_buf)
{
    uint8_t status_tx;

    if (tx_buf == NULL)
    {
        return;
    }

    status_tx = mctp_di_spt_transmit_request(mctp_cfg.spt_channel,
                            (uint8_t *)&tx_buf->pkt,
                            tx_buf->pkt.data[MCTP_PKT_BYTE_CNT_POS] + 3U,
                            (uint8_t)true);

    if(0U != status_tx)
    {
        /* MCTP Xmit ERROR */
    }
    else
    {
        /* MCTP Xmit OK */
    }
} /* End mctp_transmit_spt() */

/******************************************************************************/
/** Once the packet trasmission is initiated over SPT, this function will be
* called by SPT layer to return the status code to mctp layer.
* Based on status code, mctp layer will schedule re-transmission of packet or
* drop the packet / mark buffer available.
* @param status Status code returned by SPT layer
* @param *buffer_ptr Pointer to packet buffer
* @param *newTxParams Pointer to structure for new SPT Tx - Not Used
* @return Release  code to SPT layer.
*******************************************************************************/
uint8_t mctp_spt_tx_done(uint8_t channel, uint8_t status, uint8_t *buffer_ptr, SPT_MAPP_CBK_NEW_TX *newTxParams)
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
    
    if(msg_type_tx == MCTP_IC_MSGTYPE_SPDM)
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
    if (msg_type_tx == MCTP_IC_MSGTYPE_PLDM)
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
    /* packet was transmitted successfully over SPT */
    case (uint8_t)SPT_SUCCESS_TX:
        /* update buffer parameters and configure events */
        mctp_sptdone_handler(tx_buf);

        mctp_wait_spt_callback = 0x0;

        ret_val = (uint8_t)SPT_APP_RETVAL_TX_DONE;

        /* change state to search valid tx buffer */
        mctp_tx_state = (uint8_t)MCTP_TX_NEXT;

        /* set mctp event task */
        SET_MCTP_EVENT_TASK(mctp);

        break;

    /* bus error or unknown condition */
    case (uint8_t)SPT_ERR_TIMEOUT:
    case (uint8_t)SPT_PEC_ERROR:
    case (uint8_t)SPT_BUSY_TX:
    case (uint8_t)SPT_ENABLE_ERROR:
    default:
        /* update buffer parameters and configure events */
        mctp_sptdone_handler(tx_buf);

        mctp_wait_spt_callback = 0x0;

        /* return release code to spt layer */
        ret_val = (uint8_t)SPT_APP_RETVAL_TX_DONE;

        /* change state to search valid tx buffer */
        mctp_tx_state = (uint8_t)MCTP_TX_NEXT;

        /* set mctp event task */
        SET_MCTP_EVENT_TASK(mctp);

        break;

    } /* end of switch */

    if(spdm_pend)//if eom is not 1 for the spdm messaging, trigger spdm for further transmission over mctp
    {
        trigger_spdm_event();
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

}  /* End mctp_spt_tx_done */

/******************************************************************************/
/** This will be called when spt tx status is success, nack retry exhaust,
* lab retry exhaust, bus error, or busy retry exhaust. This will configure
* buffer parameters and configure events based on packet type (req or response).
* @param *tx_buf Pointer to TX packet buffer
* @param pkt_type Packet type (request or response packet)
* @param status spt transaction status
* @return void
*******************************************************************************/
void mctp_sptdone_handler(MCTP_PKT_BUF *tx_buf)
{
    /* drop packet buffer */
    mctp_sptdone_drop(tx_buf);
} /* End mctp_sptdone_handler */

/******************************************************************************/
/** This will be called when TX packet buffer is dropped or abandoned.
* This will clear buffer parameters to mark that TX buffer available.
* @param *tx_buf Pointer to TX packet buffer
* @return void
*******************************************************************************/
void mctp_sptdone_drop(MCTP_PKT_BUF *pkt_buf)
{
    /* packet is dropped; mark that buffer available */
    pkt_buf->buf_full = (uint8_t)MCTP_EMPTY;
    /* clear buffer parameters */

    pkt_buf->request_tx_retry_count = 0;
    pkt_buf->request_per_tx_timeout_count = 0;
    pkt_buf->rx_timestamp = 0;
} /* End mctp_sptdone_drop */

/******************************************************************************/
/** Once any handler writes valid packet in TX buffer; it will call this
* function to configure/initialize tx buffer parameters and handle tx state
* for scheduling packet transmission over spt.
* @param *tx_buf Pointer to TX packet buffer
* @return void
*******************************************************************************/
void mctp_spt_txpktready_init(MCTP_PKT_BUF *tx_buf)
{
    /* configure/initialize tx packet buffer parameters */
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

} /* End mctp_spt_txpktready_init */

/******************************************************************************/
/** This is called by MCPT module whenever spt is enabled. 
*******************************************************************************/
void mctp_spt_enable()
{

    mctp_rt.ep.ec.field.smb_address     = 0x55;

    if(mctp_cfg.spt_enable)
    {
        mctp_di_spt_configure_and_enable(mctp_cfg.spt_channel, 
                                        mctp_cfg.spt_io_mode, 
                                        mctp_cfg.spt_wait_time,
                                        mctp_cfg.spt_tar_time);
    }

} /* End mctp_spt_enable */

/**   @}
 */

