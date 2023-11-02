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
#include "mctp_config.h"
#include "trace.h"

/* function declarations */
void mctp_handle_set_endpoint_id_cmd(MCTP_PKT_BUF *rx_buf, MCTP_PKT_BUF *tx_resp_buf);
void mctp_handle_get_endpoint_id_cmd(MCTP_PKT_BUF *rx_buf, MCTP_PKT_BUF *tx_resp_buf);
void mctp_handle_get_mctp_version_cmd(MCTP_PKT_BUF *rx_buf, MCTP_PKT_BUF *tx_resp_buf);
void mctp_handle_get_msg_type_support_cmd(MCTP_PKT_BUF *rx_buf, MCTP_PKT_BUF *tx_resp_buf);
void mctp_handle_get_vndr_msg_type_support_cmd(MCTP_PKT_BUF *rx_buf, MCTP_PKT_BUF *tx_resp_buf);
void mctp_handle_unsupported_cmd(MCTP_PKT_BUF *rx_buf, MCTP_PKT_BUF *tx_resp_buf);
extern MCTP_BSS_ATTR uint8_t active_pkt_msg_type_rx;
MCTP_BSS_ATTR MCTP_ROUTING_TABLE mctp_rt;

/******************************************************************************/
/** This mctp bridging function routes the packet to destination endpoint
* (EC, BIOS or BRIDGE) for packet processing.
* @param *buffer_info Pointer to MCTP_BUFFER_INFO structure of smbus layer
* @return return SUCESS = 0 or Failure = 1
*******************************************************************************/
uint8_t mctp_packet_routing(MCTP_BUFFER_INFO *buffer_info)
{
    uint8_t ret_value;

    ret_value = MCTP_SUCCESS ;

    switch(active_pkt_msg_type_rx)
    {
    case MCTP_IC_MSGTYPE_CONTROL:
        ret_value = mctp_copy_rxpkt_for_ec(buffer_info);
        break;
    case MCTP_IC_MSGTYPE_SPDM:
        ret_value = mctp_copy_rx_for_spdm_for_ec(buffer_info);
        break;
    case MCTP_IC_MSGTYPE_PLDM:
        ret_value = mctp_copy_rx_for_pldm_for_ec(buffer_info);
        break;
    default:
        ret_value = MCTP_FAILURE;
        break;
    }

    return ret_value;
} /* End mctp_packet_routing */


/******************************************************************************/
/** Handles control command packet.
* @param *rx_buf Pointer to ec rx request buffer
* @param *tx_resp_buf Pointer to ec tx response buffer
*******************************************************************************/
void mctp_ec_control_pkt_handler(MCTP_PKT_BUF *rx_buf, MCTP_PKT_BUF *tx_resp_buf)
{
    /* call appropriate function based on control command code */
    switch(rx_buf->pkt.data[MCTP_CNTL_PKT_CMDCODE_POS])
    {
    case MCTP_SET_EP_ID: /// 01
        mctp_handle_set_endpoint_id_cmd(rx_buf, tx_resp_buf);
        break;

    case MCTP_GET_EP_ID: /// 02
        mctp_handle_get_endpoint_id_cmd(rx_buf, tx_resp_buf);
        break;

    case MCTP_GET_MCTP_VER_SUPPORT: /// 04
        mctp_handle_get_mctp_version_cmd(rx_buf, tx_resp_buf);
        break;

    case MCTP_GET_MSG_TYPE_SUPPORT: /// 05
        mctp_handle_get_msg_type_support_cmd(rx_buf, tx_resp_buf);
        break;

    case MCTP_GET_VNDR_MSG_SUPPORT: /// 06
        mctp_handle_get_vndr_msg_type_support_cmd(rx_buf, tx_resp_buf);
        break;
    default:
        mctp_handle_unsupported_cmd(rx_buf, tx_resp_buf);
        break;
    }
} /* End mctp_ec_control_pkt_handler() */

/******************************************************************************/
/** Handles set endpoint id control command.
* @param *rx_buf Pointer to packet receive buffer
* @param *tx_resp_buf Pointer to packet transmit response buffer
* @return void
*******************************************************************************/
void mctp_handle_set_endpoint_id_cmd(MCTP_PKT_BUF *rx_buf, MCTP_PKT_BUF *tx_resp_buf)
{
    /* validate length of control command packet */
    if((rx_buf->pkt.field.hdr.byte_cnt) != MCTP_SET_EP_ID_REQ_SIZE)
    {
        /* invalid length. send the reply packet with error code. */
        mctp_fill_error_packet(MCTP_CODE_ERROR_INVALID_LENGTH, rx_buf, tx_resp_buf);

        /* update byte count */
        tx_resp_buf->pkt.field.hdr.byte_cnt = MCTP_CNTL_ERROR_RESP_SIZE;
    }
    /* check if invalid data */
    else if((rx_buf->pkt.data[MCTP_CNTL_RXPKT_MSGDATA_POS] > MCTP_SET_EID_DISCOVERED) ||
            (rx_buf->pkt.data[MCTP_CNTL_RXPKT_MSGDATA_POS+1] == 0xFFU) ||
            (rx_buf->pkt.data[MCTP_CNTL_RXPKT_MSGDATA_POS+1] == 0x00U))
    {
        /* invalid data. Send the reply packet with error code. */
        mctp_fill_error_packet(MCTP_CODE_ERROR_INVALID_DATA, rx_buf, tx_resp_buf);

        /* update byte count */
        tx_resp_buf->pkt.field.hdr.byte_cnt = MCTP_CNTL_ERROR_RESP_SIZE;
    }
    /* process control command */
    else
    {
        switch(rx_buf->pkt.data[MCTP_CNTL_RXPKT_MSGDATA_POS])
        {
        case MCTP_SET_EID_NORMAL:
        case MCTP_SET_EID_FORCED:

            /* set self EC eid */
            mctp_rt.ep.ec.field.current_eid = rx_buf->pkt.data[MCTP_CNTL_RXPKT_MSGDATA_POS+1];

            /* fill mctp header */
            mctp_fill_packet_header(rx_buf, tx_resp_buf);

            /* fill control message header */
            mctp_fill_control_msg_header(rx_buf, tx_resp_buf);

            /* fill the payload/message data */
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS]   = MCTP_CODE_SUCCESS;
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+1] = MCTP_EID_ACCEPTED_AND_NO_EID_POOL;
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+2] = mctp_rt.ep.ec.field.current_eid;
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+3] = MCTP_NO_DYNAMIC_EID_POOL;

            /* update byte count */
            tx_resp_buf->pkt.field.hdr.byte_cnt = MCTP_SET_EP_ID_RESP_SIZE;

            mctp_rtupdate_eid_type((uint8_t)MCTP_RT_EC_INDEX);
            mctp_rtupdate_eid_state((uint8_t)MCTP_RT_EC_INDEX);

            break;

        case MCTP_SET_EID_RESET:

            /* invalid data. Send the reply packet with error code. */
            mctp_fill_error_packet(MCTP_CODE_ERROR_INVALID_DATA, rx_buf, tx_resp_buf);

            /* update byte count */
            tx_resp_buf->pkt.field.hdr.byte_cnt = MCTP_CNTL_ERROR_RESP_SIZE;

            break;

        case MCTP_SET_EID_DISCOVERED:

            /* fill mctp header */
            mctp_fill_packet_header(rx_buf, tx_resp_buf);

            /* fill control message header */
            mctp_fill_control_msg_header(rx_buf, tx_resp_buf);

            /* fill the payload/message data */
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS]   = MCTP_CODE_SUCCESS;
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+1] = MCTP_EID_ACCEPTED_AND_NO_EID_POOL;
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+2] = mctp_rt.ep.ec.field.current_eid;
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+3] = MCTP_NO_DYNAMIC_EID_POOL;

            /* update byte count */
            tx_resp_buf->pkt.field.hdr.byte_cnt = MCTP_SET_EP_ID_RESP_SIZE;
            mctp_cfg.mctp_discovery = (uint8_t)true;
            break;

        default:
            /* Invalid */;
            break;
        } /* End switch */
    } /* End else */
} /* End mctp_handle_set_endpoint_id_cmd() */

/******************************************************************************/
/** Handles get endpoint id control command.
* @param *rx_buf Pointer to packet receive buffer
* @param *tx_resp_buf Pointer to packet transmit response buffer
* @return void
*******************************************************************************/
void mctp_handle_get_endpoint_id_cmd(MCTP_PKT_BUF *rx_buf, MCTP_PKT_BUF *tx_resp_buf)
{
    /* validate length of control command packet */
    if((rx_buf->pkt.field.hdr.byte_cnt) != MCTP_GET_EP_ID_REQ_SIZE)
    {
        /* invalid length. send the reply packet with error code. */
        mctp_fill_error_packet(MCTP_CODE_ERROR_INVALID_LENGTH, rx_buf, tx_resp_buf);

        /* update byte count */
        tx_resp_buf->pkt.field.hdr.byte_cnt = MCTP_CNTL_ERROR_RESP_SIZE;
    }
    /* process control command */
    else
    {
        /* fill mctp header */
        mctp_fill_packet_header(rx_buf, tx_resp_buf);

        /* fill control message header */
        mctp_fill_control_msg_header(rx_buf, tx_resp_buf);

        /* fill the payload/message data */
        tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS]   = (uint8_t)MCTP_CODE_SUCCESS;
        tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+1] = mctp_rt.ep.ec.field.current_eid;
        tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+2] = (uint8_t)(mctp_rt.ep.ec.byte[0] & 0x33U);
        tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+3] = (uint8_t)mctp_cfg.smbus_fairness; /// Fairness Enable

        /* update byte count */
        tx_resp_buf->pkt.field.hdr.byte_cnt = MCTP_GET_EP_ID_RESP_SIZE;
    }
} /* End mctp_handle_get_endpoint_id_cmd() */

/******************************************************************************/
/** Handles get mctp version support control command.
* @param *rx_buf Pointer to packet receive buffer
* @param *tx_resp_buf Pointer to packet transmit response buffer
* @return void
*******************************************************************************/
void mctp_handle_get_mctp_version_cmd(MCTP_PKT_BUF *rx_buf, MCTP_PKT_BUF *tx_resp_buf)
{
    /* validate length of control command packet */
    if((rx_buf->pkt.field.hdr.byte_cnt) != MCTP_GET_MCTP_VER_REQ_SIZE)
    {
        /* invalid length. send the reply packet with error code. */
        mctp_fill_error_packet(MCTP_CODE_ERROR_INVALID_LENGTH, rx_buf, tx_resp_buf);

        /* update byte count */
        tx_resp_buf->pkt.field.hdr.byte_cnt = MCTP_CNTL_ERROR_RESP_SIZE;
    }
    /* process control command */
    else
    {
        switch(rx_buf->pkt.data[MCTP_CNTL_RXPKT_MSGDATA_POS])
        {
        case MCTP_BASE_VERSION:
            /* fill mctp header */
            mctp_fill_packet_header(rx_buf, tx_resp_buf);

            /* fill control message header */
            mctp_fill_control_msg_header(rx_buf, tx_resp_buf);

            /* fill the payload/message data */
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS]   = MCTP_CODE_SUCCESS;
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+1] = MCTP_VER_NUM_ENTRY_COUNT;
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+2] = MCTP_VER_NUM_MAJOR;
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+3] = MCTP_VER_NUM_MINOR;
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+4] = MCTP_VER_NUM_UPDATE;
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+5] = MCTP_VER_NUM_ALPHA;

            /* update byte count */
            tx_resp_buf->pkt.field.hdr.byte_cnt = MCTP_GET_MCTP_VER_RESP_SIZE;

            break;

        case MCTP_CONTROL_MSG_VERSION:
            /* fill mctp header */
            mctp_fill_packet_header(rx_buf, tx_resp_buf);

            /* fill control message header */
            mctp_fill_control_msg_header(rx_buf, tx_resp_buf);

            /* fill the payload/message data */
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS]   = MCTP_CODE_SUCCESS;
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+1] = MCTP_VER_NUM_ENTRY_COUNT;
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+2] = MCTP_VER_NUM_MAJOR;
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+3] = MCTP_VER_NUM_MINOR;
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+4] = MCTP_VER_NUM_UPDATE;
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+5] = MCTP_VER_NUM_ALPHA;

            /* update byte count */
            tx_resp_buf->pkt.field.hdr.byte_cnt = MCTP_GET_MCTP_VER_RESP_SIZE;

            break;

        case MCTP_VNDR_VERSION:
            /* fill mctp header */
            mctp_fill_packet_header(rx_buf, tx_resp_buf);

            /* fill control message header */
            mctp_fill_control_msg_header(rx_buf, tx_resp_buf);

            /* fill the payload/message data */
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS]   = MCTP_CODE_SUCCESS;
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+1] = MCTP_VER_NUM_ENTRY_COUNT;
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+2] = MCTP_VER_NUM_MAJOR;
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+3] = MCTP_VER_NUM_MINOR;
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+4] = MCTP_VER_NUM_UPDATE;
            tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+5] = MCTP_VER_NUM_ALPHA;

            /* update byte count */
            tx_resp_buf->pkt.field.hdr.byte_cnt = MCTP_GET_MCTP_VER_RESP_SIZE;

            break;

        default:
            /* send the reply packet with error code: message type not supported */
            mctp_fill_error_packet(MCTP_CODE_ERROR_INVALID_DATA, rx_buf, tx_resp_buf);

            /* update byte count */
            tx_resp_buf->pkt.field.hdr.byte_cnt = MCTP_CNTL_ERROR_RESP_SIZE;

            break;
        }
    }
} /* End mctp_handle_get_mctp_version_cmd() */

/******************************************************************************/
/** Handles get message type support control command.
* @param *rx_buf Pointer to packet receive buffer
* @param *tx_resp_buf Pointer to packet transmit response buffer
* @return void
*******************************************************************************/
void mctp_handle_get_msg_type_support_cmd(MCTP_PKT_BUF *rx_buf, MCTP_PKT_BUF *tx_resp_buf)
{
    /* validate length of control command packet */
    if((rx_buf->pkt.field.hdr.byte_cnt) != MCTP_GET_MSG_TYPE_REQ_SIZE)
    {
        /* invalid length. send the reply packet with error code. */
        mctp_fill_error_packet(MCTP_CODE_ERROR_INVALID_LENGTH, rx_buf, tx_resp_buf);

        /* update byte count */
        tx_resp_buf->pkt.field.hdr.byte_cnt = MCTP_CNTL_ERROR_RESP_SIZE;
    }
    /* process control command */
    else
    {
        /* fill mctp header */
        mctp_fill_packet_header(rx_buf, tx_resp_buf);

        /* fill control message header */
        mctp_fill_control_msg_header(rx_buf, tx_resp_buf);

        /* fill the payload/message data */
        tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS]   = MCTP_CODE_SUCCESS;
        tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+1] = MCTP_MSG_TYPE_COUNT;
        tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+2] = MCTP_MSG_TYPE_VENDR;

        /* update byte count */
        tx_resp_buf->pkt.field.hdr.byte_cnt = MCTP_GET_MSG_TYPE_RESP_SIZE;
    }
} /* End mctp_handle_get_msg_type_support_cmd() */

/******************************************************************************/
/** Handles get vendor message type support control command.
* @param *rx_buf Pointer to packet receive buffer
* @param *tx_resp_buf Pointer to packet transmit response buffer
* @return void
*******************************************************************************/
void mctp_handle_get_vndr_msg_type_support_cmd(MCTP_PKT_BUF *rx_buf, MCTP_PKT_BUF *tx_resp_buf)
{
    uint8_t vndr_id_sel;

    vndr_id_sel = rx_buf->pkt.data[MCTP_CNTL_RXPKT_MSGDATA_POS];

    /* validate length of control command packet */
    if((rx_buf->pkt.field.hdr.byte_cnt) != MCTP_GET_VNDR_MSG_REQ_SIZE)
    {
        /* invalid length. send the reply packet with error code. */
        mctp_fill_error_packet(MCTP_CODE_ERROR_INVALID_LENGTH, rx_buf, tx_resp_buf);

        /* update byte count */
        tx_resp_buf->pkt.field.hdr.byte_cnt = MCTP_CNTL_ERROR_RESP_SIZE;
    }    /* check if invalid data */
    else if(0U != vndr_id_sel) /*Check if Vendor ID select is not one of 0h*/
    {
        /* invalid data. Send the reply packet with error code. */
        mctp_fill_error_packet(MCTP_CODE_ERROR_INVALID_DATA, rx_buf, tx_resp_buf);

        /* update byte count */
        tx_resp_buf->pkt.field.hdr.byte_cnt = MCTP_CNTL_ERROR_RESP_SIZE;
    }
    /* process control command */
    else
    {
        /* fill mctp header */
        mctp_fill_packet_header(rx_buf, tx_resp_buf);

        /* fill control message header */
        mctp_fill_control_msg_header(rx_buf, tx_resp_buf);

        /* fill the payload/message data */
        tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS]   = MCTP_CODE_SUCCESS;
        tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+1] = VNDR_ID_SEL_REF ;
        tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+2] = PCI_VNDR_ID_REF;
        tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+3] = VENDOR_ID_PS_M_REF;
        tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+4] = VENDOR_ID_PS_L_REF;
        tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+5] = VENDOR_VDM_VERSION;
        tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS+6] = MCTP_PKT_VEND_MENI_CMD_REF;

        /* update byte count */
        tx_resp_buf->pkt.field.hdr.byte_cnt = MCTP_GET_VNDR_MSG_RESP_SIZE;
    }
} /* End mctp_handle_get_vndr_msg_type_support_cmd() */

/******************************************************************************/
/** Handles unsupported control command.
* @param *rx_buf Pointer to packet receive buffer
* @param *tx_resp_buf Pointer to packet transmit response buffer
* @return void
*******************************************************************************/
void mctp_handle_unsupported_cmd(MCTP_PKT_BUF *rx_buf, MCTP_PKT_BUF *tx_resp_buf)
{
    /* unsupported command. send the reply packet with error code. */
    mctp_fill_error_packet(MCTP_CODE_ERROR_UNSUPPORTED_CMD, rx_buf, tx_resp_buf);

    /* update byte count */
    tx_resp_buf->pkt.field.hdr.byte_cnt = MCTP_CNTL_ERROR_RESP_SIZE;

} /* End mctp_handle_unsupported_cmd() */

/******************************************************************************/
/** Fill mctp error packet by updating mctp header fields, control message
* header fields and completion code with error code.
* @param error_type type of error
* @param *rx_buf Pointer to packet receive buffer
* @param *tx_resp_buf Pointer to packet transmit response buffer
* @return void
*******************************************************************************/
void mctp_fill_error_packet(uint8_t error_type, MCTP_PKT_BUF *rx_buf, MCTP_PKT_BUF *tx_resp_buf)
{
    /* Fill the mctp header */
    mctp_fill_packet_header(rx_buf, tx_resp_buf);

    /* Fill the control message header */
    mctp_fill_control_msg_header(rx_buf, tx_resp_buf);

    /* Append the error code at the end */
    tx_resp_buf->pkt.data[MCTP_CNTL_TXPKT_CMPLCODE_POS] = error_type;

} /* End mctp_fill_error_packet() */

/******************************************************************************/
/** Fills the transmit packet mctp header fields.
* @param *rx_buf Pointer to packet receive buffer
* @param *tx_resp_buf Pointer to packet transmit response buffer
* @return void
*******************************************************************************/
void mctp_fill_packet_header(MCTP_PKT_BUF *rx_buf, MCTP_PKT_BUF *tx_resp_buf)
{
    uint8_t rx_src_addr;
    uint8_t rx_dst_addr;
    uint8_t rx_src_eid;
    uint8_t rx_msg_tag;
    uint8_t rx_cmd_code;

    /* these intermediate varibles are required; since  rx_buf and
     * tx_resp_buf are now same buffers */
    rx_src_addr = rx_buf->pkt.field.hdr.src_addr;
    rx_dst_addr = rx_buf->pkt.field.hdr.dst_addr;
    rx_src_eid  = rx_buf->pkt.field.hdr.src_eid;
    rx_msg_tag  = rx_buf->pkt.field.hdr.msg_tag;
    rx_cmd_code = rx_buf->pkt.field.hdr.cmd_code;

    /* destination slave address */
    tx_resp_buf->pkt.field.hdr.dst_addr   = rx_src_addr;
    tx_resp_buf->pkt.field.hdr.rw_dst     = 0;
    /* mctp command code */
    tx_resp_buf->pkt.field.hdr.cmd_code   = rx_cmd_code;
    /* source slave address */
    tx_resp_buf->pkt.field.hdr.src_addr   = rx_dst_addr;
    tx_resp_buf->pkt.field.hdr.ipmi_src   = 1;
    /* mctp reserved */
    tx_resp_buf->pkt.field.hdr.mctp_rsvd  = 0;
    /* header version supported by this implementation */
    tx_resp_buf->pkt.field.hdr.hdr_ver    = 1;
    /* destination eid */
    tx_resp_buf->pkt.field.hdr.dst_eid    = rx_src_eid;
    /* source eid = eid of self/EC */
    tx_resp_buf->pkt.field.hdr.src_eid    = mctp_rt.ep.ec.field.current_eid;

    /* for single packet message */
    tx_resp_buf->pkt.field.hdr.som        = 1;
    tx_resp_buf->pkt.field.hdr.eom        = 1;
    tx_resp_buf->pkt.field.hdr.pkt_seq    = 0;
    /* for response packet */
    tx_resp_buf->pkt.field.hdr.tag_owner  = 0;
    /* message tag */
    tx_resp_buf->pkt.field.hdr.msg_tag    = rx_msg_tag;

} /* End mctp_fill_packet_header() */

/******************************************************************************/
/** Fills the transmit packet mctp control message header fields.
* @param *rx_buf Pointer to packet receive buffer
* @param *tx_resp_buf Pointer to packet transmit response buffer
* @return void
*******************************************************************************/
void mctp_fill_control_msg_header(MCTP_PKT_BUF *rx_buf, MCTP_PKT_BUF *tx_resp_buf)
{
    /* integrity check */
    tx_resp_buf->pkt.field.hdr.integrity_check = 0;
    /* control message type */
    tx_resp_buf->pkt.field.hdr.msg_type   = MCTP_MSGTYPE_CONTROL;
    /* for response packet */
    tx_resp_buf->pkt.field.hdr.req_bit    = 0;
    tx_resp_buf->pkt.field.hdr.dgram_bit  = 0;
    /* mctp reserved */
    tx_resp_buf->pkt.field.hdr.rsvd       = 0;
    /* instance id */
    tx_resp_buf->pkt.field.hdr.inst_id    = rx_buf->pkt.field.hdr.inst_id;

    /* command code byte */
    /* this is what we are responding to */
    tx_resp_buf->pkt.data[MCTP_CNTL_PKT_CMDCODE_POS] = rx_buf->pkt.data[MCTP_CNTL_PKT_CMDCODE_POS];

} /* End mctp_fill_control_msg_header() */

/******************************************************************************/
/**  mctp_clean_up_buffer_states.
*   Clena up the internal flags for future use case
*******************************************************************************/
void mctp_clean_up_buffer_states(void)
{
    uint8_t i = 0;

    for (i = 0 ; i < 2; i ++) {
        mctp_rx[i].buf_size = 0;
        mctp_rx[i].message_tag = 0;
        mctp_rx[i].message_type = MCTP_IC_MSGTYPE_UNKNWN;
        mctp_rx[i].packet_seq = 0;
    }

} /* End mctp_clean_up_buffer_states() */

uint8_t packetize_data(MCTP_BUFFER_INFO *buffer_info, MCTP_PKT_BUF *rx_buf)
{
    uint8_t i;
    uint8_t ret_val = MCTP_SUCCESS;
    uint8_t rx_packet_len = 0;

    MCTP_IDENTITY *mctp_ctx = NULL;
    mctp_ctx = mctp_msg_ctxt_lookup(buffer_info->buffer_ptr);

    rx_packet_len = (uint8_t)(((buffer_info->buffer_ptr[MCTP_PKT_BYTE_CNT_POS]) + 3U)&UINT8_MAX);

    if (mctp_ctx == NULL)
    {
        ret_val = MCTP_FAILURE;
        return ret_val;
    }

    mctp_ctx->buf_size = rx_packet_len;

    for(i = 0; i < mctp_ctx->buf_size; i++)
    {
        rx_buf->pkt.data[i] = buffer_info->buffer_ptr[i];
    }

    mctp_ctx->buf_index = (uint8_t)((mctp_ctx->buf_index + mctp_ctx->buf_size)&UINT8_MAX);

    if (mctp_ctx->buf_index > INPUT_BUF_MAX_BYTES)//if no of bytes received cross max input buffer size of 1023
    {
        mctp_ctx->buf_index = 0;
        mctp_base_packetizing_val_set(mctp_ctx->message_type, false);
        ret_val = MCTP_FAILURE;
    }
    else if((buffer_info->buffer_ptr[MCTP_PKT_TO_MSGTAG_POS] & MCTP_EOM_REF_MSK) == MCTP_EOM_REF)
    {
        mctp_base_packetizing_val_set(mctp_ctx->message_type, false);
        mctp_ctx->buf_index = 0;
    }
    else
    {
        /* Invalid */;
    }
    return ret_val;
}

/**********************************************************************************************/
/** This is called when packet received over smbus is targeted for EC and message type is PLDM.
* @param *buffer_info Pointer to BUFFER_INFO structure of smbus layer
* @return void
***********************************************************************************************/
uint8_t mctp_copy_rx_for_pldm_for_ec(MCTP_BUFFER_INFO *buffer_info)
{
    uint8_t i;
    uint8_t ret_val = MCTP_SUCCESS;
    MCTP_PKT_BUF *pldm_msg_rx_buf = NULL;
    uint8_t is_packetizing = 0x00;
    is_packetizing = mctp_base_packetizing_val_get(MCTP_IC_MSGTYPE_PLDM);

    //trace0(MCTP_TRACE, MCTP, 0, "mplen");

    pldm_msg_rx_buf = (MCTP_PKT_BUF *) &mctp_pktbuf[MCTP_BUF4];

    {
        if(MCTP_EMPTY == pldm_msg_rx_buf->buf_full)
        {
            if(is_packetizing)
            {
                ret_val = packetize_data(buffer_info, pldm_msg_rx_buf);
                if(ret_val == MCTP_SUCCESS)
                {
                    is_packetizing = mctp_base_packetizing_val_get(MCTP_IC_MSGTYPE_PLDM);
                    if(is_packetizing == false)
                    {
                        pldm_msg_rx_buf->rx_timestamp = buffer_info->TimeStamp;
                    }
                }
                else
                {
                    ret_val = MCTP_FAILURE;
                    return ret_val;
                }
            }
            else
            {
                //trace0(MCTP_TRACE, MCTP, 0, "mplbf");
                /* copy packet from smbus buffer to pldm ec rcv buffer */
                for(i = 0; i < buffer_info->DataLen; i++)
                {
                    pldm_msg_rx_buf->pkt.data[i] = buffer_info->buffer_ptr[i];
                }
                /* store smbus layer timestamp i.e. time when packet was
                 * received by smbus */
                pldm_msg_rx_buf->rx_timestamp = buffer_info->TimeStamp;
            }
            // mctp_base_packetizing_val_set(MCTP_IC_MSGTYPE_PLDM, false);
            /* mark ec rx buffer pending for further processing */
            pldm_msg_rx_buf->buf_full = MCTP_RX_PENDING;
            mctp_di_send_slave_response_packet((uint8_t*)pldm_msg_rx_buf, sizeof(MCTP_PKT_BUF), true);
            pldm_msg_rx_buf->buf_full = MCTP_EMPTY;
        }
    }

    //trace0(MCTP_TRACE, MCTP, 3, "mpled");
    return ret_val;
} /* End mctp_copy_rx_for_pldm_for_ec */

/******************************************************************************/
/** This is called when packet received over smbus is targeted for EC and message type is for spdm.
* @param *buffer_info Pointer to BUFFER_INFO structure of smbus layer
* @return void
*******************************************************************************/
uint8_t mctp_copy_rx_for_spdm_for_ec(MCTP_BUFFER_INFO *buffer_info)
{
    uint8_t i;
    uint8_t ret_val = MCTP_SUCCESS;
    uint8_t pkt_type;
    MCTP_CONTEXT *mctpContext = NULL;
    uint8_t get_packet_len = 0x00;

    mctpContext = mctp_ctxt_get();
    if(NULL == mctpContext)
    {
        return MCTP_FAILURE;
    }
    MCTP_PKT_BUF *spdm_msg_rx_buf = NULL;
    bool is_packetizing;
    is_packetizing = mctp_base_packetizing_val_get(MCTP_IC_MSGTYPE_SPDM);
    spdm_msg_rx_buf = (MCTP_PKT_BUF *) &mctp_pktbuf[MCTP_BUF3];
    get_packet_len = (uint8_t)(((buffer_info->buffer_ptr[MCTP_PKT_BYTE_CNT_POS]) + 3U) & UINT8_MAX);
    
    {
        if((uint8_t)MCTP_EMPTY == spdm_msg_rx_buf->buf_full)
        {
            if(is_packetizing)
            {
                ret_val = packetize_data(buffer_info, spdm_msg_rx_buf);
                if(ret_val == MCTP_SUCCESS)
                {
                    is_packetizing = mctp_base_packetizing_val_get(MCTP_IC_MSGTYPE_SPDM);
                    if(is_packetizing == false)
                    {
                        spdm_msg_rx_buf->rx_timestamp = buffer_info->TimeStamp;
                    }
                }
                else
                {
                    ret_val = MCTP_FAILURE;
                    return ret_val;
                }
            }
            else
            {
                //not for packetized data
                for(i = 0; i < get_packet_len ; i++)
                {
                    spdm_msg_rx_buf->pkt.data[i] = buffer_info->buffer_ptr[i];
                }
                spdm_msg_rx_buf->rx_timestamp = buffer_info->TimeStamp;
            }
            // mctp_base_packetizing_val_set(MCTP_IC_MSGTYPE_SPDM, false);
            spdm_msg_rx_buf->buf_full = (uint8_t)MCTP_RX_PENDING;

            mctpContext->check_spdm_cmd = spdm_msg_rx_buf->pkt.data[SPDM_HEADER_COMMAND_POS];

            mctp_di_send_slave_response_packet((uint8_t*)spdm_msg_rx_buf, sizeof(MCTP_PKT_BUF), false);
            spdm_msg_rx_buf->buf_full = MCTP_EMPTY;
        }
    }
    return ret_val;
}

/******************************************************************************/
/** This is called when packet received over smbus is targeted for EC.
* @param *buffer_info Pointer to MCTP_BUFFER_INFO structure of smbus layer
* @return void
*******************************************************************************/
uint8_t mctp_copy_rxpkt_for_ec(MCTP_BUFFER_INFO *buffer_info)
{
    uint8_t i;
    uint8_t pkt_type;
    uint8_t ret_val = MCTP_SUCCESS;
    MCTP_PKT_BUF *mctp_msg_rx_buf = NULL;

    /* get mctp packet type, request or response or other */
    pkt_type = mctp_get_packet_type(buffer_info->buffer_ptr);

    mctp_msg_rx_buf = (MCTP_PKT_BUF *) &mctp_pktbuf[MCTP_BUF1];

    /* if rx pkt is request pkt */
    if(MCTP_REQ_PKT == pkt_type)
    {
            /* if mctp ec rx request buffer available */
        {
            if((uint8_t)MCTP_EMPTY == mctp_msg_rx_buf->buf_full)
            {
                /* copy packet from smbus buffer to spdm ec rcv buffer */
                for(i = 0; i < buffer_info->DataLen; i++)
                {
                    mctp_msg_rx_buf->pkt.data[i] = buffer_info->buffer_ptr[i];
                }

                /* store smbus layer timestamp i.e. time when packet was
                 * received by smbus */
                mctp_msg_rx_buf->rx_timestamp = buffer_info->TimeStamp;

                /* mark ec rx buffer pending for further processing */
                mctp_msg_rx_buf->buf_full = (uint8_t)MCTP_RX_PENDING;
                SET_MCTP_EVENT_TASK(mctp);
            }
        }
    }

    return ret_val;
} /* End mctp_copy_rxpkt_for_ec */