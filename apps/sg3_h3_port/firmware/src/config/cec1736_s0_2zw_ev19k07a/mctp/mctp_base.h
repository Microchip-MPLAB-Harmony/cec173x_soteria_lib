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

#ifndef MCTP_BASE_H
#define MCTP_BASE_H

#include <stddef.h>
#include <stdint.h>

#include "mctp_common.h"
#include "platform.h"

/* port to FreeRtos from Threadx */
typedef unsigned int TX_EVENT_FLAGS_GROUP;

#ifdef __cplusplus
extern "C" {
#endif

#define MCTP_TRUE                   1
#define MCTP_FALSE                  0

#define MCTP_SUCCESS                0U
#define MCTP_FAILURE                1U

#define MCTP_PKT_BUF_DATALEN       74U
#define MCTP_PKT_BUF_NUM            5U

#define MCTP_PACKET_MIN            12U
#define MCTP_PACKET_MAX            73U

#define MCTP_BYTECNT_OFFSET         3U
#define MCTP_BYTE_CNT_OFFSET        2u
#define MCTP_PEC_BYTE               1U
#define PAYLOAD                     8U
#define MCTP_BYTECNT_MIN            5U
#define MCTP_BYTECNT_MAX           69U

#define MCTP_SMBUS_HDR_CMD_CODE  0x0FU

#define MCTP_MSGTYPE_CONTROL        0U
#define MCTP_MSGTYPE_PLDM           1U
#define MCTP_MSGTYPE_SPDM           5U

#define MCTP_SPDM_CMD_GET_CERT      0x82U

#define MCTP_BUF1                   0U
#define MCTP_BUF2                   1U
#define MCTP_BUF3                   2U
#define MCTP_BUF4                   3U
#define MCTP_BUF5                   4U

#define MCTP_PKT_DST_ADDR_POS       0U
#define MCTP_PKT_CMD_CODE_POS       1U
#define MCTP_PKT_BYTE_CNT_POS       2U
#define MCTP_PKT_SRC_ADDR_POS       3U
#define MCTP_PKT_HDR_VER_POS        4U
#define MCTP_PKT_DST_EID_POS        5U
#define MCTP_PKT_SRC_EID_POS        6U
#define MCTP_PKT_TO_MSGTAG_POS      7U
#define MCTP_PKT_IC_MSGTYPE_POS     8U
#define MCTP_PKT_RQ_D_POS           9U

#define MCTP_PKT_TO_VEND_ID_POS_M   9U
#define MCTP_PKT_TO_VEND_ID_POS_L   10U

#define MCTP_PKT_VEND_RQ_D_POS      11U
#define MCTP_PKT_VEND_MENI_CMD_POS  12U

#define INCR_NEXT                   0x1U
#define MCTP_NULL_EID               0x00U
#define MCTP_EID_LOW                0x08U
#define MCTP_EID_HIGH               0xFFU

#define MCTP_HDR_VER_REF            0x01U
#define MCTP_HDR_VER_REF_MSK        0x0FU

#define MCTP_SOM_EOM_REF            0xC0U
#define MCTP_SOM_EOM_REF_MSK        0xC0U
#define MCTP_SOM_REF                0x80U
#define MCTP_EOM_REF                0x40U
#define MCTP_SOM_REF_MSK            0x80U
#define MCTP_EOM_REF_MSK            0x40U
#define MCTP_MSG_TAG_REF_MASK       0x07U
#define MCTP_MSG_TAG_TO_REF_MASK    0x08U
#define MCTP_MSG_PKSEQ_REF_MASK     0x30U
#define MCTP_MSG_PKSEQ_SHIFT        0x4U
#define MCTP_MSG_PKSEQ_MAX_MASK     0x3U

#define MCTP_IC_MSGTYPE_VNDR        VNDR_LAHO_VERSION
#define MCTP_IC_MSGTYPE_CONTROL     0x00U
#define MCTP_IC_MSGTYPE_LHUP_RESP   0x01U
#define MCTP_IC_MSGTYPE_LHUP_DGRM   0x02U
#define MCTP_IC_MSGTYPE_SPDM        0x05U
#define MCTP_IC_MSGTYPE_PLDM        0X01U
#define MCTP_IC_MSGTYPE_UNKNWN      0xFFU
#define SPDM_TIMEOUT_MS             135U //135 ms
#define MCTP_TIMEOUT_MS             100U //100 ms

#define VENDOR_ID_PS_M_REF          0x80U
#define VENDOR_ID_PS_L_REF          0x86U
#define VNDR_ID_SEL_REF             0xFFU
#define PCI_VNDR_ID_REF             0x00U
#define MCTP_PKT_VEND_MENI_CMD_REF  0x03U
#define VENDOR_VDM_VERSION          0x01U
#define NONE_REF_VALUE              0x00U

#define MCTP_PKT_VEND_RQ_D_POS_MSK  0xC0U
#define MCTP_PKT_VEND_RQ_D_REF      0xC0U

#define MCTP_HDR_MASK_TO            0x08U
#define MCTP_HDR_MASK_RQ            0x80U
#define MCTP_HDR_MASK_D             0x40U

#define MCTP_RESP_PKT               0U
#define MCTP_REQ_PKT                1U
#define MCTP_DTGM_PKT               2U
#define MCTP_OTHER_PKT              0xFFU

#define MCTP_NULL                   0x00U

#define MCTP_MAX_NUM_APP            1U
#define INVALID_APP_CODE            0xFFU

#define INSTANCE_ID                 0x01U

/*SPDM header bit pos*/
#define SPDM_MSG_TYPE_POS           8U // For multiple response, msg type will not be present in packets other than first packet 
#define SPDM_HEADER_VERSION_POS     9U
#define SPDM_HEADER_COMMAND_POS     10U
#define SPDM_HEADER_DATA_POS        11U
/* PLDM header bit position */
#define PLDM_PAYLOAD_START_MUTLIPLE_PKT_POS 8U
#define PLDM_HEADER_VERSION_PLDM_TYPE_POS   10U
#define PLDM_HEADER_COMMAND_CODE_POS        11U
#define PLDM_HEADER_COMPLETION_CODE_POS     12U // completion code is only for response.
#define PLDM_HEADER_DATA_POS_FOR_RESP       13U
#define PLDM_HEADER_DATA_POS_FOR_REQ        12U

#define PLDM_SEND_VERIFY_COMPLETE_CMD       23U
#define PLDM_TYPE5_AND_HEADER_VERSION       0x05U

/* Status codes for application callback from MCTP */
enum STATUS_TO_APP
{
    APP_RX_REQUEST = 0,
    APP_TX_REQUEST_DONE,
    APP_TX_REQUEST_FAIL,
    APP_TX_RESPONSE_DONE,
    APP_TX_RESPONSE_FAIL,
};

/** MCTP buffer status */
/*
Buffer 1 can be in state MCTP_EMPTY, MCTP_PACKETIZING, MCTP_RX_PENDING.
Buffer 2 can be in state MCTP_EMPTY, MCTP_PACKETIZING, MCTP_RX_PENDING.
Buffer 3 can be in state MCTP_EMPTY, MCTP_TX_PENDING, MCTP_RX_PENDING.
Buffer 4 can be in state MCTP_EMPTY, MCTP_TX_PENDING.
*/
enum MCTP_BUF_STATUS
{
    MCTP_EMPTY = 0                /**< Empty */
    , MCTP_TX_PENDING             /**< need TX over smbus */
    , MCTP_TX_DONE_WAIT_FOR_RESP  /**< smbus TX done, waiting for response */
    , MCTP_RX_PENDING             /**< RX received from smbus, need to do mctp control/pldm process */
    , MCTP_PACKETIZING            /**< Packetizing in Progress */

};

/** mctp transmit state machine status */
enum MCTP_TX_STATES
{
    MCTP_TX_IDLE = 0, /**< No TX buffer is pending */
    MCTP_TX_NEXT, /**< Search for any pending TX buffer */
    MCTP_TX_WAIT_SMBUS_CHAN_STAT_GET, /**< Check smbus busy status */
    MCTP_TX_SMBUS_ACQUIRE, /**< Check smbus busy and timeout conditions */
    MCTP_TX_IN_PROGRESS /**< Transmission over smbus is in progress */
};
//TODO need to clean up the structure for proper definition
/******************************************************************************/
/** MCTP packet header. It includes SMBus specific header, MCTP transport
 * header and message header fields.
*******************************************************************************/
typedef struct MCTP_HEADER
{
    /** Destination Read/Write */
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
    uint8_t rw_dst:1,
          /** Destination Slave Address */
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
          dst_addr:7;

    /** Command Code */
    uint8_t cmd_code;

    /** Bye Count */
    uint8_t byte_cnt;

    /** IPMI over SMBus/I2C */
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
    uint8_t ipmi_src:1,
          /** Source Slave Address */
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
          src_addr:7;

    /** Header Version */
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
    uint8_t hdr_ver:4,
          /** MCTP reserved */
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
          mctp_rsvd:4;

    /** Destination Endpoint ID */
    uint8_t dst_eid;

    /** Source Endpoint ID */
    uint8_t src_eid;

    /** Message Tag */
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
    uint8_t msg_tag:3,
          /** Tag Owner */
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
          tag_owner:1,
          /** Packet Sequence Number */
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
          pkt_seq:2,
          /** End Of Message */
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
          eom:1,
          /** Start Of Message */
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
          som:1;

    /** Message Type */
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
    uint8_t msg_type:7,
          /** Integrity Check */
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
          integrity_check:1;

    /** Instance ID */
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
    uint8_t inst_id:5,
          /** Reserved */
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
          rsvd:1,
          /** Datagram bit */
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
          dgram_bit:1,
          /** Request bit */
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
          req_bit:1;

} MCTP_HEADER;

/******************************************************************************/
/** Structure holding the MCTP packet header and message payload.
*******************************************************************************/
typedef struct MCTP_PACKET
{
    /** mctp header */
    MCTP_HEADER hdr;

    /** mctp message body */
    uint8_t msg[MCTP_PKT_BUF_DATALEN - sizeof(MCTP_HEADER)];

} MCTP_PACKET;

/******************************************************************************/
/** MCTP packet buffer data
*******************************************************************************/
typedef union MCTP_BUFDATA
{
    /** To access packet data in bytes form */
    uint8_t data[sizeof(MCTP_PACKET)];

    /** To access packet data in fields form i.e. packet parsing */
    MCTP_PACKET field;

} MCTP_BUFDATA;

/******************************************************************************/
/** MCTP packet buffer structure holding packet buffer data and
 * buffer parameters / attributes
*******************************************************************************/
typedef struct MCTP_PKT_BUF
{
    /** MCTP packet data buffer. */
    MCTP_BUFDATA pkt;

    /** Buffer 1 can be in state MCTP_EMPTY, MCTP_RX_PENDING.
     * Buffer 2 can be in state MCTP_EMPTY, MCTP_RX_BIOS_DONE.
     * Buffer 3 can be in state MCTP_EMPTY, MCTP_TX_PENDING, MCTP_RX_PENDING.
     * Buffer 4 can be in state MCTP_EMPTY, MCTP_TX_PENDING, MCTP_TX_DONE_WAIT_FOR_RESP.
     * Buffer 5 can be in state MCTP_EMPTY, MCTP_TX_PENDING.
     * Buffer 6 can be in state MCTP_EMPTY, MCTP_TX_PENDING, MCTP_TX_DONE_WAIT_FOR_RESP.
     * Buffer 7 can be in state MCTP_EMPTY, MCTP_TX_PENDING, MCTP_RX_PENDING. */

    uint8_t buf_full;

    /** Applicable only for transmit buffer 3, 4, 5 and 6.
     * This counts number of nack retries. */
    uint8_t smbus_nack_retry_count;

    /** Applicable only for transmit buffer 3, 4, 5 and 6.
     * This counts number of lost arbitration retries. */
    uint8_t smbus_lab_retry_count;

    /** Applicable only for transmit buffer 3, 4, 5 and 6.
     * This counts number of smbus master busy retries. */
    uint8_t smbus_acquire_retry_count;

    /** Applicable only for transmit request packet buffers 4 and 6.
     * This is used for timeout condition of 360ms while waiting for
     * matching response packet from MC.
     * = timestamp as per mctp firmware specifications */
    uint16_t request_per_tx_timeout_count;

    /** Applicable only for transmit request packet buffers 4 and 6.
     * It is incremented each time a re-transmission is attempted after
     * the request_per_tx_timeout_count expires (360ms).
     * After retry count becomes 3, transmission of packet is abandoned
     * and buffer is marked available.
     * = timeout_retry_count as per mctp firmware specifications */
    uint16_t request_tx_retry_count;

    /* Holds rx packet timestamp; i.e. time when packet was received
     * by smbus layer */
    uint16_t rx_smbus_timestamp;

} MCTP_PKT_BUF;

/******************************************************************************/
/** MCTP Configuration parameters / attributes
*******************************************************************************/
typedef struct MCTP_CFG_PARA
{

    /* Bit[0] Enable
    0 = SMBus x is not enabled. When the SMBus is not enabled, the hardware will not
    transmit or respond to any SMBus commands.
    1 = SMBus x is enabled. The SMBus has been configured and enabled. The SMBus controller
    will generate and respond to master/slave commands.

    Bit[1] Master Speed Select
    This bits determines the SMBus Clock frequency.
    0 = 100kHz
    1 = 400kHz
    Bit[2] Fairness Enable If MCTP shares the SMBus Controller with another SMBus master function, the fairness
    algorithm must be enabled. (ex. MCTP and PCH)
    0 = fairness algorithm not enabled
    1 = fairness algorithm enabled.
    Bit[3] SMBus Busy Status
    While the SMBus Network Layer is processing an SMBus master/slave command the SMBus Busy
    Status bit will be asserted*/
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
    uint8_t smbus_enable:1,
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
          smbus_fairness:1,
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
          smbus_status:1,
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
          smbus_rsvd:4;
    /* Internal Flag to notify the Application the Discovery is done*/
    uint8_t mctp_discovery;

    /* byte value for selecting the i2c clock frequency upto 1 Mhz*/
    uint8_t smbus_speed;

} MCTP_CFG_PARA;

/******************************************************************************/
/** MCTP self idnetification flags for internal processing
*******************************************************************************/
typedef struct MCTP_IDENTITY
{
    /*Holds the current packet Sequence number SMBus*/
    uint8_t packet_seq;
    /*Holds the current message type received Via SMBus*/
    uint8_t message_type;
    /* To hold the message tag value for the incomign start of Packet for Application*/
    uint8_t message_tag;
    /* To hold the index value of the buffer pointer*/
    uint16_t buf_index;
    /* To Hold the current buffer length*/
    uint16_t buf_size;
    /* To hold the msg register current status */
    uint8_t app_register;
    /* To hold the error if any from SMBus transfer*/
    uint8_t smbus_error;
} MCTP_IDENTITY;

typedef uint32_t * EventGroupHandle_t;
typedef uint32_t     TickType_t;
typedef unsigned long    UBaseType_t;
struct xSTATIC_MINI_LIST_ITEM
{
    #if ( configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES == 1 )
        TickType_t xDummy1;
    #endif
    TickType_t xDummy2;
    void * pvDummy3[ 2 ];
};
typedef struct xSTATIC_MINI_LIST_ITEM StaticMiniListItem_t;
typedef struct xSTATIC_LIST
{
    #if ( configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES == 1 )
        TickType_t xDummy1;
    #endif
    UBaseType_t uxDummy2;
    void * pvDummy3;
    StaticMiniListItem_t xDummy4;
    #if ( configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES == 1 )
        TickType_t xDummy5;
    #endif
} StaticList_t;
typedef struct xSTATIC_EVENT_GROUP
{
    TickType_t xDummy1;
    StaticList_t xDummy2;

    #if ( configUSE_TRACE_FACILITY == 1 )
        UBaseType_t uxDummy3;
    #endif

    #if ( ( configSUPPORT_STATIC_ALLOCATION == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) )
        uint8_t ucDummy4;
    #endif
} StaticEventGroup_t;
/******************************************************************************/
/**  MCTP Context Information
*******************************************************************************/
typedef struct MCTP_CONTEXT
{
    uint8_t i2c_bus_freq;

    uint8_t i2c_slave_addr;

    uint8_t eid;

    uint8_t check_spdm_cmd;

    /* Event group handle */
    EventGroupHandle_t xmctp_EventGroupHandle;

    /* Event group buffer*/
    StaticEventGroup_t xmctp_CreatedEventGroup;

} MCTP_CONTEXT;


/******************************************************************************/
/** Initializes the mctp module.
* Mainly does initialization specific to mctp buffers, buffer parameters,
* variables and mctp-smbus/emi interface.
* @param void
* @return void
*******************************************************************************/
void mctp_init_task(void);

/******************************************************************************/
/** This is called whenever kernel schedules mctp event task.
* Mctp module calls SET_MCTP_EVENT_TASK(mctp) for scheduling mctp event task.
* This event task is called whenever packet is received over smbus, or
* packet is to be transmitted over smbus.
* @param void
* @return void
*******************************************************************************/
void mctp_event_task(void);

/******************************************************************************/
/** UPDATES packetizing variable to check if input data spans more than one 
* mctp packet
* @param NULL
* @return true/false
*******************************************************************************/
extern bool mctp_base_packetizing_val_get(void);

/******************************************************************************/
/** UPDATES packetizing variable to check if input data spans more than one
* mctp packet
* @param val = true/false
* @return None
*******************************************************************************/
extern void mctp_base_packetizing_val_set(bool val);

/******************************************************************************/
/** Validates packet received over smbus.
* @param *pktbuf Pointer to smbus layer packet buffer
* @return MCTP_TRUE if packet is found valid, else return MCTP_FALSE
*******************************************************************************/
uint8_t mctp_packet_validation(uint8_t *pkt_buf);

/******************************************************************************/
/** Get packet type - response or request or other.
* @param *buffer_ptr Pointer to packet buffer
* @return Packet type - request, response or other.
*******************************************************************************/
uint8_t mctp_get_packet_type(uint8_t *buffer_ptr);

/******************************************************************************/
/** For Calculating the time difference between current and the time given
* @param start_time_val Start time counter value
* @return Return the difference between start and end timing
*******************************************************************************/
uint32_t mctp_timer_difference(uint32_t start_time_val);

/******************************************************************************/
/** Store I2C parameters into MCTP context structure
* @param void
* @return void
*******************************************************************************/
void mctp_update_i2c_params(MCTP_CONTEXT* ret_mctp_ctxt);

/****************************************************************/
/** mctp_i2c_update
* Updates I2C bus parameters
* @param void
* @return void
*****************************************************************/
void mctp_i2c_update(uint8_t slv_addr, uint8_t freq, uint8_t eid);

/****************************************************************/
/** sb_mctp_enable
* Enable MCTP module
* @param void
* @return void
*****************************************************************/
void sb_mctp_enable(void);

/******************************************************************************/
/** UPDATES CURRENT_EID FIELD OF RESPECTIVE ENDPOINT OF MCTP BRIDGE ROUTING TABLE
* @param i Endpoint entry id
* @return void
*******************************************************************************/
extern void mctp_rtupdate_current_eid(uint8_t i);

/******************************************************************************/
/** UPDATES EID_TYPE FIELD OF RESPECTIVE ENDPOINT OF MCTP BRIDGE ROUTING TABLE
* @param i Endpoint entry id
* @return void
*******************************************************************************/
extern void mctp_rtupdate_eid_type(uint8_t i);

/******************************************************************************/
/** UPDATES EID_STATE FIELD OF RESPECTIVE ENDPOINT OF MCTP BRIDGE ROUTING TABLE
* @param i Endpoint entry id
* @return void
*******************************************************************************/
extern void mctp_rtupdate_eid_state(uint8_t i);

extern MCTP_BSS_ATTR struct MCTP_CFG_PARA mctp_cfg;
extern MCTP_BSS_ATTR uint8_t is_attest_port_enabled;
extern MCTP_BSS_ATTR struct MCTP_IDENTITY mctp_self;

extern MCTP_BSS_ATTR uint8_t mctp_tx_state;

extern MCTP_BSS_ATTR uint8_t mctp_wait_smbus_callback;

extern MCTP_BSS_ATTR uint8_t store_msg_type_tx; // pldm or spdm or mctp - when transmitting multiple/single pkt through smbus

MCTP_CONTEXT* mctp_ctxt_get(void);
#ifdef __cplusplus
}
#endif

#endif /* MCTP_BASE_H */

/**   @}
 */
