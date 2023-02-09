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

#ifndef MCTP_CONTROL_H
#define MCTP_CONTROL_H

#include "mctp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/** MCTP_NUM_ENDPOINTS index
*******************************************************************************/
enum MCTP_NUM_ENDPOINTS
{
    MCTP_RT_EC_INDEX = 0 /**< Routing EC index*/
    , MCTP_RT_MC_INDEX   /**< Routing MC index*/
    , MCTP_ENDPOINTS_MAX /**< Max Index number*/
};

/******************************************************************************/
/** MCTP_EID_TYPE_VALUES
*******************************************************************************/
enum MCTP_EID_TYPE_VALUES
{
    MCTP_DYNAMIC_EID = 0, /**< Dynamic EID only. The Default EID field should be 0 */
    MCTP_STATIC_UNUSED,   /**< Static EID supported. This value should not be used */
    MCTP_STATIC_DEQC,     /**< Static EID supported, Default EID is the same as Current EID */
    MCTP_STATIC_DNEQC     /**< Static EID supported, Default EID is not same as Current EID */
};

/******************************************************************************/
/** MCTP_EID_STATE_VALUES
*******************************************************************************/
enum MCTP_EID_STATE_VALUES
{
    MCTP_STATIC_ASSIGNED = 0, /**< Static, assigned */
    MCTP_DYNAMIC_ASSIGNED,    /**< Dynamic, assigned */
    MCTP_DYNAMIC_NOTASSIGNED  /**< Dynamic, not assigned */
};

/******************************************************************************/
/** MCTP_RT_ENDPOINT_ENTRIES
*******************************************************************************/
typedef struct MCTP_RT_ENDPOINT_ENTRIES
{
    /** EID TYPE */
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
    uint8_t eid_type:2,
          /** EID STATE */
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
          eid_state:2,
          /** ENDPOINT TYPE */
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
          endpoint_type:2,
          /** DISCOVERED FLAG */
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
          disc:1,
          /** RESERVED */
// coverity[misra_c_2012_rule_6_1_violation:FALSE]
          rsvd:1;

    /** DEFAULT EID  */
    uint8_t default_eid;

    /** CURRENT EID  */
    uint8_t current_eid;

    /** SMB ADDRESS */
    uint8_t smb_address;

} MCTP_RT_ENDPOINT_ENTRIES;

/******************************************************************************/
/** MCTP_RT_EP_ENTRIES
*******************************************************************************/
typedef union MCTP_RT_EP_ENTRIES
{
    /** To access data in bytes form */
    uint8_t byte[4];

    /** To access data in fields form i.e. field parsing */
    MCTP_RT_ENDPOINT_ENTRIES field;

} MCTP_RT_EP_ENTRIES;

/******************************************************************************/
/** MCTP_ROUTING_TABLE
*******************************************************************************/

typedef struct MCTP_ROUTING_TABLE_STRUCT1
{
    /**< BRIDGE ENDPOINT ROUTING TABLE ENTRIES
     * ep[0] = endpoint 1 entry = ec entries
     * ep[1] = endpoint 2 entry = mc entries
     */
    MCTP_RT_EP_ENTRIES ep[2];

} MCTP_ROUTING_TABLE_STRUCT1;

typedef struct MCTP_ROUTING_TABLE_STRUCT2
{
    /**< EC ENDPOINT ROUTING TABLE ENTRIES */
    MCTP_RT_EP_ENTRIES ec;

    /**< MC ENDPOINT ROUTING TABLE ENTRIES */
    MCTP_RT_EP_ENTRIES mc;

} MCTP_ROUTING_TABLE_STRUCT2;

typedef union MCTP_ROUTING_TABLE
{
    MCTP_ROUTING_TABLE_STRUCT1 epA;
    MCTP_ROUTING_TABLE_STRUCT2 ep;

} MCTP_ROUTING_TABLE;



/* control commands */
#define MCTP_SET_EP_ID             0x01
#define MCTP_GET_EP_ID             0x02
#define MCTP_GET_MCTP_VER_SUPPORT  0x04
#define MCTP_GET_MSG_TYPE_SUPPORT  0x05
#define MCTP_GET_VNDR_MSG_SUPPORT  0x06

#define VNDR_LAHO_VERSION          0x7E

/* control command request/response sizes */
#define MCTP_HEADER_SZ      4u
#define MCTP_SMB_SLADDR_SZ  1u

#define MCTP_TOT_HDR_SZ     (MCTP_HEADER_SZ + MCTP_SMB_SLADDR_SZ)

/*Expected Size for Request Control packets*/
#define MCTP_SET_EP_ID_REQ      5U
#define MCTP_GET_EP_ID_REQ      3U
#define MCTP_GET_MCTP_VER_REG   4U
#define MCTP_GET_MSG_TYPE_REG   3U
#define MCTP_GET_VNDR_MSG_REG   4U

/*Expected Size for Response Control Packets*/
#define MCTP_SET_EP_ID_RESP     7U
#define MCTP_GET_EP_ID_RESP     7U
#define MCTP_GET_MCTP_VER_RESP  9U
#define MCTP_GET_MSG_TYPE_RESP  6U
#define MCTP_GET_VNDR_MSG_RESP  10U

#define MCTP_CNTL_ERROR_RESP    4U

/** MCTP Request Byte Count Expected*/
#define MCTP_SET_EP_ID_REQ_SIZE      (MCTP_TOT_HDR_SZ + MCTP_SET_EP_ID_REQ)     /// 10
#define MCTP_GET_EP_ID_REQ_SIZE      (MCTP_TOT_HDR_SZ + MCTP_GET_EP_ID_REQ)     /// 8
#define MCTP_GET_MCTP_VER_REQ_SIZE   (MCTP_TOT_HDR_SZ + MCTP_GET_MCTP_VER_REG)  /// 9
#define MCTP_GET_MSG_TYPE_REQ_SIZE   (MCTP_TOT_HDR_SZ + MCTP_GET_MSG_TYPE_REG)  /// 8
#define MCTP_GET_VNDR_MSG_REQ_SIZE   (MCTP_TOT_HDR_SZ + MCTP_GET_VNDR_MSG_REG)  /// 9

/** MCTP Response Byte Counts to update*/
#define MCTP_SET_EP_ID_RESP_SIZE     (MCTP_TOT_HDR_SZ + MCTP_SET_EP_ID_RESP)    /// 12
#define MCTP_GET_EP_ID_RESP_SIZE     (MCTP_TOT_HDR_SZ + MCTP_GET_EP_ID_RESP)    /// 12
#define MCTP_GET_MCTP_VER_RESP_SIZE  (MCTP_TOT_HDR_SZ + MCTP_GET_MCTP_VER_RESP) /// 14
#define MCTP_GET_MSG_TYPE_RESP_SIZE  (MCTP_TOT_HDR_SZ + MCTP_GET_MSG_TYPE_RESP) /// 11
#define MCTP_GET_VNDR_MSG_RESP_SIZE  (MCTP_TOT_HDR_SZ + MCTP_GET_VNDR_MSG_RESP) /// 15
#define MCTP_CNTL_ERROR_RESP_SIZE    (MCTP_TOT_HDR_SZ + MCTP_CNTL_ERROR_RESP)   /// 9

/* control message completion codes */
#define MCTP_CODE_SUCCESS                  0x00
#define MCTP_CODE_ERROR_GENERAL            0x01
#define MCTP_CODE_ERROR_INVALID_DATA       0x02
#define MCTP_CODE_ERROR_INVALID_LENGTH     0x03
#define MCTP_CODE_ERROR_NOT_READY          0x04
#define MCTP_CODE_ERROR_UNSUPPORTED_CMD    0x05
#define MCTP_CODE_MSG_TYPE_NOT_SUPPORTED   0x80

/* control packet byte positions */
#define MCTP_CNTL_PKT_CMDCODE_POS            10
#define MCTP_CNTL_RXPKT_MSGDATA_POS          11
#define MCTP_CNTL_TXPKT_CMPLCODE_POS         11

/* set endpoint command specific */
#define MCTP_SET_EID_NORMAL                0x00
#define MCTP_SET_EID_FORCED                0x01
#define MCTP_SET_EID_RESET                 0x02
#define MCTP_SET_EID_DISCOVERED            0x03U
#define MCTP_EID_ACCEPTED_AND_NO_EID_POOL  0x00
#define MCTP_EID_REJECTED_AND_NO_EID_POOL  0x10
#define MCTP_NO_DYNAMIC_EID_POOL           0x00

/* get endpoint command specific */
#define MCTP_SIMPLE_ENDPOINT_DYNAMIC_EID   0x00
#define MCTP_SIMPLE_ENDPOINT_STATIC_EID    0x01
#define MCTP_MEDIUM_SPECIFIC_INFO          0x00

/* get mctp version specific */
#define MCTP_BASE_VERSION                  0xFF
#define MCTP_CONTROL_MSG_VERSION           0x00
#define MCTP_VNDR_VERSION                  VNDR_LAHO_VERSION
#define MCTP_VER_NUM_ENTRY_COUNT           0x01
#define MCTP_VER_NUM_MAJOR                 0xF1
#define MCTP_VER_NUM_MINOR                 0xF0
#define MCTP_VER_NUM_UPDATE                0xFF
#define MCTP_VER_NUM_ALPHA                 0x00

/* get msg type support specific */
#define MCTP_MSG_TYPE_COUNT                0x01
#define MCTP_MSG_TYPE_NUMLIST              0x01
#define MCTP_MSG_TYPE_VENDR                VNDR_LAHO_VERSION

/* function declarations */
void mctp_ec_control_pkt_handler(MCTP_PKT_BUF *rx_buf, MCTP_PKT_BUF *tx_resp_buf);

extern void mctp_fill_error_packet(uint8_t error_type, MCTP_PKT_BUF *rx_buf, MCTP_PKT_BUF *tx_resp_buf);
extern void mctp_fill_packet_header(MCTP_PKT_BUF *rx_buf, MCTP_PKT_BUF *tx_resp_buf);
extern void mctp_fill_control_msg_header(MCTP_PKT_BUF *rx_buf, MCTP_PKT_BUF *tx_resp_buf);
extern uint8_t mctp_packet_routing(I2C_BUFFER_INFO *buffer_info);
extern void mctp_clean_up_buffer_states(void);

extern MCTP_BSS_ATTR MCTP_ROUTING_TABLE mctp_rt;

#ifdef __cplusplus
}
#endif

#endif /*MCTP_CONTROL_H*/
/**   @}
 */
