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

/** @file pldm_pkt_prcs.h
 *  MEC1324 Peripheral common header file
 */
/** @defgroup MEC1324 Peripherals
 */

/*******************************************************************************
 *  MCHP version control information (Perforce):
 *
 *  FILE:     $ $
 *  REVISION: $Revision: #19 $
 *  DATETIME: $DateTime: 2023/02/02 13:26:58 $
 *  AUTHOR:   $Author: i53517 $
 *
 *  Revision history (latest first):
 *      # 1: Initial revision for the MCTP porting
 ***********************************************************************************
*/

#ifndef PLDM_PKT_PRCS_H
#define PLDM_PKT_PRCS_H

#include "pldm_common.h"

/* Message payload lengths */
#define PLDM_GET_COMMANDS_REQ_BYTES 5
#define PLDM_GET_VERSION_REQ_BYTES 6
#define PLDM_MAX_PAYLOAD_BUFF_SIZE 1024 
#define AP_CFG_STAGED_LOCATION 2
#define HASH_TABLE_VERSION     12

#define NUMBER_ECFW_SUPPORT 2
#define NUMBER_APCFG_SUPPORT 2
#define NUMBER_HT_SUPPORT 12
#define NUMBER_ECFWKHB_SUPPORT 3

#define NO_OF_HDR_BYTES_FRM_BYTE_CNT_OFFSET_SOM_BIT_SET 10
#define NO_OF_HDR_BYTES_FRM_BYTE_CNT_OFFSET 5u
#define PLDM_RESPONSE_DATA_POS 8

// for multiple pkts
#define MAX_NO_OF_PLDM_PAYLOAD 60u
#define NO_OF_BYTES_PLDM_HEADER_TYPE_CMD_CODE_AND_PAYLOAD 62u
#define MAX_NO_OF_PLDM_PAYLOAD_NOT_FIRST_PKT 64u
#define BYTE_CNT_FOR_LAST_PKT_MSG 5u
#define BYTE_CNT_FOR_ONE_PKT 9u

#define FD_T1 120000 // FD_T1 timeout 120000ms for Req fw data responses

#define SG3_CRISIS 1
#define SG3_POSTAUTH 0

#define PLDM_CRISIS_BA_VERSION 0x0001

/** pldm transmit state machine status */
enum PLDM_TX_STATES
{
    PLDM_TX_IDLE = 0, /**< No TX buffer is pending */
    PLDM_TX_IN_PROGRESS, /**< Transmission over smbus is in progress */
    PLDM_NON_PACKETIZING,
    PLDM_PACKETIZING,
    PLDM_RX_LAST_PKT,
    PLDM_ERROR_PACKETIZING
};


enum PLDM_STATES
{
    PLDM_IDLE_STATE = 0,
    PLDM_LEARN_COMPONENTS,
    PLDM_READY_TRANSFER,
    PLDM_DOWNLOAD,
    PLDM_VERIFY,
    PLDM_APPLY,
    PLDM_ACTIVATE
};

#define PLDM_MAX_TYPES 64
#define PLDM_GET_TYPES_RESP_BYTES 9
#define PLDM_MAX_CMDS_PER_TYPE 256
#define PLDM_INSTANCE_MAX 31

#define PROGRESS_PERCENT 0x65

#define NUMBER_OF_256B_TRANSFER_FROM_HOST_FOR_EC_FW 897
#define NUMBER_OF_59B_REQUEST_DATA 4

#define REQUEST_59B 59
#define REQUEST_20B 20

// PLDM Types
#define PLDM_FIRMWARE_UPDATE 0X5
#define PLDM_CONTROL_AND_DISCOVERY 0X0

// PLDM control and discovery command code
#define PLDM_GET_ID_REQ                   0X2
#define PLDM_GET_PLDM_VERSION_REQ         0X3
#define PLDM_GET_PLDM_TYPES               0X4
#define PLDM_GET_PLDM_COMMANDS            0X5

// PLDM firmware update messages command code
#define PLDM_QUERY_DEVICE_IDENTIFIERS_REQ 0X1
#define PLDM_PASS_COMPONENT_TABLE_REQ     0X13
#define PLDM_GET_FIRMWARE_PARAMETERS_REQ  0X2
#define PLDM_UPDATE_COMPONENT_REQ         0X14
#define PLDM_TRANSFER_COMPLETE_REQ        0X16
#define PLDM_VERIFY_COMPLETE_REQ          0X17
#define PLDM_APPLY_COMPLETE_REQ           0X18
#define PLDM_ACTIVATE_FIRMWARE_REQ        0X1A
#define PLDM_CANCEL_UPDATE_COMPONENT_REQ  0X1C
#define PLDM_CANCEL_UPDATE_REQ            0X1D
#define PLDM_REQUEST_UPDATE_REQ           0x10
#define PLDM_REQUEST_FIRMWARE_DATA_REQ    0X15
#define PLDM_REQUEST_GET_STATUS           0X1B

#define PLDM_SPECIFIC_HEADER_BYTES        4 // includes mctp msg type (pldm), instance id, pldm hdr version & type,
// msg command code

#define PLDM_REQ_UPDATE_RESPONSE_DATA_BYTES sizeof(REQUEST_UPDATE_RESPONSE) // includes completion code, meta data length, get pkg cmd
#define PLDM_REQ_FIRMWARE_DATA_REQ_DATA_BYTES sizeof(REQUEST_FIRMWARE_DATA) // includes offset, length
#define PLDM_REQ_APPLY_COMPLETE_REQ_DATA_BYTES sizeof(REQUEST_APPLY_COMPLETE)
#define PLDM_REQ_TRANSFER_COMPLETE_REQ_DATA_BYTES sizeof(REQUEST_TRANSFER_COMPLETE)
#define PLDM_REQ_VERIFY_COMPLETE_REQ_DATA_BYTES sizeof(REQUEST_VERIFY_COMPLETE)
#define PLDM_REQ_UPDATE_COMP_RESPONSE_DATA_BYTES sizeof(REQUEST_UPDATE_COMPONENT_RESPONSE) // includes completion code as well
#define PLDM_REQ_ACTIVATE_FIRMWARE_RESPONSE_DATA_BYTES sizeof(REQUEST_ACTIVATE_FIRMWARE_RESP)
#define PLDM_REQ_CANCEL_UPDATE_COMP_RESPONSE_DATA_BYTES sizeof(REQUEST_CANCEL_UPDATE_COMPONENT_RESPONSE)
#define PLDM_REQ_CANCEL_UPDATE_RESPONSE_DATA_BYTES sizeof(REQUEST_CANCEL_UPDATE_RESPONSE)
#define PLDM_REQ_PASS_COMPONENT_TABLE_RESPONSE_DATA_BYTES sizeof(REQUEST_PASS_COMPONENT_TABLE_RESPONSE)
#define PLDM_REQ_GET_FIRMWARE_PARAMETERS_RESPONSE_COMMON_DATA_BYTES 91
#define PLDM_REQ_GET_STATUS_RESPONSE_DATA_BYTES sizeof(GET_STATUS_REQ)

#define PLDM_UPDATE_COMP_REQUEST_SIZE sizeof(REQUEST_UPDATE_COMPONENT)
#define PLDM_GET_FIRMWARE_PARAMETERS_ECFW_APCFG_CNT 4

#define PLDM_REQ_GET_TID_RESPONSE_DATA_BYTES 2
#define PLDM_REQ_GET_PLDM_VERSION_DATA_BYTES 10
#define PLDM_REQ_GET_PLDM_TYPES_DATA_BYTES 9
#define PLDM_REQ_GET_PLDM_COMMANDS_DATA_BYTES 33

#define PLDM_PASS_COMPONENT_TABLE_CAN_BE_UPDATED 0
#define PLDM_PASS_COMPONENT_TABLE_MAY_BE_UPDATED 1
#define PLDM_PASS_COMPONENT_RESPCODE_COMP_NOT_SUPPORTED 0x06
#define PLDM_PASS_COMPONENT_RESPCODE_COMP_VERSION_STR_IDENTICAL 0xA

#define PLDM_UPDATE_COMP_REQ_RES_COMP_CAN_BE_UPDATED 0X0
#define PLDM_UPDATE_COMP_REQ_RES_COMP_CANNOT_BE_UPDATED 0X1

// PLDM supported component identifiers during crisi
#define PLDM_NUMBER_OF_COMP_SUPPORTED_DURING_CRISIS 5

// PLDM FW update type
#define PLDM_FW_TYPE_EC_FW 0x0
#define PLDM_FW_TYPE_AP_CFG 0x1
#define PLDM_FW_TYPE_HASH_TABLE 0x2
#define PLDM_FW_TYPE_ECFWKHB_TOO 0x3 // TAG1 KHB1 for TOO
#define PLDM_FW_TYPE_ECFW0_KHB 0x4 // non TOO TAG0 KHB
#define PLDM_FW_TYPE_ECFW1_KHB 0x5 // non TOO TAG1 KHB
#define PLDM_FW_BYTE_MATCH_INT_SPI 0x6

// OTP offset for query device identifiers
#define DEVICE_PART_NUMBER_OFFSET 56
#define DEVICE_SERIAL_NUMBER_OFFSET 980

#define MAX_NUM_BYTES_PKT           64u

#define PLDM_DEFAULT_CAP_DURING_UPDATE 0x00000024
#define PLDM_NUMBER_OF_COMPONENTS_SUPPORTED 4  // EC_FW Tag 0 & 1; AP_CFG 0 & 1

#define PLDM_COMP_UPDATE_FAILURE_RECOVERY_CAP (1 << 0)
#define PLDM_COMP_UPDATE_RETRY_CAP (1 << 1)
#define PLDM_COMP_UPDATE_HOST_FUNCTIONALITY (1 << 2)

#define PLDM_TRANSFER_START 0x1
#define PLDM_TRANSFER_MIDDLE 0x2
#define PLDM_TRANSFER_END 0x4
#define PLDM_TRANSFER_START_AND_END 0x5

#define PLDM_QUERY_DEVICE_DES1_TYPE 0x0000
#define PLDM_QUERY_DEVICE_DES1_LEN 0x0002
#define PLDM_QUERY_DEVICE_DES1_VAL 0x1055

#define PLDM_QUERY_DEVICE_DES2_TYPE 0xFFFF
#define PLDM_QUERY_DEVICE_DES2_LEN 0x000C

#define PLDM_QUERY_DEVICE_DES_TOTAL_LEN 0x00000016
#define PLDM_QUERY_DEVICE_DES_COUNT 0x02

#define PLDM_REQUEST_UPDATE_DATA_LEN sizeof(REQUEST_REQUEST_UPDATE)
#define PLDM_REQUEST_ACTIVATE_FIRMWARE_LEN sizeof(REQUEST_ACTIVATE_FIRMWARE)

#define PLDM_EOM 0x40

// Byte match int spi update masking
#define BYTE_MATCH_INT_SPI_IMGID_MSK 0x000F
#define BYTE_MATCH_INT_SPI_COMP_MSK 0x00F0
#define BYTE_MATCH_INT_SPI_AP_MSK 0x0F00

typedef struct PLDM_AP_CFG
{
    /* PLDM override device descriptor status */
    bool PLDM_override_device_descriptors;

    /* PLDM override capability upgrade */
    bool PLDM_override_capability_upgrade;

    /* PLDM override component classification */
    bool PLDM_override_comp_classification;

    /* PLDM device identifier length */
    uint32_t pldm_device_identifier_len;

    /* PLDM device descriptor count */
    uint32_t pldm_des_cnt;

    /* PLDM descriptor */
    uint8_t descriptor[200];

    /* PLDM capabilities during update */
    uint16_t cap_during_update;

    /* PLDM component classification */
    uint16_t comp_classification;

    /* PLDM terminal ID */
    uint8_t tid;

    /** Staged TAG0 Payload Base Address */         // 0x10
    uint32_t staged_TAG0_payload_BA;

    /** Restore TAG0 Payload Base Address */       // 0x14
    uint32_t restore_TAG0_payload_BA;

    /** Staged TAG1 Payload Base Address */
    uint32_t staged_TAG1_payload_BA;

    /** Restore TAG1 Payload Base Address */
    uint32_t restore_TAG1_payload_BA;

    /** TAG0 Max Payload Size */                // 0x20
    uint16_t TAG0_max_payload_size;

    /** TAG1 Max Payload Size */
    uint16_t TAG1_max_payload_size;

    /** APCFG0 base address */
    uint32_t AP_CFG_table0_base_addr;

    /** APCFG1 base address */
    uint32_t AP_CFG_table1_base_addr;

    uint32_t ht_addr_staged[AP_MAX][COMPONENT_MAX];

    uint32_t ht_addr_staged1[AP_MAX][COMPONENT_MAX];

    uint32_t ht_addr_staged2[AP_MAX][COMPONENT_MAX];

    uint32_t restore_img_loc[AP_MAX];

    uint32_t ap_cfg_staged_loc[AP_CFG_STAGED_LOCATION];

    uint8_t no_of_HT_support_AP0C0;

    uint8_t no_of_HT_support_AP0C1;

    uint8_t no_of_HT_support_AP1C0;

    uint8_t no_of_HT_support_AP1C1;

    uint16_t apcfg0_rev;

    uint16_t apcfg1_rev;

    uint16_t ht_version[HASH_TABLE_VERSION];

    uint8_t use_c1_ht_for_c0[AP_MAX];

    BYTE_MATCH_DETAILS byte_match_details[NO_OF_BYTE_MATCH_SUPPORT_IN_INT_FLASH];

} PLDM_AP_CFG;

typedef enum
{
    PLDM_RESPONSE,		   //!< PLDM response
    PLDM_REQUEST,		   //!< PLDM request
    PLDM_RESERVED,		   //!< Reserved
    PLDM_ASYNC_REQUEST_NOTIFY, //!< Unacknowledged PLDM request messages
} MessageType;

enum PLDM_UPDATE_COMP_COMP_CAP_RES_CODE
{
    COMP_CAN_BE_UPDATED = 0x0,
    COMP_COMPARISON_FLAG_IDENTICAL,
    COMP_COMPARISON_FLAG_LOWER,
    INVALID_COMP_COMAPARISON_STAMP,
    COMP_CONFLICT,
    PRE_REQ_NOT_MET,
    COMP_NOT_SUPPORTED,
    SECURITY_PREVENT_DOWNGRADE,
    INCOMPLETE_COMP_IMAGE_SET,
    DETAILS_MISMATCH,
    COMP_VERSION_STRING_IDENTICAL,
    COMP_VERSION_STRING_LOWER
};

enum GET_STATUS_REASON_CODE
{
    INITIALIZATION_OF_FD,
    ACTIVATE_FIRMWARE_RECEIVED,
    CANCEL_UPDATE_RECEIVED,
    TIMEOUT_LEARN_COMP,
    TIMEOUT_READY_XFER,
    TIMEOUT_DOWNLOAD,
    TIMEOUT_VERIFY,
    TIMEOUT_APPLY
};

typedef struct GET_STATUS_REQ
{
    uint8_t completion_code;
    uint8_t current_state;
    uint8_t previous_state;
    uint8_t aux_state;
    uint8_t aux_state_status;
    uint8_t progress_percent;
    uint8_t reason_code;
    uint32_t update_option_flags_enabled;
} __attribute__((packed)) GET_STATUS_REQ;

typedef struct DEVICE_DESCRIPTORS
{
    uint16_t descriptor_type;
    uint16_t descriptor_length;
    uint32_t descriptor_value;
} __attribute__((packed)) DEVICE_DESCRIPTORS;

typedef struct QUERY_DEVICE_IDENTIFIERS_REQ_MESSAGE_RES_FIELDS
{
    uint8_t completion_code;
    uint32_t device_identifiers_length;
    uint8_t descriptor_count;
    uint8_t descriptor[200];
} __attribute__((packed)) QUERY_DEVICE_IDENTIFIERS_REQ_MESSAGE_RES_FIELDS;

typedef struct REQUEST_FIRMWARE_DATA
{
    uint32_t offset;
    uint32_t length;
} __attribute__((packed)) REQUEST_FIRMWARE_DATA;

typedef struct RESPONSE_DATA
{
    uint8_t completion_code;
} __attribute__((packed)) RESPONSE_DATA;

typedef struct REQUEST_TRANSFER_COMPLETE
{
    uint8_t transfer_result;
} __attribute__((packed)) REQUEST_TRANSFER_COMPLETE;

typedef struct REQUEST_TRANSFER_COMPLETE_RESPONSE
{
    uint8_t completion_code;
} __attribute__((packed)) REQUEST_TRANSFER_COMPLETE_RESPONSE;

typedef struct REQUEST_VERIFY_COMPLETE
{
    uint8_t verify_result;
} __attribute__((packed)) REQUEST_VERIFY_COMPLETE;

typedef struct REQUEST_VERIFY_COMPLETE_RESPONSE
{
    uint8_t completion_code;
} __attribute__((packed)) REQUEST_VERIFY_COMPLETE_RESPONSE;

typedef struct REQUEST_APPLY_COMPLETE
{
    uint8_t apply_status;
    uint16_t comp_activation_mthd_modification;
} __attribute__((packed)) REQUEST_APPLY_COMPLETE;

typedef struct REQUEST_APPLY_COMPLETE_RESPONSE
{
    uint8_t completion_code;
} __attribute__((packed)) REQUEST_APPLY_COMPLETE_RESPONSE;

typedef struct REQUEST_ACTIVATE_FIRMWARE
{
    bool self_contained_activation_request;
} __attribute__((packed)) REQUEST_ACTIVATE_FIRMWARE;

typedef struct REQUEST_ACTIVATE_FIRMWARE_RESP
{
    uint8_t completion_code;
    uint16_t est_time_for_self_contained_activation;
} __attribute__((packed)) REQUEST_ACTIVATE_FIRMWARE_RESP;

/** pldm firmware update completion error codes */
enum PLDM_FW_UPDATE_ERROR_CODES
{
    PLDM_SUCCESS = 0X0,
    PLDM_ERROR,
    PLDM_ERROR_INVALID_DATA,
    PLDM_ERROR_INVALID_LENGTH,
    PLDM_ERROR_NOT_READY,
    PLDM_ERROR_UNSUPPORTED_CMD,
    PLDM_ERROR_INVALID_PLDM_TYPE,
    NOT_IN_UPDATE_MODE = 0X80,
    ALREADY_IN_UPDATE_MODE,
    DATA_OUT_OF_RANGE,
    INVALID_TRANSFER_LENGTH,
    INVALID_STATE_FOR_COMMAND,
    INCOMPLETE_UPDATE,
    BUSY_IN_BACKGROUND,
    CANCEL_PENDING,
    COMMAND_NOT_EXPECTED,
    RETRY_REQUEST_FW_DATA,
    UNABLE_TO_INITIATE_UPDATE,
    ACTIVATION_NOT_REQUIRED,
    SELF_CONTAINED_ACTIVATED_NOT_PERMITTED,
    NO_DEVICE_METADATA,
    RETRY_REQUEST_UPDATE,
    NO_PACKAGE_DATA,
    INVALID_TRANSFER_HANDLE,
    INVALID_TRANSFER_OPERATION_FLAG,
    ACTIVATE_PENDING_IMAGE_NOT_PERMITTED,
    PACKAGE_DATA_ERROR
};

typedef struct REQUEST_REQUEST_UPDATE
{
    uint32_t max_transfer_size;
    uint16_t no_of_comp;
    uint8_t max_outstanding_transfer_req;
    uint16_t package_data_length;
    uint8_t comp_image_set_version_string_type;
    uint8_t comp_image_set_version_string_length;
    uint8_t comp_image_set_version_string[ASCII_SIZE];
} __attribute__((packed)) REQUEST_REQUEST_UPDATE;

typedef struct REQUEST_PASS_COMPONENT_TABLE
{
    uint8_t transfer_flag;
    uint16_t comp_classification;
    uint16_t comp_identifier;
    uint8_t comp_classification_index;
    uint32_t comp_comparison_stamp;
    uint8_t comp_version_string_type;
    uint8_t comp_version_string_length;
    uint8_t comp_version_string[COMP_STRING_TYPE_SIZE];
} __attribute__((packed)) REQUEST_PASS_COMPONENT_TABLE;

typedef struct REQUEST_PASS_COMPONENT_TABLE_RESPONSE
{
    uint8_t completion_code;
    uint8_t component_response;
    uint8_t component_response_code;
} __attribute__((packed)) REQUEST_PASS_COMPONENT_TABLE_RESPONSE;

typedef struct REQUEST_UPDATE_COMPONENT
{
    uint16_t comp_classification;
    uint16_t comp_identifier;
    uint8_t comp_classification_index;
    uint32_t comp_comparison_stamp;
    uint32_t comp_image_size;
    uint32_t update_option_flags;
    uint8_t comp_version_string_type;
    uint8_t comp_version_string_length;
    uint8_t comp_version_string[COMP_STRING_TYPE_SIZE];
} __attribute__((packed)) REQUEST_UPDATE_COMPONENT;

typedef struct REQUEST_UPDATE_COMPONENT_RESPONSE
{
    uint8_t completion_code;
    uint8_t comp_capability_resp;
    uint8_t comp_capability_resp_code;
    uint32_t update_option_flag_enabled;
    uint16_t est_time_sending_reqfwdata;
} __attribute__((packed)) REQUEST_UPDATE_COMPONENT_RESPONSE;

typedef struct REQUEST_UPDATE_RESPONSE
{
    uint8_t completion_code;
    uint16_t fw_dev_metadata_len;
    uint8_t get_pkg_data_cmd;
} __attribute__((packed)) REQUEST_UPDATE_RESPONSE;

typedef struct REQUEST_CANCEL_UPDATE_COMPONENT
{
    uint8_t completion_code;
} __attribute__((packed)) REQUEST_CANCEL_UPDATE_COMPONENT_RESPONSE;

typedef struct REQUEST_CANCEL_UPDATE_RESPONSE
{
    uint8_t completion_code;
    uint8_t non_func_comp_indication;
    uint64_t non_func_comp_bitmap;
} __attribute__((packed)) REQUEST_CANCEL_UPDATE_RESPONSE;


/** @struct pldm_header_info
 *
 *  The information needed to prepare PLDM header and this is passed to the
 *  pack_pldm_header and unpack_pldm_header API.
 */
struct pldm_header_info
{
    MessageType msg_type;	 //!< PLDM message type
    uint8_t instance;	 //!< PLDM instance id
    uint8_t pldm_type;	 //!< PLDM type
    uint8_t command;	 //!< PLDM command code
    uint8_t completion_code; //!< PLDM completion code, applies for response
};

/** @struct pldm_get_tid_resp
 *
 *  Structure representing PLDM get tid response.
 */

struct pldm_get_tid_resp
{
    uint8_t completion_code; //!< completion code
    uint8_t tid;		 //!< PLDM GetTID TID field
} __attribute__((packed));

/** @struct pldm_get_version_req
 *
 *  Structure representing PLDM get version request.
 */
struct pldm_get_version_req
{
    uint32_t
    transfer_handle; //!< handle to identify PLDM version data transfer
    uint8_t transfer_opflag; //!< PLDM GetVersion operation flag
    uint8_t type; //!< PLDM Type for which version information is being
    //!< requested
} __attribute__((packed));

/** @struct pldm_get_version_resp
 *
 *  Structure representing PLDM get version response.
 */

struct pldm_get_version_resp
{
    uint8_t completion_code;       //!< completion code
    uint32_t next_transfer_handle; //!< next portion of PLDM version data
    //!< transfer
    uint8_t transfer_flag;	       //!< PLDM GetVersion transfer flag
    uint8_t version_data[1];       //!< PLDM GetVersion version field
} __attribute__((packed));

/** @struct pldm_version
 *
 *
 */
typedef struct pldm_version
{
    uint8_t major;
    uint8_t minor;
    uint8_t update;
    uint8_t alpha;
} __attribute__((packed)) ver32_t;

typedef union
{
    uint8_t byte;
    struct
    {
        uint8_t bit0 : 1;
        uint8_t bit1 : 1;
        uint8_t bit2 : 1;
        uint8_t bit3 : 1;
        uint8_t bit4 : 1;
        uint8_t bit5 : 1;
        uint8_t bit6 : 1;
        uint8_t bit7 : 1;
    } __attribute__((packed)) bits;
} bitfield8_t;

/** @struct pldm_get_types_resp
 *
 *  Structure representing PLDM get types response.
 */
struct pldm_get_types_resp
{
    uint8_t completion_code; //!< completion code
    bitfield8_t types[8]; //!< each bit represents whether a given PLDM Type
    //!< is supported
} __attribute__((packed));

/** @struct pldm_get_commands_resp
 *
 *  Structure representing PLDM get commands response.
 */
struct pldm_get_commands_resp
{
    uint8_t completion_code;  //!< completion code
    bitfield8_t commands[32]; //!< each bit represents whether a given PLDM
    //!< command is supported
} __attribute__((packed));

#define PLDM_HDR_VERSION_PLDM_FW_UPDATE_TYPE 0x05
#define PLDM_HDR_VERSION_PLDM_DIS_CONTROL_TYPE 0X00


/******************************************************************************/
/** This is called whenever kernel schedules pldm event task.
* mctp module calls SET_PLDM_EVENT_FLAG(pldm) for scheduling pldm event task.
* This event task is called whenever mctp packet is received with pldm message type over smbus,
* @param void
* @return void
*******************************************************************************/
void pldm1_event_task(void);

/******************************************************************************/
/** function for decoding the response of reqfirmwaredata and copying to flash
 * once 256bytes of data is received.
* @param data_ptr - pointer to data array
* @param size - size to copy into data array
* @return NULL
*******************************************************************************/
void pldm_pkt_process_request_firmware_update_response(void);

/******************************************************************************/
/** This function is for restoring configs back for AP access
* @param None
* @return void
*******************************************************************************/
void restore_configs();

/******************************************************************************/
/** once pldm msg is ready to transmit, pldm state machine is set to transmit mode
* function to load the pldm data to mctp tx buffer
* @param none
* @return void
*******************************************************************************/
void pldm_pkt_tx_packet(void);

/******************************************************************************/
/** Function to load the 1Kb pldm input buffer for the pldm response bytes
* @param none
* @return 1 if payload size goes greater than 1024 else success
*******************************************************************************/
uint8_t pldm_pkt_fill_buffer(MCTP_PKT_BUF *pldm_msg_rx_buf, PLDM_CONTEXT *pldmContext, uint16_t len, uint8_t offset);

/******************************************************************************/
/** PLDMResp_timer_callback();
* PLDMResp timer callback
* @param TimerHandle_t pxTimer
* @return None
*******************************************************************************/
void PLDMResp_timer_callback(TimerHandle_t pxTimer);

/******************************************************************************/
/** is_comp_iden_supported();
* check if comp identifier in pass component and update component messages are
* supported
* @param uint16_t comp_iden
* @return true or false
*******************************************************************************/
bool is_comp_iden_supported(uint16_t comp_iden);

/******************************************************************************/
/** pldm_init_flags();
* Init Pldm flags
* @param None
* @return None
*******************************************************************************/
void pldm_init_flags();

/******************************************************************************/
/** pldm_apcfg_device_identifier_length();
* Returns device descriptor length
* @param None
* @return length of device descriptor
*******************************************************************************/
uint32_t pldm_apcfg_device_identifier_length(void);

/******************************************************************************/
/** pldm_apcfg_descriptor_count();
* Returns device descriptor count
* @param None
* @return count no of device descriptor available
*******************************************************************************/
uint8_t pldm_apcfg_descriptor_count(void);

/******************************************************************************/
/** pldm_apcfg_descriptor();
* Returns device descriptor
* @param descriptor array
* @return None
*******************************************************************************/
void pldm_apcfg_descriptor(uint8_t *descriptor);

/******************************************************************************/
/** pldm_apcfg_override_cap_upgrade();
* Returns flag of pldm override capabilities during upgrade
* @param None
* @return true = override, false = no override
*******************************************************************************/
bool pldm_apcfg_override_cap_upgrade(void);

/******************************************************************************/
/** pldm_apcfg_capabilities_upgrade();
* Returns pldm capabilities during upgrade
* @param None
* @return cap capapbilites configured
*******************************************************************************/
uint16_t pldm_apcfg_capabilities_upgrade(void);

/******************************************************************************/
/** pldm_apcfg_override_comp_classification();
* Returns flag of pldm override component classification
* @param None
* @return true = override, false = no override
*******************************************************************************/
bool pldm_apcfg_override_comp_classification(void);

/******************************************************************************/
/** pldm_apcfg_tid();
* Returns pldm terminal id
* @param None
* @return tid
*******************************************************************************/
uint8_t pldm_apcfg_tid(void);

/******************************************************************************/
/** pldm_apcfg_component_classification();
* Returns pldm component classification
* @param None
* @return comp_classification
*******************************************************************************/
uint16_t pldm_apcfg_component_classification(void);

/******************************************************************************/
/** pldm_pkt_get_config_from_apcfg
* This function is used for getting AP_CFG during initialization for PLDM
* @param pldmContext
* @return void
*******************************************************************************/
void pldm_pkt_get_config_from_apcfg(PLDM_CONTEXT *pldmContext);

/******************************************************************************/
/** pldm_initiate_verify_req_to_update_agent
* This function is for sending verify complete to UA
* @param verify_state verify success or failure
* @return void
*******************************************************************************/
void pldm_initiate_verify_req_to_update_agent(uint8_t verify_state);

/******************************************************************************/
/** pldm_initiate_apply_req_to_update_agent
* This function is for sending apply complete to UA
* @param apply_state apply success or failure
* @return void
*******************************************************************************/
void pldm_initiate_apply_req_to_update_agent(uint8_t apply_state);

/******************************************************************************/
/** pldm_apply_complete_response
* This function is for any action required internal to UA after sending 
* apply complete response 
* @param apply_state apply success or failure
* @return void
*******************************************************************************/
void pldm_apply_complete_response(uint16_t comp_identifier);

/******************************************************************************/
/** get_pldm_override_device_desc();
* Returns pldm override device descriptor
* @param None
* @return overrite - true, false
*******************************************************************************/
bool get_pldm_override_device_desc();

/******************************************************************************/
/** pldm_get_AP_custom_configs_from_apcfg();
* This function gets the PLDM_AP_CFG feilds required for AP image update
* Refer PLDM_AP_CFG
* @param None
* @return overrite - true, false
*******************************************************************************/
void pldm_get_AP_custom_configs_from_apcfg();

#endif