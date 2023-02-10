/*****************************************************************************
* Copyright (c) 2022 Microchip Technology Inc.
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

/** @file spdm_pkt_prcs.h
 *  Source file for SPDM packet processing
 */

/*******************************************************************************
 *  MCHP version control information (Perforce):
 *
 *  FILE:     $ $
 *  REVISION: $Revision: #26 $
 *  DATETIME: $DateTime: 2022/09/28 08:34:45 $
 *  AUTHOR:   $Author: i67071 $
 *
 *  Revision history (latest first):
 *      # 1: Initial revision for the MCTP porting
 ***********************************************************************************
*/

#include "mctp.h"
#include "../mctp/mctp_control.h"
#include "spdm_task.h"
#include "spdm_common.h"

#define MAX_SHA384_BUF_SIZE                            384 // 48 bytes * 8 chains

/** spdm transmit state machine status */
enum SPDM_TX_STATES
{
    SPDM_TX_IDLE = 0, /**< No TX buffer is pending */
    SPDM_TX_IN_PROGRESS, /**< Transmission over smbus is in progress */
    SPDM_PRCS_RX_STATE, /**< Check spdm request process */
    SPDM_NON_PACKETIZING,
    SPDM_PACKETIZING,
    SPDM_RX_LAST_PKT,
    ERROR_SPDM_PACKETIZING
};

typedef struct CERTIFICATE
{
    uint8_t tail_ptr_val; //tail pointer
    uint32_t mem_addr;//memory address in sram/internal spi
    uint32_t cert_size;

} CERTIFICATE;


//chain structure
typedef struct CERT_CHAIN
{
    uint8_t head_ptr_val; //head pointer for the chain
    uint8_t root_cert_hash[SPDM_SHA384_LEN];
    uint32_t chain_length;

} __attribute__((packed)) CERT_CHAIN;


//slot structure
typedef struct CERT_SLOT
{
    uint8_t chain_present; //for flag check if chain present
    uint8_t chain_no; //to store which chain is present in current slot
    CERT_CHAIN chain;//head ptr val for the chain

} __attribute__((packed)) CERT_SLOT;

enum MSR_INDICES
{
    INDEX1 = 1,
    INDEX2 = 2,
    INDEX3 = 3,
    INDEX4 = 4
};

enum CHLNGE_RESPONSE_TX_STATES
{
    TXSTATE_1 = 0,
    TXSTATE_2 = 1,
    TXSTATE_3 = 2,
    TXSTATE_4 = 3,
    TXSTATE_5,
    TXSTATE_6,
    END_OF_TX
};

enum MSR_RESPONSE_TRANSMISSION_STATES
{
    FILL_MSR_RCRD = 0,
    FILL_REM_RCRD = 1,
    FILL_MSR_RND =  2,
    FILL_REM_RND =  3,
    FILL_MSR_OPQ = 4,
    FILL_MSR_OPQ_PEND =  5,
    FILL_MSR_SIGN_R = 6,
    FILL_MSR_SIGN_R_PEND = 7,
    FILL_MSR_SIGN_S = 8,
    FILL_MSR_SIGN_S_PEND = 9,
    END_OF_TRANSACTION = 10
};

//measurement format - dmtf
typedef struct MEASUREMENT_FRMT
{
    uint8_t dmtf_msr_val_type;
    uint8_t dmtf_msr_val_size[2];
    uint8_t dmtf_msr_val[SPDM_SHA384_LEN];

} __attribute__((packed)) MEASUREMENT_FRMT;

//measurement block structure
typedef struct MEASUREMENT_BLOCK
{
    uint8_t index;
    uint8_t msr_specific;
    uint8_t msr_size[2];
    MEASUREMENT_FRMT msr_frmt;//msr_format

} __attribute__((packed)) MEASUREMENT_BLOCK;

/*Expected Size for Response Control Packets*/
#define SPDM_GET_VER_RESP  7

#define SPDM_GET_VER_RESP_SIZE  (MCTP_TOT_HDR_SZ + SPDM_GET_VER_RESP)

#define SPDM_GET_CAP_RESP  13

#define SPDM_GET_CAP_RESP_SIZE  (MCTP_TOT_HDR_SZ + SPDM_GET_CAP_RESP)

#define SPDM_NEG_ALGO_RESP_BYTES  41

#define SPDM_NEG_ALG_RESP_SIZE  (MCTP_TOT_HDR_SZ + SPDM_NEG_ALGO_RESP_BYTES)

#define SPDM_ERROR_RESPONSE_BYTES  37

#define SPDM_ERROR_RESPONSE_BYTES_SIZE  (MCTP_TOT_HDR_SZ + SPDM_ERROR_RESPONSE_BYTES)

#define SPDM_GET_DIGEST_RESPONSE_BYTES  5

#define SPDM_GET_DIGEST_RESPONSE_SIZE  (MCTP_TOT_HDR_SZ + SPDM_GET_DIGEST_RESPONSE_BYTES)

#define SPDM_GET_CERT_RESPONSE_BYTES  9

#define SPDM_GET_CERT_RESPONSE_BYTES_SIZE  (MCTP_TOT_HDR_SZ + SPDM_GET_CERT_RESPONSE_BYTES)

#define SPDM_CHALLENGE_AUTH_RESPONSE_BYTES  437

#define SPDM_CHALLENGE_AUTH_RESPONSE_BYTES_SZ  (MCTP_TOT_HDR_SZ + SPDM_CHALLENGE_AUTH_RESPONSE_BYTES)

#define SPDM_MEASUREMENT_RESPONSE_BYTES     299

#define SPDM_MEASUREMENT_RESPONSE_BYTES_SZ  (MCTP_TOT_HDR_SZ + SPDM_MEASUREMENT_RESPONSE_BYTES)

#define SLOT_REQUEST_OFFSET                      2u

#define MSR_SUMMARY_HASH_TYPE_OFFSET             3u

#define SPDM_MEASUREMENT_HASH_SUMMARY_SZ         48


//block specific
#define MSR_BLOCK_SPEC_FLDS_SZ                   4u

//DMTF specific
#define MSR_BLOCK_DMTF_FLDS_SZ                   3u

#define DMTF_MSR_VAL_OFFSET                      3

#define SPDM_MSG_PLD_SPECIFIC_BYTES              1 //MSG_TYPE 

#define SPDM_HDR_DATA_START_OFFSET               2

#define DIGEST_PLD_LEN_EXCLUDING_DIGEST          4U // SPDMVersion, RequestResponseCode, Param1(rsrvd), Param2(slot mask); 
// Refer Successful DIGESTS response message format in SPDM Spec

#define DIGEST_RSP_DIGEST_OFFSET                 4U // SPDMVersion, RequestResponseCode, Param1(rsrvd), Param2(slot mask)

//As per certificate chain format table offset
#define SPDM_CERT_CHAIN_TBL_OFFSET              10

#define SPDM_CHALLENGE_AUTH_NONCE_OFFSET        (4 + SPDM_SHA384_LEN)

#define NOUNCE_DATA_SIZE                        32

#define OPAQUE_DATA_SZ                          256

#define OPAQUE_FIELDS_SZ                        2

#define OPAQUE_DATA_LEN                         2

#define SIGNATURE_R_TERM_SZ                     48u

#define SIGNATURE_S_TERM_SZ                     48u

#define VERSION_DEFAULT              0x10
#define CURRENT_VERSION              0x11
/* spdm commands */
#define SPDM_GET_VERSION             0x84
#define SPDM_GET_VERSION_RESP        0x04

#define SPDM_GET_CAPABILITIES        0xE1
#define SPDM_GET_CAPABILITIES_RESP   0x61

#define SPDM_NEG_ALGO                0xE3
#define SPDM_NEG_ALGO_RESP           0x63

#define SPDM_GET_DIGEST              0x81
#define SPDM_GET_DIGEST_RESPONSE     0x01

#define SPDM_GET_CERT                0x82
#define SPDM_GET_CERT_RESPONSE       0x02

#define SPDM_CHALLENGE_AUTH_RQ       0x83
#define SPDM_CHALLENGE_AUTH_RSP      0x03

#define SPDM_GET_MEASUREMENT         0xE0
#define SPDM_GET_MEASUREMENT_RSP     0x60

#define SPDM_ERROR_RESP              0x7F
#define SPDM_ERROR_INVLD_RQ          0x01
#define SPDM_ERROR_UNXPTD_RQ_CODE    0x04
#define SPDM_ERROR_USPRTD_RQ_CODE    0x07
#define SPDM_ERROR_MJR_VRS_MISMATCH  0x41
#define SPDM_ERROR_RESYNCH           0x43
#define SPDM_ERROR_UNSPECIFIED       0x05

#define SPDM_RESP_IF_RDY_RQ          0xFF

//SPDM_GET_VERSION command response bits
#define SPDM_VER_NUM_ENTRY_COUNT           0x01
#define SPDM_VER_NUM_MAJOR_MINOR           0x11
#define SPDM_VER_NUM_UPDATE_ALPHA          0x00u
#define SPDM_MAX_VER_ENTRY_COUNT           0x1d //29
////SPDM_GET_CAPABILITIES command response bits
//0x11 = 17 - ctexponent = 17; 2^ctexponent = 2^17 = 132 ms timeout for commands
#define SPDM_CAP_CT_EXP                    0x11
#define SPDM_CAP_FLAG_BYTE1                0x16
#define SPDM_CAP_FLAG_BYTE2                0x00
#define SPDM_CAP_FLAG_BYTE3                0x00
#define SPDM_CAP_FLAG_BYTE4                0x00

#define SPDM_CAP_FLAG_BYTE4_OFFSET         6u
#define SPDM_CAP_FLAG_BYTE3_OFFSET         7u
#define SPDM_CAP_FLAG_BYTE2_OFFSET         8u
#define SPDM_CAP_FLAG_BYTE1_OFFSET         9u

#define SPDM_CMD_PASS                          1U
#define SPDM_CMD_FAIL                          0U
enum BYTE_OFFSETS
{
    BYTE0 = 0,
    BYTE1,
    BYTE2,
    BYTE3
};


////SPDM_NEG_ALGO command response bit values
#define SPDM_NEG_ALG_NO_ALG_STRC           0x01
#define SPDM_NEG_ALG_LENGTH_RESP           0x28 //40
#define SPDM_NEG_ALG_MSR_SPC               0x01
#define SPDM_NEG_ALG_MSR_HASH_ALGO         0x04 //sha384
#define SPDM_NEG_ALG_BASE_ASYM_SEL         0x80
#define SPDM_NEG_ALG_BASE_HASH_SEL         0x02
#define SPDM_NEG_ALG_NO_EXT_SIG            0x00
#define SPDM_NEG_ALG_NO_EXT_HASH           0x00

#define ALG_TYPE                    0x04 //ReqBaseAsymAlg
#define ALG_CNT                     0x20
#define TPM_ALG_ECDSA_ECC_NIST_P384 0x80
#define MAX_NUM_BYTES_PLD           64u

#define MCTP_BYTE_CNT_OFFSET                           2u
#define NO_OF_MCTP_HDR_BYTES_FRM_BYTE_CNT_OFFSET       6
#define MAX_SLOTS                                      8
#define MAX_CERT_PER_CHAIN                             8
#define END_OF_CHAIN                                   0x40
#define CERT_MAX_SIZE                                  0x400
#define NO_CHAIN_VAL                                   0x88
#define NO_CHAIN_VAL_NIBBLE_0_MSK                      0x08
#define NO_CHAIN_VAL_NIBBLE_1_MSK                      0x80
#define CHAIN_VAL_MSK_NIBBLE0_MSK                      0x07
#define CHAIN_VAL_MSK_NIBBLE1_MSK                      0x70

#define MAX_CERTIFICATES                               64
#define MAX_SLOTS                                      8

#define MAX_SIZE_CERTIFICATE                           1024


#define CERT_DATA_BYTE2                                0x02U
#define CERT_DATA_BYTE3                                0x03U
#define CERT_BYTE0_BYTE3_COUNT                         0x04U

#define CERT2_BASE_ADDR_OTP_OFFSET                     860

#define TEST_INPUT_DATA_SZ                             25

#define START_INDEX                                    0x01
#define NO_OF_MSR_INDICES                              0x04
#define SIZE_OF_ONE_MSR_BLOCK                          55u

#define DMTF_FRMT                                      0x01

#define HW_CONFIG_DATA_SZ                              0x01

#define TCB_MSR_SIZE                                   (2*MSR_BLOCK_DMTF_FLDS_SZ + 96u)

#define MSR_RSP_RECORD_OFFSET                          8u

#define MSR_BYTES_SZ_TILL_RECRD_LEN                    8u

#define RAW_BIT_STEAM_SELECT_VALUE                     0x80

#define IMMUTABLE_ROM_SEL_DGST_RAW                     0x00 //digest; 7th bit 0 is for digest format

#define MUTABLE_FW_SEL_DGST_RAW                        0x01 //digest; 7th bit 0 is for digest format

#define HW_CONFIG_SEL_DGST_RAW                         0x02 //digest; 7th bit 0 is for digest format

#define FW_CONFIG_SEL_DGST_RAW                         0x03 //digest; 7th bit 0 is for digest format

#define MAX_PKT_SIZE                                   74

#define DEVAK_KEY_MAILBOX_OFFSET                       900 //(127100 - 126800)

#define MAX_CERT_CHAIN_IN_FIRST_ITER                   55U // MAX_NUM_BYTES_PLD - 1 - 8

#define ROOTHASH_OFFSET_IN_CERT_CHAIN                  4U

#define USE_OTP_GENERATED_PRIVATE_KEY                  0U

#define USE_PUF_GENERATED_PRIVATE_KEY                  1U

#define INVALID_SIGNATURE_FLAGS                        2U

#define INVALID_TAG_IMAGE                              2U

#define PUF0_OTP_LOCK_MASK                             (BIT_2_MASK | BIT_3_MASK)

#define DEVAK_DEVIK_KEY_SELECT_MASK                    (BIT_2_MASK | BIT_3_MASK)

#define MEAS_PLD_LEN_WITH_SIGN                          37U
#define MEAS_PLD_LEN_WITHOUT_SIGN                       04U

#define CERT_CHAIN_INVALID                              1
#define CERT_CHAIN_VALID                                0

#define ROOT_START_OFFSET                              52U // Root certificate start offset in chain
#define ROOT_HASH_END_OFFSET                           51U // Root hash end offset in chain
#define SLOT_NUM_CHAIN_LENGTH_BYTES                     6U // account for slot number reserved portion length chain length in cert response

/******************************************************************************/
/** Structure holding the version entry table fields
*******************************************************************************/
typedef union
{
    uint16_t table_value;
    struct
    {
        uint16_t alpha:3,
               update_ver_no:4,
               minor_ver:8,
               major_ver:12;
    };
} __attribute__((packed)) VERSION_NUM_ENTRY_TABLE;


/******************************************************************************/
/** Structure holding the response fields for get version response
*******************************************************************************/
typedef struct TABLE_VALUE
{
    uint8_t table_value[2];
} TABLE_VALUE;

typedef struct GET_VERSION_RESP_FIELDS
{
    uint8_t version;
    uint8_t resp_code;
    uint8_t reserved0[3];
    uint8_t entry_count;
    TABLE_VALUE table_entry[SPDM_VER_NUM_ENTRY_COUNT];

} __attribute__((packed)) GET_VERSION_RESP_FIELDS;


/******************************************************************************/
/** Structure holding the response fields for get capability response
*******************************************************************************/

typedef struct GET_CAP_RESP_FIELDS
{
    uint8_t version;
    uint8_t resp_code;
    uint8_t reserved0[3];
    uint8_t cte_expo;
    uint8_t reserved1[2];
    uint8_t flags[4];
} __attribute__((packed)) GET_CAP_RESP_FIELDS;


/******************************************************************************/
/** Structure holding the response fields for negotiate algo response
*******************************************************************************/

typedef struct REQ_BASE_ASYM_ALG
{
    uint8_t AlgType;
    uint8_t AlgCount;
    uint16_t AlgSupported;
} REQ_BASE_ASYM_ALG;

typedef struct REQ_ALGO
{
    uint8_t algo_struct[4];
} REQ_ALGO;

typedef struct NEG_ALGO_RESP_FIELDS
{
    uint8_t version;
    uint8_t resp_code;
    uint8_t no_of_algo_struct;
    uint8_t reserved;
    uint8_t len_of_resp[2];
    uint8_t msr_specific;
    uint8_t reserved1;
    uint8_t msr_hash_algo[4];
    uint8_t base_asym_sel[4];
    uint8_t base_hash_sel[4];
    uint8_t reserved2[12];
    uint8_t ext_asym_sel_cnt;
    uint8_t ext_hash_sel_cnt;
    uint8_t reserved3[2];
    REQ_ALGO req_algo_struct;
} __attribute__((packed)) NEG_ALGO_RESP_FIELDS;

/******************************************************************************/
/** Structure holding the response fields for get digest response
*******************************************************************************/

typedef struct GET_DIGEST_FIELDS
{
    uint8_t version;
    uint8_t resp_code;
    uint8_t reserved;
    uint8_t slot_mask;

} __attribute__((packed)) GET_DIGEST_FIELDS;

/******************************************************************************/
/** Structure holding the response fields for get certificate response
*******************************************************************************/

typedef struct GET_CERTIFICATE_FIELDS
{
    uint8_t version;
    uint8_t resp_code;
    uint8_t slot_no;
    uint8_t reserved;
    uint8_t portion_len[2];
    uint8_t remain_len[2];
    uint8_t len_of_cert_chain[2];
    uint8_t reserved1[2];
    uint8_t root_cert_hash[SPDM_SHA384_LEN];

} __attribute__((packed)) GET_CERTIFICATE_FIELDS;


/******************************************************************************/
/** Structure holding the response fields for challenge response
*******************************************************************************/

typedef struct CHALLENGE_FIELDS
{
    uint8_t version;
    uint8_t resp_code;
    uint8_t resp_attr;
    uint8_t slot_mask;
    uint8_t hash_of_cert_chain[SPDM_SHA384_LEN];
    uint8_t random_val[32];
    uint8_t opaq_len[2];
    uint8_t opaq_data[2];

} __attribute__((packed)) CHALLENGE_FIELDS;


/******************************************************************************/
/** Structure holding the response fields for measurement response
*******************************************************************************/

typedef struct MEASUREMENT_FIELDS
{
    uint8_t version;
    uint8_t resp_code;
    uint8_t param1;
    uint8_t param2;
    uint8_t num_of_blocks;
    uint8_t msr_rec_len[3];
    uint8_t opaq_len[2];
    uint8_t opaq_data[OPAQUE_DATA_SZ];

} __attribute__((packed)) MEASUREMENT_FIELDS;


typedef struct MEASUREMENT_VARIABLES
{
    uint8_t msr_hash_rqstd;
    uint8_t msr_hash_sum_offset;
    uint8_t sign_needed;
    uint16_t opq_offset;
    uint8_t msr_operation;
    uint8_t msr_response_byte_iter;
    uint8_t msr_rnd_bytes_remaining;
    uint16_t msr_opq_bytes_remaining;
    uint16_t msr_bytes_copied;
    uint16_t concat_msr_offset;
    uint32_t msr_record_size;
    uint32_t msr_rcrd_len;
    uint32_t msr_rcrd_bytes_to_sent;

} __attribute__((packed)) MEASUREMENT_VARIABLES;

/******************************************************************************/
/** This is called whenever kernel schedules spdm event task.
 * mctp module calls SET_MCTP_EVENT_TASK(spdm) for scheduling spdm event task.
 * This event task is called whenever mctp packet is received with spdm message type over smbus,
 * @param spdmContext - SPDM module context
 * @return void
 *******************************************************************************/
void spdm_event_task(SPDM_CONTEXT *spdmContext);

/******************************************************************************/
/** function load the get version response bytes to spdm local buffer
 * @param spdmContext - SPDM module context
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @return void
 *******************************************************************************/
void spdm_pkt_process_get_version_cmd(MCTP_PKT_BUF *test_buf, SPDM_CONTEXT *spdmContext);

/******************************************************************************/
/** function to load the get capabilities response bytes to spdm local buffer
 * @param spdmContext - SPDM module context
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @return void
 *******************************************************************************/
void spdm_pkt_process_get_capabilties_cmd(MCTP_PKT_BUF *test_buf, SPDM_CONTEXT *spdmContext);

/******************************************************************************/
/** function to load the negotiate algo response bytes to spdm local buffer
 * @param spdmContext - SPDM module context
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @return void
 *******************************************************************************/
void spdm_pkt_process_neg_alg_cmd(MCTP_PKT_BUF *test_buf, SPDM_CONTEXT *spdmContext);

/******************************************************************************/
/** function to load the digest response bytes to spdm local buffer
 * @param spdmContext - SPDM module context
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @return void
 *******************************************************************************/
void spdm_pkt_process_get_digest_cmd(MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext);

/******************************************************************************/
/** Function to parse GET_CERTIFICATE request and fill part of spdm local buffer based on the request
 * @param spdmContext - SPDM module context
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @return void
 *******************************************************************************/
uint8_t spdm_pkt_process_get_cert_cmd(MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext);

/******************************************************************************/
/** Function to parse CHALLENGE_AUTH request and fill part of spdm local buffer based on the request
 * @param spdmContext - SPDM module context
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @return void
 *******************************************************************************/
uint8_t spdm_pkt_challenge_auth(MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext);

/******************************************************************************/
/** function to validate spdm data received
 * @param spdmContext - SPDM module context
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @param get_cmd                   - RequestResponseCode for individual SPDM Commands Request as per SPDM spec 
 *                                    eg: 0x84 - GET_VERSION
 * @return error_handle       : value greater than 0 depending on the error present in payload 
 *                                    0 implies No error
 *******************************************************************************/
uint8_t spdm_pkt_validate_and_process_spdm_msg(uint8_t, MCTP_PKT_BUF *spdm_tx, SPDM_CONTEXT *spdmContext);

/******************************************************************************/
/** once spdm msg is ready to transmit, trigger mctp; call this
 * function to configure/initialize tx buffer parameters and handle tx state
 * for scheduling packet transmission over smbus.
 * @param tx_buf Pointer to TX packet buffer
 * @return void
 *******************************************************************************/
void spdm_pkt_msg_ready_trigger_mctp(MCTP_PKT_BUF *tx_buf);

/******************************************************************************/
/** Function to load the 1Kb spdm input buffer for the spdm request bytes
 * @param spdmContext     - SPDM module context
 * @param spdm_msg_rx_buf - Buffer containing SPDM Request
 * @return 1 if payload size goes greater than 1024 
 *         0 success 
 *******************************************************************************/
uint8_t spdm_pkt_fill_buffer(MCTP_PKT_BUF *spdm_msg_rx_buf, SPDM_CONTEXT *spdmContext);

/******************************************************************************/
/** Function to get the length for runtime hashing
 * @param None
 * @return None 
 *******************************************************************************/
void spdm_get_len_for_runtime_hash(SPDM_CONTEXT *spdmContext);

/******************************************************************************/
/** fill_cert_buffer
* Fill the Certificate buffer with slot, chain and certificate details.
* @param void
* @return void
*******************************************************************************/
void fill_cert_buffer();