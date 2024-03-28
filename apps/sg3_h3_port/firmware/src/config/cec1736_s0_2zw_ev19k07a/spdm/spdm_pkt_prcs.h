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

#include "mctp.h"
#include "../mctp/mctp_control.h"
#include "spdm_task.h"
#include "spdm_common.h"

#define MAX_SHA384_BUF_SIZE                            384 // 48 bytes * 8 chains

#define NO_OF_APFW_MSR_SUPPORTED 30 // 2 apcfg + 12 HT + 16 APFW images
#define HB_TIMEOUT_IN_MS (2* (HEARTBEAT_PERIOD * 1000)) //The Responder shall terminate the session if session traffic is not received in twice HeartbeatPeriod
#define SPI_DATA_BUF_SIZE              0x2000

/* Secure Session hashing states */
enum SPDM_SS_HASHING_STATE
{
    HASH_CTX_KEY_EXCHANGE = 0,
    HASH_CTX_TH1_DATA,
    HASH_CTX_FINISH,
    HASH_CTX_FINISH_VERIFY_DATA,
    HASH_CTX_FINISH_RSP_VERIFY_DATA,
    HASH_CTX_TH2_DATA,
    HASH_CTX_MAX
};

/** spdm transmit state machine status */
enum SPDM_TX_STATES
{
    SPDM_TX_IDLE = 0, /**< No TX buffer is pending */
    SPDM_TX_IN_PROGRESS, /**< Transmission over smbus is in progress */
    SPDM_PRCS_RX_STATE, /**< Check spdm request process */
    SPDM_NON_PACKETIZING,
    SPDM_PACKETIZING,
    SPDM_RX_LAST_PKT,
    SPDM_ERROR_PACKETIZING
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
	uint8_t is_cert_chain_valid; //Check if cert chain pointed by the slot is valid
} __attribute__((packed)) CERT_SLOT;

enum MSR_INDICES
{
    INDEX1 = 1,
    INDEX2 = 2,
    INDEX3 = 3,
    INDEX4 = 4,
    INDEX5 = 5
};

enum CHLNGE_RESPONSE_TX_STATES
{
    TXSTATE_1 = 0,
    TXSTATE_2 = 1,
    TXSTATE_3 = 2,
    TXSTATE_4 = 3,
    TXSTATE_5,
    TXSTATE_6,
    CHG_RESP_END_OF_TX
};

enum KEY_EXCHANGE_TX_STATES
{
    STATE_1 =0,
    STATE_2 =1,
    STATE_3 =2,
    STATE_4,
    STATE_5,
    STATE_6,
    KEY_EXCHNG_END_STATE
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
    MSR_RESP_END_OF_TRANSACTION = 10
};

enum MSR_RESPONSE_TRANSMISSION_STATES_SS
{
    FILLMSRRCRD = 0,
    FILLREMRCRD = 1,
    FILLMSRRND =  2,
    FILLREMRND =  3,
    FILLMSROPQ = 4,
    FILLMSROPQ_PEND =  5,
    FILLMSRSIGNR = 6,
    FILLMSRSIGNRPEND = 7,
    FILLMSRSIGNS = 8,
    FILLMSRSIGNSPEND = 9,
    MSR_RESP_SS_ENDOFTRANSACTION = 10 
};

enum MSR_RESPONSE_MULTPLE_PKT_SS
{
    SOM,
    REMAINING,
    MSR_RESP_MUL_PKT_SS_END
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

typedef struct MEASUREMENT_MANIFEST
{
    uint16_t image_descriptor;
    uint8_t dmtf_msr_val[SPDM_SHA384_LEN];
}__attribute__((packed)) MEASUREMENT_MANIFEST;

typedef struct MEASUREMENT_BLOCK_MANIFEST
{
    uint8_t index;
    uint8_t msr_specific;
    uint16_t msr_size;
    uint8_t dmtf_msr_val_type;
    uint16_t dmtf_msr_val_size;
    MEASUREMENT_MANIFEST msr_manifest[NO_OF_APFW_MSR_SUPPORTED];
}__attribute__((packed)) MEASUREMENT_BLOCK_MANIFEST;

/*Expected Size for Response Control Packets*/
#define SPDM_GET_VER_RESP  7

#define SPDM_GET_VER_RESP_SIZE  (MCTP_TOT_HDR_SZ + SPDM_GET_VER_RESP)

#define SPDM_GET_CAP_RESP  13

#define SPDM_GET_CAP_RESP_SIZE  (MCTP_TOT_HDR_SZ + SPDM_GET_CAP_RESP)

#define SPDM_NEG_ALGO_RESP_BYTES  53

#define SPDM_NEG_ALG_RESP_SIZE  (MCTP_TOT_HDR_SZ + SPDM_NEG_ALGO_RESP_BYTES)

#define SPDM_ERROR_RESPONSE_BYTES  5 // optional Extended error data is unused in spdm, therefore, the packet size will be 5 (including crc)

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

// secure session
#define SPDM_KEY_EXCHANGE_RESPONSE_BYTES 299 // includes message type, other key exchange fields except msrt hash

#define SPDM_KEY_EXCHANGE_RESPONSE_SIZE (MCTP_TOT_HDR_SZ + SPDM_KEY_EXCHANGE_RESPONSE_BYTES)

#define SPDM_HEART_BEAT_RESPONSE_BYTES 5

#define SPDM_HEART_BEAT_RESPONSE_SIZE (MCTP_TOT_HDR_SZ + SPDM_HEART_BEAT_RESPONSE_BYTES)

#define SPDM_FINISH_RESPONSE_BYTES 53 // including msg type, excluding responser verify data

#define SPDM_FINISH_RESPONSE_SIZE (MCTP_TOT_HDR_SZ + SPDM_FINISH_RESPONSE_BYTES)

#define SPDM_END_SESSION_RESPONSE_BYTES 5

#define SPDM_END_SESSION_SIZE (MCTP_TOT_HDR_SZ + SPDM_END_SESSION_RESPONSE_BYTES)

#define SPDM_FINISH_REQUEST_SIZE 4

// secure session offset
// key_exchange
#define SS_KEY_EXCHANGE_REQUESTER_PUB_KEY        40u
#define SS_KEY_EXCHANGE_REQ_SESSION_ID           4u
#define SS_KEY_EXCHANGE_SLOT_REQUEST_OFFSET      3u
#define SS_KEY_EXCHANGE_MSR_OFFSET               2u
// finish
#define SS_FINISH_SIGNED_MUTH_AUTH               2u
#define SS_FINISH_SLOTID                         3u
#define SS_FINISH_SIGNATURE_OFFSET               4u
#define SS_FINISH_REQ_VERIFY_DATA                100u

// end_session
#define BYTE_CNT_END_SESION_RESP_FIELDS          4u

//heartbeat
#define BYTE_CNT_HEARTBEAT_RESP_FIELDS           4u

// deliver encap response
#define SS_DELIVER_ENCAP_RESP_REQUEST_ID_OFFSET  2u
#define SS_DELIVER_ENCAP_RESP_OPCODE_OFFSET      5u // either get digest or get certificate

// MCTP header
#define BYTE_CNT_MCTP_HEADER                     6u // from source address till message type

// secure session macro values
#define DHE_384_PUB_KEY_SIZE                     96u
#define SS_ECDSA_384_SIGN_SIZE                   96u
#define HEARTBEAT_PERIOD                         10u // 10 sec
#define RESP_SESSION_ID_RANDOM_NO_SIZE           2u // responder's session id of size 2bytes
#define SS_ENCAP_RESP_REQUEST_ID_RANDOM_NO_SIZE     1u
#define RESP_ENCAP_REQ_REQUEST_ID                1u
#define KEY_EXCHANGE_MUTHAUTHREQUESTED           0x2 // bit 1 set in MuthAuthRequested 
#define MSR_NO_SUMMARY                           0
#define MSR_TCB                                  1
#define MSR_ALL                                  0xFF
#define SECURED_MESSAGE_OPAQUE_DATA_SPEC_ID      0x444D5446
#define SECURED_MESSAGE_OPAQUE_VERSION           0x1
#define SPDM_REGISTRY_ID_DMTF                    0
#define SECURED_MESSAGE_OPAQUE_ELEMENT_SMDATA_DATA_VERSION  1
#define SECURED_MESSAGE_OPAQUE_ELEMENT_SMDATA_ID_SUPPORTED_VERSION 1
#define SPDM_VERSION_NUMBER_SHIFT_BIT 8
#define SPDM_OPAQUE_DATA_VERSION_COUNT 1

#define SPDM_VERSION_1_1_BIN_CONCAT_LABEL "spdm1.1 "
#define SPDM_BIN_STR_0_LABEL "derived"
#define SPDM_BIN_STR_1_LABEL "req hs data"
#define SPDM_BIN_STR_2_LABEL "rsp hs data"
#define SPDM_BIN_STR_3_LABEL "req app data"
#define SPDM_BIN_STR_4_LABEL "rsp app data"
#define SPDM_BIN_STR_5_LABEL "key"
#define SPDM_BIN_STR_6_LABEL "iv"
#define SPDM_BIN_STR_7_LABEL "finished"
#define SPDM_BIN_STR_8_LABEL "exp master"
#define SPDM_BIN_STR_9_LABEL "traffic upd"

#define SPDM_CRYPTO_HKDF_EXTRACT 0 
#define SPDM_CRYPTO_HKDF_EXPAND  1

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

#define SPDM_MUT_AUTH_GET_CERTIFICATE_LENGTH 0x3B9
#define SPDM_MUT_AUTH_GET_CERT_DEFAULT_OFFSET 0U

#define SPDM_CHALLENGE_AUTH_NONCE_OFFSET        (4 + SPDM_SHA384_LEN)

#define CHALLENGE_VERSION_OFFSET                      0
#define CHALLENGE_RESP_CODE_OFFSET                    1
#define CHALLENGE_REQ_SLOT_OFFSET                     2
#define CHALLENGE_SLOT_DATA_OFFSET                    3
#define CHALLENGE_CHAIN_HASH_OFFSET                   4
#define CHALLENGE_NONCE_OFFSET                        52
#define CHAL_NEXT_OFS_AFTER_NONCE_END         84

#define NOUNCE_DATA_SIZE                        32

#define OPAQUE_DATA_SZ                          256

#define OPAQUE_FIELDS_SZ                        2

#define OPAQUE_DATA_LEN                         2
#define OPAQUE_DATA_ELEMENTS                    1

#define SIGNATURE_R_TERM_SZ                     48u

#define SIGNATURE_S_TERM_SZ                     48u

#define SIGNATURE_SIZE 96

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

// Secure Session
#define SPDM_KEY_EXCHANGE            0xE4
#define SPDM_KEY_EXCHANGE_RSP        0x64

#define SPDM_FINISH                  0xE5
#define SPDM_FINISH_RSP              0x65

#define SPDM_END_SESSION             0xEC
#define SPDM_END_SESSION_RSP         0x6C

#define SPDM_HEARTBEAT               0xE8
#define SPDM_HEARTBEAT_ACK           0x68

#define SPDM_ENCAP_REQ_REQ           0xEA
#define SPDM_ENCAP_REQ_RSP           0x6A

#define DELIVER_ENCAP_RSP_REQ        0xEB
#define DELIVER_ENCAP_RSP_RSP        0x6B

#define SPDM_ERROR_RESP              0x7F
#define SPDM_ERROR_INVLD_RQ          0x01
#define SPDM_ERROR_UNXPTD_RQ_CODE    0x04
#define SPDM_ERROR_USPRTD_RQ_CODE    0x07
#define SPDM_ERROR_MJR_VRS_MISMATCH  0x41
#define SPDM_ERROR_RESYNCH           0x43
#define SPDM_ERROR_UNSPECIFIED       0x05
#define SPDM_ERROR_BUSY              0x03

// spdm error codes for secure session
#define SPDM_ERROR_INVALID_SESSION   0x02
#define SPDM_ERROR_DECRYPT_ERROR     0x06
#define SPDM_ERROR_REQUEST_IN_FLIGHT 0x08
#define SPDM_ERROR_SESSION_LIMIT_EXCEEDED 0x0A
#define SPDM_ERROR_INVALID_RESPONSE_CODE 0x09


#define SPDM_RESP_IF_RDY_RQ          0xFF

//SPDM_GET_VERSION command response bits
#define SPDM_VER_NUM_ENTRY_COUNT           0x01
#define SPDM_VER_NUM_MAJOR_MINOR           0x11
#define SPDM_VER_NUM_UPDATE_ALPHA          0x00u
#define SPDM_MAX_VER_ENTRY_COUNT           0x1d //29
////SPDM_GET_CAPABILITIES command response bits
//0x11 = 17 - ctexponent = 17; 2^ctexponent = 2^17 = 132 ms timeout for commands
#define SPDM_CAP_CT_EXP                    0x11
#define SPDM_CAP_FLAG_BYTE1                0xD6 // secure session - bit 7 MAC_CAP, bit 6 ENCRYPT_CAP
#define SPDM_CAP_FLAG_BYTE2                0x33 // secure session - bit 5 HBEAT_CAP, bit 4 ENCAP_CAP, bit 1 KEY_EXC_CAP, bit 0 MUT_AUTH_CAP
#define SPDM_CAP_FLAG_BYTE3                0x00
#define SPDM_CAP_FLAG_BYTE4                0x00

#define SPDM_CAP_FLAG_BYTE4_OFFSET         6u
#define SPDM_CAP_FLAG_BYTE3_OFFSET         7u
#define SPDM_CAP_FLAG_BYTE2_OFFSET         8u
#define SPDM_CAP_FLAG_BYTE1_OFFSET         9u

#define SPDM_CMD_CHAIN_INVALID                 2U
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
#define SPDM_NEG_ALG_NO_ALG_STRC           0x04 // Secure Session DHE, AEAD, ReqBaseAsymAlg, KeySchedule
#define SPDM_NEG_ALG_LENGTH_RESP           0x34 // 52 - breakup - already supported  - 40 + Secure Session 12 for additional 3 RespAlgStruct
#define SPDM_NEG_ALG_MSR_SPC               0x01
#define SPDM_NEG_ALG_MSR_HASH_ALGO         0x04 //sha384
#define SPDM_NEG_ALG_BASE_ASYM_SEL         0x80
#define SPDM_NEG_ALG_BASE_HASH_SEL         0x02
#define SPDM_NEG_ALG_NO_EXT_SIG            0x00
#define SPDM_NEG_ALG_NO_EXT_HASH           0x00

// DHE
#define ALG_TYPE_DHE                0x02
#define ALG_CNT_DHE                 0x20
#define DHE_SECP_384r1              0x0010

// AED
#define ALG_TYPE_AEAD               0x03
#define ALG_CNT_AEAD                0x20
#define AES_256_GCM                 0x0002

//ReqBaseAsymAlg
#define ALG_TYPE_REQ_BASE_ASYM_ALG  0x04
#define ALG_CNT_REQ_BASE_ASYM_ALG   0x20
#define TPM_ALG_ECDSA_ECC_NIST_P384 0x0080

// KeySchedule
#define ALG_TYPE_KEY_SCHEDULE       0x05
#define ALG_CNT_KEY_SCHEDULE        0x20
#define SPDM_KEY_SCHEDULE           0x0001

#define MAX_NUM_BYTES_PLD           64u


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
#define NO_OF_MSR_INDICES                              0x04U
#define NO_OF_MSR_MANIFEST                             0x01U
#define TOTAL_NO_OF_MSR_INDICES                        (NO_OF_MSR_MANIFEST + NO_OF_MSR_INDICES)

#define SIZE_OF_ONE_MSR_BLOCK                          55u

#define SIZE_OF_AP_MSR_BLOCK_FORMAT                    1507u

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

#define AP_CONFIG_SELCT_DGST_RAM                       0x04 //digest

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
#define MEAS_PLD_LEN_WITHOUT_SIGN                       4U
#define MEAS_SPI_DATA_START                             3U // For measurements, we start at index 3 for secure session purpose

#define MEASUREMENT_VERSION_OFFSET                      3
#define MEASUREMENT_RESP_CODE_OFFSET                    4
#define MEASUREMENT_INDICES_OFFSET                      5
#define MEASUREMENT_REQ_SLOT_OFFSET                     6
#define MEASUREMENT_NUM_BLOCKS_OFFSET                   7
#define MEASUREMENT_REC_LEN_0                           8
#define MEASUREMENT_REC_LEN_1                           9
#define MEASUREMENT_REC_LEN_2                           10
#define MEAS_NEXT_OFS_AFTER_REC_LEN           11
#define CERT_CHAIN_INVALID                              1
#define CERT_CHAIN_VALID                                0

#define ROOT_START_OFFSET                              52U // Root certificate start offset in chain
#define ROOT_HASH_END_OFFSET                           51U // Root hash end offset in chain
#define SLOT_NUM_CHAIN_LENGTH_BYTES                     6U // account for slot number reserved portion length chain length in cert response

#define DMTF_MSR_VAL_SIZE 2U

#define OFFSET_FOR_CERT_START 52U
#define OFFSET_FOR_ROOT_HASH  4U
#define SERIAL_NUM_LEN 13U
#define SIG_ALGO_LEN 12U
#define CERT_PARSER_DEFAULT_VAL 1U
#define RAW_LENGTH_OF_DATA_FOR_SIGN 4U
#define SS_ENCAP_REQUST_MOVE_1BYTE 1U
#define APP_DATA_SIZE_MINUS_MSG_TYPE_SIZE 1U
#define APP_DATA_SIZE_ADD_MSG_TYPE_SIZE 1U
#define BIT_16 16U
#define MCTP_TRANSPORT_LAYER_SIZE 64U
#define MCTP_TRANSPORT_LAYER_MINUS_MSG_TYPE_SIZE 63U
#define OFFSET_FOR_START 0U
#define BIT_8 8U
#define CERT_PARSE_PUB_KEY_SHIFT 2U
#define CERT_PARSE_EXTENSIONS 0xA3
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
    uint8_t algo_struct[16];
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

// typedef struct CHALLENGE_FIELDS
// {
//     uint8_t version;
//     uint8_t resp_code;
//     uint8_t resp_attr;
//     uint8_t slot_mask;
//     uint8_t hash_of_cert_chain[SPDM_SHA384_LEN];
//     uint8_t random_val[32];
//     uint8_t opaq_len[2];
//     uint8_t opaq_data[2];

// } __attribute__((packed)) CHALLENGE_FIELDS;


// /******************************************************************************/
// /** Structure holding the response fields for measurement response
// *******************************************************************************/

// typedef struct MEASUREMENT_FIELDS
// {
//     uint8_t version;
//     uint8_t resp_code;
//     uint8_t param1;
//     uint8_t param2;
//     uint8_t num_of_blocks;
//     uint8_t msr_rec_len[3];
//     uint8_t opaq_len[2];
//     uint8_t opaq_data[OPAQUE_DATA_SZ];

// } __attribute__((packed)) MEASUREMENT_FIELDS;


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
/** Structure holding the response fields for KEY_EXCHANGE
*******************************************************************************/
typedef struct KEY_EXCHANGE_FIELDS
{
    uint8_t version;
    uint8_t resp_code;
    uint8_t param1;
    uint8_t param2;
    uint16_t rsp_session_id;
    uint8_t muth_auth_requested;
    uint8_t slot_id;
    uint8_t random_val[NOUNCE_DATA_SIZE];
    uint8_t exchange_data[PUB_KEY_CODE_LENGTH];
    // uint8_t msr_hash[SPDM_SHA384_LEN];
    // uint16_t opaque_data_len;
    // uint8_t opaque_data;
    // uint8_t signature[48];
    // uint8_t req_verify_data[SPDM_SHA384_LEN];
} __attribute__((packed)) KEY_EXCHANGE_FIELDS;

/******************************************************************************/
/** Structure holding the response fields for FINISH
*******************************************************************************/
typedef struct FINISH_RSP_FIELDS
{
    uint8_t version;
    uint8_t resp_code;
    uint8_t param1;
    uint8_t param2;
    uint8_t res_verify_data[SPDM_SHA384_LEN];
} __attribute__((packed)) FINISH_RSP_FIELDS;

/******************************************************************************/
/** Structure holding the response fields for END_SESSION
*******************************************************************************/
typedef struct SS_END_SESSION_RSP_FIELDS
{
    uint8_t version;
    uint8_t resp_code;
    uint8_t param1;
    uint8_t param2;
} __attribute__((packed)) SS_END_SESSION_RSP_FIELDS;

/******************************************************************************/
/** Structure holding the response fields for HEARTBEAT
*******************************************************************************/
typedef struct HEARTBEAT_RSP_FIELDS
{
    uint8_t version;
    uint8_t resp_code;
    uint8_t param1;
    uint8_t param2;
} __attribute__((packed)) HEARTBEAT_RSP_FIELDS;

/******************************************************************************/
/** Structure holding the response fields for ENCAPSULATED_REQ response messgae
*******************************************************************************/
typedef struct SPDM_ENCAP_REQ_RSP_FIELDS
{
    uint8_t version;
    uint8_t resp_code;
    uint8_t param1;
    uint8_t param2;
} __attribute__((packed)) SPDM_ENCAP_REQ_RSP_FIELDS;

/******************************************************************************/
/** Structure holding the response fields for GET_DIGEST request messgae
*******************************************************************************/
typedef struct GET_DIGEST_REQ_FIELDS
{
    uint8_t version;
    uint8_t resp_code;
    uint8_t param1;
    uint8_t param2;
} __attribute__((packed)) GET_DIGEST_REQ_FIELDS;

/******************************************************************************/
/** Structure holding the response fields for GET_CERTIFICATE request messgae
*******************************************************************************/
typedef struct GET_CERT_REQ_FIELDS
{
    uint8_t version;
    uint8_t resp_code;
    uint8_t param1;
    uint8_t param2;
    uint16_t offset;
    uint16_t length;
} __attribute__((packed)) GET_CERT_REQ_FIELDS;

/******************************************************************/
/** Structure holding the opaque element table in response in KEY_EXCHANGE
*******************************************************************/
typedef struct OPAQUE_ELEMENT_SUPPORTED_VERSION
{
    uint8_t sm_data_version;
    uint8_t sm_data_id;
    uint8_t version_count;
    uint16_t version_number;
    uint8_t reserved; //align padding
} __attribute__((packed)) OPAQUE_ELEMENT_SUPPORTED_VERSION;

/******************************************************************/
/** Structure holding the opaque element table in response in KEY_EXCHANGE
*******************************************************************/
typedef struct OPAQUE_ELEMENT_TABLE_HEADER
{
    uint8_t id;
    uint8_t vendor_len;
    uint16_t opaque_element_data_len;
} __attribute__((packed)) OPAQUE_ELEMENT_TABLE_HEADER;

/******************************************************************/
/** Structure holding the general opaque response in KEY_EXCHANGE
*******************************************************************/
typedef struct GENERAL_OPAQUE_DATA
{
    uint16_t opaque_len;
    uint32_t spec_id;
    uint8_t opaque_version;
    uint8_t total_elements;
    uint16_t reserved;
    OPAQUE_ELEMENT_TABLE_HEADER opaque_element_table_header;
    OPAQUE_ELEMENT_SUPPORTED_VERSION opaque_element_supported_version;
} __attribute__((packed)) GENERAL_OPAQUE_DATA;


/******************************************************************/
/** Structure holding the encap request and response msg function pointer
*******************************************************************/
typedef uint8_t (*spdm_get_encap_request_func)(
   void *encap_request, uint8_t *size);

typedef uint8_t (*spdm_process_encap_response_func)(void);

typedef struct SS_ENCAP_RESPONSE_STRUCT {
    uint8_t request_op_code;
    spdm_get_encap_request_func get_encap_request;
    spdm_process_encap_response_func process_encap_response;
} SS_ENCAP_RESPONSE_STRUCT;

typedef struct {
    uint32_t session_id;
    uint16_t seq_num;
    uint16_t length; /* The length of the remaining data, including application_data_length(O), payload, Random(O) and MAC.*/
} spdm_secured_message_a_data_header_t;

typedef struct {
    uint16_t application_data_length; /* The length of the payload*/
} spdm_secured_message_cipher_header_t;

// X509 certificate parse
typedef struct {
    uint8_t tbs_Certificate[13];

    // serial number
    uint8_t serial_number_start;
    uint16_t serial_number_len;
    uint8_t *serial_number; 
    
    // signature algo
    uint8_t sig_algo[12]; // includes tag, length, data - for ecdsa-with-SHA384

    // issuer
    uint8_t issuer_start; 
    uint16_t issuer_len;
    uint8_t *issuer_data;

    // validity
    uint8_t validity_start; // includes tag, length, data
    uint16_t validity_len;
    uint8_t *validity_data;

    // subject
    uint8_t subject_start;
    uint16_t subject_len;
    uint8_t *subject_data;

    // subject public key info
    uint8_t spki_start;
    uint16_t spki_len;
    uint8_t spki_alg_oid_start;
    uint16_t spki_alg_oid_len;
    uint8_t *spki_alg_oid_data;
    uint8_t public_key_start;
    uint16_t public_key_len;
    uint8_t *public_key;

    // Extensions
    uint8_t ext_start;
    uint16_t ext_len;
    uint8_t *ext_data;

    // Signature algorithm
    uint8_t sig_alg[12];

    // signature 
    uint8_t sig_start;
    uint16_t sig_len;
    uint8_t *signature;
} X509_struct_t;

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
 * @param len - length of data to be copied
 * @param offset - offset at which data starts
 * @return 1 if payload size goes greater than 1024
 *         0 success
 *******************************************************************************/
uint8_t spdm_pkt_fill_buffer(MCTP_PKT_BUF *spdm_msg_rx_buf, SPDM_CONTEXT *spdmContext, uint8_t len, uint8_t offset);

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

/******************************************************************************/
/** function to return signature type to be used
 * @param None
 * @return         : 0U (USE_OTP_GENERATED_PRIVATE_KEY)
 *                   1U (USE_PUF_GENERATED_PRIVATE_KEY)
 *                   2U (INVALID_SIGNATURE_FLAGS)
 *******************************************************************************/
uint8_t signature_type();

/******************************************************************************/
/** function to store signature type
 * @param void
 * @return void
 *******************************************************************************/
void spdm_pkt_store_signature_type(void);

/******************************************************************************/
/** function to update APFW data stored in MSR after reauth is done and calculate
 * concatanate hash
 * @param spdmContext - SPDM module context
 * @return void
 *******************************************************************************/
void spdm_update_ap_msr_data(SPDM_CONTEXT *spdmContext);

/******************************************************************************/
/** Function to derive final session id
 * @param req_session_id - Session ID of requester (host)
 * @param resp_session_id  - Session ID of responder
 * @return session_id - final session ID - concatenation of req's and resp's
 *******************************************************************************/
uint32_t spdm_derive_session_id(uint16_t req_session_id, uint16_t resp_session_id);

/******************************************************************************/
/** spdm_crypto_ops_generate_dhe_key_pair();
* Generate DHE key pair and get the public key
* @param pub_key  Pointer to save the public key
* @param pvt_key  Pointer to save the private key
* @param session_id  usage context value
* @return 1 - fail, 0 - pass
*******************************************************************************/
uint8_t spdm_crypto_ops_generate_dhe_key_pair(uint8_t *pub_key, uint8_t *pvt_key, uint32_t session_id);

/******************************************************************************/
/** function to parse the enc response received
 * @param encap_request the response received from host
 * @param size pointer to size
 * @return true/false
 *******************************************************************************/
uint8_t spdm_process_encapsulated_response(void *encap_request, uint8_t *size);

/******************************************************************************/
/** function to encap get_certificate request for mutual authentication
 * @param get_cert_request - pointer to hold GET_CERT req
 * @param size  - size of get_certificate request
 * @return true/false
 *******************************************************************************/
uint8_t spdm_get_encap_request_get_certificate(void *get_cert_request, uint8_t *size);

/******************************************************************************/
/** function to parse get_certificate response for mutual authentication
 * @param None
 * @return true/false
 *******************************************************************************/
uint8_t spdm_process_encap_response_certificate(void);

/******************************************************************************/
/** Function to generate session handshake key
 * @param spdmContext - SPDM module context
 * @return true or false
 *******************************************************************************/
int8_t spdm_generate_session_handshake_key(SPDM_CONTEXT *spdmContext);

/******************************************************************************/
/** Function to derive AEAD and IV for secure session
 * @param spdmContext - spdmContext pointer
 * @param major_secret  - pointer to major secret used for encryption/decryption
 * @param key - derived key pointer
 * @param iv - derived key pointer
 * @return true/false
 *******************************************************************************/
int8_t spdm_generate_aead_key_and_iv(SPDM_CONTEXT *spdmContext,
     uint8_t *major_secret, uint8_t *key, uint8_t *iv);

/******************************************************************************/
/** Function to generate finished key
 * @param spdmContext - spdmContext pointer
 * @param handshake_secret  - pointer to handshake secret used for encryption/decryption
 * @param finished_key - derived finished key pointer
 * @return true/false
 *******************************************************************************/
int8_t spdm_generate_finished_key(SPDM_CONTEXT *spdmContext, uint8_t *handshake_secret, uint8_t *finished_key);

/******************************************************************************/
/** function to concatanate bin string for key schedule
 * @param spdm_version - SPDM version
 * @param label - label data pointer
 * @param label_size - label size
 * @param context - context for bin string derivation pointer
 * @param length - length of context
 * @param hash_size - size of hash
 * @param out_bin - output data pointer
 * @param out_bin_size - output size pointer
 * @return void
 *******************************************************************************/
void spdm_bin_concat(uint16_t spdm_version,
                        const char *label, size_t label_size,
                        const uint8_t *context, uint16_t length,
                        uint8_t hash_size, uint8_t *out_bin,
                        uint8_t *out_bin_size);

/******************************************************************************/
/** function to parse certificate
 * @param cert_data - pointer to cert data
 * @return void
 *******************************************************************************/
uint8_t spdm_x509_parse_certificate(uint8_t *cert_data, uint16_t len);

/******************************************************************************/
/** function to encrypt spdm data
 * @param spdmContext pointer to spdm Context
 * @param is_request_message request or response message
 * @param app_message_size size of application message
 * @param app_msg pointer to app msg data
 * @param secured_msg pointer to encrypted msg
 * @return true/false
 *******************************************************************************/
uint8_t spdm_pkt_process_encrypt_spdm_data(SPDM_CONTEXT *spdmContext, uint8_t is_request_message, uint16_t app_message_size, uint8_t *app_msg, uint8_t *secured_msg);

/******************************************************************************/
/** function to get length of certificate
 * @param cert_data - pointer to cert data
 * @return void
 *******************************************************************************/
uint16_t spdm_x509_get_cert_len(uint8_t *cert_data);

/******************************************************************************/
/** function to parse certificate
 * @param cert_data - pointer to cert data
 * @param len - has the length of certificate
 * @return void
 *******************************************************************************/
uint8_t spdm_x509_parse_certificate_chain();

/******************************************************************************/
/** function to move to next opcode as part of mutual auth
 * @param None
 * @return None
 *******************************************************************************/
void spdm_encap_move_to_next_opcode();

/******************************************************************************/
/** spdm_secure_session_set_session_state();
* Sets secure session state
* @param spdmContext pointer to spdmContext
* @param val state value to be set
* @return None
*******************************************************************************/
void spdm_secure_session_set_session_state(SPDM_CONTEXT *spdmContext, SPDM_SESSION_STATE val);

/******************************************************************************/
/** once spdm msg is ready to transmit, spdm state machine is set to transmit mode
 * function to load the spdm data to mctp tx buffer
 * @param none
 * @return void
 *******************************************************************************/
void spdm_pkt_tx_packet();

/******************************************************************************
* Cleans up keys, major secrets derivated
* @param spdmContext pointer to spdmContext
* @return None
*******************************************************************************/
void spdm_secure_session_cleanup(SPDM_CONTEXT *spdmContext);

/******************************************************************************/
/** SPDMHB_timer_callback();
* SPDMHB timer callback
* @param TimerHandle_t pxTimer
* @return None
*******************************************************************************/
void SPDMHB_timer_callback(TimerHandle_t pxTimer);

/******************************************************************************/
/** function to frame error response message
 * @param error_handle - error value
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @param get_cmd  - Current request / response code
 * @return None
 *******************************************************************************/
void spdm_fill_error_response(uint8_t error_handle, MCTP_PKT_BUF *spdm_buf_tx, uint8_t get_cmd);