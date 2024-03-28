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

/** @file spdm_pkt_prcs.c
 * Source file for SPDM packet processing
 */

#include "spdm_task.h"
#include "../mctp/mctp_smbus.h"
#include "../mctp/mctp_base.h"
#include "spdm_pkt_prcs.h"
#include "spdm.h"
#include "spdm_common.h"
#include "pldm/pldm_common.h"
#include "app.h"

#define SPI_DATA_MAX_BUFF              4096U
#define CONCAT_MSRMENT_BLOCKS_SIZE     (SIZE_OF_ONE_MSR_BLOCK * NO_OF_MSR_INDICES) + SIZE_OF_AP_MSR_BLOCK_FORMAT
#define SS_ENCAP_BUF_SIZE              20

// extern SPDM_BSS1_ATTR DI_CONTEXT_SPDM *spdm_di_context;
extern SPDM_BSS1_ATTR uint8_t curr_ec_id;
SPDM_BSS1_ATTR_8ALIGNED uint8_t hw_config_buff[SRAM_MBOX_HW_CONFIG_SIZE] __attribute__((aligned(8)));

SPDM_BSS0_ATTR_8ALIGNED uint8_t get_mctp_pld[MAX_SIZE_CERTIFICATE] __attribute__((aligned(8)));
SPDM_BSS1_ATTR_8ALIGNED MCTP_PKT_BUF mctp_pktbuf_tx  __attribute__((aligned(8)));
SPDM_BSS1_ATTR_8ALIGNED MCTP_PKT_BUF spdm_pktbuf_rx  __attribute__((aligned(8)));

SPDM_BSS1_ATTR_8ALIGNED MCTP_PKT_BUF spdm_pktbuf[1] __attribute__((aligned(8)));
SPDM_BSS1_ATTR uint8_t spdm_tx_state;

SPDM_BSS2_ATTR_8ALIGNED uint8_t spi_data[SPI_DATA_BUF_SIZE] __attribute__((aligned(8))); // This buffer is externed and used to read/Write certificate
SPDM_BSS1_ATTR VERSION_NUM_ENTRY_TABLE version_tbl_buf[2 * SPDM_VER_NUM_ENTRY_COUNT];
SPDM_BSS1_ATTR_8ALIGNED REQ_BASE_ASYM_ALG struct_algo[SPDM_NEG_ALG_NO_ALG_STRC] __attribute__((aligned(8)));
SPDM_BSS2_ATTR uint8_t spdm_rqst_cur_state;
SPDM_BSS1_ATTR MCTP_PKT_BUF *mctp_buf_tx;
SPDM_BSS1_ATTR MCTP_PKT_BUF *spdm_buf_tx;
SPDM_BSS1_ATTR uint32_t packet_sz;
SPDM_BSS1_ATTR uint8_t pkt_seq_mctp;
SPDM_BSS1_ATTR uint8_t first_pkt;
SPDM_BSS1_ATTR uint8_t tbl_entry;
SPDM_BSS1_ATTR uint16_t pld_index;
SPDM_BSS1_ATTR uint16_t time_stamp;
SPDM_BSS1_ATTR uint32_t cert_len;
SPDM_BSS2_ATTR uint32_t cert2_base_addr;
SPDM_BSS2_ATTR uint8_t chain_avail_bits_mask;
SPDM_BSS2_ATTR uint8_t slot_position;
SPDM_BSS2_ATTR_8ALIGNED CERT_SLOT slot_buf[MAX_SLOTS]  __attribute__((aligned(8))); // 8 slots structure; each slot containing a chain

SPDM_BSS2_ATTR CERTIFICATE cert_buf[MAX_CERTIFICATES] __attribute__((aligned(8))); // 64 certificates structure

// For challenge authentication, required buffers
SPDM_BSS1_ATTR_4ALIGNED uint8_t pvt_key[PVT_KEY_CODE_LENGTH] __attribute__((aligned(4))); // This buffer holds AK_PVT_KEY / AK_PVT_KEY_CODE
// For secure session, need to store the DHE pvt & public key
SPDM_BSS1_ATTR_4ALIGNED uint8_t ss_pvt_key[PVT_KEY_CODE_LENGTH] __attribute__((aligned(4)));
SPDM_BSS1_ATTR_4ALIGNED uint8_t ss_pub_key[PUB_KEY_CODE_LENGTH] __attribute__((aligned(4)));
SPDM_BSS2_ATTR_8ALIGNED uint8_t responder_verify_data[SPDM_SHA384_LEN] __attribute__((aligned(8)));

SPDM_BSS1_ATTR_8ALIGNED ecdsa_signature_t ecdsa_signature __attribute__((aligned(8)));

// end of challenge authentication, required buffers
SPDM_BSS1_ATTR_8ALIGNED uint8_t random_no[CURVE_384_SZ]  __attribute__((aligned(8)));
//--GET CERTIFICATE RELATED VARIABLES AND BUFFERS ---//
extern SPDM_BSS1_ATTR uint8_t AP_CFG_cert_buffer[sizeof(CFG_CERT)];
SPDM_BSS1_ATTR CFG_CERT *ptr_cert_buffer;

SPDM_BSS2_ATTR uint8_t requested_slot;
SPDM_BSS2_ATTR uint16_t cert_offset_to_sent;
SPDM_BSS2_ATTR uint16_t challenge_tracker;
SPDM_BSS2_ATTR uint16_t meas_tracker;
SPDM_BSS1_ATTR uint16_t RemainderLength;
SPDM_BSS1_ATTR uint16_t PortionLength;
SPDM_BSS1_ATTR uint16_t opaq_size_remaining;
SPDM_BSS1_ATTR volatile uint8_t first_get_cert_response_sent;
SPDM_BSS1_ATTR volatile uint16_t bytes_sent_over_mctp_for_cert;
SPDM_BSS1_ATTR uint16_t remaining_bytes_to_sent;
SPDM_BSS1_ATTR uint16_t pending_bytes;
SPDM_BSS1_ATTR uint16_t global_offset_for_pending_bytes;
SPDM_BSS1_ATTR uint8_t requester_cert_chain_slot;

// CHALLENGE COMMAND RELATED VARIABLES AND BUFFERS//

SPDM_BSS2_ATTR uint16_t actual_challenge_length;
SPDM_BSS2_ATTR uint16_t actual_meas_length;
SPDM_BSS1_ATTR_8ALIGNED uint8_t nonce_data[NOUNCE_DATA_SIZE]  __attribute__((aligned(8)));
SPDM_BSS2_ATTR uint8_t first_challenge_resp_sent;
SPDM_BSS2_ATTR uint8_t first_meas_resp_sent;
SPDM_BSS2_ATTR uint8_t digest_state_machine;
SPDM_BSS2_ATTR uint8_t nonce_offset;
SPDM_BSS2_ATTR uint16_t signature_offset;

// KEY EXCHANGE related variables and buffers
SPDM_BSS2_ATTR uint8_t exchange_data_offset;
SPDM_BSS2_ATTR uint8_t key_exchange_rsp_multiple_pkt_state;

SPDM_BSS1_ATTR_8ALIGNED uint8_t hash_of_req_buffer[SPDM_SHA384_LEN] __attribute__((aligned(8)));
SPDM_BSS1_ATTR_8ALIGNED uint8_t hash_of_chains[MAX_SHA384_BUF_SIZE] __attribute__((aligned(8)));
SPDM_BSS1_ATTR_8ALIGNED uint8_t hash_of_req_chains[SPDM_SHA384_LEN] __attribute__((aligned(8))); // hash of requester chain for mutual authentication

SPDM_BSS2_ATTR_8ALIGNED uint16_t req_and_response_sz[2] __attribute__((aligned(8)));

// response fields pointers for each command responses
SPDM_BSS1_ATTR_8ALIGNED GET_VERSION_RESP_FIELDS get_ver_resp_object __attribute__((aligned(8)));
SPDM_BSS1_ATTR_8ALIGNED uint8_t get_version_response[sizeof(get_ver_resp_object)] __attribute__((aligned(8))); 

SPDM_BSS1_ATTR_8ALIGNED GET_CAP_RESP_FIELDS get_cap_resp_object __attribute__((aligned(8)));
SPDM_BSS1_ATTR_8ALIGNED uint8_t get_cap_response[sizeof(get_cap_resp_object)] __attribute__((aligned(8)));

SPDM_BSS2_ATTR_8ALIGNED NEG_ALGO_RESP_FIELDS nego_algo_resp_object __attribute__((aligned(8)));
SPDM_BSS2_ATTR_8ALIGNED uint8_t nego_algo_response[sizeof(nego_algo_resp_object)] __attribute__((aligned(8)));
SPDM_BSS2_ATTR_8ALIGNED GET_DIGEST_FIELDS get_digest_resp_object __attribute__((aligned(8)));
SPDM_BSS2_ATTR_8ALIGNED uint8_t get_digest_response[sizeof(get_digest_resp_object)] __attribute__((aligned(8)));
SPDM_BSS2_ATTR_8ALIGNED GET_CERTIFICATE_FIELDS get_cert_resp_object __attribute__((aligned(8)));
SPDM_BSS2_ATTR_8ALIGNED uint8_t get_cert_response[sizeof(get_cert_resp_object)]  __attribute__((aligned(8)));

// Secure Session
SPDM_BSS1_ATTR_8ALIGNED KEY_EXCHANGE_FIELDS key_exchange_resp_obj __attribute__((aligned(8)));
SPDM_BSS1_ATTR_8ALIGNED FINISH_RSP_FIELDS finish_rsp_fields __attribute__((aligned(8)));
SPDM_BSS1_ATTR uint32_t final_session_id;
SPDM_BSS1_ATTR_8ALIGNED GENERAL_OPAQUE_DATA opaque_data __attribute__((aligned(8)));
SPDM_BSS1_ATTR_8ALIGNED HASH_CTX_DATA ctx_ptr __attribute__((aligned(8))); // for SPDM normal message
SPDM_BSS0_ATTR HASH_CTX_DATA ctx_ptr_arr_ss[HASH_CTX_MAX] __attribute__((aligned(8))); 
SPDM_BSS1_ATTR_8ALIGNED uint8_t th_hash[SHA384_LEN_BYTES] __attribute__((aligned(8)));
SPDM_BSS1_ATTR_8ALIGNED uint8_t requester_public_key[97] __attribute__((aligned(8))); // extra 1 byte to include 0x04 for shared secret derivation
SPDM_BSS1_ATTR uint8_t message_type; // secured or non-secured message
//-----MEASUREMENT COMMAND OPERATIONS RELATED BUFFERS AND VARIABLES------//;

// concatenated measurement blocks structure
SPDM_BSS1_ATTR_8ALIGNED MEASUREMENT_BLOCK msr_block_buffer[NO_OF_MSR_INDICES] __attribute__((aligned(8)));

// AP measurement value
SPDM_BSS1_ATTR_8ALIGNED MEASUREMENT_BLOCK_MANIFEST msr_block_manifest  __attribute__((aligned(8)));

// hash of all msr block
SPDM_BSS1_ATTR_8ALIGNED uint8_t hash_concat_msrments_val[SPDM_SHA384_LEN]  __attribute__((aligned(8)));

// hash of all msr block
SPDM_BSS1_ATTR_8ALIGNED uint8_t hash_concat_tcb_msrments_val[SPDM_SHA384_LEN]  __attribute__((aligned(8)));

// will have all msr blocks concatenated for sending in response
SPDM_BSS2_ATTR_8ALIGNED uint8_t concat_msrment_blocks[CONCAT_MSRMENT_BLOCKS_SIZE] __attribute__((aligned(8)));

SPDM_BSS1_ATTR_8ALIGNED uint8_t msr_opaque_buf[OPAQUE_DATA_SZ + OPAQUE_DATA_LEN]  __attribute__((aligned(8)));

SPDM_BSS1_ATTR_8ALIGNED MEASUREMENT_VARIABLES measurement_var  __attribute__((aligned(8)));

void read_chain_from_offset(uint16_t start_offset, uint16_t end_offset, uint32_t spi_data_offset, uint8_t slot);
SPDM_BSS1_ATTR uint16_t read_bytes_from_chain;
SPDM_BSS2_ATTR uint16_t START_OFFSET_IN_BUFFER;
SPDM_BSS2_ATTR uint16_t BUFFER_END_OFFSET;
SPDM_BSS2_ATTR uint16_t length;

// concurret flash access
SPDM_BSS1_ATTR uint8_t spdm_flash_busy;
extern PLDM_BSS1_ATTR uint8_t pldm_flash_busy;
SPDM_BSS2_ATTR uint8_t is_puf;
extern SPDM_BSS1_ATTR uint8_t count_of_ap_msr_data_received;

extern SPDM_BSS1_ATTR uint8_t pldm_ap_cleaned_on_reauth;
extern SPDM_BSS1_ATTR uint8_t msr_ap_cleaned_on_reauth;

// encapsulated request 
SPDM_BSS1_ATTR_8ALIGNED uint8_t encapreq_response[SS_ENCAP_BUF_SIZE]  __attribute__((aligned(8)));
SPDM_BSS1_ATTR uint8_t request_op_code;
SPDM_BSS2_ATTR uint16_t get_cert_offset, get_cert_length;
SPDM_BSS0_ATTR_8ALIGNED uint8_t buf_for_encryption[SPI_DATA_BUF_SIZE]  __attribute__((aligned(8)));


// offset and remaining data for SS GET_MSR
SPDM_BSS2_ATTR uint32_t offset_for_ss_get_msr;
SPDM_BSS2_ATTR uint32_t data_size_for_ss_get_msr;
SPDM_BSS2_ATTR uint8_t state_for_msr_ss;
SPDM_BSS2_ATTR uint32_t remaining_for_ss, offset_for_msr_ss;
SPDM_BSS2_ATTR_8ALIGNED X509_struct_t X509_struct  __attribute__((aligned(8)));
SPDM_BSS2_ATTR_8ALIGNED uint8_t bin_str[128] __attribute__((aligned(8)));
SPDM_BSS2_ATTR uint8_t bin_str_size;
SPDM_BSS1_ATTR uint8_t salt[SPDM_SHA384_LEN] __attribute__((aligned(8)));
SPDM_BSS2_ATTR uint8_t is_root_cert;
SPDM_BSS1_ATTR_8ALIGNED uint8_t zero_filled_buffer[MAX_HASH_SIZE]  __attribute__((aligned(8)));

SPDM_BSS1_ATTR     size_t total_secured_message_size; // data in buf_encryption - starting from session id
extern SPDM_BSS1_ATTR_8ALIGNED uint8_t root_pub_key_hash[SB_KEY_HASH_SIZE_MAX]  __attribute__((aligned(8)));
SPDM_BSS1_ATTR uint8_t SPDMHB_timer_started;
SPDM_BSS1_ATTR uint16_t size_for_msr;
SPDM_BSS1_ATTR uint8_t error_handle_secure_session;

/******************************************************************************/
/** spdm_get_measurements
 * This function can be used to get hash of measurement data. 
 * @param buff_ptr         Pointer to hold the measurement data
 * @param index            measurement id i.e. 1,2,3,4
 * @return                 None
 * @note
 ******************************************************************************/
void spdm_get_measurements(uint8_t *buffer_ptr, uint8_t index)
{
    uint8_t otp_array[286];
    switch (index)
    {
        case INDEX1:
            SRAM_MBOX_API_rom_hash_read(buffer_ptr);
            break;

        case INDEX2:
            SRAM_MBOX_API_ec_fw_hash_read(buffer_ptr);
            break;

        case INDEX3:
            // read hw config data
            SRAM_MBOX_API_hw_config_read(&hw_config_buff[0]);

            hw_config_buff[0] &= 0x1F; // Mask from 0 to 5th bit

            // calculate hash of hw config data
            spdm_crypto_ops_calc_hash(&hw_config_buff[0], HW_CONFIG_DATA_SZ, spdmContext);

            // store the hash
            memcpy(buffer_ptr, &spdmContext->sha_digest[0], SPDM_SHA384_LEN);
            break;

        case INDEX4:
            // Read OTP 576 to 863, except 857 & 858
            efuse_read_data(576, &otp_array[0], 281);
            efuse_read_data(859, &otp_array[281], 5);

            // calculate hash of FW config data
            spdm_crypto_ops_calc_hash(&otp_array[0], sizeof(otp_array), spdmContext);

            // store the hash
            memcpy(buffer_ptr, &spdmContext->sha_digest[0], SPDM_SHA384_LEN);

            break;
    }
}

/******************************************************************************/
/** get certificate length and update total length of chain into cert_len variable
 * @param  offset - offset of the spi buffer where the certificate data length values resides
 * @param  update_cert_size - Pointer in which cert size has to be updated
 * @return CERT_CHAIN_VALID (0U) or CERT_CHAIN_INVALID(1U)
 *******************************************************************************/
uint8_t spdm_pkt_update_cert_data_len(uint32_t offset, uint32_t *update_cert_size)
{
    uint8_t ret_sts = CERT_CHAIN_INVALID;

    if(SPI_DATA_MAX_BUFF > offset)
    {
        uint32_t LSB = (uint32_t)((offset + CERT_DATA_BYTE2)&UINT32_MAX);
        uint32_t MSB = (uint32_t)((offset + CERT_DATA_BYTE3)&UINT32_MAX);
        uint32_t cert_sz  =0x0u;
        uint32_t spi_dat_sz = (uint32_t)((spi_data[LSB] & UINT8_MAX) << 8U);
        cert_sz = (uint32_t)(((uint32_t)((spi_dat_sz | spi_data[MSB])&UINT32_MAX) +
                              CERT_BYTE0_BYTE3_COUNT)&UINT32_MAX);
        if ((is_add_safe(LSB, offset) == 0) || (is_add_safe(MSB, offset) == 0) ||
                (is_add_safe(cert_sz, spi_dat_sz | spi_data[MSB]) == 0))
        {
            // Handle Error
            cert_sz = CERT_BYTE0_BYTE3_COUNT;
        }
        ret_sts = CERT_CHAIN_VALID;

        // If certificate size is greater than 1024 or if there is no certificate data; return 1KB of present data
        if ((cert_sz > 1024) || (cert_sz == CERT_BYTE0_BYTE3_COUNT))
        {
            cert_sz = 1024;
            ret_sts = CERT_CHAIN_INVALID; // Invalid Cert
        }

        *update_cert_size = cert_sz;
        cert_len = cert_len + cert_sz;
        if(is_add_safe(cert_len, cert_sz) == 0)
        {
            // Handle Error
            ret_sts = CERT_CHAIN_INVALID;
        }
    }

    return ret_sts;
}

/******************************************************************************/
/** function to store signature type
 * @param void
 * @return void
 *******************************************************************************/
void spdm_pkt_store_signature_type(void)
{
    is_puf = signature_type();
}

/******************************************************************************/
/** function to store hash of each chain in a buffer and keep for later use
 * @param spdmContext - SPDM module context
 * @return void
 *******************************************************************************/
void spdm_pkt_store_hash_of_chain(SPDM_CONTEXT *spdmContext)
{
    uint8_t slot = 0;
    uint8_t current_cert_ptr = 0;
    uint8_t get_tail_ptr = 0;
    uint8_t root_cert = true;
    uint32_t offset = 0;
    uint32_t get_mem = 0;

    if (NULL == spdmContext)
    {
        return;
    }
    memset(hash_of_chains, 0, MAX_SHA384_BUF_SIZE);

    if (cert2_base_addr != 0)
    {
        spdm_di_qmspi_clr_port_init_status(SPI_SELECT_INT_COMP_0);
        spdm_di_init_flash_component(SPI_SELECT_INT_COMP_0);
        for (slot = 0; slot < MAX_SLOTS; slot++)
        {
            if (slot_buf[slot].chain_present)
            {
                spdmContext->no_of_chains_avail++;
                cert_len = 0;
                offset = ROOT_START_OFFSET;
                root_cert = true;
                uint8_t cert_chain_valid_status = CERT_CHAIN_VALID; // This local variable is used to check validity of chain
                memset(spi_data, 0U, SPI_DATA_MAX_BUFF);
                current_cert_ptr = slot_buf[slot].chain.head_ptr_val; // get head pointer - root cert

                uint8_t counter = 0; /* Max 8 certificates in a chain are supported */
                
                // Get number of certificates in the chain 
                while (current_cert_ptr != END_OF_CHAIN)
                {
                    counter++;
                    if ((current_cert_ptr >= MAX_CERTIFICATES) || (counter > MAX_CERT_PER_CHAIN)) /* The first check is used to prevent access of cert_buf array at out of bound location 
                                            Second check used to prevent infinite while loop (If tail pointer of a certificate points to itself) */
                    {
                        cert_chain_valid_status = CERT_CHAIN_INVALID;
                        break;
                    }
                    current_cert_ptr = cert_buf[current_cert_ptr].tail_ptr_val;
                }

                // Compute RootHash, Chain length
                if(cert_chain_valid_status == CERT_CHAIN_VALID)
                {
                    current_cert_ptr = slot_buf[slot].chain.head_ptr_val; // get head pointer - root cert
                    // GET CHAIN LENGTH, ROOT Certificate and ROOTHASH
                    while ((cert_buf[current_cert_ptr].tail_ptr_val) != END_OF_CHAIN)
                    {
                        get_mem = cert_buf[current_cert_ptr].mem_addr;
                        if (root_cert)
                        {
                            
                            
                            if (spdm_read_certificate(get_mem, &spi_data[offset], 1024, current_cert_ptr))
                            {
                                // spdmContext->spdm_state_info = SPDM_IDLE;
                                cert_chain_valid_status = CERT_CHAIN_INVALID;
                                break;
                            }
                            cert_chain_valid_status |= spdm_pkt_update_cert_data_len(offset, &cert_buf[current_cert_ptr].cert_size);

                            if (cert_chain_valid_status == CERT_CHAIN_VALID)
                            {
                                // calculate hash of the root certificate and store it in spi_data
                                spdm_crypto_ops_calc_hash(&spi_data[offset], cert_buf[current_cert_ptr].cert_size, spdmContext);
                                memcpy(&slot_buf[slot].chain.root_cert_hash[0], &spdmContext->sha_digest[0], SPDM_SHA384_LEN);
                                memcpy(&spi_data[ROOTHASH_OFFSET_IN_CERT_CHAIN], &slot_buf[slot].chain.root_cert_hash[0], SPDM_SHA384_LEN);
                            }
                            else
                            {
                                /* Set roothash to 0U since root cert is invalid
                                However, since variable is global, which is initialized to 0U by default, memset to 0U is not needed */
                                break;
                            }
                            root_cert = false;
                            offset += cert_buf[current_cert_ptr].cert_size;
                        }
                        else
                        {
                            if (spdm_read_certificate(get_mem, &spi_data[offset], 4, current_cert_ptr))
                            {
                                // spdmContext->spdm_state_info = SPDM_IDLE;
                                cert_chain_valid_status = CERT_CHAIN_INVALID;
                                break;
                            }
                            cert_chain_valid_status |= spdm_pkt_update_cert_data_len(offset, &cert_buf[current_cert_ptr].cert_size);
                        }

                        get_tail_ptr = current_cert_ptr;
                        current_cert_ptr = cert_buf[get_tail_ptr].tail_ptr_val;
                    }

                    if (cert_chain_valid_status == CERT_CHAIN_VALID) // Get Tail certificate Length only if all certificates are valid
                    {
                        if (!(spdm_read_certificate(0, &spi_data[offset], 4, current_cert_ptr)))
                        {
                            cert_chain_valid_status |= spdm_pkt_update_cert_data_len(offset, &cert_buf[current_cert_ptr].cert_size);
                        }
                        else
                        {
                            cert_chain_valid_status = CERT_CHAIN_INVALID;
                        }
                    }
                }
                
                // If all the certificate in the chain are valid, proceed to compute chainHash which will be used in Challenge
                if(cert_chain_valid_status == CERT_CHAIN_VALID) // If any of the certificate is invalid, set hash of the chain to zero
                {
                    // Refer Certificate chain format in spdm spec, chain length includes length, reserved, hash bytes
                    slot_buf[slot].chain.chain_length = cert_len + 2U + 2U + SPDM_SHA384_LEN;
                    spi_data[0] = slot_buf[slot].chain.chain_length & 0xFF;           // Length of certificate chain - Byte 0
                    spi_data[1] = (slot_buf[slot].chain.chain_length & 0xFF00) >> 8U; // Length of certificate chain - Byte 1 (Little Endian)

                    current_cert_ptr = slot_buf[slot].chain.head_ptr_val; // get head pointer - root cert
                    offset = ROOT_START_OFFSET + cert_buf[current_cert_ptr].cert_size;

                    // This is used to hash request and responses and return signature of all requests and responses along with MEASUREMENTS;
                    // However, same is being reused since current function is executed during boot and SPDM requests will be received after completion of moving state machine to SPDM_CMD_PROCESS_MODE
                    spdmContext->get_requests_state = HASH_INIT_MODE;
                    spdmContext->request_or_response = 1;
                    spdm_crypto_ops_run_time_hashing(&spi_data[0U], 0, spdmContext, &ctx_ptr, spdmContext->get_requests_state);

                    // Step 1 : Compute run-time hash of chain-length(2 bytes), reserved(2 bytes), roothash(48 bytes), Root certificate
                    spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
                    if(offset > UINT16_MAX)
                    {
                        //trace0(ERROR_HANDLE_TRACE, SPDM_TSK, 0, "sshc1");
                    }
                    else
                    {
                        req_and_response_sz[1] = (uint16_t)(offset); // size of chain-length(2 bytes) + reserved(2 bytes) + roothash(48 bytes) + Root certificate
                    }
                    spdm_get_len_for_runtime_hash(spdmContext);
                    spdm_crypto_ops_run_time_hashing(&spi_data[0U], length, spdmContext, &ctx_ptr, spdmContext->get_requests_state);
                    offset = ROOT_START_OFFSET;

                    get_tail_ptr = current_cert_ptr;
                    current_cert_ptr = cert_buf[get_tail_ptr].tail_ptr_val;

                    while ((cert_buf[current_cert_ptr].tail_ptr_val) != END_OF_CHAIN)
                    {
                        get_mem = cert_buf[current_cert_ptr].mem_addr;
                        if (spdm_read_certificate(get_mem, &spi_data[offset], cert_buf[current_cert_ptr].cert_size, current_cert_ptr))
                        {
                            // spdmContext->spdm_state_info = SPDM_IDLE;
                            slot_buf[slot].is_cert_chain_valid = CERT_CHAIN_INVALID;
                            break;
                        }
                        req_and_response_sz[1] = cert_buf[current_cert_ptr].cert_size;
                        spdm_get_len_for_runtime_hash(spdmContext);
                        spdm_crypto_ops_run_time_hashing(&spi_data[offset], length, spdmContext, &ctx_ptr, spdmContext->get_requests_state);

                        get_tail_ptr = current_cert_ptr;
                        current_cert_ptr = cert_buf[get_tail_ptr].tail_ptr_val;
                    }

                    uint16_t size = 0U;
                    if(cert_buf[current_cert_ptr].cert_size > UINT16_MAX)
                    {
                        // trace0(ERROR_HANDLE_TRACE, SPDM_TSK, 0, "sshc2");
                    }
                    else
                    {
                        size = (uint16_t)(cert_buf[current_cert_ptr].cert_size);
                    }
                    if (spdm_read_certificate(0, &spi_data[offset], size, current_cert_ptr))
                    {
                            slot_buf[slot].is_cert_chain_valid = CERT_CHAIN_INVALID;
                            break;
                    }
                    req_and_response_sz[1] = cert_buf[current_cert_ptr].cert_size;
                    spdm_get_len_for_runtime_hash(spdmContext);
                    spdm_crypto_ops_run_time_hashing(&spi_data[offset], length, spdmContext, &ctx_ptr, spdmContext->get_requests_state);


                    spdmContext->get_requests_state = END_OF_HASH;
                    spdm_crypto_ops_run_time_hashing(NULL, 0, spdmContext,&ctx_ptr, spdmContext->get_requests_state);
                    memcpy(&hash_of_chains[SPDM_SHA384_LEN * slot], &spdmContext->sha_digest[0], SPDM_SHA384_LEN);
                }
                else
                {
                    slot_buf[slot].is_cert_chain_valid = CERT_CHAIN_INVALID;
                    /* Ideally, Set hash of cert chain to 0U since one of the cert is invalid
                    However, since variable is global, which is initialized to 0U by default, memset to 0U is not needed */
                }

            }
        }
        spdm_di_spi_tristate(INT_SPI);
        spdmContext->spdm_state_info = SPDM_CMD_PROCESS_MODE;
    }
    else
    {
        spdmContext->spdm_state_info = SPDM_IDLE;
    }
}

/******************************************************************************/
/** function to initialize immutable rom measurement block
 * @param spdmContext - SPDM module context
 * @return void
 *******************************************************************************/
void spdm_pkt_init_immutable_rom_block(SPDM_CONTEXT *spdmContext)
{
    if (NULL == spdmContext)
    {
        return;
    }
    uint8_t indx = (INDEX1 - 1);
    // INDEX 1 FOR IMMUTABLE ROM MEASUREMENT FIXED AS PER SPEC
    msr_block_buffer[indx].index = INDEX1;
    msr_block_buffer[indx].msr_specific = DMTF_FRMT;
    memset(msr_block_buffer[indx].msr_size, 0, 2);
    memset(msr_block_buffer[indx].msr_frmt.dmtf_msr_val_size, 0, 2);

    msr_block_buffer[indx].msr_frmt.dmtf_msr_val_type = IMMUTABLE_ROM_SEL_DGST_RAW; // digest type supported for now

    // 7th bit 1 means raw bit stream selected which is TBD for immutable rom
    if (((msr_block_buffer[indx].msr_frmt.dmtf_msr_val_type & RAW_BIT_STEAM_SELECT_VALUE) == 0x00))
    {
        // size of measurement record
        msr_block_buffer[indx].msr_size[0] = MSR_BLOCK_DMTF_FLDS_SZ + SPDM_SHA384_LEN;

        // size of dmtf measurement value
        msr_block_buffer[indx].msr_frmt.dmtf_msr_val_size[0] = SPDM_SHA384_LEN;
        spdm_get_measurements(&msr_block_buffer[indx].msr_frmt.dmtf_msr_val[0], INDEX1);

        uint8_t num_of_bytes_in_current_block = msr_block_buffer[indx].msr_size[0] + MSR_BLOCK_SPEC_FLDS_SZ;
        if (is_add_safe(num_of_bytes_in_current_block, MSR_BLOCK_SPEC_FLDS_SZ) == 0) // Coverity INT30-C fix
        {
            //trace0(ERROR_HANDLE_TRACE, SPDM_TSK, 0, "sirb");
        }
        // concatenate first block
        memcpy(&concat_msrment_blocks[indx], (uint8_t *)&msr_block_buffer[indx], num_of_bytes_in_current_block);

        measurement_var.msr_record_size = num_of_bytes_in_current_block;

        req_and_response_sz[1] = msr_block_buffer[indx].msr_size[0];

        spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
        spdm_get_len_for_runtime_hash(spdmContext);
        spdm_crypto_ops_run_time_hashing((uint8_t *)&msr_block_buffer[indx].msr_frmt, length, spdmContext, &ctx_ptr, spdmContext->get_requests_state);
    }
}

/******************************************************************************/
/** function to initialize apfw hash measurement block
 * @param spdmContext - SPDM module context
 * @return void
 *******************************************************************************/
void spdm_pkt_init_apfw_hash(SPDM_CONTEXT *spdmContext)
{
    if (NULL == spdmContext)
    {
        return;
    }
    uint8_t indx = (INDEX5 - 1);
    // INDEX 5 FOR APFW details
    msr_block_manifest.index = INDEX5;
    msr_block_manifest.msr_specific = DMTF_FRMT;
    msr_block_manifest.dmtf_msr_val_type = AP_CONFIG_SELCT_DGST_RAM; // default as hash
    uint16_t num_of_bytes_in_current_block = 0x0U;

    if ((msr_block_manifest.dmtf_msr_val_type & RAW_BIT_STEAM_SELECT_VALUE) == 0x00)
    {
        // size of measurement
        msr_block_manifest.msr_size = MSR_BLOCK_DMTF_FLDS_SZ + ((SPDM_SHA384_LEN +2 )* count_of_ap_msr_data_received);

        // size of dmtf measurement value
        msr_block_manifest.dmtf_msr_val_size = ((SPDM_SHA384_LEN + 2)* count_of_ap_msr_data_received);
    }

    num_of_bytes_in_current_block = msr_block_manifest.msr_size + MSR_BLOCK_SPEC_FLDS_SZ;
    if (is_add_safe(num_of_bytes_in_current_block, MSR_BLOCK_SPEC_FLDS_SZ) == 0) // Coverity INT30-C fix
    {
        // trace0(ERROR_HANDLE_TRACE, SPDM_TSK, 0, "siah");
    }
    // concatenate fifth block
    memcpy(&concat_msrment_blocks[measurement_var.msr_record_size], &msr_block_manifest, num_of_bytes_in_current_block);

    measurement_var.msr_record_size = measurement_var.msr_record_size + num_of_bytes_in_current_block;
    if (is_add_safe(measurement_var.msr_record_size, num_of_bytes_in_current_block) == 0)
    {
        // Handle Error
        measurement_var.msr_record_size = 0;
    }
    req_and_response_sz[1] = msr_block_manifest.msr_size;
    spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
    spdm_get_len_for_runtime_hash(spdmContext);
    spdm_crypto_ops_run_time_hashing((uint8_t *)&msr_block_manifest.dmtf_msr_val_type, length, spdmContext,&ctx_ptr, spdmContext->get_requests_state);  
}

/******************************************************************************/
/** function to initialize mutable fw measurement block
 * @param spdmContext - SPDM module context
 * @return void
 *******************************************************************************/
void spdm_pkt_init_mutable_fw(SPDM_CONTEXT *spdmContext)
{
    if (NULL == spdmContext)
    {
        return;
    }
    uint8_t indx = (INDEX2 - 1);
    // INDEX 2 FOR MUTABLE FW MEASUREMENT FIXED AS PER SPEC
    msr_block_buffer[indx].index = INDEX2;
    msr_block_buffer[indx].msr_specific = DMTF_FRMT;
    memset(msr_block_buffer[indx].msr_size, 0, 2);
    memset(msr_block_buffer[indx].msr_frmt.dmtf_msr_val_size, 0, 2);

    msr_block_buffer[indx].msr_frmt.dmtf_msr_val_type = MUTABLE_FW_SEL_DGST_RAW; // digest type supported for now

    // 7th bit 1 means raw bit stream selected which is TBD for mutable ecfw
    if ((msr_block_buffer[indx].msr_frmt.dmtf_msr_val_type & RAW_BIT_STEAM_SELECT_VALUE) == 0x00)
    {
        // size of measurement record
        msr_block_buffer[indx].msr_size[0] = MSR_BLOCK_DMTF_FLDS_SZ + SPDM_SHA384_LEN;

        // size of dmtf measurement value
        msr_block_buffer[indx].msr_frmt.dmtf_msr_val_size[0] = SPDM_SHA384_LEN;

        spdm_get_measurements(&msr_block_buffer[indx].msr_frmt.dmtf_msr_val[0], INDEX2);

        uint8_t num_of_bytes_in_current_block = msr_block_buffer[indx].msr_size[0] + MSR_BLOCK_SPEC_FLDS_SZ;
        if (is_add_safe(num_of_bytes_in_current_block, MSR_BLOCK_SPEC_FLDS_SZ) == 0) // Coverity INT30-C fix
        {
            // Handle Error
        }

        // concatenate second block
        memcpy(&concat_msrment_blocks[measurement_var.msr_record_size], &msr_block_buffer[indx], num_of_bytes_in_current_block);

        measurement_var.msr_record_size = measurement_var.msr_record_size + num_of_bytes_in_current_block;
        if (is_add_safe(measurement_var.msr_record_size, num_of_bytes_in_current_block) == 0)
        {
            // Handle Error
            measurement_var.msr_record_size = 0;
        }

        req_and_response_sz[1] = msr_block_buffer[indx].msr_size[0];
        spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
        spdm_get_len_for_runtime_hash(spdmContext);
        spdm_crypto_ops_run_time_hashing((uint8_t *)&msr_block_buffer[indx].msr_frmt, length, spdmContext,&ctx_ptr, spdmContext->get_requests_state);
    }
}

/******************************************************************************/
/** function to initialize hw config measurement block
 * @param spdmContext - SPDM module context
 * @return void
 *******************************************************************************/
void spdm_pkt_init_hwconfig(SPDM_CONTEXT *spdmContext)
{
    if (NULL == spdmContext)
    {
        return;
    }
    uint8_t num_of_bytes_in_current_block = 0U;
    uint8_t indx = (INDEX3 - 1);

    // INDEX 3 FOR MUTABLE FW MEASUREMENT FIXED AS PER SPEC
    msr_block_buffer[indx].index = INDEX3;
    msr_block_buffer[indx].msr_specific = DMTF_FRMT;
    memset(msr_block_buffer[indx].msr_size, 0, 2);
    memset(msr_block_buffer[indx].msr_frmt.dmtf_msr_val_size, 0, 2);
    msr_block_buffer[indx].msr_frmt.dmtf_msr_val_type = HW_CONFIG_SEL_DGST_RAW; // digest selected default

    // 7th bit 1 means raw bit stream selected for hw config else digest
    if ((msr_block_buffer[indx].msr_frmt.dmtf_msr_val_type & RAW_BIT_STEAM_SELECT_VALUE) == 0x00)
    {
        // size of measurement
        msr_block_buffer[indx].msr_size[0] = MSR_BLOCK_DMTF_FLDS_SZ + SPDM_SHA384_LEN;

        // size of dmtf measurement value
        msr_block_buffer[indx].msr_frmt.dmtf_msr_val_size[0] = SPDM_SHA384_LEN;

        // store the hash
        spdm_get_measurements(&msr_block_buffer[indx].msr_frmt.dmtf_msr_val[0], INDEX3);

        num_of_bytes_in_current_block = msr_block_buffer[indx].msr_size[0] + MSR_BLOCK_SPEC_FLDS_SZ;
        if (is_add_safe(num_of_bytes_in_current_block, MSR_BLOCK_SPEC_FLDS_SZ) == 0) // Coverity INT30-C fix
        {
            // Handle Error
        }

        // concatenate third block
        memcpy(&concat_msrment_blocks[measurement_var.msr_record_size], &msr_block_buffer[indx], num_of_bytes_in_current_block);

        measurement_var.msr_record_size = measurement_var.msr_record_size + num_of_bytes_in_current_block;
        if (is_add_safe(measurement_var.msr_record_size, num_of_bytes_in_current_block) == 0)
        {
            // Handle Error
            measurement_var.msr_record_size = 0;
        }
    }
    else
    {
        // size of measurement
        msr_block_buffer[indx].msr_size[0] = MSR_BLOCK_DMTF_FLDS_SZ + HW_CONFIG_DATA_SZ;

        msr_block_buffer[indx].msr_frmt.dmtf_msr_val_size[0] = HW_CONFIG_DATA_SZ;

        spdm_get_measurements(&msr_block_buffer[indx].msr_frmt.dmtf_msr_val[0], INDEX3);


        if ((msr_block_buffer[indx].msr_size[0] + MSR_BLOCK_SPEC_FLDS_SZ) > UINT8_MAX) // Coverity INT31-C Fix
        {
            // handle error
        }
        else
        {
            num_of_bytes_in_current_block = (uint8_t)(msr_block_buffer[indx].msr_size[0] + MSR_BLOCK_SPEC_FLDS_SZ);
        }
        // concatenate third block
        if (num_of_bytes_in_current_block > 0) {
            memcpy(&concat_msrment_blocks[measurement_var.msr_record_size], &msr_block_buffer[indx], num_of_bytes_in_current_block);
        }
        measurement_var.msr_record_size = measurement_var.msr_record_size + num_of_bytes_in_current_block;
        if (is_add_safe(measurement_var.msr_record_size, num_of_bytes_in_current_block) == 0)
        {
            // Handle Error
            measurement_var.msr_record_size = 0;
        }
    }
    req_and_response_sz[1] = msr_block_buffer[indx].msr_size[0];
    spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
    spdm_get_len_for_runtime_hash(spdmContext);
    spdm_crypto_ops_run_time_hashing((uint8_t *)&msr_block_buffer[indx].msr_frmt, length, spdmContext,&ctx_ptr, spdmContext->get_requests_state);
}

/******************************************************************************/
/** function to initialize ecfw config measurement block
 * @param spdmContext - SPDM module context
 * @return void
 *******************************************************************************/
void spdm_pkt_init_ecfwconfig(SPDM_CONTEXT *spdmContext)
{
    if (NULL == spdmContext)
    {
        return;
    }
    uint8_t indx = (INDEX4 - 1);
    // INDEX 3 FOR MUTABLE FW MEASUREMENT FIXED AS PER SPEC
    msr_block_buffer[indx].index = INDEX4;
    msr_block_buffer[indx].msr_specific = DMTF_FRMT;
    msr_block_buffer[indx].msr_frmt.dmtf_msr_val_type = FW_CONFIG_SEL_DGST_RAW; // default as hash
    uint8_t num_of_bytes_in_current_block = 0x0U;
    memset(msr_block_buffer[indx].msr_size, 0, 2);

    memset(&msr_block_buffer[indx].msr_frmt.dmtf_msr_val[0], 0, SPDM_SHA384_LEN); // setting to 0; future support

    if ((msr_block_buffer[indx].msr_frmt.dmtf_msr_val_type & RAW_BIT_STEAM_SELECT_VALUE) == 0x00)
    {
        // size of measurement
        msr_block_buffer[indx].msr_size[0] = MSR_BLOCK_DMTF_FLDS_SZ + SPDM_SHA384_LEN;

        // size of dmtf measurement value
        msr_block_buffer[indx].msr_frmt.dmtf_msr_val_size[0] = SPDM_SHA384_LEN;

        spdm_get_measurements(&msr_block_buffer[indx].msr_frmt.dmtf_msr_val[0], INDEX4);
    }
    else
    {
        msr_block_buffer[indx].msr_frmt.dmtf_msr_val_size[1] = HW_CONFIG_DATA_SZ;
        msr_block_buffer[indx].msr_size[1] = MSR_BLOCK_DMTF_FLDS_SZ + HW_CONFIG_DATA_SZ;
    }

    num_of_bytes_in_current_block = msr_block_buffer[indx].msr_size[0] + MSR_BLOCK_SPEC_FLDS_SZ;
    if (is_add_safe(num_of_bytes_in_current_block, MSR_BLOCK_SPEC_FLDS_SZ) == 0) // Coverity INT30-C fix
    {
        // Handle Error
    }

    // concatenate fourth block
    memcpy(&concat_msrment_blocks[measurement_var.msr_record_size], &msr_block_buffer[indx], num_of_bytes_in_current_block);

    measurement_var.msr_record_size = measurement_var.msr_record_size + num_of_bytes_in_current_block;
    if (is_add_safe(measurement_var.msr_record_size, num_of_bytes_in_current_block) == 0)
    {
        // Handle Error
        measurement_var.msr_record_size = 0;
    }
    req_and_response_sz[1] = msr_block_buffer[indx].msr_size[0];
    spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
    spdm_get_len_for_runtime_hash(spdmContext);
    spdm_crypto_ops_run_time_hashing((uint8_t *)&msr_block_buffer[indx].msr_frmt, length, spdmContext,&ctx_ptr, spdmContext->get_requests_state);
}

/******************************************************************************/
/** function to initialize measurement blocks
 * @param spdmContext - SPDM module context
 * @return void
 *******************************************************************************/
void spdm_pkt_init_measurement_block(SPDM_CONTEXT *spdmContext)
{
    if (NULL == spdmContext)
    {
        return;
    }
    uint8_t tcb_buff[TCB_MSR_SIZE] = {0x00};
    uint8_t tcb0_len = 0x00;
    uint8_t tcb1_len = 0x00;
    uint16_t size = 0x00;
    // calculate hash of all msr blocks
    // switch hash engine to init mode first for run time hashing
    spdmContext->get_requests_state = HASH_INIT_MODE;

    spdm_crypto_ops_run_time_hashing(NULL, 0, spdmContext, &ctx_ptr, spdmContext->get_requests_state);

    spdmContext->request_or_response = 1; // hash data is meant for responses

    spdm_pkt_init_immutable_rom_block(spdmContext);
    spdm_pkt_init_mutable_fw(spdmContext);
    spdm_pkt_init_hwconfig(spdmContext);
    spdm_pkt_init_ecfwconfig(spdmContext);
    spdm_pkt_init_apfw_hash(spdmContext);

    
    spdmContext->get_requests_state = END_OF_HASH;
    spdm_crypto_ops_run_time_hashing(NULL, 0, spdmContext,&ctx_ptr, spdmContext->get_requests_state);
    memcpy(&hash_concat_msrments_val[0], &spdmContext->sha_digest, SPDM_SHA384_LEN);

    // calculate hash of tcb blocks alone: immutable rom and hw config
    memcpy(&size, &msr_block_buffer[(INDEX1 - 1)].msr_frmt.dmtf_msr_val_size[0], DMTF_MSR_VAL_SIZE);
    tcb0_len = ((MSR_BLOCK_DMTF_FLDS_SZ + size)&UINT8_MAX);
    if (is_add_safe(tcb0_len, MSR_BLOCK_DMTF_FLDS_SZ) == 0) // Coverity INT30-C fix
    {
        // Handle Error
    }
    memcpy(&size, &msr_block_buffer[(INDEX3 - 1)].msr_frmt.dmtf_msr_val_size[0], DMTF_MSR_VAL_SIZE);
    tcb1_len = ((MSR_BLOCK_DMTF_FLDS_SZ + size)&UINT8_MAX);
    if (is_add_safe(tcb1_len, MSR_BLOCK_DMTF_FLDS_SZ) == 0)
    {
        // Handle Error
    }
    memcpy(&tcb_buff[0], (uint8_t *)&msr_block_buffer[(INDEX1 - 1)].msr_frmt, tcb0_len);
    memcpy(&tcb_buff[tcb0_len], (uint8_t *)&msr_block_buffer[(INDEX3 - 1)].msr_frmt, tcb1_len);
    spdm_crypto_ops_calc_hash(&tcb_buff[0], (tcb0_len + tcb1_len), spdmContext);
    memcpy(&hash_concat_tcb_msrments_val[0], &spdmContext->sha_digest, SPDM_SHA384_LEN);
}

/******************************************************************************/
/** function to initialize the spdm module local buffers and state machine
 * @param spdmContext - SPDM module context
 * @return void
 *******************************************************************************/
void spdm_init_task(SPDM_CONTEXT *spdmContext)
{
    uint8_t iter = 0;
    uint8_t j = 0;

    if (NULL == spdmContext)
    {
        return;
    }

    for (j = 0; j < MCTP_PKT_BUF_DATALEN; j++)
    {
        spdm_pktbuf[0].pkt.data[j] = 0;
    }

    spdm_pktbuf[0].buf_full = MCTP_EMPTY;
    spdm_pktbuf[0].smbus_nack_retry_count = 0;
    spdm_pktbuf[0].smbus_acquire_retry_count = 0;
    spdm_pktbuf[0].smbus_lab_retry_count = 0;
    spdm_pktbuf[0].request_tx_retry_count = 0;
    spdm_pktbuf[0].request_per_tx_timeout_count = 0;
    spdm_pktbuf[0].rx_timestamp = 0;
    spdm_tx_state = SPDM_TX_IDLE;
    // initialize table entry buffers
    for (iter = 0; iter < SPDM_VER_NUM_ENTRY_COUNT; iter++)
    {
        version_tbl_buf[iter].major_ver = SPDM_VER_NUM_MAJOR_MINOR + iter;
        version_tbl_buf[iter].update_ver_no = SPDM_VER_NUM_UPDATE_ALPHA;
        get_ver_resp_object.table_entry[iter].table_value[0] = version_tbl_buf[iter].update_ver_no;
        get_ver_resp_object.table_entry[iter].table_value[1] = version_tbl_buf[iter].major_ver;
    }

    // initialize opaque data buffer
    opaque_data.opaque_len = sizeof(opaque_data) - OPAQUE_DATA_LEN;
    opaque_data.spec_id = SECURED_MESSAGE_OPAQUE_DATA_SPEC_ID;
    opaque_data.opaque_version = SECURED_MESSAGE_OPAQUE_VERSION;
    opaque_data.total_elements = OPAQUE_DATA_ELEMENTS;
    opaque_data.reserved = 0;
    opaque_data.opaque_element_table_header.id = SPDM_REGISTRY_ID_DMTF;
    opaque_data.opaque_element_table_header.vendor_len = 0;
    opaque_data.opaque_element_table_header.opaque_element_data_len = sizeof(OPAQUE_ELEMENT_SUPPORTED_VERSION);
    opaque_data.opaque_element_supported_version.sm_data_version = SECURED_MESSAGE_OPAQUE_ELEMENT_SMDATA_DATA_VERSION;
    opaque_data.opaque_element_supported_version.sm_data_id = SECURED_MESSAGE_OPAQUE_ELEMENT_SMDATA_ID_SUPPORTED_VERSION;
    opaque_data.opaque_element_supported_version.version_count = SPDM_OPAQUE_DATA_VERSION_COUNT;
    opaque_data.opaque_element_supported_version.version_number = CURRENT_VERSION << SPDM_VERSION_NUMBER_SHIFT_BIT;
    opaque_data.opaque_element_supported_version.reserved = 0;

    packet_sz = 0x00;
    pkt_seq_mctp = 0x00;
    first_pkt = true; 
    tbl_entry = 0x00;
    bytes_sent_over_mctp_for_cert = 0x00;
    // DHE
    struct_algo[0].AlgType = ALG_TYPE_DHE;
    struct_algo[0].AlgCount = ALG_CNT_DHE;
    struct_algo[0].AlgSupported = DHE_SECP_384r1;
    // AEAD
    struct_algo[1].AlgType = ALG_TYPE_AEAD;
    struct_algo[1].AlgCount = ALG_CNT_AEAD;
    struct_algo[1].AlgSupported = AES_256_GCM;
    // ReqBaseAsymAlgo
    struct_algo[2].AlgType = ALG_TYPE_REQ_BASE_ASYM_ALG;
    struct_algo[2].AlgCount = ALG_CNT_REQ_BASE_ASYM_ALG;
    struct_algo[2].AlgSupported = TPM_ALG_ECDSA_ECC_NIST_P384;
    // KeySchedule
    struct_algo[3].AlgType = ALG_TYPE_KEY_SCHEDULE;
    struct_algo[3].AlgCount = ALG_CNT_KEY_SCHEDULE;
    struct_algo[3].AlgSupported = SPDM_KEY_SCHEDULE;

    first_get_cert_response_sent = 0;
    first_challenge_resp_sent = CHG_RESP_END_OF_TX;
    signature_offset = 0;
    nonce_offset = 0;
    memset(AP_CFG_cert_buffer, 0, sizeof(CFG_CERT));

    spdmContext->current_request_length = 0;
    spdmContext->previous_request_length = 0;
    spdmContext->total_request_bytes = 0;
    spdmContext->no_of_chains_avail = 0;

    memset(req_and_response_sz, 0, 2);

    memset(spdmContext->cert_bytes_pending_to_sent, 0, 8);
    memset(ecdsa_signature.signature_r_term, 0, CURVE_384_SZ);
    memset(ecdsa_signature.signature_s_term, 0, CURVE_384_SZ);
    spdmContext->spdm_state_info = SPDM_INIT_CERT_PARAMS;
    spdmContext->get_requests_state = HASH_INIT_MODE;

    memset(&measurement_var, 0, sizeof(MEASUREMENT_VARIABLES));
    spdm_pkt_init_measurement_block(spdmContext);

    spdm_secure_session_set_session_state(spdmContext, SPDM_SESSION_STATE_NOT_STARTED);

    SET_SPDM_EVENT_FLAG(); // pldm ap cfg populated via spdm ap cfg cert data
}

/******************************************************************************/
/** function to initialize cert params to default state
 * @param spdmContext - SPDM module context
 * @return void
 *******************************************************************************/
void spdm_pkt_initialize_cert_params_to_default(SPDM_CONTEXT *spdmContext)
{
    uint8_t slot = 0;
    uint8_t cert_count = 0;
    uint8_t efuse_data = 0x00;

    if (NULL == spdmContext)
    {
        return;
    }

    for (slot = 0; slot < MAX_SLOTS; slot++)
    {
        slot_buf[slot].chain_no = NO_CHAIN_VAL;
        slot_buf[slot].chain_present = false;
        slot_buf[slot].chain.head_ptr_val = END_OF_CHAIN;
        slot_buf[slot].is_cert_chain_valid = CERT_CHAIN_VALID;
    }
    for (cert_count = 0; cert_count < MAX_CERTIFICATES; cert_count++)
    {
        cert_buf[cert_count].mem_addr = 0x00;
        cert_buf[cert_count].tail_ptr_val = END_OF_CHAIN;
    }

    get_cert2_base_address(&cert2_base_addr);

    spdmContext->spdm_state_info = SPDM_GET_CERT_FROM_APCFG;
    SET_SPDM_EVENT_FLAG();
}

/******************************************************************************/
/** function populate the version entry table based on version supported
 * @param spdm_buf_tx- tx buffer of spdm with MCTP packet definition fields
 * @return void
 *******************************************************************************/
void spdm_pkt_fill_spdm_buf_ver_entry_tbl(MCTP_PKT_BUF *spdm_buf_tx)
{
    uint8_t iter = 0;
    uint8_t max_lim = 0x00;
    if (SPDM_VER_NUM_ENTRY_COUNT >
            SPDM_MAX_VER_ENTRY_COUNT) // entry table max size in one packet can only be (2*29 + 6) = 64 bytes: one entry is 2 bytes
    {
        for (iter = 0; iter < SPDM_MAX_VER_ENTRY_COUNT; iter++)
        {
            if ((tbl_entry + iter) >= SPDM_VER_NUM_ENTRY_COUNT) // if table entries go greater than max entries for next loop
            {
                max_lim = 1;
                break;
            }
            else
            {
                spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 4 + (iter * 2)] = version_tbl_buf[tbl_entry + iter].major_ver;
                spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 5 + (iter * 2)] = version_tbl_buf[tbl_entry + iter].update_ver_no;
            }
        }
        if (!max_lim)
        {
            tbl_entry = tbl_entry + SPDM_MAX_VER_ENTRY_COUNT;
        }
        else
        {
            tbl_entry = 0;
        }
    }
    else
    {
        for (iter = 0; iter < SPDM_VER_NUM_ENTRY_COUNT; iter++)
        {
            spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 5 + (iter * 2)] = version_tbl_buf[iter].major_ver;
            spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 4 + (iter * 2)] = version_tbl_buf[iter].update_ver_no;
        }
    }
}

/******************************************************************************/
/** function to handle tx packet buffering for the chain digest values sent for each spdm response
 * @param spdm_buf_tx- tx buffer of spdm with MCTP packet definition fields
 * @return void
 *******************************************************************************/
void spdm_pkt_fill_spdm_buf_digest(MCTP_PKT_BUF *spdm_buf_tx)
{
    uint8_t is_chain_available_in_slot = 0U;
    uint8_t offset = 0U;
    uint8_t vacancy = 0U;
    uint8_t bytes_to_transfer = 0U;

    SPDM_CONTEXT *spdmContext = NULL;
    spdmContext = spdm_ctxt_get();
    if (NULL == spdmContext)
    {
        return;
    }

    /*
        Consider an array
         ------------------------------------------------------------------------------------------------------------
        |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |    |
         ------------------------------------------------------------------------------------------------------------

        vacancy signifies max possible bytes that can be filled at any point of time
        offset is the counter to track index as the array gets filled
    */

    switch (digest_state_machine)
    {
    case TXSTATE_1:
    {
        /* This is the first response so it contains MSG_TYPE */
        vacancy = MAX_NUM_BYTES_PLD - SPDM_MSG_PLD_SPECIFIC_BYTES - DIGEST_PLD_LEN_EXCLUDING_DIGEST;
        offset = SPDM_HEADER_VERSION_POS + DIGEST_RSP_DIGEST_OFFSET;
        digest_state_machine = TXSTATE_2;
        break;
    }
    case TXSTATE_2:
    {
        /* Rest of the response contains only Digest */
        vacancy = MAX_NUM_BYTES_PLD;
        offset = SPDM_MSG_TYPE_POS;
        digest_state_machine = TXSTATE_2;
        break;
    }
    }

    uint8_t i;
    for (i = slot_position; i < MAX_SLOTS; i++)
    {
        if (((chain_avail_bits_mask & (1U << slot_position)) >> slot_position) > UINT16_MAX) // Coverity INT31-C Fix
        {
            // handle error
        }
        else
        {
            is_chain_available_in_slot = (chain_avail_bits_mask & (1U << slot_position)) >> slot_position;
        }
        if (is_chain_available_in_slot & 0x01)
        {
            if (pending_bytes) // the slot's digest is not completely sent, some bytes are pending to be transferred
            {
                if (is_sub_safe(SPDM_SHA384_LEN, pending_bytes) == 0)
                {
                    // Handle Error
                    break;
                }
                else
                {
                    global_offset_for_pending_bytes = SPDM_SHA384_LEN - pending_bytes;
                }
                bytes_to_transfer = pending_bytes;
                if(bytes_to_transfer > vacancy)
                {
                    /* If bytes_to_transfer > vacancy, transfer only what can be transferred
                       update pending_bytes so that it can be transferred in the next transaction */
                    pending_bytes = bytes_to_transfer - vacancy;
                    bytes_to_transfer = vacancy;
                }
                else
                {
                    /* All bytes transsferred, so on more pending bytes from current slot */
                    pending_bytes = 0U;
                }
            }
            else // No pending bytes in current slot, the slot needs to be sent completely from Offset 0
            {
                bytes_to_transfer = SPDM_SHA384_LEN;
                global_offset_for_pending_bytes = 0U;
                /* If bytes_to_transfer > vacancy, transfer only what can be transferred
                    update pending_bytes so that it can be transferred in the next transaction */
                if(bytes_to_transfer > vacancy)
                {
                    pending_bytes = bytes_to_transfer - vacancy;
                    bytes_to_transfer = vacancy;
                }
            }
            if (bytes_to_transfer > 0) {
                memcpy(&spdm_buf_tx->pkt.data[offset],
                       &hash_of_chains[(slot_position * SPDM_SHA384_LEN) + global_offset_for_pending_bytes], bytes_to_transfer);
            }
            vacancy -= bytes_to_transfer; // Decrement vacancy by bytes_transferred
            offset  = offset + bytes_to_transfer; // Incremenet offset
            if(is_add_safe(offset, bytes_to_transfer) == 0)
            {
                // Handle Error
            }

        }
        if(vacancy == 0U)
        {
            /*  IF vacancy = 0, and if there are no pending bytes from the slot's digest, update slot_position
                ELSE, it means slot's digest has pending bytes to be sent, so dont update slot position */
            if(pending_bytes == 0U)
            {
                slot_position = slot_position + 1;
                if (is_add_safe(slot_position, 1) == 0)
                {
                    // Handle Error
                    slot_position = 0;
                }
            }
            break;
        }
        else
        {
            /* Either current slot position is completely sent or there is no chain in the current slot;
               so increment slot_position for the next iteration */
            slot_position = slot_position + 1;
            if (is_add_safe(slot_position, 1) == 0)
            {
                // Handle Error
                slot_position = 0;
            }
        }
    }
    // If Control reaches here, it signifies End Of Transaction, so update packet size
    if(i == MAX_SLOTS)
    {
        packet_sz = MAX_NUM_BYTES_PLD - vacancy;
    }
}

/******************************************************************************/
/** function to handle tx packet buffering for the all supported certificate values sent for each
 * spdm response
 * @param spdm_buf_tx- tx buffer of spdm with MCTP packet definition fields
 * @param spdmContext - Context of SPDM module
 * @return void
 *******************************************************************************/
void spdm_pkt_fill_spdm_buf_certificate(MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext)
{
    /*
        Aim: Using spi_data buffer, buffer out CERTIFICATE Response
        Known data from GET_CERT request : start_offset of the chain , size requested

        Logic Used :
            Fill the spi_data buffer with chain data of length  = min(SPI_DATA_MAX_BUFF,Length_requested)
            Keep buffering out spdm packets in multiples of MAX_NUM_BYTES_PLD
            When there aren't enough bytes to be sent from the buffer: Say, only 40 bytes are in buffer and 64 bytes needs to be sent
                -> Store existing data into a local buffer(temp) - store 40 bytes into temp
                -> Fill spi_data buffer with next chain data of length = min(SPI_DATA_MAX_BUFF, pending bytes to be sent)
                -> Get rest of data into temp buffer from spi_data buffer  - append remaining 24 bytes into spi_data
                -> Copy temp to spdm pkt data
                -> For successive packets - Continue sending rest of packets from spi_data buffer
    */

    if (NULL == spdmContext)
    {
        return;
    }
    spdm_flash_busy = true;

    uint16_t read_frm_offset = spdmContext->cert_offset_to_read[requested_slot];

    uint16_t end_offset_of_requested_chain = (uint16_t)((read_frm_offset + spdmContext->cert_bytes_to_sent_curr[requested_slot] - 1) & UINT16_MAX);

    if (first_get_cert_response_sent == 0)
    {
        read_bytes_from_chain = (spdmContext->cert_bytes_to_sent_curr[requested_slot] > SPI_DATA_MAX_BUFF) ? SPI_DATA_MAX_BUFF : spdmContext->cert_bytes_to_sent_curr[requested_slot];
        START_OFFSET_IN_BUFFER = read_frm_offset;
        BUFFER_END_OFFSET = (uint16_t)((START_OFFSET_IN_BUFFER + read_bytes_from_chain - 1) & UINT16_MAX);

        read_chain_from_offset(START_OFFSET_IN_BUFFER, BUFFER_END_OFFSET, 0, requested_slot); // Fill the spi_data buffer with chain data of length  = min(SPI_DATA_MAX_BUFF,Length_requested)

        spdmContext->request_or_response = 1;
        req_and_response_sz[1] = read_bytes_from_chain;
        spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
        spdm_get_len_for_runtime_hash(spdmContext);
        spdm_crypto_ops_run_time_hashing(&spi_data[0], length, spdmContext,&ctx_ptr, spdmContext->get_requests_state);

        // MAX_CERT_CHAIN_IN_FIRST_ITER = 55 bytes of certificate chain can be transferred along with other fields in the first iteration
        if (spdmContext->cert_bytes_requested[requested_slot] <= MAX_CERT_CHAIN_IN_FIRST_ITER)
        {
            memcpy(&spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + SLOT_NUM_CHAIN_LENGTH_BYTES], &spi_data[read_frm_offset - START_OFFSET_IN_BUFFER], spdmContext->cert_bytes_requested[requested_slot]);
            packet_sz = spdmContext->cert_bytes_to_sent_curr[requested_slot] + MCTP_TOT_HDR_SZ + 1 + 8; // 8 corresponds to length of all fields excluding cert_chain
        }
        else
        {
            memcpy(&spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + SLOT_NUM_CHAIN_LENGTH_BYTES], &spi_data[read_frm_offset - START_OFFSET_IN_BUFFER], MAX_CERT_CHAIN_IN_FIRST_ITER);
        }
        bytes_sent_over_mctp_for_cert = MAX_CERT_CHAIN_IN_FIRST_ITER;
        remaining_bytes_to_sent = (uint16_t)((MAX_CERT_CHAIN_IN_FIRST_ITER + read_frm_offset) & UINT16_MAX); // This variable functions as chain offset tracker

        first_get_cert_response_sent = MAX_NUM_BYTES_PLD; // this var used to differentiate between first response and rest
    }
    else if (bytes_sent_over_mctp_for_cert < spdmContext->cert_bytes_to_sent_curr[requested_slot])
    {
        if ((spdmContext->cert_bytes_to_sent_curr[requested_slot] - bytes_sent_over_mctp_for_cert) <= MAX_NUM_BYTES_PLD)
        {
            uint16_t size_to_be_transferred = spdmContext->cert_bytes_to_sent_curr[requested_slot] - bytes_sent_over_mctp_for_cert;
            packet_sz = (uint32_t)(size_to_be_transferred + MCTP_TOT_HDR_SZ);
            if((remaining_bytes_to_sent + size_to_be_transferred - 1) > BUFFER_END_OFFSET ) /* When there aren't enough bytes to be sent from the buffer */
            {
                uint8_t temp[MAX_NUM_BYTES_PLD];
                uint16_t pending_size = 0U;
                uint16_t size_available_in_buffer = 0U;

                size_available_in_buffer = (uint16_t)((BUFFER_END_OFFSET - remaining_bytes_to_sent + 1U) & UINT16_MAX);

                if (is_sub_safe(size_to_be_transferred, size_available_in_buffer) == 0)
                {
                    // Handle Error
                }
                else
                {
                    pending_size = size_to_be_transferred - size_available_in_buffer;
                }
                memcpy(&temp, &spi_data[remaining_bytes_to_sent], size_available_in_buffer); // Store existing data into a local buffer(temp)

                read_bytes_from_chain = (uint16_t)((end_offset_of_requested_chain - BUFFER_END_OFFSET) & UINT16_MAX);

                if(read_bytes_from_chain > SPI_DATA_MAX_BUFF)
                {
                    read_bytes_from_chain = SPI_DATA_MAX_BUFF;
                }

                if(BUFFER_END_OFFSET + 1 > UINT16_MAX )
                {
                    // handle error
                }
                else
                {
                    START_OFFSET_IN_BUFFER = (uint16_t)(BUFFER_END_OFFSET + 1);
                }

                BUFFER_END_OFFSET   = (uint16_t)((START_OFFSET_IN_BUFFER  + read_bytes_from_chain - 1) & UINT16_MAX);
                read_chain_from_offset(START_OFFSET_IN_BUFFER, BUFFER_END_OFFSET, 0, requested_slot); // Fill spi_data buffer with next chain data of length = min(SPI_DATA_MAX_BUFF, pending bytes to be sent)

                spdmContext->request_or_response = 1;
                req_and_response_sz[1] = read_bytes_from_chain;
                spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
                spdm_get_len_for_runtime_hash(spdmContext);
                spdm_crypto_ops_run_time_hashing(&spi_data[0], length, spdmContext,&ctx_ptr, spdmContext->get_requests_state);
                if (pending_size > 0)
                {
                    if (size_available_in_buffer < MAX_NUM_BYTES_PLD)
                    {
                        memcpy(&temp[size_available_in_buffer], &spi_data[0], pending_size); // Get rest of data into temp buffer from spi_data buffer
                    }
                }
                memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS], &temp[0], size_to_be_transferred); // Copy temp to spdm pkt data
            }
            else
            {
                memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS], &spi_data[remaining_bytes_to_sent - START_OFFSET_IN_BUFFER], size_to_be_transferred);
            }
            bytes_sent_over_mctp_for_cert = spdmContext->cert_bytes_to_sent_curr[requested_slot];
        }
        else
        {
            uint16_t size_to_be_transferred = MAX_NUM_BYTES_PLD;
            if((remaining_bytes_to_sent + size_to_be_transferred - 1 ) > BUFFER_END_OFFSET ) /* When there aren't enough bytes to be sent from the buffer */
            {
                uint8_t temp[MAX_NUM_BYTES_PLD];
                uint16_t pending_size = 0U;
                uint16_t size_available_in_buffer = 0U;

                size_available_in_buffer = (uint16_t)((BUFFER_END_OFFSET - remaining_bytes_to_sent + 1U) & UINT16_MAX);
                pending_size = (uint16_t)((size_to_be_transferred - size_available_in_buffer) & UINT16_MAX);

                memcpy(&temp, &spi_data[remaining_bytes_to_sent], size_available_in_buffer);// Store existing data into a local buffer(temp)

                read_bytes_from_chain = (uint16_t)((end_offset_of_requested_chain - BUFFER_END_OFFSET) & UINT16_MAX);

                if (read_bytes_from_chain > SPI_DATA_MAX_BUFF)
                {
                    read_bytes_from_chain = SPI_DATA_MAX_BUFF;
                }

                START_OFFSET_IN_BUFFER = (uint16_t)((BUFFER_END_OFFSET + 1) & UINT16_MAX);
                BUFFER_END_OFFSET   = (uint16_t)((START_OFFSET_IN_BUFFER  + read_bytes_from_chain - 1) & UINT16_MAX);
                read_chain_from_offset(START_OFFSET_IN_BUFFER, BUFFER_END_OFFSET, 0, requested_slot);// Fill spi_data buffer with next chain data of length = min(SPI_DATA_MAX_BUFF, pending bytes to be sent)

                spdmContext->request_or_response = 1;
                req_and_response_sz[1] = read_bytes_from_chain;
                spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
                spdm_get_len_for_runtime_hash(spdmContext);
                spdm_crypto_ops_run_time_hashing(&spi_data[0], length, spdmContext,&ctx_ptr, spdmContext->get_requests_state);


                memcpy(&temp[size_available_in_buffer], &spi_data[0], pending_size); // Get rest of data into temp buffer from spi_data buffer
                memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS], &temp[0], size_to_be_transferred); // Copy temp to spdm pkt data
            }
            else
            {
                memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS], &spi_data[remaining_bytes_to_sent - START_OFFSET_IN_BUFFER], size_to_be_transferred);
            }
            bytes_sent_over_mctp_for_cert = (uint16_t)((bytes_sent_over_mctp_for_cert + MAX_NUM_BYTES_PLD) & UINT16_MAX);
            remaining_bytes_to_sent = (uint16_t)((remaining_bytes_to_sent + MAX_NUM_BYTES_PLD) & UINT16_MAX);
        }
    }
    spdm_flash_busy = false;
}

/******************************************************************************/
/** function to handle tx packet buffering for the required data sent for
 * spdm challenge authentication response
 * @param spdm_buf_tx- tx buffer of spdm with MCTP packet definition fields
 * @param spdmContext - Context of SPDM module
 * @return void
 *******************************************************************************/
void spdm_pkt_fill_spdm_buf_challenge_auth_resp(MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext)
{
    uint8_t offset = 0U;
    uint16_t length = 0;
    if (NULL == spdmContext)
    {
        return;
    }

    switch (first_challenge_resp_sent)
    {
    case TXSTATE_1:
        memcpy(&spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS], &spi_data[0], MCTP_TRANSPORT_LAYER_MINUS_MSG_TYPE_SIZE);
        challenge_tracker =  MCTP_TRANSPORT_LAYER_MINUS_MSG_TYPE_SIZE;
        first_challenge_resp_sent = TXSTATE_2;

        break;
    case TXSTATE_2:
        if(safe_sub_16(actual_challenge_length, challenge_tracker, &length))
        {
            return;
        }
        if(length > MAX_NUM_BYTES_PLD)
        {
            length = MAX_NUM_BYTES_PLD;
        }
        else // last packet
        {
            packet_sz = length + 5;
        }
        memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS],  &spi_data[challenge_tracker], length);
        if (safe_add_16(challenge_tracker, length, &challenge_tracker))
        {
            return;
        }
        break;
    default:
        first_challenge_resp_sent = CHG_RESP_END_OF_TX;
        packet_sz = 0;
        break;
    }
}

/******************************************************************************/
/** Function to fill spdm_buffer with random number
 * @param spdm_buf_bytes  - Offset in spdm_buffer to start filling random number
 * @param offset          - start copying from offset specified by this field from random_number buffer
 * @param bytes_to_sent   - Number of bytes to copy
 * @return spdm_buf_bytes - Total Number of bytes of random number copied (Inclusive of all transaction, not just the current one)
 *******************************************************************************/
uint16_t spdm_pkt_fill_from_rnd_no(uint16_t spdm_buf_bytes, uint8_t offset, uint16_t bytes_to_sent)
{
    for (uint16_t i = 0; i < bytes_to_sent; i++)
    {
        spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + spdm_buf_bytes + i] = nonce_data[offset + i];
    }

    if ((spdm_buf_bytes + bytes_to_sent) > UINT16_MAX) // Coverity INT31-C Fix
    {
        // handle error
    }
    else
    {
        spdm_buf_bytes = (uint16_t)(spdm_buf_bytes + bytes_to_sent);
    }
    return spdm_buf_bytes;
}

/******************************************************************************/
/** Handle tx packet buffering when responding to KEY_EXCHANGE request
 * @param spdm_buf_tx- tx buffer of spdm with MCTP packet definition fields
 * @param spdmContext - Context of SPDM module
 * @return void
 *******************************************************************************/
void spdm_pkt_fill_spdm_buf_key_exchange_rsp(MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext)
{
    if (NULL == spdmContext)
    {
        return;
    }

    switch (key_exchange_rsp_multiple_pkt_state)
    {
    case STATE_1:
        // add 64 - 38 = 26 bytes of exchange data buffered
        for (uint8_t i = 0; i < 23 ; i++)
        {
            spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 38 + i] = key_exchange_resp_obj.exchange_data[i];
        }

        exchange_data_offset = (23);

        key_exchange_rsp_multiple_pkt_state = STATE_2;

        break;

    case STATE_2:
        for (uint16_t i = 0; i < MAX_NUM_BYTES_PLD; i++)
        {
            spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + i] = key_exchange_resp_obj.exchange_data[exchange_data_offset + i];
        }
        if (safe_add_8(MAX_NUM_BYTES_PLD, exchange_data_offset, &exchange_data_offset))
        {
            return;
        }
        // 96 - 23 - 64 = 9 bytes of exchange data to be sent
        key_exchange_rsp_multiple_pkt_state = STATE_3;
        break;
    
    case STATE_3:
        if (exchange_data_offset < DHE_384_PUB_KEY_SIZE) {
            for (uint16_t i = 0; i < (DHE_384_PUB_KEY_SIZE - exchange_data_offset); i++)
            {
                spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + i] = key_exchange_resp_obj.exchange_data[exchange_data_offset + i];
            }
        } else {
            return;
        }
        remaining_bytes_to_sent = (uint16_t)((MAX_NUM_BYTES_PLD - (DHE_384_PUB_KEY_SIZE - exchange_data_offset)) & UINT16_MAX);
        // if measurement hash summary field present
        // 55bytes can be sent
        if (measurement_var.msr_hash_rqstd)
        {
            if (measurement_var.msr_hash_rqstd == 0xff)
            {
                // measurement hash bytes that can be copied is  64 - 9 = 55
                memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + 9], &hash_concat_msrments_val[0],
                       SPDM_SHA384_LEN);
            }
            else
            {
                // measurement hash bytes that can be copied is 64 - 9 = 55
                memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + 9], &hash_concat_tcb_msrments_val[0],
                       SPDM_SHA384_LEN);
            }

            // 55 - 48 = 7bytes of data can be sent
            if (safe_sub_16(remaining_bytes_to_sent, SPDM_SHA384_LEN, &remaining_bytes_to_sent))
            {
                return;
            }
            opaq_size_remaining = sizeof(opaque_data);
            memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + 57], &opaque_data, 7);   
            // remaining 10bytes of opaque data, followed by signature
            key_exchange_rsp_multiple_pkt_state = STATE_4;
        }
        else
        {
            // 64 - 9 = 55 bytes, 55 bytes can be sent
            opaq_size_remaining = sizeof(opaque_data);
            memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + 9], &opaque_data, sizeof(opaque_data)); //size of 20bytes
            // 55 - 20 = 35 bytes can be sent
            for (uint16_t i = 0; i < 35; i++)
            {
                spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + 29 + i] = ecdsa_signature.ecdsa_signature[i];
            }
            // remaining 58bytes of signature can be sent
        }
        key_exchange_rsp_multiple_pkt_state = STATE_4;
        break;
    case STATE_4:
        if (measurement_var.msr_hash_rqstd) {
            memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS], &(opaque_data.total_elements), 13);
            // 51 bytes can be sent
            for (uint8_t i = 0; i < 51; i++) {
                spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + 13 + i] = ecdsa_signature.ecdsa_signature[i];
            }
            key_exchange_rsp_multiple_pkt_state = STATE_5;
            // 44 bytes should be sent
        } else {
            // 61 bytes can be sent
            for (uint8_t i = 0; i < 61; i++) {
                spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + i] = ecdsa_signature.ecdsa_signature[35 + i];
            }
            memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + 61], responder_verify_data, 3); // 48 -3 = 45 bytes of responderverifydata to be sent
            key_exchange_rsp_multiple_pkt_state = STATE_5;
        }
        break;
    case STATE_5:
        if (measurement_var.msr_hash_rqstd) {
            for (uint8_t i = 0; i < 45; i++) {
                spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + i] = ecdsa_signature.ecdsa_signature[51 + i];
            }
            // 64 - 44 = 20 bytes of responder verify data can be sent
            memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + 45], &responder_verify_data[0], 19); // 48 - 20  = 28 bytes of responderverifydata to be sent
            key_exchange_rsp_multiple_pkt_state = STATE_6;

        } else {
            memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS], &responder_verify_data[3], 45); 
            key_exchange_rsp_multiple_pkt_state = KEY_EXCHNG_END_STATE;
            packet_sz = 45 + 5;
        }
        break;
    case STATE_6:
        if (measurement_var.msr_hash_rqstd) {
            // 26 bytes of responder verify data must be sent
            memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS], &responder_verify_data[19], 29); 
            key_exchange_rsp_multiple_pkt_state = KEY_EXCHNG_END_STATE;
        }
        packet_sz = 29 + 5;
        break;
    default:
        packet_sz = 0;
        break;
    }
}

/******************************************************************************/
/** Fill tx packet buffer for encrypted data as part of Secure Session
 * @param spdm_buf_tx- tx buffer of spdm with MCTP packet definition fields
 * @param spdmContext - Context of SPDM module
 * @return void
 *******************************************************************************/
void spdm_pkt_fill_spdm_buf_ss(MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext)
{
    if (NULL == spdmContext)
    {
        return;
    }

    switch(state_for_msr_ss)
    {
        case SOM:
            if (remaining_for_ss > MCTP_TRANSPORT_LAYER_MINUS_MSG_TYPE_SIZE) {
                memcpy(&spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS], &buf_for_encryption[0], MCTP_TRANSPORT_LAYER_MINUS_MSG_TYPE_SIZE); 
                state_for_msr_ss = REMAINING;
                if (safe_add(offset_for_msr_ss, MCTP_TRANSPORT_LAYER_MINUS_MSG_TYPE_SIZE, &offset_for_msr_ss))
                {
                    return;
                }
                if (safe_sub(remaining_for_ss, MCTP_TRANSPORT_LAYER_MINUS_MSG_TYPE_SIZE, &remaining_for_ss))
                {
                    return;
                }
            } else {
                memcpy(&spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS], &buf_for_encryption[0], remaining_for_ss); 
                state_for_msr_ss = MSR_RESP_MUL_PKT_SS_END;
            }
            break;
        case REMAINING:
            if (remaining_for_ss > MAX_NUM_BYTES_PLD)
            {
                memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS], &buf_for_encryption[offset_for_msr_ss], MAX_NUM_BYTES_PLD);
                if (safe_add(offset_for_msr_ss, MAX_NUM_BYTES_PLD, &offset_for_msr_ss))
                {
                    return;
                }
                remaining_for_ss = remaining_for_ss - MAX_NUM_BYTES_PLD;
            } else {
                memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS], &buf_for_encryption[offset_for_msr_ss], remaining_for_ss); 
                state_for_msr_ss = MSR_RESP_MUL_PKT_SS_END;
                packet_sz = remaining_for_ss + 5;
            }
            break;
        default:
            break;
    }

}
/******************************************************************************/
/** Handle tx packet buffering when responding to GET_MEASUREMENT request
 * @param spdm_buf_tx- tx buffer of spdm with MCTP packet definition fields
 * @param spdmContext - Context of SPDM module
 * @param is_secure_msg - in secure session or not
 * @return void
 *******************************************************************************/
void spdm_pkt_fill_spdm_buf_measurements_resp(MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext, bool is_secure_msg)
{
    uint8_t offset = 0U;
    uint16_t length = 0;
    uint16_t result = 0;
    if (NULL == spdmContext)
    {
        return;
    }

    switch (first_meas_resp_sent)
    {
        case TXSTATE_1:
            memcpy(&spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS], &spi_data[MEAS_SPI_DATA_START], MCTP_TRANSPORT_LAYER_MINUS_MSG_TYPE_SIZE);
            meas_tracker = MEAS_SPI_DATA_START + MCTP_TRANSPORT_LAYER_MINUS_MSG_TYPE_SIZE;
            first_meas_resp_sent = TXSTATE_2;
            break;
        case TXSTATE_2:
            if (safe_sub_16(actual_meas_length, meas_tracker, &result))
            {
                return;
            }

            if (safe_add_16(result, MEAS_SPI_DATA_START, &length))
            {
                return;
            }

            if (length > MAX_NUM_BYTES_PLD)
            {
                length = MAX_NUM_BYTES_PLD;
            }
            else
            {
                packet_sz = length + 5;
            }

            memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS], &spi_data[meas_tracker], length);
            if (safe_add_16(meas_tracker, length, &meas_tracker))
            {
                return;
            }
            break;
        default:
            first_meas_resp_sent = CHG_RESP_END_OF_TX;
            packet_sz = 0;
            break;
    }
}


/******************************************************************************/
/** function to populate spdm data and fill MCTP fields in mctp tx buffer fields before sending the response to MCTP module
 * @param spdm_buf_tx  - Fill spdm response in spdm_buf_tx pointer
 * @param mctp_buf     - mctp_buf pointer needs to be populated with MCTP fields and Payload which is finally sent to MCTP layer
 * @param cmd_resp     - SPDM current response command
 * @param spdmContext  - SPDM module context
 * @return void
 *******************************************************************************/
void spdm_pkt_populate_mctp_packet_for_resp(MCTP_PKT_BUF *spdm_buf_tx, MCTP_PKT_BUF *mctp_buf, uint8_t cmd_resp,
        SPDM_CONTEXT *spdmContext)
{
    if (NULL == spdmContext)
    {
        return;
    }

    if (first_pkt)
    {
        first_pkt = false;
        mctp_buf->pkt.field.hdr.som = 1;
    }
    else
    {
        mctp_buf->pkt.field.hdr.som = 0;
    }

    /* destination slave address */
    mctp_buf->pkt.field.hdr.dst_addr = spdmContext->host_slv_addr;
    mctp_buf->pkt.field.hdr.rw_dst = 0;
    /*  Command Code */
    mctp_buf->pkt.field.hdr.cmd_code = spdmContext->spdm_cmd_code;
    /* Source Slave address*/
    mctp_buf->pkt.field.hdr.src_addr = spdmContext->ec_slv_addr;
    mctp_buf->pkt.field.hdr.ipmi_src = 1;
    /* mctp reserved */
    mctp_buf->pkt.field.hdr.mctp_rsvd = 0;
    /* header version supported by this implementation */
    mctp_buf->pkt.field.hdr.hdr_ver = 1;
    /* destination eid */
    mctp_buf->pkt.field.hdr.dst_eid = spdmContext->host_eid;
    /* source eid = eid of self/EC */
    //        mctp_buf->pkt.field.hdr.src_eid   = mctp_rt.ep.ec.field.current_eid;
    mctp_buf->pkt.field.hdr.src_eid = curr_ec_id;//spdmContext->ec_eid;
    /* message tag */
    mctp_buf->pkt.field.hdr.msg_tag = spdmContext->message_tag;
    /* for response packet */
    mctp_buf->pkt.field.hdr.tag_owner = 0;
    /* Packet sequence number */
    mctp_buf->pkt.field.hdr.pkt_seq = pkt_seq_mctp;
    if (message_type == MCTP_IC_MSGTYPE_SECURED_MESSAGE) {
        mctp_buf->pkt.field.hdr.msg_type = MCTP_IC_MSGTYPE_SECURED_MESSAGE;
    } else {
        mctp_buf->pkt.field.hdr.msg_type = MCTP_MSGTYPE_SPDM;
    }

    /* integrity check */
    mctp_buf->pkt.field.hdr.integrity_check = 0;

    mctp_buf->rx_timestamp = spdm_buf_tx->rx_timestamp;

    switch (cmd_resp)
    {
    case SPDM_GET_VERSION_RESP:
        spdm_pkt_fill_spdm_buf_ver_entry_tbl(spdm_buf_tx);
        if (packet_sz > MCTP_BYTECNT_MAX)
        {
            packet_sz = packet_sz + MCTP_TOT_HDR_SZ;
        }
        break;
    case SPDM_GET_DIGEST_RESPONSE:
        spdm_pkt_fill_spdm_buf_digest(spdm_buf_tx);
        if (packet_sz < MCTP_BYTECNT_MAX)
        {
            packet_sz = packet_sz + MCTP_BYTECNT_MIN;
        }
        break;
    case SPDM_GET_CERT_RESPONSE:
        if (spdmContext->requested_slot[requested_slot])
        {
            spdm_pkt_fill_spdm_buf_certificate(spdm_buf_tx, spdmContext);
            if (packet_sz > MCTP_BYTECNT_MAX)
            {
                packet_sz = packet_sz + MCTP_TOT_HDR_SZ;
            }
        }
        else
        {
            return;
        }
        break;
    case SPDM_CHALLENGE_AUTH_RSP:
        spdm_pkt_fill_spdm_buf_challenge_auth_resp(spdm_buf_tx, spdmContext);
        if (packet_sz > MCTP_BYTECNT_MAX)
        {
            packet_sz = packet_sz + MCTP_TOT_HDR_SZ;
        }
        break;
    case SPDM_GET_MEASUREMENT_RSP:
        if (message_type != MCTP_IC_MSGTYPE_SECURED_MESSAGE) {
            spdm_pkt_fill_spdm_buf_measurements_resp(spdm_buf_tx, spdmContext, false);
            if (packet_sz > MCTP_BYTECNT_MAX)
            {
                packet_sz = packet_sz + MCTP_TOT_HDR_SZ;
            }
        } else {
            spdm_pkt_fill_spdm_buf_ss(spdm_buf_tx, spdmContext);
            if (packet_sz > MCTP_BYTECNT_MAX)
            {
                packet_sz = packet_sz + MCTP_TOT_HDR_SZ;
            }
        }
        break;
    case SPDM_KEY_EXCHANGE_RSP:
        spdm_pkt_fill_spdm_buf_key_exchange_rsp(spdm_buf_tx, spdmContext);
        if (packet_sz > MCTP_BYTECNT_MAX)
        {
            packet_sz = packet_sz + MCTP_TOT_HDR_SZ;
        }
        break;
    case SPDM_ENCAP_REQ_RSP:
    case DELIVER_ENCAP_RSP_RSP:
    case SPDM_FINISH_RSP:
    case SPDM_END_SESSION_RSP:
    case SPDM_HEARTBEAT_ACK:
    case SPDM_ERROR_RESP:
        if (message_type == MCTP_IC_MSGTYPE_SECURED_MESSAGE) {  
            spdm_pkt_fill_spdm_buf_ss(spdm_buf_tx, spdmContext);
            if (packet_sz > MCTP_BYTECNT_MAX)
            {
                packet_sz = packet_sz + MCTP_TOT_HDR_SZ;
            }
        }
        break;
    default:
        /* Invalid case */
        break;
    }

    if (packet_sz > MCTP_BYTECNT_MAX)
    {
        mctp_buf->pkt.field.hdr.byte_cnt = MCTP_BYTECNT_MAX;
        mctp_buf->pkt.field.hdr.eom = 0;
        if (mctp_buf->pkt.field.hdr.som == 1)
        {
            memcpy(&mctp_buf->pkt.data[SPDM_HEADER_VERSION_POS], &spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS],
                   MAX_NUM_BYTES_PLD);
            packet_sz -= MAX_NUM_BYTES_PLD;
        }
        else
        {
            memcpy(&mctp_buf->pkt.data[SPDM_MSG_TYPE_POS], &spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS], MAX_NUM_BYTES_PLD + 1);
            packet_sz -= (MAX_NUM_BYTES_PLD + 1);
        }
    }
    else
    {
        mctp_buf->pkt.field.hdr.eom = 1;
        mctp_buf->pkt.field.hdr.byte_cnt = packet_sz;
        if (mctp_buf->pkt.field.hdr.som == 1)
        {
            memcpy(&mctp_buf->pkt.data[SPDM_HEADER_VERSION_POS], &spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS],
                   packet_sz - MCTP_BYTECNT_MIN);
        }
        else
        {
            memcpy(&mctp_buf->pkt.data[SPDM_MSG_TYPE_POS], &spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS], packet_sz - MCTP_BYTECNT_MIN);
        }
        packet_sz = 0;
    }

    pkt_seq_mctp += 1;
    if (pkt_seq_mctp > 3)
        pkt_seq_mctp = 0;
}


/******************************************************************************/
/** function load the get version response bytes to spdm local buffer
 * @param spdmContext - SPDM module context
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @return void
 *******************************************************************************/
void spdm_pkt_process_get_version_cmd(MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext)
{
    if (NULL == spdmContext)
    {
        return;
    }
    memset(get_version_response, 0, sizeof(GET_VERSION_RESP_FIELDS));
    spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS] = VERSION_DEFAULT; // V1.0
    get_ver_resp_object.version = VERSION_DEFAULT;
    memset(get_ver_resp_object.reserved0, 0, 3);

    spdm_buf_tx->pkt.data[SPDM_HEADER_COMMAND_POS] = SPDM_GET_VERSION_RESP;
    get_ver_resp_object.resp_code = SPDM_GET_VERSION_RESP;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 3] = SPDM_VER_NUM_ENTRY_COUNT;
    get_ver_resp_object.entry_count = SPDM_VER_NUM_ENTRY_COUNT;

    packet_sz = SPDM_GET_VER_RESP_SIZE + (2 * SPDM_VER_NUM_ENTRY_COUNT);
    spdmContext->current_resp_cmd = SPDM_GET_VERSION_RESP;

    // call hash engine to feed the response bytes to run time hash sequence
    spdmContext->request_or_response = 1;
    req_and_response_sz[1] = sizeof(GET_VERSION_RESP_FIELDS);

    memcpy(get_version_response, (uint8_t *)&get_ver_resp_object, sizeof(GET_VERSION_RESP_FIELDS));

    // calculate run time hash of response
    spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
    spdmContext->secure_session_get_requests_state = RUN_TIME_HASH_MODE;

    spdm_get_len_for_runtime_hash(spdmContext);
    spdm_crypto_ops_run_time_hashing(&get_version_response[0], length, spdmContext, &ctx_ptr, spdmContext->get_requests_state);

    for (uint8_t i = HASH_CTX_KEY_EXCHANGE; i < HASH_CTX_MAX; i++)
    {
        spdm_crypto_ops_run_time_hashing(&get_version_response[0], length, spdmContext, &ctx_ptr_arr_ss[i], spdmContext->secure_session_get_requests_state);
    }
}

/******************************************************************************/
/** function to load the get capabilities response bytes to spdm local buffer
 * @param spdmContext - SPDM module context
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @return void
 *******************************************************************************/
void spdm_pkt_process_get_capabilties_cmd(MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext)
{
    if (NULL == spdmContext)
    {
        return;
    }
    spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS] = CURRENT_VERSION; // V1.1
    get_cap_resp_object.version = CURRENT_VERSION;
    memset(get_cap_resp_object.reserved0, 0, 3);
    memset(get_cap_resp_object.reserved1, 0, 2);

    spdm_buf_tx->pkt.data[SPDM_HEADER_COMMAND_POS] = SPDM_GET_CAPABILITIES_RESP;
    get_cap_resp_object.resp_code = SPDM_GET_CAPABILITIES_RESP;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 3] = SPDM_CAP_CT_EXP;
    get_cap_resp_object.cte_expo = SPDM_CAP_CT_EXP;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + SPDM_CAP_FLAG_BYTE4_OFFSET] = SPDM_CAP_FLAG_BYTE1;
    get_cap_resp_object.flags[BYTE0] = SPDM_CAP_FLAG_BYTE1;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + SPDM_CAP_FLAG_BYTE3_OFFSET] = SPDM_CAP_FLAG_BYTE2;
    get_cap_resp_object.flags[BYTE1] = SPDM_CAP_FLAG_BYTE2;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + SPDM_CAP_FLAG_BYTE2_OFFSET] = SPDM_CAP_FLAG_BYTE3;
    get_cap_resp_object.flags[BYTE2] = SPDM_CAP_FLAG_BYTE3;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + SPDM_CAP_FLAG_BYTE1_OFFSET] = SPDM_CAP_FLAG_BYTE4;
    get_cap_resp_object.flags[BYTE3] = SPDM_CAP_FLAG_BYTE4;

    spdm_buf_tx->pkt.field.hdr.byte_cnt = SPDM_GET_CAP_RESP_SIZE;
    packet_sz = SPDM_GET_CAP_RESP_SIZE;
    spdmContext->current_resp_cmd = SPDM_GET_CAPABILITIES_RESP;

    memcpy(get_cap_response, (uint8_t *)&get_cap_resp_object, sizeof(GET_CAP_RESP_FIELDS));

    spdmContext->request_or_response = 1;
    req_and_response_sz[1] = sizeof(GET_CAP_RESP_FIELDS);
    // calculate run time hash of response
    spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
    spdmContext->secure_session_get_requests_state = RUN_TIME_HASH_MODE;

    spdm_get_len_for_runtime_hash(spdmContext);
    spdm_crypto_ops_run_time_hashing(&get_cap_response[0], length, spdmContext,&ctx_ptr, spdmContext->get_requests_state);
    for (uint8_t i = HASH_CTX_KEY_EXCHANGE; i < HASH_CTX_MAX; i++)
    {
        spdm_crypto_ops_run_time_hashing(&get_cap_response[0], length, spdmContext, &ctx_ptr_arr_ss[i], spdmContext->secure_session_get_requests_state);
    }
}

/******************************************************************************/
/** function to load the negotiate algo response bytes to spdm local buffer
 * @param spdmContext - SPDM module context
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @return void
 *******************************************************************************/
void spdm_pkt_process_neg_alg_cmd(MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext)
{
    if (NULL == spdmContext)
    {
        return;
    }
    spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS] = CURRENT_VERSION; // V1.1
    nego_algo_resp_object.version = CURRENT_VERSION;
    nego_algo_resp_object.reserved = 0x00;
    nego_algo_resp_object.reserved1 = 0x00;
    memset(nego_algo_resp_object.reserved2, 0, 12);
    memset(nego_algo_resp_object.reserved3, 0, 2);

    spdm_buf_tx->pkt.data[SPDM_HEADER_COMMAND_POS] = SPDM_NEG_ALGO_RESP;
    nego_algo_resp_object.resp_code = SPDM_NEG_ALGO_RESP;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS] = SPDM_NEG_ALG_NO_ALG_STRC;
    nego_algo_resp_object.no_of_algo_struct = SPDM_NEG_ALG_NO_ALG_STRC;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 2] = SPDM_NEG_ALG_LENGTH_RESP;

    memcpy(nego_algo_resp_object.len_of_resp, (uint8_t *)&spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 2], 2);

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 4] = SPDM_NEG_ALG_MSR_SPC;
    nego_algo_resp_object.msr_specific = SPDM_NEG_ALG_MSR_SPC;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 6] = SPDM_NEG_ALG_MSR_HASH_ALGO;
    memcpy(nego_algo_resp_object.msr_hash_algo, (uint8_t *)&spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 6], 4);

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 10] = SPDM_NEG_ALG_BASE_ASYM_SEL;
    memcpy(nego_algo_resp_object.base_asym_sel, (uint8_t *)&spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 10], 4);

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 14] = SPDM_NEG_ALG_BASE_HASH_SEL;
    memcpy(nego_algo_resp_object.base_hash_sel, (uint8_t *)&spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 14], 4);

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 30] = SPDM_NEG_ALG_NO_EXT_SIG;
    nego_algo_resp_object.ext_asym_sel_cnt = SPDM_NEG_ALG_NO_EXT_SIG;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 31] = SPDM_NEG_ALG_NO_EXT_HASH;
    nego_algo_resp_object.ext_hash_sel_cnt = SPDM_NEG_ALG_NO_EXT_HASH;

    // DHE
    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 34] = struct_algo[0].AlgType;
    nego_algo_resp_object.req_algo_struct.algo_struct[0] = struct_algo[0].AlgType;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 35] = struct_algo[0].AlgCount;
    nego_algo_resp_object.req_algo_struct.algo_struct[1] = struct_algo[0].AlgCount;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 36] = struct_algo[0].AlgSupported;
    nego_algo_resp_object.req_algo_struct.algo_struct[2] = struct_algo[0].AlgSupported;

    // AEAD
    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 38] = struct_algo[1].AlgType;
    nego_algo_resp_object.req_algo_struct.algo_struct[4] = struct_algo[1].AlgType;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 39] = struct_algo[1].AlgCount;
    nego_algo_resp_object.req_algo_struct.algo_struct[5] = struct_algo[1].AlgCount;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 40] = struct_algo[1].AlgSupported;
    nego_algo_resp_object.req_algo_struct.algo_struct[6] = struct_algo[1].AlgSupported;

    // ReqAsymAlg
    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 42] = struct_algo[2].AlgType;
    nego_algo_resp_object.req_algo_struct.algo_struct[8] = struct_algo[2].AlgType;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 43] = struct_algo[2].AlgCount;
    nego_algo_resp_object.req_algo_struct.algo_struct[9] = struct_algo[2].AlgCount;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 44] = struct_algo[2].AlgSupported;
    nego_algo_resp_object.req_algo_struct.algo_struct[10] = struct_algo[2].AlgSupported;

    // KeySchedule
    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 46] = struct_algo[3].AlgType;
    nego_algo_resp_object.req_algo_struct.algo_struct[12] = struct_algo[3].AlgType;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 47] = struct_algo[3].AlgCount;
    nego_algo_resp_object.req_algo_struct.algo_struct[13] = struct_algo[3].AlgCount;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 48] = struct_algo[3].AlgSupported;
    nego_algo_resp_object.req_algo_struct.algo_struct[14] = struct_algo[3].AlgSupported;

    spdm_buf_tx->pkt.field.hdr.byte_cnt = SPDM_NEG_ALG_RESP_SIZE;
    packet_sz = SPDM_NEG_ALG_RESP_SIZE;
    spdmContext->current_resp_cmd = SPDM_NEG_ALGO_RESP;

    memcpy(nego_algo_response, (uint8_t *)&nego_algo_resp_object, sizeof(NEG_ALGO_RESP_FIELDS));

    spdmContext->request_or_response = 1;
    req_and_response_sz[1] = sizeof(NEG_ALGO_RESP_FIELDS);
    // calculate run time hash of response
    spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
    spdmContext->secure_session_get_requests_state = RUN_TIME_HASH_MODE;
    spdm_get_len_for_runtime_hash(spdmContext);
    spdm_crypto_ops_run_time_hashing(&nego_algo_response[0], length, spdmContext,&ctx_ptr, spdmContext->get_requests_state);

    for (uint8_t i = HASH_CTX_KEY_EXCHANGE; i < HASH_CTX_MAX; i++)
    {
        spdm_crypto_ops_run_time_hashing(&nego_algo_response[0], length, spdmContext, &ctx_ptr_arr_ss[i], spdmContext->secure_session_get_requests_state);
    }
}

/******************************************************************************/
/** function to form the GET_DIGEST request message
 * @param spdmContext - SPDM module context
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @return void
 *******************************************************************************/
uint8_t spdm_get_encap_request_get_digest(void *get_digest_request, uint8_t *size)
{
    if (get_digest_request == NULL)
    {
        return FALSE;
    }
    GET_DIGEST_REQ_FIELDS *get_digest;
    get_digest = get_digest_request;
    get_digest->version = CURRENT_VERSION;
    get_digest->resp_code = SPDM_GET_DIGEST;
    get_digest->param1 = 0;
    get_digest->param2 = 0;

    uint8_t size_of_req = sizeof(GET_DIGEST_REQ_FIELDS);

    *size = size_of_req;

    return TRUE;
}

/******************************************************************************/
/** function to parse GET_DIGEST response
 * @param None
 * @return void
 *******************************************************************************/
uint8_t spdm_process_encap_response_digest()
{
    if (requester_cert_chain_slot < PRECISION_GENERIC(1U)) // Coverity INT34-C
    {
        if (!(get_mctp_pld[7] & ( 1 << requester_cert_chain_slot)))
        {
            return FALSE;
        }
    }
    memcpy(hash_of_req_chains, &get_mctp_pld[8], SHA384_LEN_BYTES);
    return TRUE;
}

/******************************************************************************/
/** function to load the digest response bytes to spdm local buffer
 * @param spdmContext - SPDM module context
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @return void
 *******************************************************************************/
void spdm_pkt_process_get_digest_cmd(MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext)
{
    if (NULL == spdmContext)
    {
        return;
    }
    uint8_t is_chain_available_in_slot = 0x00;
    uint8_t iter = 0x00;

    spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS] = CURRENT_VERSION; // V1.1
    get_digest_resp_object.version = CURRENT_VERSION;
    get_digest_resp_object.reserved = 0x00;

    spdm_buf_tx->pkt.data[SPDM_HEADER_COMMAND_POS] = SPDM_GET_DIGEST_RESPONSE;
    get_digest_resp_object.resp_code = SPDM_GET_DIGEST_RESPONSE;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 1] = chain_avail_bits_mask;
    get_digest_resp_object.slot_mask = chain_avail_bits_mask;

    packet_sz = SPDM_GET_DIGEST_RESPONSE_SIZE + ((spdmContext->no_of_chains_avail) * SPDM_SHA384_LEN); // pkt size updated here
    spdmContext->current_resp_cmd = SPDM_GET_DIGEST_RESPONSE;

    memcpy(get_digest_response, (uint8_t *)&get_digest_resp_object, sizeof(GET_DIGEST_FIELDS));

    spdmContext->request_or_response = 1;
    req_and_response_sz[1] = sizeof(GET_DIGEST_FIELDS);

    // calculate run time hash of response
    spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
    spdm_get_len_for_runtime_hash(spdmContext);
    spdm_crypto_ops_run_time_hashing(&get_digest_response[0], length, spdmContext,&ctx_ptr, spdmContext->get_requests_state);

    for (iter = 0; iter < MAX_SLOTS; iter++)
    {

        if (((chain_avail_bits_mask & (1U << iter)) >> iter) > UINT8_MAX) // Coverity INT31-C Fix
        {
            //trace0(ERROR_HANDLE_TRACE, SPDM_TSK, 0, "spgdc");
        }
        else
        {
            is_chain_available_in_slot = (chain_avail_bits_mask & (1U << iter)) >> iter;
        }
        if (is_chain_available_in_slot & 0x01)
        {
            req_and_response_sz[1] = SPDM_SHA384_LEN;
            spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
            spdm_get_len_for_runtime_hash(spdmContext);
            spdm_crypto_ops_run_time_hashing(&hash_of_chains[(iter * SPDM_SHA384_LEN)], length, spdmContext,&ctx_ptr, 
                                             spdmContext->get_requests_state);
        }
    }
    digest_state_machine = TXSTATE_1;
    remaining_bytes_to_sent = 0;
}

/******************************************************************************/
/** function to get certificate number corresponding to offset in chain
 * @param start_offset    - start_offset of certificate whose position in chain needs to be identified
 * @param slot            - chain slot
 * @return certificate number
 *******************************************************************************/
uint8_t get_cert_num(uint16_t start_offset, uint8_t slot)
{
    if(start_offset < ROOT_START_OFFSET)
    {
        return 0U;
    }

    uint16_t loop_start_offset = start_offset - ROOT_START_OFFSET; // @offset 52 root certificate starts

    uint8_t cert_num_in_chain = 1U;
    uint8_t current_cert_ptr = slot_buf[slot].chain.head_ptr_val; // get head pointer - root cert

    // Find the certificate number to which start_offset begins:
    for (uint8_t i = 0; i < MAX_CERT_PER_CHAIN; i++)
    {

        // int32_t temp = loop_start_offset - cert_buf[current_cert_ptr].cert_size; // Cert_size is computed and stored when storing hash of chain (state: SPDM_CALC_HASH_CHAIN)
        if (loop_start_offset == cert_buf[current_cert_ptr].cert_size)
        {
            break;
        }
        else if (loop_start_offset < cert_buf[current_cert_ptr].cert_size)
        {
            cert_num_in_chain = (uint8_t)((cert_num_in_chain - 1) & UINT8_MAX);
            break;
        }
        else
        {
            loop_start_offset = loop_start_offset - cert_buf[current_cert_ptr].cert_size;
        }
        current_cert_ptr = cert_buf[current_cert_ptr].tail_ptr_val;

        if(cert_num_in_chain + 1 > UINT8_MAX)
        {
            // handle error
        }
        else
        {
            cert_num_in_chain = (uint8_t)(cert_num_in_chain + 1);
        }
    }
    return cert_num_in_chain;
}

/******************************************************************************/
/** function to  get cert start offset corresponding to a certificate number in chain
 * @param cert_num_in_chain - certificate number in chain
 * @param slot            - chain slot
 * @param cert_ptr        - update cert ptr with its location in flash
 * @return certificate's start offset in chain
 *******************************************************************************/
uint16_t get_cert_start_offset(uint8_t cert_num_in_chain, uint8_t slot, uint8_t *cert_ptr)
{
    uint16_t cert_start_offset = ROOT_START_OFFSET;
    uint8_t current_cert_ptr = slot_buf[slot].chain.head_ptr_val;

    for(int8_t i = 0; i <= (cert_num_in_chain - 1); i++)
    {
        uint64_t temp = cert_start_offset + cert_buf[current_cert_ptr].cert_size;
        if(temp > UINT32_MAX)
        {
            // handle error
        }
        else
        {
            cert_start_offset = temp;
        }
        current_cert_ptr = cert_buf[current_cert_ptr].tail_ptr_val;
    }

    *cert_ptr = current_cert_ptr;

    return cert_start_offset;
}

/******************************************************************************/
/** function to read and store chain data from start_offset to end_offset into spi_data buffer
 * @param start_offset    - start_offset of the chain to be read
 * @param end_offset      - end_offset of the chain to be read (inclusive)
 * @param spi_data_offset - offset to be put in spi_data buffer
 * @return void
 *******************************************************************************/
void read_chain_from_offset(uint16_t start_offset, uint16_t end_offset, uint32_t spi_data_offset, uint8_t slot)
{
    uint32_t pre_checks = (end_offset < start_offset) || (spi_data_offset >= SPI_DATA_MAX_BUFF) || ((spi_data_offset + end_offset - start_offset + 1) > SPI_DATA_MAX_BUFF) || (start_offset >= slot_buf[slot].chain.chain_length) || (slot > 7);
    if(pre_checks == 1)
    {
        return;
    }

    // end_offset cannot be greater than chain_length
    end_offset = (end_offset >= slot_buf[slot].chain.chain_length) ? slot_buf[slot].chain.chain_length - 1 : end_offset;
    int32_t size_to_read = end_offset - start_offset + 1;

    // First 52 bytes of the chain includes length of the certificate(2 bytes), reserved bytes(2), roothash(48)
    if(start_offset < ROOT_START_OFFSET)
    {

        uint16_t limit_end_offset = end_offset > 51 ? 51 : end_offset;
        uint8_t temp[ROOT_START_OFFSET];
        uint16_t copy_bytes = 0U;

        // Store first 52 bytes into temp buffer
        temp[0] = (slot_buf[slot].chain.chain_length & 0xff);
        temp[1] = (slot_buf[slot].chain.chain_length >> 8);
        memcpy(&temp[4], &slot_buf[slot].chain.root_cert_hash[0], SPDM_SHA384_LEN);

        // Now copy bytes from temp to spi_data
        copy_bytes = limit_end_offset - start_offset + 1;
        memcpy(&spi_data[spi_data_offset], &temp[start_offset], copy_bytes);

        spi_data_offset = spi_data_offset + copy_bytes; // Increment spi_data_offset to store subsequent chain data
        start_offset = start_offset + copy_bytes; // Increment start_offset with bytes_copied
        size_to_read = size_to_read - copy_bytes; // decrement size_to_read with bytes_copied
        if(size_to_read <= 0)
        {
            return; // Implies end_offset requested was within 0 to 51
        }
    }



    uint8_t  cert_num_in_chain = get_cert_num(start_offset, slot);

    uint8_t current_cert_ptr = 0;
    
    // Now location of the certificate in chain is known ;
    // traverse the chain again to find the starting offset of the certificate which will be used to find the offset at which particular certificate is requested
    uint16_t cert_start_offset = get_cert_start_offset(cert_num_in_chain, slot, &current_cert_ptr);

    uint8_t first_cert = 1U;
    size_to_read = end_offset - start_offset + 1;
    spdm_di_qmspi_clr_port_init_status(SPI_SELECT_INT_COMP_0);
    spdm_di_init_flash_component(SPI_SELECT_INT_COMP_0);
      
    while (((cert_buf[current_cert_ptr].tail_ptr_val) != END_OF_CHAIN) && (size_to_read > 0)) /* Keep reading the certificates until req. size (or)
                                                                                                     ak_cert which needs to be read from SRAM_MBOX is reached */
    {
        uint32_t get_mem = cert_buf[current_cert_ptr].mem_addr;
        uint32_t spi_read_size = cert_buf[current_cert_ptr].cert_size;
        if(spi_read_size == 0U)
        {
            current_cert_ptr = cert_buf[current_cert_ptr].tail_ptr_val;
            continue;
        }
        if(first_cert == 1U)
        {
            get_mem = get_mem + start_offset - cert_start_offset;
            first_cert = 0U;
            spi_read_size = spi_read_size - (start_offset - cert_start_offset);
        }
        else
        {
            /* Invalid case for size */
            ;
        }
        spi_read_size = (spi_read_size > size_to_read) ?  size_to_read : spi_read_size; // Limit spi_read_size based on length to read

        if (spdm_read_certificate(get_mem, &spi_data[spi_data_offset], spi_read_size, current_cert_ptr))
        {
            spdm_di_spi_tristate(INT_SPI);
            return;
        }

        size_to_read = size_to_read - spi_read_size; // decrement size_to_read with bytes_copied
        spi_data_offset = spi_data_offset + spi_read_size;  // Increment spi_data_offset to store subsequent chain data
        current_cert_ptr = cert_buf[current_cert_ptr].tail_ptr_val; // Move to next pointer
    }
    if(size_to_read > 0) // If still pending bytes are to be read -> Implies tail poiner is reached and SRAM_MBOX needs to be read
    {
        uint32_t spi_read_size = cert_buf[current_cert_ptr].cert_size;
        uint16_t ak_cert_start_offset = 0u;
        spi_read_size = (spi_read_size > size_to_read) ?  size_to_read : spi_read_size;
        if(first_cert == 1U)
        {
            ak_cert_start_offset =  start_offset - cert_start_offset;
        }
        else
        {
            ak_cert_start_offset = 0U;
        }

        uint16_t ak_bytes = spi_read_size > 1024 ? 1024 : (uint16_t)(spi_read_size);
        spdm_read_certificate(ak_cert_start_offset, &spi_data[spi_data_offset], ak_bytes, current_cert_ptr);
    }
    spdm_di_spi_tristate(INT_SPI);
}


/******************************************************************************/
/** Function to parse GET_CERTIFICATE request and fill part of spdm local buffer based on the request
 * @param spdmContext - SPDM module context
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @return void
 *******************************************************************************/
uint8_t spdm_pkt_process_get_cert_cmd(MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext)
{
    uint16_t cert_chain_frmt_tbl_sz = 0x00;
    uint32_t local_offset_in_cert_req = 0;

    if (NULL == spdmContext)
    {
        return SPDM_CMD_FAIL;
    }

    requested_slot = (get_mctp_pld[SLOT_REQUEST_OFFSET] & 0xff); // attain the value of slot requested

    memset(get_cert_response, 0, sizeof(GET_CERTIFICATE_FIELDS));
    spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS] = CURRENT_VERSION; // V1.1
    get_cert_resp_object.version = CURRENT_VERSION;

    spdm_buf_tx->pkt.data[SPDM_HEADER_COMMAND_POS] = SPDM_GET_CERT_RESPONSE;
    get_cert_resp_object.resp_code = SPDM_GET_CERT_RESPONSE;

    if (cert2_base_addr != 0)
    {
        if (slot_buf[requested_slot].is_cert_chain_valid == CERT_CHAIN_INVALID)
        {
            return SPDM_CMD_CHAIN_INVALID;
        }
        // check if chain present
        if (slot_buf[requested_slot].chain_present)
        {
            spdmContext->requested_slot[requested_slot] = 0x01;
            spdmContext->cert_bytes_requested[requested_slot] = ((get_mctp_pld[7U] << 8U) | get_mctp_pld[6]);
            spdmContext->cert_offset_to_read[requested_slot] = ((get_mctp_pld[5U] << 8U) |
                    get_mctp_pld[4]); // attain the value of offset requested
            local_offset_in_cert_req = spdmContext->cert_offset_to_read[requested_slot];

            // check if there are pending bytes to sent from EC for the certificates
            // if no pending certificate chain bytes, proceed for transmission
            // check the required length from host
            if ((spdmContext->cert_bytes_pending_to_sent[requested_slot] == 0))
            {
                // if requested bytes is non zero
                if (spdmContext->cert_bytes_requested[requested_slot] != 0)
                {
                    // check if the total chain length in this slot is within the requested bytes
                    if (spdmContext->cert_bytes_requested[requested_slot] + local_offset_in_cert_req < slot_buf[requested_slot].chain.chain_length)
                    {
                        // PortionLength
                        spdmContext->cert_bytes_to_sent_curr[requested_slot] = spdmContext->cert_bytes_requested[requested_slot];

                        if ((slot_buf[requested_slot].chain.chain_length - spdmContext->cert_bytes_to_sent_curr[requested_slot] - local_offset_in_cert_req) > UINT16_MAX) // Coverity INT31-C Fix
                        {
                            // handle error
                        }
                        else
                        {
                            spdmContext->cert_bytes_pending_to_sent[requested_slot] = slot_buf[requested_slot].chain.chain_length - spdmContext->cert_bytes_to_sent_curr[requested_slot] - local_offset_in_cert_req;
                        }
                    }
                    // check if the requested length is for full chain
                    else if (((spdmContext->cert_bytes_requested[requested_slot] == 0xffff)
                              && (spdmContext->cert_offset_to_read[requested_slot] == 0))
                             || (spdmContext->cert_bytes_requested[requested_slot] == slot_buf[requested_slot].chain.chain_length))
                    {
                        // PortionLength
                        spdmContext->cert_bytes_to_sent_curr[requested_slot] = slot_buf[requested_slot].chain.chain_length;
                        spdmContext->cert_bytes_pending_to_sent[requested_slot] = 0U;
                    }
                    else if (spdmContext->cert_bytes_requested[requested_slot] + local_offset_in_cert_req >= slot_buf[requested_slot].chain.chain_length)
                    {
                        // PortionLength
                        spdmContext->cert_bytes_to_sent_curr[requested_slot] = slot_buf[requested_slot].chain.chain_length - local_offset_in_cert_req;
                        // RemainderLength
                        spdmContext->cert_bytes_pending_to_sent[requested_slot] = 0U;
                    }
                    else
                    {
                        // invalid length
                    }
                }
            }
            // if there are pending bytes from EC side
            else
            {
                // PortionLength
                spdmContext->cert_bytes_to_sent_curr[requested_slot] = spdmContext->cert_bytes_pending_to_sent[requested_slot];
                // RemainderLength
                spdmContext->cert_bytes_pending_to_sent[requested_slot] = 0;
            }

            PortionLength = spdmContext->cert_bytes_to_sent_curr[requested_slot];

            RemainderLength = spdmContext->cert_bytes_pending_to_sent[requested_slot];

            cert_chain_frmt_tbl_sz = PortionLength + 4 + SPDM_SHA384_LEN;

            spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS] = requested_slot;
            get_cert_resp_object.slot_no = requested_slot;
            get_cert_resp_object.reserved = 0x00;
            memset(get_cert_resp_object.reserved1, 0, 2);

            spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 3] = (PortionLength >> 8);
            get_cert_resp_object.portion_len[1] = (PortionLength >> 8);

            spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 2] = (PortionLength & 0xff);
            get_cert_resp_object.portion_len[0] = (PortionLength & 0xff);

            spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 5] = (RemainderLength >> 8);
            get_cert_resp_object.remain_len[1] = (RemainderLength >> 8);

            spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 4] = (RemainderLength & 0xff);
            get_cert_resp_object.remain_len[0] = (RemainderLength & 0xff);

            spdmContext->request_or_response = 1;
            req_and_response_sz[1] = 8U; // Hash size includes version, resp_code, slot_no, reserved, portion_len[2], remain_len[2] totalling 8 bytes
            spdm_get_len_for_runtime_hash(spdmContext);
            spdm_crypto_ops_run_time_hashing((uint8_t *)&get_cert_resp_object, length, spdmContext, &ctx_ptr, spdmContext->get_requests_state);

            // certificate chain format data starts
            memset(spi_data, 0U, SPI_DATA_MAX_BUFF);

            if ((slot_buf[requested_slot].chain.chain_length >> 8U) > UINT8_MAX) // Coverity INT31-C Fix
            {
                // handle error
            }
            else
            {
                spi_data[1] = (slot_buf[requested_slot].chain.chain_length >> 8);
            }
            spi_data[0] = (slot_buf[requested_slot].chain.chain_length & 0xff);
            get_cert_resp_object.len_of_cert_chain[0] = slot_buf[requested_slot].chain.chain_length & 0xff;

            // fill response buffer with root hash
            memcpy(get_cert_resp_object.root_cert_hash, &slot_buf[requested_slot].chain.root_cert_hash[0], SPDM_SHA384_LEN);
            memcpy(&spi_data[4], &slot_buf[requested_slot].chain.root_cert_hash[0], SPDM_SHA384_LEN);

            memcpy(get_cert_response, (uint8_t *)&get_cert_resp_object, sizeof(GET_CERTIFICATE_FIELDS));

            packet_sz = SPDM_GET_CERT_RESPONSE_BYTES_SIZE + cert_chain_frmt_tbl_sz;
            spdmContext->current_resp_cmd = SPDM_GET_CERT_RESPONSE;

            spdmContext->cert_bytes_pending_to_sent[requested_slot] = 0;
        }
        else
        {
            return SPDM_CMD_FAIL;
        }
    }
    else
    {
        return SPDM_CMD_FAIL;
    }
    return SPDM_CMD_PASS;
}

/******************************************************************************/
/** function to reverse array from 0th offset till offset of length len
 * @param array - array to be reversed
 * @param len   - length of array to be reversed
 * @return void
 *******************************************************************************/
void reverse(uint8_t array[], uint8_t len)
{
    uint8_t temp;
    for (int i = 0; i < len / 2; i++)
    {
        temp = array[i];
        array[i] = array[len - i - 1];
        array[len - i - 1] = temp;
    }
}

/******************************************************************************/
/** function to initalize variables for Mutual Authentication
 * @param None
 * @return void
 *******************************************************************************/
void spdm_init_muth_auth_state()
{
    request_op_code = SPDM_GET_DIGEST;
    get_cert_offset = SPDM_MUT_AUTH_GET_CERT_DEFAULT_OFFSET;
    get_cert_length = SPDM_MUT_AUTH_GET_CERTIFICATE_LENGTH;
}

/******************************************************************************/
/** Function to parse KEY_EXCHANGE request and fill part of spdm local buffer based on the request
 * @param spdmContext - SPDM module context
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @return true or false
 *******************************************************************************/
uint8_t spdm_secure_session_key_exchange(MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext)
{
    uint8_t cmd_ret_val = SPDM_CMD_PASS;
    uint8_t ret_val;
    uint8_t size;
    int status;
    uint16_t requester_session_id = 0, responder_session_id = 0;

    if (NULL == spdmContext)
    {
        error_handle_secure_session = SPDM_ERROR_INVLD_RQ;
        return SPDM_CMD_FAIL;
    }

    // processing request message parameters
    requested_slot = (get_mctp_pld[SS_KEY_EXCHANGE_SLOT_REQUEST_OFFSET] &
                      0xff);                          // attain the value of slot requested

    if (!(slot_buf[requested_slot].chain_present))
    {
        error_handle_secure_session = SPDM_ERROR_INVLD_RQ;
        return SPDM_CMD_FAIL;
    }
    // take in hash of certificate chain for transcript hash
    spdmContext->request_or_response = 0; // 0 means request
    req_and_response_sz[0] = SPDM_SHA384_LEN;
    spdmContext->secure_session_get_requests_state = RUN_TIME_HASH_MODE;
    spdm_get_len_for_runtime_hash(spdmContext);

    for (uint8_t i = HASH_CTX_KEY_EXCHANGE; i < HASH_CTX_MAX; i++)
    {
        spdm_crypto_ops_run_time_hashing(&hash_of_chains[requested_slot * SPDM_SHA384_LEN], length, spdmContext, &ctx_ptr_arr_ss[i], spdmContext->secure_session_get_requests_state);
    }

    spdmContext->request_or_response = 0; // 0 means request
    req_and_response_sz[0] = spdmContext->current_request_length;;
    spdmContext->secure_session_get_requests_state = RUN_TIME_HASH_MODE;
    spdm_get_len_for_runtime_hash(spdmContext);

    for (uint8_t i = HASH_CTX_KEY_EXCHANGE; i < HASH_CTX_MAX; i++)
    {
        spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], length, spdmContext, &ctx_ptr_arr_ss[i], spdmContext->secure_session_get_requests_state);
    }

    measurement_var.msr_hash_rqstd = (get_mctp_pld[SS_KEY_EXCHANGE_MSR_OFFSET] &
                                      0xff); // whether measurement hash requested

    memcpy(&requester_session_id, &(get_mctp_pld[SS_KEY_EXCHANGE_REQ_SESSION_ID]), 2);                          // get the requester session id value

    memcpy(&requester_public_key[1], &get_mctp_pld[SS_KEY_EXCHANGE_REQUESTER_PUB_KEY], 96);//DHE_384_PUB_KEY_SIZE);
    requester_public_key[0] = 0x04;
    // filling of response message
    spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS] = CURRENT_VERSION; // V1.1
    key_exchange_resp_obj.version = CURRENT_VERSION;

    spdm_buf_tx->pkt.data[SPDM_HEADER_COMMAND_POS] = SPDM_KEY_EXCHANGE_RSP;
    key_exchange_resp_obj.resp_code = SPDM_KEY_EXCHANGE_RSP;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS] = HEARTBEAT_PERIOD;
    key_exchange_resp_obj.param1 = HEARTBEAT_PERIOD; //heartbeat period of 1sec

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 1] = 0;
    key_exchange_resp_obj.param2 = 0;

    // random value 2bytes for responder's session id
    ret_val = spdm_crypto_ops_gen_random_no((uint8_t *)&responder_session_id, RESP_SESSION_ID_RANDOM_NO_SIZE);
    if (ret_val != MCHP_OK)
    {
        memset(&responder_session_id, 0x00, RESP_SESSION_ID_RANDOM_NO_SIZE);
    }
    key_exchange_resp_obj.rsp_session_id = responder_session_id;
    memcpy(&spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 2], &responder_session_id, RESP_SESSION_ID_RANDOM_NO_SIZE);
    final_session_id = spdm_derive_session_id(requester_session_id, responder_session_id);

    key_exchange_resp_obj.muth_auth_requested = KEY_EXCHANGE_MUTHAUTHREQUESTED;
    spdm_init_muth_auth_state();
    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 4] = KEY_EXCHANGE_MUTHAUTHREQUESTED;

    key_exchange_resp_obj.slot_id = 0; //slot ID to be followed in encapsulated request message as part of mutual authentication
    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 5] = 0;

    spdmContext->current_resp_cmd = SPDM_KEY_EXCHANGE_RSP;

    // random value 32bytes 
    ret_val = spdm_crypto_ops_gen_random_no(&nonce_data[0], NOUNCE_DATA_SIZE);

    if (ret_val != MCHP_OK)
    {
        memset(&nonce_data[0], 0x00, NOUNCE_DATA_SIZE);
    }

    memcpy(&key_exchange_resp_obj.random_val[0], &nonce_data[0], NOUNCE_DATA_SIZE);
    memcpy(&spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 6], &nonce_data[0], NOUNCE_DATA_SIZE);

    spdm_crypto_ops_generate_dhe_key_pair(&ss_pub_key[0], &ss_pvt_key[0], final_session_id);
    memcpy(&key_exchange_resp_obj.exchange_data[0], &ss_pub_key[0], PUB_KEY_CODE_LENGTH);

    spdm_crypto_ops_generate_dhe_secret(&requester_public_key[0], &ss_pvt_key[0], &spdmContext->secure_session_info.secure_session_master_secret.dhe_secret[0]);

    // add key exchange response for transcript hash till exchange data
    spdmContext->request_or_response = 1; // 0 means request
    req_and_response_sz[1] = sizeof(key_exchange_resp_obj);
    spdmContext->secure_session_get_requests_state = RUN_TIME_HASH_MODE;
    spdm_get_len_for_runtime_hash(spdmContext);

    for (uint8_t i = HASH_CTX_KEY_EXCHANGE; i < HASH_CTX_MAX; i++)
    {
        spdm_crypto_ops_run_time_hashing(&key_exchange_resp_obj.version, length, spdmContext, &ctx_ptr_arr_ss[i], spdmContext->secure_session_get_requests_state);
    }

    if (measurement_var.msr_hash_rqstd)
    {
        req_and_response_sz[1] = SPDM_MEASUREMENT_HASH_SUMMARY_SZ;
        if (measurement_var.msr_hash_rqstd == 0xff)
        {
            spdmContext->request_or_response = 1; // 0 means request
            spdmContext->secure_session_get_requests_state = RUN_TIME_HASH_MODE;
            spdm_get_len_for_runtime_hash(spdmContext);

            for (uint8_t i = HASH_CTX_KEY_EXCHANGE; i < HASH_CTX_MAX; i++)
            {
                spdm_crypto_ops_run_time_hashing(&hash_concat_msrments_val[0], length, spdmContext, &ctx_ptr_arr_ss[i], spdmContext->secure_session_get_requests_state);
            }
        }
        else
        {
            spdmContext->request_or_response = 1; // 1 means response
            spdmContext->secure_session_get_requests_state = RUN_TIME_HASH_MODE;
            spdm_get_len_for_runtime_hash(spdmContext);

            for (uint8_t i = HASH_CTX_KEY_EXCHANGE; i < HASH_CTX_MAX; i++)
            {
                spdm_crypto_ops_run_time_hashing(&hash_concat_tcb_msrments_val[0], length, spdmContext, &ctx_ptr_arr_ss[i], spdmContext->secure_session_get_requests_state);
            }
        }
        packet_sz = SPDM_KEY_EXCHANGE_RESPONSE_SIZE + SPDM_MEASUREMENT_HASH_SUMMARY_SZ;
    } else {
        packet_sz = SPDM_KEY_EXCHANGE_RESPONSE_SIZE;
    }

    // opaque data length
    spdmContext->request_or_response = 1; // 0 means request
    req_and_response_sz[1] = 2;
    spdmContext->secure_session_get_requests_state = RUN_TIME_HASH_MODE;
    spdm_get_len_for_runtime_hash(spdmContext);

    for (uint8_t i = HASH_CTX_KEY_EXCHANGE; i < HASH_CTX_MAX; i++)
    {
        spdm_crypto_ops_run_time_hashing((uint8_t *)&opaque_data.opaque_len, length, spdmContext, &ctx_ptr_arr_ss[i], spdmContext->secure_session_get_requests_state);
    }

    // opaque data  
    spdmContext->request_or_response = 1; // 0 means request
    req_and_response_sz[1] = sizeof(opaque_data) - 2;
    spdmContext->secure_session_get_requests_state = RUN_TIME_HASH_MODE;
    spdm_get_len_for_runtime_hash(spdmContext);

    for (uint8_t i = HASH_CTX_KEY_EXCHANGE; i < HASH_CTX_MAX; i++)
    {
        spdm_crypto_ops_run_time_hashing((uint8_t *)&opaque_data.spec_id, length, spdmContext, &ctx_ptr_arr_ss[i], spdmContext->secure_session_get_requests_state);
    }

    // get the intermediate hashing till opaque data for signature cal
    spdmContext->secure_session_get_requests_state = END_OF_HASH;
    spdm_crypto_ops_run_time_hashing(NULL, 0, spdmContext, &ctx_ptr_arr_ss[HASH_CTX_KEY_EXCHANGE], spdmContext->secure_session_get_requests_state);
    memcpy(&th_hash[0], &spdmContext->sha_digest[0], SPDM_SHA384_LEN); // this will be TH hash

    // generate signature for the response using AK pvt key
    memcpy(&hash_of_req_buffer[0], &spdmContext->sha_digest[0], SPDM_SHA384_LEN);
    spdm_crypto_ops_gen_signature(); 

    spdmContext->request_or_response = 1; // 0 means request
    req_and_response_sz[1] = 96;
    spdmContext->secure_session_get_requests_state = RUN_TIME_HASH_MODE;
    spdm_get_len_for_runtime_hash(spdmContext);

    for (uint8_t i = HASH_CTX_TH1_DATA; i < HASH_CTX_MAX; i++)
    {
        spdm_crypto_ops_run_time_hashing((uint8_t *)&ecdsa_signature.ecdsa_signature[0], length, spdmContext, &ctx_ptr_arr_ss[i], spdmContext->secure_session_get_requests_state);
    }

    // get the intermediate hashing till signature for TH1 hashing
    spdmContext->secure_session_get_requests_state = END_OF_HASH;
    spdm_crypto_ops_run_time_hashing(NULL, 0, spdmContext, &ctx_ptr_arr_ss[HASH_CTX_TH1_DATA], spdmContext->secure_session_get_requests_state);
    memcpy(&th_hash[0], &spdmContext->sha_digest, SPDM_SHA384_LEN); //this will be TH1 hash

    if (spdm_generate_session_handshake_key(spdmContext)) {
        error_handle_secure_session = SPDM_ERROR_UNSPECIFIED;
        return SPDM_CMD_FAIL;
    }
 
    spdmContext->request_or_response = 1; // 0 means request
    req_and_response_sz[1] = SHA_384_LEN;
    spdmContext->secure_session_get_requests_state = RUN_TIME_HASH_MODE;
    spdm_get_len_for_runtime_hash(spdmContext);

    for (uint8_t i = HASH_CTX_FINISH; i < HASH_CTX_MAX; i++)
    {
        spdm_crypto_ops_run_time_hashing((uint8_t *)&responder_verify_data[0], length, spdmContext, &ctx_ptr_arr_ss[i], spdmContext->secure_session_get_requests_state);
    }

    spdm_secure_session_set_session_state(spdmContext, SPDM_SESSION_STATE_HANDSHAKING);

    return cmd_ret_val;
}

/******************************************************************************/
/** Function to generate session data key
 * @param spdmContext - SPDM module context
 * @return true or false
 *******************************************************************************/
int8_t spdm_generate_session_data_key(SPDM_CONTEXT *spdmContext)
{
    int8_t status;
    uint8_t hash_size;

    hash_size = SPDM_SHA384_LEN;

    // create bin string0
    bin_str_size = sizeof(bin_str);
    spdm_bin_concat(CURRENT_VERSION << SPDM_VERSION_NUMBER_SHIFT_BIT,
                       SPDM_BIN_STR_0_LABEL,
                       sizeof(SPDM_BIN_STR_0_LABEL) - 1, NULL,
                       SPDM_SHA384_LEN, SPDM_SHA384_LEN, bin_str,
                       &bin_str_size);
    // create salt from hanshake secret
    memset(&salt[0], 0, sizeof(salt));
    status = spdm_crypto_ops_hkdf(
        SPDM_CRYPTO_HKDF_EXPAND, SHA_MODE_384, bin_str, bin_str_size, 
        &(spdmContext->secure_session_info.secure_session_master_secret.handshake_secret[0]),
        SPDM_SHA384_LEN, &salt[0], SPDM_SHA384_LEN);
    memset(bin_str, 0 , sizeof(bin_str));
    if (status)
    {
        return status;
    }
    // create 0_filled from salt
    memset(zero_filled_buffer, 0, sizeof(zero_filled_buffer));
    status = spdm_crypto_ops_hkdf(
        SPDM_CRYPTO_HKDF_EXTRACT, SHA_MODE_384, salt, hash_size,
        zero_filled_buffer, hash_size, 
        &spdmContext->secure_session_info.secure_session_master_secret.master_secret[0], hash_size);
    if (status)
    {
        return status;
    }
    // create bin str3
    bin_str_size = sizeof(bin_str);
    spdm_bin_concat(CURRENT_VERSION << SPDM_VERSION_NUMBER_SHIFT_BIT,
                       SPDM_BIN_STR_3_LABEL, sizeof(SPDM_BIN_STR_3_LABEL) - 1,
                       th_hash, (uint16_t)hash_size, hash_size,
                       bin_str, &bin_str_size);
    // create request direction data secret
    status = spdm_crypto_ops_hkdf(
        SPDM_CRYPTO_HKDF_EXPAND, SHA_MODE_384, bin_str, bin_str_size,
        &spdmContext->secure_session_info.secure_session_master_secret.master_secret[0],
        hash_size, 
        &spdmContext->secure_session_info.secure_session_application_secret.request_data_secret[0],
        hash_size);
    memset(bin_str, 0 , sizeof(bin_str));
    if (status)
    {
        return status;
    }
    // create bin str4
    bin_str_size = sizeof(bin_str);
    spdm_bin_concat(CURRENT_VERSION << SPDM_VERSION_NUMBER_SHIFT_BIT,
                       SPDM_BIN_STR_4_LABEL, sizeof(SPDM_BIN_STR_4_LABEL) - 1,
                       th_hash, (uint16_t)hash_size, hash_size,
                       bin_str, &bin_str_size);

    // create response direction data secret
    status = spdm_crypto_ops_hkdf(
        SPDM_CRYPTO_HKDF_EXPAND, SHA_MODE_384, bin_str, bin_str_size,
        &spdmContext->secure_session_info.secure_session_master_secret.master_secret[0],
        hash_size, 
        &spdmContext->secure_session_info.secure_session_application_secret.response_data_secret[0],
        hash_size);
    memset(bin_str, 0 , sizeof(bin_str));
    if (status)
    {
        return status;
    }
    status = spdm_generate_aead_key_and_iv(
        spdmContext,
        &(spdmContext->secure_session_info.secure_session_application_secret.request_data_secret[0]),
        &(spdmContext->secure_session_info.secure_session_application_secret.request_data_encryption_key[0]),
        &(spdmContext->secure_session_info.secure_session_application_secret.request_data_salt[0]));
    if (status)
    {
        return status;
    }
    spdmContext->secure_session_info.secure_session_application_secret.request_data_sequence_number = 0;

    status = spdm_generate_aead_key_and_iv(
        spdmContext,
        &(spdmContext->secure_session_info.secure_session_application_secret.response_data_secret[0]),
        &(spdmContext->secure_session_info.secure_session_application_secret.response_data_encryption_key[0]),
        &(spdmContext->secure_session_info.secure_session_application_secret.response_data_salt[0]));

    spdmContext->secure_session_info.secure_session_application_secret.response_data_sequence_number = 0;

    return status;
}

/******************************************************************************/
/** Function to generate session handshake key
 * @param spdmContext - SPDM module context
 * @return true or false
 *******************************************************************************/
int8_t spdm_generate_session_handshake_key(SPDM_CONTEXT *spdmContext)
{
    int8_t status;

    // create the handshake secret
    memset(salt, 0, sizeof(salt));
    status = spdm_crypto_ops_hkdf(SPDM_CRYPTO_HKDF_EXTRACT, SHA_MODE_384, &salt[0], SPDM_SHA384_LEN, 
                &(spdmContext->secure_session_info.secure_session_master_secret.dhe_secret[0]),
                MAX_DHE_KEY_SIZE,
                &(spdmContext->secure_session_info.secure_session_master_secret.handshake_secret[0]),
                SPDM_SHA384_LEN);

    if (status)
    {
        return status;
    }
    // create bin1 string
    bin_str_size = sizeof(bin_str);
    spdm_bin_concat(CURRENT_VERSION << SPDM_VERSION_NUMBER_SHIFT_BIT,
                       SPDM_BIN_STR_1_LABEL, sizeof(SPDM_BIN_STR_1_LABEL) - 1,
                       &th_hash[0], SPDM_SHA384_LEN, SPDM_SHA384_LEN,
                       bin_str, &bin_str_size);

    // create request direction handshake secret
    status = spdm_crypto_ops_hkdf(SPDM_CRYPTO_HKDF_EXPAND, SHA_MODE_384, bin_str, bin_str_size,
            &(spdmContext->secure_session_info.secure_session_master_secret.handshake_secret[0]),
            SPDM_SHA384_LEN, 
            &(spdmContext->secure_session_info.secure_session_handshake_secret.request_handshake_secret[0]),
                            SPDM_SHA384_LEN);

    memset(bin_str, 0 , sizeof(bin_str));

    if (status)
    {
        return status;
    }

    // create bin2 string
    bin_str_size = sizeof(bin_str);
    spdm_bin_concat(CURRENT_VERSION << SPDM_VERSION_NUMBER_SHIFT_BIT,
                       SPDM_BIN_STR_2_LABEL, sizeof(SPDM_BIN_STR_2_LABEL) - 1,
                       &th_hash[0], SPDM_SHA384_LEN, SPDM_SHA384_LEN,
                       bin_str, &bin_str_size);

    // create response direction handshake secret
    status = spdm_crypto_ops_hkdf(SPDM_CRYPTO_HKDF_EXPAND, SHA_MODE_384, bin_str, bin_str_size,
        &(spdmContext->secure_session_info.secure_session_master_secret.handshake_secret[0]),
        SPDM_SHA384_LEN, 
        &(spdmContext->secure_session_info.secure_session_handshake_secret.response_handshake_secret[0]),
        SPDM_SHA384_LEN);

    memset(bin_str, 0 , sizeof(bin_str));

    if (status)
    {
        return status;
    }
    // create request direction finished key
    status = spdm_generate_finished_key(spdmContext,
                    &(spdmContext->secure_session_info.secure_session_handshake_secret.request_handshake_secret[0]),
                    &(spdmContext->secure_session_info.secure_session_handshake_secret.request_finished_key[0]));
    if (status)
    {
        return status;
    }
    // create response direction finished key
    status = spdm_generate_finished_key(spdmContext,
                    &(spdmContext->secure_session_info.secure_session_handshake_secret.response_handshake_secret[0]),
                    &(spdmContext->secure_session_info.secure_session_handshake_secret.response_finished_key[0]));
    if (status)
    {
        return status;
    }
    // generate AEAD and IV request direction
    status = spdm_generate_aead_key_and_iv(spdmContext,
                                           &(spdmContext->secure_session_info.secure_session_handshake_secret.request_handshake_secret[0]),
                                           &(spdmContext->secure_session_info.secure_session_handshake_secret.request_handshake_encryption_key[0]),
                                           &(spdmContext->secure_session_info.secure_session_handshake_secret.request_handshake_salt[0]));
    if (status)
    {
        return status;
    }
    spdmContext->secure_session_info.secure_session_handshake_secret.request_handshake_sequence_number = 0;

    // generate AEAD and IV response direction
    status = spdm_generate_aead_key_and_iv(spdmContext,
                                           &(spdmContext->secure_session_info.secure_session_handshake_secret.response_handshake_secret[0]),
                                           &(spdmContext->secure_session_info.secure_session_handshake_secret.response_handshake_encryption_key[0]),
                                           &(spdmContext->secure_session_info.secure_session_handshake_secret.response_handshake_salt[0]));
    if (status)
    {
        return status;
    }
    spdmContext->secure_session_info.secure_session_handshake_secret.response_handshake_sequence_number = 0;

    status = spdm_crypto_ops_hkdf(SPDM_CRYPTO_HKDF_EXTRACT, SHA_MODE_384, &th_hash[0], SPDM_SHA384_LEN, 
                &(spdmContext->secure_session_info.secure_session_handshake_secret.response_finished_key[0]),
                MAX_DHE_KEY_SIZE,
                &(responder_verify_data[0]),
                SPDM_SHA384_LEN);

    return status;
}

/******************************************************************************/
/** Function to derive AEAD and IV for secure session
 * @param spdmContext - spdmContext pointer
 * @param major_secret  - pointer to major secret used for encryption/decryption
 * @param key - derived key pointer
 * @param iv - derived key pointer
 * @return true/false
 *******************************************************************************/
int8_t spdm_generate_aead_key_and_iv(SPDM_CONTEXT *spdmContext, uint8_t *major_secret, uint8_t *key, uint8_t *iv)
{
    int8_t status;

    // create bin str5 string
    bin_str_size = sizeof(bin_str);
    spdm_bin_concat(CURRENT_VERSION << SPDM_VERSION_NUMBER_SHIFT_BIT,
                       SPDM_BIN_STR_5_LABEL, sizeof(SPDM_BIN_STR_5_LABEL) - 1,
                       NULL, MAX_AEAD_KEY_SIZE, MAX_HASH_SIZE, bin_str,
                       &bin_str_size);

    // generate encryption key
    status = spdm_crypto_ops_hkdf(SPDM_CRYPTO_HKDF_EXPAND, SHA_MODE_384, bin_str, bin_str_size,
                                 major_secret, MAX_HASH_SIZE,
                                 key, MAX_AEAD_KEY_SIZE);
    memset(bin_str, 0 , sizeof(bin_str));
    if (status)
    {
        return status;
    }
    bin_str_size = sizeof(bin_str);
    spdm_bin_concat(CURRENT_VERSION << SPDM_VERSION_NUMBER_SHIFT_BIT,
                       SPDM_BIN_STR_6_LABEL, sizeof(SPDM_BIN_STR_6_LABEL) - 1,
                       NULL, MAX_AEAD_IV_SIZE, MAX_HASH_SIZE, bin_str,
                       &bin_str_size);

    // generate iv
    status = spdm_crypto_ops_hkdf(SPDM_CRYPTO_HKDF_EXPAND, SHA_MODE_384, bin_str, bin_str_size,
                                 major_secret, MAX_HASH_SIZE, 
                                 iv, MAX_AEAD_IV_SIZE);
    memset(bin_str, 0 , sizeof(bin_str));

    return status;
}

/******************************************************************************/
/** Function to generate finished key
 * @param spdmContext - spdmContext pointer
 * @param handshake_secret  - pointer to handshake secret used for encryption/decryption
 * @param finished_key - derived finished key pointer
 * @return true/false
 *******************************************************************************/
int8_t spdm_generate_finished_key(SPDM_CONTEXT *spdmContext, uint8_t *handshake_secret, uint8_t *finished_key)
{
    uint8_t status;

    bin_str_size = sizeof(bin_str);
    spdm_bin_concat(CURRENT_VERSION << SPDM_VERSION_NUMBER_SHIFT_BIT,
                       SPDM_BIN_STR_7_LABEL, sizeof(SPDM_BIN_STR_7_LABEL) - 1,
                       NULL, SPDM_SHA384_LEN, SPDM_SHA384_LEN, 
                       bin_str, &bin_str_size);

    status = spdm_crypto_ops_hkdf(SPDM_CRYPTO_HKDF_EXPAND, SHA_MODE_384, bin_str,
                                 bin_str_size,
                                 handshake_secret, SPDM_SHA384_LEN,  finished_key, SPDM_SHA384_LEN);
    memset(bin_str, 0 , sizeof(bin_str));

    return status;  
}

/******************************************************************************/
/** Function to derive final session id
 * @param req_session_id - Session ID of requester (host)
 * @param resp_session_id  - Session ID of responder
 * @return session_id - final session ID - concatenation of req's and resp's
 *******************************************************************************/
uint32_t spdm_derive_session_id(uint16_t req_session_id, uint16_t resp_session_id)
{
    uint32_t session_id;
    session_id = (uint32_t)(resp_session_id << BIT_16) | req_session_id;
    return session_id;
}

/******************************************************************************/
/** Function to parse END_SESSION request and fill part of spdm local buffer based on the request
 * @param spdmContext - SPDM module context
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @return void
 *******************************************************************************/
uint8_t spdm_secure_session_end_session(MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext)
{
    if (NULL == spdmContext)
    {
        error_handle_secure_session = SPDM_ERROR_UNSPECIFIED;
        return SPDM_CMD_FAIL;
    }

    // filling of response message
    spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS] = CURRENT_VERSION; // V1.1

    spdm_buf_tx->pkt.data[SPDM_HEADER_COMMAND_POS] = SPDM_END_SESSION_RSP;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS] = 0;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 1] = 0;

    spdm_buf_tx->pkt.field.hdr.byte_cnt = BYTE_CNT_MCTP_HEADER + BYTE_CNT_END_SESION_RESP_FIELDS;
    spdmContext->current_resp_cmd = SPDM_END_SESSION_RSP;

    if (message_type == MCTP_IC_MSGTYPE_SECURED_MESSAGE) {
        uint16_t plaintext_size = 0;
        plaintext_size = SPDM_END_SESSION_RESPONSE_BYTES; // encrypted app data (app data is from spdm msg type, so increase the packet_sz + 1), application data len is added as part of encrypt API
        spdm_buf_tx = (MCTP_PKT_BUF *)&spdm_pktbuf[0];
        offset_for_ss_get_msr = 3; // leave application length of size 2bytes and message type of size 1byte, taken from offset of SPDM header version
        spi_data[2] = MCTP_MSGTYPE_SPDM; // plain text data - application len of 2bytes, msg type 1 byte, followed by the application data
        data_size_for_ss_get_msr = 0;
        state_for_msr_ss = SOM;
        offset_for_msr_ss = 0;
        memcpy(&spi_data[offset_for_ss_get_msr], &spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS], 4);
        spdmContext->current_resp_cmd = SPDM_END_SESSION_RSP;
        if (spdm_pkt_process_encrypt_spdm_data(spdmContext, false, plaintext_size, spi_data, buf_for_encryption))
        {
            error_handle_secure_session = SPDM_ERROR_UNSPECIFIED;
            return FALSE;
        }
        remaining_for_ss = total_secured_message_size;
        if (safe_add(total_secured_message_size, APP_DATA_SIZE_ADD_MSG_TYPE_SIZE + MCTP_TOT_HDR_SZ, &packet_sz))
        {
            error_handle_secure_session = SPDM_ERROR_UNSPECIFIED;
            return FALSE;
        }    
    } else {
        packet_sz = SPDM_END_SESSION_SIZE;
    }

    spdm_secure_session_set_session_state(spdmContext, SPDM_SESSION_STATE_NOT_STARTED);

    if (SPDMHB_timer_started) 
    {
        spdm_hb_response_timeout_stop();
        SPDMHB_timer_started = false;
    }

    spdm_secure_session_cleanup(spdmContext);

    return TRUE;
}

/******************************************************************************/
/** Function to parse KEY_EXCHANGE request and fill part of spdm local buffer based on the request
 * @param spdmContext - SPDM module context
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @return void
 *******************************************************************************/
uint8_t spdm_secure_session_heartbeat(MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext)
{
    if (NULL == spdmContext)
    {
        error_handle_secure_session = SPDM_ERROR_UNSPECIFIED;
        return SPDM_CMD_FAIL;
    } 

    if (SPDMHB_timer_started)
    {
        SPDMHB_timer_started = false;
        spdm_hb_response_timeout_stop();
        spdm_hb_response_timeout_start();
        SPDMHB_timer_started = true;
    }

    // filling of response message
    spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS] = CURRENT_VERSION; // V1.1

    spdm_buf_tx->pkt.data[SPDM_HEADER_COMMAND_POS] = SPDM_HEARTBEAT_ACK;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS] = 0;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 1] = 0;

    spdm_buf_tx->pkt.field.hdr.byte_cnt = BYTE_CNT_MCTP_HEADER + BYTE_CNT_HEARTBEAT_RESP_FIELDS;

    spdmContext->current_resp_cmd = SPDM_HEARTBEAT_ACK;

    if (message_type == MCTP_IC_MSGTYPE_SECURED_MESSAGE) {
        uint16_t plaintext_size = 0;
        plaintext_size = SPDM_HEART_BEAT_RESPONSE_BYTES; // encrypted app data (app data is from spdm msg type, so increase the packet_sz + 1), application data len is added as part of encrypt API
        spdm_buf_tx = (MCTP_PKT_BUF *)&spdm_pktbuf[0];
        offset_for_ss_get_msr = 3; // leave application length of size 2bytes and message type of size 1byte, taken from offset of SPDM header version
        spi_data[2] = MCTP_MSGTYPE_SPDM; // plain text data - application len of 2bytes, msg type 1 byte, followed by the application data
        data_size_for_ss_get_msr = 0;
        state_for_msr_ss = SOM;
        offset_for_msr_ss = 0;
        memcpy(&spi_data[offset_for_ss_get_msr], &spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS], 4);
        spdmContext->current_resp_cmd = SPDM_HEARTBEAT_ACK;
        if(spdm_pkt_process_encrypt_spdm_data(spdmContext, false, plaintext_size, spi_data, buf_for_encryption))
        {
            error_handle_secure_session = SPDM_ERROR_UNSPECIFIED;
            return FALSE;
        }
        remaining_for_ss = total_secured_message_size;
        if (safe_add(total_secured_message_size, APP_DATA_SIZE_ADD_MSG_TYPE_SIZE + MCTP_TOT_HDR_SZ, &packet_sz))
        {
            return FALSE;
        }
    } else {
        packet_sz = SPDM_HEART_BEAT_RESPONSE_SIZE;
    }

    return TRUE;
}

/******************************************************************************/
/** Function to parse KEY_EXCHANGE request and fill part of spdm local buffer based on the request
 * @param spdmContext - SPDM module context
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @return void
 *******************************************************************************/
uint8_t spdm_secure_session_finish(MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext)
{
    uint8_t slot_id = 0, sign_incl_muth_auth = 0;
    uint8_t transcript_hash[SPDM_SHA384_LEN];
    uint8_t sign_from_request[96];
    uint8_t ret_val = 0;

    if (NULL == spdmContext)
    {
        error_handle_secure_session = SPDM_ERROR_UNSPECIFIED;
        return SPDM_CMD_FAIL;
    }

    // processing request message parameters
    sign_incl_muth_auth = (get_mctp_pld[SS_FINISH_SIGNED_MUTH_AUTH] &
                      0xff);

    if (!sign_incl_muth_auth)
    {
        error_handle_secure_session = SPDM_ERROR_INVLD_RQ;
        return SPDM_CMD_FAIL;       
    }

    slot_id = (get_mctp_pld[SS_FINISH_SLOTID] &
                      0xff);

    if (slot_id != requester_cert_chain_slot)
    {
        error_handle_secure_session = SPDM_ERROR_INVLD_RQ;
        return SPDM_CMD_FAIL;
    }

    memcpy(&sign_from_request[0], &get_mctp_pld[SS_FINISH_SIGNATURE_OFFSET], SIGNATURE_SIZE);

    memcpy(&transcript_hash, &get_mctp_pld[SS_FINISH_REQ_VERIFY_DATA], SPDM_SHA384_LEN);

    // take in hash of certificate chain for transcript hash
    spdmContext->request_or_response = 0; // 0 means request
    req_and_response_sz[0] = SPDM_SHA384_LEN;
    spdmContext->secure_session_get_requests_state = RUN_TIME_HASH_MODE;
    spdm_get_len_for_runtime_hash(spdmContext);

    for (uint8_t i = HASH_CTX_FINISH; i < HASH_CTX_MAX; i++)
    {
        spdm_crypto_ops_run_time_hashing(&hash_of_req_chains[0], length, spdmContext, &ctx_ptr_arr_ss[i], spdmContext->secure_session_get_requests_state);
    }

    // take in hash of finish request header fields for transcript hash
    spdmContext->request_or_response = 0; // 0 means request
    req_and_response_sz[0] = SPDM_FINISH_REQUEST_SIZE; // size of finish request header fields
    spdmContext->secure_session_get_requests_state = RUN_TIME_HASH_MODE;
    spdm_get_len_for_runtime_hash(spdmContext);

    for (uint8_t i = HASH_CTX_FINISH; i < HASH_CTX_MAX; i++)
    {
        spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], length, spdmContext, &ctx_ptr_arr_ss[i], spdmContext->secure_session_get_requests_state);
    }

    // take signature out and verify it with the public key derived earlier
    spdmContext->secure_session_get_requests_state = END_OF_HASH;
    spdm_crypto_ops_run_time_hashing(NULL, 0, spdmContext, &ctx_ptr_arr_ss[HASH_CTX_FINISH], spdmContext->secure_session_get_requests_state);
    // verify signature of the request using requester's leaf pub key
    ret_val = spdm_di_crypto_send_ecdsa_sign_verify_request(&requester_public_key[0], 
                &sign_from_request[0], &spdmContext->sha_digest[0], SHA_384_LEN, SB_AUTHALGO_ECDSA_P384, false);

    if (ret_val) 
    {
        error_handle_secure_session = SPDM_ERROR_DECRYPT_ERROR;
        return SPDM_CMD_FAIL;
    }

    spdmContext->request_or_response = 0; // 0 means request
    req_and_response_sz[0] = SS_ECDSA_384_SIGN_SIZE; 
    spdmContext->secure_session_get_requests_state = RUN_TIME_HASH_MODE;
    spdm_get_len_for_runtime_hash(spdmContext);

    for (uint8_t i = HASH_CTX_FINISH_VERIFY_DATA; i < HASH_CTX_MAX; i++)
    {
        spdm_crypto_ops_run_time_hashing(&sign_from_request[0], length, spdmContext, &ctx_ptr_arr_ss[i], spdmContext->secure_session_get_requests_state);
    }

    spdmContext->secure_session_get_requests_state = END_OF_HASH;
    spdm_crypto_ops_run_time_hashing(NULL, 0, spdmContext, &ctx_ptr_arr_ss[HASH_CTX_FINISH_VERIFY_DATA], spdmContext->secure_session_get_requests_state);

    // take requesterverifydata out and verify it
    spdm_crypto_ops_run_time_hmac(spdmContext, &spdmContext->sha_digest[0],
                    &(spdmContext->secure_session_info.secure_session_handshake_secret.request_finished_key[0])); // fill responder verify data

    if ((memcmp(&responder_verify_data[0], &transcript_hash[0], SPDM_SHA384_LEN))) // responder_verify_data - from crypto, transcript from request message
    {
        error_handle_secure_session = SPDM_ERROR_DECRYPT_ERROR;
        return SPDM_CMD_FAIL;
    }

    spdmContext->request_or_response = 0; // 0 means request
    req_and_response_sz[0] = SHA_384_LEN; // size of finish request header fields
    spdmContext->secure_session_get_requests_state = RUN_TIME_HASH_MODE;
    spdm_get_len_for_runtime_hash(spdmContext);

    for (uint8_t i = HASH_CTX_FINISH_RSP_VERIFY_DATA; i < HASH_CTX_MAX; i++)
    {
        spdm_crypto_ops_run_time_hashing(&transcript_hash[0], length, spdmContext, &ctx_ptr_arr_ss[i], spdmContext->secure_session_get_requests_state);
    }

    // filling of response message
    spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS] = CURRENT_VERSION; // V1.1
    finish_rsp_fields.version = CURRENT_VERSION;

    spdm_buf_tx->pkt.data[SPDM_HEADER_COMMAND_POS] = SPDM_FINISH_RSP;
    finish_rsp_fields.resp_code = SPDM_FINISH_RSP;

    finish_rsp_fields.param1 = 0;
    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS] = finish_rsp_fields.param1;
    finish_rsp_fields.param2 = 0;
    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 1] = finish_rsp_fields.param2;
    spdmContext->current_resp_cmd = SPDM_FINISH_RSP;

    // take in hash of finish response for transcript hash
    spdmContext->request_or_response = 1; // 0 means request
    req_and_response_sz[1] = sizeof(FINISH_RSP_FIELDS) - sizeof (finish_rsp_fields.res_verify_data);
    spdmContext->secure_session_get_requests_state = RUN_TIME_HASH_MODE;
    spdm_get_len_for_runtime_hash(spdmContext);

    for (uint8_t i = HASH_CTX_FINISH_RSP_VERIFY_DATA; i < HASH_CTX_MAX; i++)
    {
        spdm_crypto_ops_run_time_hashing(&finish_rsp_fields.version, length, spdmContext, &ctx_ptr_arr_ss[i], spdmContext->secure_session_get_requests_state);
    }

    // get the intermediate hashing till signature for transcript hashing required for responderVerifyData
    spdmContext->secure_session_get_requests_state = END_OF_HASH;
    spdm_crypto_ops_run_time_hashing(NULL, 0, spdmContext, &ctx_ptr_arr_ss[HASH_CTX_FINISH_RSP_VERIFY_DATA], spdmContext->secure_session_get_requests_state);
    memcpy(&transcript_hash[0], &spdmContext->sha_digest[0], SPDM_SHA384_LEN); 

    spdmContext->hmac_state = HASH_INIT_MODE;
    spdm_crypto_ops_run_time_hmac(spdmContext, transcript_hash,
                    &(spdmContext->secure_session_info.secure_session_handshake_secret.response_finished_key[0])); // fill responder verify data

    memcpy(&finish_rsp_fields.res_verify_data, responder_verify_data, SPDM_SHA384_LEN);

    spdmContext->request_or_response = 1; // 0 means request
    req_and_response_sz[1] = sizeof(finish_rsp_fields.res_verify_data);
    spdmContext->secure_session_get_requests_state = RUN_TIME_HASH_MODE;
    spdm_get_len_for_runtime_hash(spdmContext);
    spdm_crypto_ops_run_time_hashing(&finish_rsp_fields.res_verify_data[0], length, spdmContext, &ctx_ptr_arr_ss[HASH_CTX_TH2_DATA], spdmContext->secure_session_get_requests_state);

    spdmContext->secure_session_get_requests_state = END_OF_HASH;
    spdm_crypto_ops_run_time_hashing(NULL, 0, spdmContext, &ctx_ptr_arr_ss[HASH_CTX_TH2_DATA], spdmContext->secure_session_get_requests_state);
    memcpy(&th_hash[0], &spdmContext->sha_digest[0], SPDM_SHA384_LEN); 

    if (spdm_generate_session_data_key(spdmContext))
    {
        error_handle_secure_session = SPDM_ERROR_UNSPECIFIED;
        return SPDM_CMD_FAIL;
    }

    memcpy(&spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 2], &finish_rsp_fields.res_verify_data[0], 48);

    if (message_type == MCTP_IC_MSGTYPE_SECURED_MESSAGE) {
        uint16_t plaintext_size = 0;
        plaintext_size = SPDM_FINISH_RESPONSE_BYTES; // encrypted app data (app data is from spdm msg type, so increase the packet_sz + 1), application data len is added as part of encrypt API
        offset_for_ss_get_msr = 3; // leave application length of size 2bytes and message type of size 1byte, taken from offset of SPDM header version
        spi_data[2] = MCTP_MSGTYPE_SPDM; // plain text data - application len of 2bytes, msg type 1 byte, followed by the application data
        data_size_for_ss_get_msr = 0;
        state_for_msr_ss = SOM;
        offset_for_msr_ss = 0;
        memcpy(&spi_data[offset_for_ss_get_msr], &finish_rsp_fields.version, 52); // buffer all MSR data for encryption into spi_data[] array and then send it out
        spdmContext->current_resp_cmd = SPDM_FINISH_RSP;
        if(spdm_pkt_process_encrypt_spdm_data(spdmContext, false, plaintext_size, spi_data, buf_for_encryption))
        {
            error_handle_secure_session = SPDM_ERROR_UNSPECIFIED;
            return FALSE;
        }
        remaining_for_ss = total_secured_message_size;
        packet_sz = (uint32_t)((total_secured_message_size + APP_DATA_SIZE_ADD_MSG_TYPE_SIZE + MCTP_TOT_HDR_SZ) & UINT32_MAX);
    } else {
        packet_sz = SPDM_FINISH_RESPONSE_SIZE;
    }

    spdm_secure_session_set_session_state(spdmContext, SPDM_SESSION_STATE_ESTABLISHED);

    // start the HB timer - twice the HEARTBEATPERIOD
    spdm_hb_response_timeout_start();
    SPDMHB_timer_started = true;

    return SPDM_CMD_PASS;
}

/******************************************************************************/
/** Function to parse CHALLENGE_AUTH request and fill part of spdm local buffer based on the request
 * @param spdmContext - SPDM module context
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @return void
 *******************************************************************************/
uint8_t spdm_pkt_challenge_auth(MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext)
{
    uint8_t iter = 0;

    uint8_t cmd_ret_val = SPDM_CMD_PASS;
    uint8_t ret_val;
    uint16_t fill_challenge_resp_offset = 0;
    if (NULL == spdmContext)
    {
        return 0xff;
    }   

    spi_data[CHALLENGE_VERSION_OFFSET] = CURRENT_VERSION;
    spi_data[CHALLENGE_RESP_CODE_OFFSET] = SPDM_CHALLENGE_AUTH_RSP;
    
    spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS] = CURRENT_VERSION; // V1.1

    spdm_buf_tx->pkt.data[SPDM_HEADER_COMMAND_POS] = SPDM_CHALLENGE_AUTH_RSP;

    requested_slot = (get_mctp_pld[SLOT_REQUEST_OFFSET] &
                      0xff);                          // attain the value of slot requested
    measurement_var.msr_hash_rqstd = (get_mctp_pld[MSR_SUMMARY_HASH_TYPE_OFFSET] &
                                      0xff); // whether measurement hash requested
    if(requested_slot > (MAX_SLOTS - 1))
    {
        return SPDM_CMD_FAIL;
    }

    if (cert2_base_addr != 0)
    {
        // check if chain present
        if (slot_buf[requested_slot].chain_present)
        {
            // Response Attribute Field
            spi_data[CHALLENGE_REQ_SLOT_OFFSET] = requested_slot;

            // Slot data
            spi_data[CHALLENGE_SLOT_DATA_OFFSET] = (1 << requested_slot);

            // at offset 4:
            // Hash of the certificate chain
            memcpy(&spi_data[CHALLENGE_CHAIN_HASH_OFFSET], &hash_of_chains[(requested_slot * SPDM_SHA384_LEN)], SPDM_SHA384_LEN);

            // at offset 4 + SPDM_SHA384_LEN
            // random value of 256 bit as nonce generated by TRNG each time a response sent:
            ret_val = spdm_crypto_ops_gen_random_no(&nonce_data[0], NOUNCE_DATA_SIZE);

            if (ret_val != MCHP_OK)
            {
                memset(&nonce_data[0], 0x00, NOUNCE_DATA_SIZE);
            }
            memcpy(&spi_data[CHALLENGE_NONCE_OFFSET], &nonce_data[0], NOUNCE_DATA_SIZE);
            fill_challenge_resp_offset = CHAL_NEXT_OFS_AFTER_NONCE_END; // here on , depending on the request the offset varies, so taking a variable
            if (measurement_var.msr_hash_rqstd)
            {
                if (measurement_var.msr_hash_rqstd == 0xff)
                {
                    memcpy(&spi_data[fill_challenge_resp_offset], &hash_concat_msrments_val[0], SPDM_MEASUREMENT_HASH_SUMMARY_SZ);
                }
                else
                {
                    memcpy(&spi_data[fill_challenge_resp_offset], &hash_concat_tcb_msrments_val[0], SPDM_MEASUREMENT_HASH_SUMMARY_SZ);
                }
                fill_challenge_resp_offset += 48;
            }

            spi_data[fill_challenge_resp_offset] = 0;
            spi_data[fill_challenge_resp_offset+1] = 1; // Size of opaq data len in LE format (=256 , 0x0100)
            fill_challenge_resp_offset +=OPAQUE_DATA_LEN;

            memset(&spi_data[fill_challenge_resp_offset], 0, OPAQUE_DATA_SZ);
            fill_challenge_resp_offset +=OPAQUE_DATA_SZ;

            spdmContext->request_or_response = 1;
            req_and_response_sz[1] = fill_challenge_resp_offset;
            spdmContext->get_requests_state = RUN_TIME_HASH_MODE;

            // get the hash of challenge response data of requested size
            spdm_get_len_for_runtime_hash(spdmContext);
            spdm_crypto_ops_run_time_hashing(&spi_data[0], length, spdmContext, &ctx_ptr, spdmContext->get_requests_state);

            spdmContext->get_requests_state = END_OF_HASH;
            spdm_crypto_ops_run_time_hashing(NULL, 0, spdmContext,&ctx_ptr, spdmContext->get_requests_state);
            memcpy(&hash_of_req_buffer[0], &spdmContext->sha_digest, SPDM_SHA384_LEN);
            // generate signature for the response using AK pvt key
            spdm_crypto_ops_gen_signature();

            memcpy(&spi_data[fill_challenge_resp_offset], &ecdsa_signature.ecdsa_signature[0], SS_ECDSA_384_SIGN_SIZE);
            fill_challenge_resp_offset+=SS_ECDSA_384_SIGN_SIZE;
            actual_challenge_length = fill_challenge_resp_offset;
            
            // the reason for adding 5 bytes of MCTP header is that we assume there is going to be only one single transaction and add it here
            // If it spans to multiple transactions, the 5 bytes is added into packet size at time of sending
            // For the last packet, the packet size is modified accordingly based on actual challenge length
            packet_sz = fill_challenge_resp_offset + MCTP_TOT_HDR_SZ;

            spdmContext->current_resp_cmd = SPDM_CHALLENGE_AUTH_RSP;
            first_challenge_resp_sent = TXSTATE_1;
        }
        else
        {
            cmd_ret_val = SPDM_CMD_FAIL;
        }
    }
    else
    {
        cmd_ret_val = SPDM_CMD_FAIL;
    }
    return cmd_ret_val;
}

/******************************************************************************/
/** Function to parse GET_MEASUREMENTS request and fill part of spdm local buffer based on the request
 * @param spdmContext - SPDM module context
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @return void
 *******************************************************************************/
uint8_t spdm_pkt_get_measurements(MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext)
{
    uint8_t ret_val = 0x00;
    uint8_t cmd_ret_val = SPDM_CMD_PASS;
    uint8_t no_of_indices = 0x00;
    uint8_t no_of_blocks = 0x00;
    uint16_t get_len = 0x0;
    uint16_t fill_meas_resp_offset = 0;
    uint16_t src_offset =0;

    if (NULL == spdmContext)
    {
        return 0xff;
    }

    if (SPDMHB_timer_started)
    {
        SPDMHB_timer_started = false;
        spdm_hb_response_timeout_stop();
        spdm_hb_response_timeout_start();
        SPDMHB_timer_started = true;
    }
    // GET_MEASUREMENTS request attributes
    measurement_var.sign_needed = (get_mctp_pld[2] & 0xff); // Bit to check that signature will be present in responder

    measurement_var.msr_operation = (get_mctp_pld[3] & 0xff); // param2 for the measurement operation

    if (message_type == MCTP_IC_MSGTYPE_SECURED_MESSAGE)
    {
        get_len = size_for_msr;
    }
    else
    {
        get_len = spdmContext->current_request_length;
    }

    // SPDM payload byte length
    // SlotIDParam
    if (measurement_var.sign_needed)
    {
        requested_slot = (get_mctp_pld[36] & 0xf); // attain the value of slot requested
        if ((!slot_buf[requested_slot].chain_present) || (get_len != MEAS_PLD_LEN_WITH_SIGN))
        {
            return SPDM_CMD_FAIL;
        }
    }
    else
    {
        if (get_len != MEAS_PLD_LEN_WITHOUT_SIGN)
        {
            return SPDM_CMD_FAIL;
        }
        requested_slot = 0U; // Param 2 - If this message does not contain a signature, this field shall be set to 0x0 .
    }

    if (measurement_var.msr_operation == 0)
    {
        // return the total number of measurement indices on the device
        no_of_indices = TOTAL_NO_OF_MSR_INDICES;
        no_of_blocks = 0x00;
        measurement_var.msr_rcrd_len = 0x00;
        measurement_var.msr_response_byte_iter = FILL_MSR_RND;
    }
    else if (measurement_var.msr_operation == 0xff) // request all blocks
    {
        no_of_blocks = TOTAL_NO_OF_MSR_INDICES;
        measurement_var.msr_rcrd_len = measurement_var.msr_record_size; // get the size of all measurement blocks supported
        measurement_var.msr_response_byte_iter = FILL_MSR_RCRD;
    }
    else if ((measurement_var.msr_operation >= START_INDEX) && (measurement_var.msr_operation <= TOTAL_NO_OF_MSR_INDICES))
    {
        no_of_blocks = 1U;
        if (measurement_var.msr_operation <= NO_OF_MSR_INDICES)
        {
            uint32_t temp = (uint32_t)((msr_block_buffer[(measurement_var.msr_operation) - 1u].msr_size[0u]) & UINT32_MAX);
            uint32_t msr_offset = (uint32_t)((temp + MSR_BLOCK_SPEC_FLDS_SZ) & UINT32_MAX);
            if ((msr_offset > UINT32_MAX) || (is_add_safe(msr_offset,
                                                          msr_block_buffer[(measurement_var.msr_operation) - 1].msr_size[0]) == false)) // Coverity INT31-C Fix
            {
                //trace0(ERROR_HANDLE_TRACE, SPDM_TSK, 0, "spgm2");
            }
            else
            {
                measurement_var.msr_rcrd_len = msr_offset; // get the size of only requested block
            }
        }
        else
        {
            uint32_t temp = (uint32_t)((msr_block_manifest.msr_size) & UINT32_MAX);
            uint32_t msr_offset = (uint32_t)((temp + MSR_BLOCK_SPEC_FLDS_SZ) & UINT32_MAX);
            if ((msr_offset > UINT32_MAX) || (is_add_safe(msr_offset,
                                                          msr_block_manifest.msr_size) == false)) // Coverity INT31-C Fix
            {
                //trace0(ERROR_HANDLE_TRACE, SPDM_TSK, 0, "spgm3");
            }
            else
            {
                measurement_var.msr_rcrd_len = msr_offset; // get the size of only requested block
            }
        }
    }
    else
    {
        return SPDM_CMD_FAIL;
    }

    spi_data[MEASUREMENT_VERSION_OFFSET] = CURRENT_VERSION; // V1.1

    spi_data[MEASUREMENT_RESP_CODE_OFFSET] = SPDM_GET_MEASUREMENT_RSP; // response code

    spi_data[MEASUREMENT_INDICES_OFFSET] = no_of_indices; // Param1

    spi_data[MEASUREMENT_REQ_SLOT_OFFSET] = requested_slot; // Param2

    spi_data[MEASUREMENT_NUM_BLOCKS_OFFSET] = no_of_blocks; // NumberOfBlocks

    spi_data[MEASUREMENT_REC_LEN_2] = (uint8_t)((measurement_var.msr_rcrd_len >> 16u) & UINT8_MAX); // MeasurementRecordLength
    spi_data[MEASUREMENT_REC_LEN_1] = (uint8_t)((measurement_var.msr_rcrd_len >> 8u) & UINT8_MAX);  // MeasurementRecordLength
    spi_data[MEASUREMENT_REC_LEN_0] = (uint8_t)(measurement_var.msr_rcrd_len & UINT8_MAX);          // MeasurementRecordLength

    fill_meas_resp_offset = MEAS_NEXT_OFS_AFTER_REC_LEN; // First 0 to 7 bytes are fixed, from 8th bytes the response bytes changes depending on the request and we start at index 3 for secure session purpose

    if (measurement_var.msr_operation == 0xff) // request all blocks
    {
        memcpy(&spi_data[fill_meas_resp_offset], &concat_msrment_blocks[0], measurement_var.msr_rcrd_len);
    }
    else if ((measurement_var.msr_operation >= START_INDEX) && (measurement_var.msr_operation <= TOTAL_NO_OF_MSR_INDICES))
    {
        if (UINT16_MAX - fill_meas_resp_offset >= measurement_var.msr_rcrd_len)
        {
            if (measurement_var.msr_operation <= NO_OF_MSR_INDICES)
            {
                src_offset = (measurement_var.msr_operation - 1) * SIZE_OF_ONE_MSR_BLOCK;
                if(((fill_meas_resp_offset + measurement_var.msr_rcrd_len) < SPI_DATA_BUF_SIZE) && ((src_offset + measurement_var.msr_rcrd_len) < CONCAT_MSRMENT_BLOCKS_SIZE))
                {
                    memcpy(&spi_data[fill_meas_resp_offset], &concat_msrment_blocks[src_offset], measurement_var.msr_rcrd_len);
                }
                else{
                    return SPDM_CMD_FAIL;
                }
            }
            else // APFW measurement
            {
                src_offset = SIZE_OF_ONE_MSR_BLOCK * NO_OF_MSR_INDICES;
                if(((fill_meas_resp_offset + measurement_var.msr_rcrd_len) < SPI_DATA_BUF_SIZE) && ((src_offset + measurement_var.msr_rcrd_len) < CONCAT_MSRMENT_BLOCKS_SIZE))
                {
                    memcpy(&spi_data[fill_meas_resp_offset], &concat_msrment_blocks[src_offset], measurement_var.msr_rcrd_len);
                }
                else{
                    return SPDM_CMD_FAIL;
                }
            }
        }
        else
        {
            return SPDM_CMD_FAIL;
        }
    }

    if (UINT16_MAX - fill_meas_resp_offset >= measurement_var.msr_rcrd_len)
    {
        fill_meas_resp_offset += measurement_var.msr_rcrd_len;
    }
    else
    {
        return SPDM_CMD_FAIL;
    }

    // at offset 8 + MeasurementRecordLength, nonce data
    memset(nonce_data, 0, NOUNCE_DATA_SIZE);
    ret_val = spdm_crypto_ops_gen_random_no(&nonce_data[0], NOUNCE_DATA_SIZE);

    if (ret_val != MCHP_OK)
    {
        memset(&nonce_data[0], 0x00, NOUNCE_DATA_SIZE);
    }

    if(((fill_meas_resp_offset + NOUNCE_DATA_SIZE) < SPI_DATA_BUF_SIZE))
    {
        memcpy(&spi_data[fill_meas_resp_offset], &nonce_data[0], NOUNCE_DATA_SIZE);
    }
    else{
        return SPDM_CMD_FAIL;
    }
    fill_meas_resp_offset+=NOUNCE_DATA_SIZE;
    if (fill_meas_resp_offset + 1 < SPI_DATA_BUF_SIZE)
    {
        spi_data[fill_meas_resp_offset + 1] = 0x1U; // opaq size 256 bytes (=0x0100 in Hex) in LE format
        spi_data[fill_meas_resp_offset] = 0x0U;
    }
    else{
        return SPDM_CMD_FAIL;
    }

    fill_meas_resp_offset+=OPAQUE_DATA_LEN;

    if(((fill_meas_resp_offset + OPAQUE_DATA_SZ) < SPI_DATA_BUF_SIZE))
    {
        memset(&spi_data[fill_meas_resp_offset], 0U, OPAQUE_DATA_SZ);
    }
    else{
        return SPDM_CMD_FAIL;
    }
    fill_meas_resp_offset+=OPAQUE_DATA_SZ;

    spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
    spdmContext->request_or_response = 1;

    if(safe_sub_16(fill_meas_resp_offset, MEAS_SPI_DATA_START, &req_and_response_sz[1]))
    {
        return SPDM_CMD_FAIL;
    }
    spdm_get_len_for_runtime_hash(spdmContext);
    spdm_crypto_ops_run_time_hashing(&spi_data[MEAS_SPI_DATA_START], length, spdmContext, &ctx_ptr, spdmContext->get_requests_state);

    if (measurement_var.sign_needed)
    {
        spdmContext->get_requests_state = END_OF_HASH;
        spdm_crypto_ops_run_time_hashing(NULL, 0, spdmContext,&ctx_ptr, spdmContext->get_requests_state);
        memcpy(&hash_of_req_buffer[0], &spdmContext->sha_digest, SPDM_SHA384_LEN);

        // generate signature for the response using AK pvt key
        spdm_crypto_ops_gen_signature();
        memcpy(&spi_data[fill_meas_resp_offset], &ecdsa_signature.ecdsa_signature[0], 96);
        fill_meas_resp_offset+=96;
    }
    else
    {
    }

    if(safe_sub_16(fill_meas_resp_offset, MEAS_SPI_DATA_START, &actual_meas_length))
    {
        return SPDM_CMD_FAIL;
    }
    
    // the reason for adding 5 bytes of MCTP header is that we assume there is going to be only one single transaction and add it here
    // If it spans to multiple transactions, the 5 bytes is added into packet size at time of sending
    // For the last packet, the packet size is modified accordingly based on actual_meas_length
    packet_sz = actual_meas_length + MCTP_TOT_HDR_SZ;

    spdmContext->current_resp_cmd = SPDM_GET_MEASUREMENT_RSP;
    if (message_type == MCTP_IC_MSGTYPE_SECURED_MESSAGE)
    {

        if (measurement_var.sign_needed)
        {
            packet_sz = SPDM_MEASUREMENT_RESPONSE_BYTES_SZ + measurement_var.msr_rcrd_len + (2 * CURVE_384_SZ);
        }
        else
        {
            packet_sz = SPDM_MEASUREMENT_RESPONSE_BYTES_SZ + measurement_var.msr_rcrd_len;
        }

        uint16_t plaintext_size = 0;
        // encrypted app data (app data is from spdm msg type, so increase the packet_sz + 1), application data len is added as part of encrypt API
        if (safe_sub_16((uint16_t)((packet_sz)&UINT16_MAX), MCTP_TOT_HDR_SZ, &plaintext_size))
        {
            return FALSE;
        }
        spdm_buf_tx = (MCTP_PKT_BUF *)&spdm_pktbuf[0];
        offset_for_ss_get_msr = 3;       // leave application length of size 2bytes and message type of size 1byte, taken from offset of SPDM header version
        spi_data[2] = MCTP_MSGTYPE_SPDM; // plain text data - application len of 2bytes, msg type 1 byte, followed by the application data
        data_size_for_ss_get_msr = 0;
        state_for_msr_ss = SOM;
        offset_for_msr_ss = 0;
        
        if (spdm_pkt_process_encrypt_spdm_data(spdmContext, false, plaintext_size, spi_data, buf_for_encryption))
        {
            error_handle_secure_session = SPDM_ERROR_UNSPECIFIED;
            return FALSE;
        }
        if (safe_add(total_secured_message_size, APP_DATA_SIZE_ADD_MSG_TYPE_SIZE + MCTP_TOT_HDR_SZ, &packet_sz))
        {
            return FALSE;
        }
        remaining_for_ss = total_secured_message_size;
    }
    first_meas_resp_sent = TXSTATE_1;

    return cmd_ret_val;
}

/******************************************************************************/
/** function to frame error response message
 * @param error_handle - error value
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @param get_cmd  - Current request / response code
 * @return None
 *******************************************************************************/
void spdm_fill_error_response(uint8_t error_handle, MCTP_PKT_BUF *spdm_buf_tx, uint8_t get_cmd)
{
    spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS] = CURRENT_VERSION; // V1.1
    spdm_buf_tx->pkt.data[SPDM_HEADER_COMMAND_POS] = SPDM_ERROR_RESP;
    spdm_buf_tx->pkt.field.hdr.byte_cnt = SPDM_ERROR_RESPONSE_BYTES_SIZE;
    packet_sz = SPDM_ERROR_RESPONSE_BYTES_SIZE;
    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS] = error_handle;

    if ((error_handle == SPDM_ERROR_INVLD_RQ) || (error_handle == SPDM_ERROR_UNXPTD_RQ_CODE) ||
        (error_handle == SPDM_ERROR_MJR_VRS_MISMATCH) || (error_handle == SPDM_ERROR_BUSY) ||
        (error_handle == SPDM_ERROR_UNSPECIFIED) )
    {
        spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 1] = 0x0; // No extended error data is provided
    }
    else if ((error_handle == SPDM_ERROR_USPRTD_RQ_CODE) || (error_handle == SPDM_ERROR_RESYNCH))
    {
        spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 1] = get_cmd; // error data - request response code
    }

    if (message_type == MCTP_IC_MSGTYPE_SECURED_MESSAGE) {
        uint16_t plaintext_size = 0;
        plaintext_size = SPDM_ERROR_RESPONSE_BYTES; // encrypted app data (app data is from spdm msg type, so increase the packet_sz + 1), application data len is added as part of encrypt API
        spdm_buf_tx = (MCTP_PKT_BUF *)&spdm_pktbuf[0];
        offset_for_ss_get_msr = 3; // leave application length of size 2bytes and message type of size 1byte, taken from offset of SPDM header version
        spi_data[2] = MCTP_MSGTYPE_SPDM; // plain text data - application len of 2bytes, msg type 1 byte, followed by the application data
        data_size_for_ss_get_msr = 0;
        state_for_msr_ss = SOM;
        offset_for_msr_ss = 0;
        memcpy(&spi_data[offset_for_ss_get_msr], &spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS], SPDM_ERROR_RESPONSE_BYTES - 1U);
        spdmContext->current_resp_cmd = SPDM_ERROR_RESP;
        if(spdm_pkt_process_encrypt_spdm_data(spdmContext, false, plaintext_size, spi_data, buf_for_encryption))
        {
            return;
        }
        remaining_for_ss = total_secured_message_size;
        if (safe_add(total_secured_message_size, APP_DATA_SIZE_ADD_MSG_TYPE_SIZE + MCTP_TOT_HDR_SZ, &packet_sz))
        {
            error_handle_secure_session = SPDM_ERROR_UNSPECIFIED;
            return;
        }
    }
}

/******************************************************************************/
/** function to parse the incoming deliver encapsulated response for mutual authentication
 * @param spdmContext - SPDM module context
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @return None : value greater than 0 depending on the error present in payload
 *                                    0 implies No error
 *******************************************************************************/
uint8_t spdm_deliver_encap_res_request(MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext)
{
    // process incoming get_digest response or get_certificate response
    if (memcmp(&get_mctp_pld[SS_DELIVER_ENCAP_RESP_REQUEST_ID_OFFSET], &spdmContext->request_id, 1))
    {
        error_handle_secure_session = SPDM_ERROR_UNSPECIFIED;
        return SPDM_CMD_FAIL;
    } 

    uint8_t response_opcode;
    uint8_t status;
    uint8_t ret_val;
    void *encap_request;
    uint8_t size_of_encapreq_resp = 0U, total_size_of_encapreq_resp = 0U;
    SS_ENCAP_RESPONSE_STRUCT *current_encap_response = NULL;
    SS_ENCAP_RESPONSE_STRUCT encap_response_struct[2] = {
        {SPDM_GET_DIGEST_RESPONSE, spdm_get_encap_request_get_digest,
        spdm_process_encap_response_digest},
        {SPDM_GET_CERT_RESPONSE, spdm_get_encap_request_get_certificate,
        spdm_process_encap_response_certificate},      
    };

    response_opcode = get_mctp_pld[SS_DELIVER_ENCAP_RESP_OPCODE_OFFSET];
    for (uint8_t index = 0; index < 2; index++)
    {
        if (encap_response_struct[index].request_op_code == response_opcode) {
            current_encap_response = &encap_response_struct[index];
        }
    }

    if (current_encap_response == NULL)
    {
        error_handle_secure_session = SPDM_ERROR_UNSPECIFIED;
        return SPDM_CMD_FAIL;
    }

    status = current_encap_response->process_encap_response();

    if (!status)
    {
        error_handle_secure_session = SPDM_ERROR_INVALID_RESPONSE_CODE;
        return SPDM_CMD_FAIL;
    }

    // send out next request ie in case of get_digest response received, send out get_certificate request
    SPDM_ENCAP_REQ_RSP_FIELDS *spdm_encapreq_response;
    void *ptr = (void *)&encapreq_response[3];
    spdm_encapreq_response = (SPDM_ENCAP_REQ_RSP_FIELDS *)ptr;
    spdm_encap_move_to_next_opcode();
    spdm_encapreq_response->version = CURRENT_VERSION;

    spdm_encapreq_response->resp_code = DELIVER_ENCAP_RSP_RSP;
    // random value 1byte for request ID field
    ret_val = spdm_crypto_ops_gen_random_no((uint8_t *)&spdmContext->request_id, SS_ENCAP_RESP_REQUEST_ID_RANDOM_NO_SIZE);
    if (ret_val != MCHP_OK)
    {
        memset(&spdmContext->request_id, 0x00, SS_ENCAP_RESP_REQUEST_ID_RANDOM_NO_SIZE);
    }
    spdm_encapreq_response->param1 = spdmContext->request_id;
    if ((response_opcode == SPDM_GET_DIGEST_RESPONSE) || 
        ((response_opcode == SPDM_GET_CERT_RESPONSE) && (get_cert_length != 0))) {
        spdm_encapreq_response->param2 = 0x1;
    } else if ((response_opcode == SPDM_GET_CERT_RESPONSE) && (get_cert_length == 0)) {
        spdm_encapreq_response->param2 = 0x2;
    }

    encap_request = spdm_encapreq_response + SS_ENCAP_REQUST_MOVE_1BYTE;

    if (spdm_encapreq_response->param2 == 0x01) {
        spdm_process_encapsulated_response(encap_request, &size_of_encapreq_resp);
        if (safe_add_8(sizeof(SPDM_ENCAP_REQ_RSP_FIELDS), size_of_encapreq_resp, &total_size_of_encapreq_resp))
        {
            error_handle_secure_session = SPDM_ERROR_UNSPECIFIED;
            return SPDM_CMD_FAIL;
        }
    } else if (spdm_encapreq_response->param2 == 0x02) {
        encapreq_response[4] = requester_cert_chain_slot;
        total_size_of_encapreq_resp = sizeof(SPDM_ENCAP_REQ_RSP_FIELDS) + SS_ENCAP_REQUST_MOVE_1BYTE;
    }

    if (total_size_of_encapreq_resp != 0)
    {
        memcpy(&spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS], spdm_encapreq_response, total_size_of_encapreq_resp);
    }

    if (message_type == MCTP_IC_MSGTYPE_SECURED_MESSAGE) {
        uint16_t plaintext_size = 0;
        plaintext_size = total_size_of_encapreq_resp + 1; // encrypted app data (app data is from spdm msg type, so increase the packet_sz + 1), application data len is added as part of encrypt API
        offset_for_ss_get_msr = 3; // leave application length of size 2bytes and message type of size 1byte, taken from offset of SPDM header version
        encapreq_response[2] = MCTP_MSGTYPE_SPDM; // plain text data - application len of 2bytes, msg type 1 byte, followed by the application data
        data_size_for_ss_get_msr = 0;
        state_for_msr_ss = SOM;
        offset_for_msr_ss = 0;
        if (total_size_of_encapreq_resp != 0)
        {
            memcpy(&encapreq_response[offset_for_ss_get_msr], spdm_encapreq_response, total_size_of_encapreq_resp); // buffer all MSR data for encryption into spi_data[] array and then send it out
        }
        spdmContext->current_resp_cmd = SPDM_ENCAP_REQ_RSP;
        if(spdm_pkt_process_encrypt_spdm_data(spdmContext, false, plaintext_size, encapreq_response, buf_for_encryption))
        {
            error_handle_secure_session = SPDM_ERROR_UNSPECIFIED;
            return FALSE;
        }
        if (safe_add(total_secured_message_size, APP_DATA_SIZE_ADD_MSG_TYPE_SIZE + MCTP_TOT_HDR_SZ, &packet_sz))
        {
            error_handle_secure_session = SPDM_ERROR_UNSPECIFIED;
            return SPDM_CMD_FAIL;
        }
        remaining_for_ss = total_secured_message_size;
    } else {
        if (safe_add(total_size_of_encapreq_resp, APP_DATA_SIZE_ADD_MSG_TYPE_SIZE + MCTP_TOT_HDR_SZ, &packet_sz))
        {
            error_handle_secure_session = SPDM_ERROR_UNSPECIFIED;
            return SPDM_CMD_FAIL;
        }
    }

    return true;
    
}
/******************************************************************************/
/** function to parse the incoming encapsulated request for mutual authentication
 * @param spdmContext - SPDM module context
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @return None : value greater than 0 depending on the error present in payload
 *                                    0 implies No error
 *******************************************************************************/
uint8_t spdm_encap_req_request(MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext)
{
    void *encap_request;
    uint8_t size_of_encapreq_resp;
    uint8_t ret_val;
    uint8_t total_size_of_encapreq_resp;

    if (NULL == spdmContext)
    {
        error_handle_secure_session = SPDM_ERROR_UNSPECIFIED;
        return SPDM_CMD_FAIL;
    }

    SPDM_ENCAP_REQ_RSP_FIELDS *spdm_encapreq_response;
    void *ptr = (void *)encapreq_response; //coverity fix
    spdm_encapreq_response = (SPDM_ENCAP_REQ_RSP_FIELDS *)ptr;
    spdm_encapreq_response->version = CURRENT_VERSION;

    spdm_encapreq_response->resp_code = SPDM_ENCAP_REQ_RSP;
    
    // random value 1byte for request ID field
    ret_val = spdm_crypto_ops_gen_random_no((uint8_t *)&spdmContext->request_id, SS_ENCAP_RESP_REQUEST_ID_RANDOM_NO_SIZE);

    if (ret_val != MCHP_OK)
    {
        memset(&spdmContext->request_id, 0x00, SS_ENCAP_RESP_REQUEST_ID_RANDOM_NO_SIZE);
    }

    spdm_encapreq_response->param1 = spdmContext->request_id;
    spdm_encapreq_response->param2 = 0;

    encap_request = spdm_encapreq_response + 1;
    ret_val = spdm_process_encapsulated_response(encap_request, &size_of_encapreq_resp);

    if (ret_val == false)
    {
        error_handle_secure_session = SPDM_ERROR_UNSPECIFIED;
        return SPDM_CMD_FAIL;
    }

    if (safe_add_8(sizeof(SPDM_ENCAP_REQ_RSP_FIELDS), size_of_encapreq_resp, &total_size_of_encapreq_resp))
    {
        error_handle_secure_session = SPDM_ERROR_UNSPECIFIED;
        return SPDM_CMD_FAIL;
    }

    memcpy(&spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS], spdm_encapreq_response, total_size_of_encapreq_resp);

    if (message_type == MCTP_IC_MSGTYPE_SECURED_MESSAGE) {
        uint16_t plaintext_size = 0;
        plaintext_size = total_size_of_encapreq_resp + 1; // encrypted app data (app data is from spdm msg type, so increase the packet_sz + 1), application data len is added as part of encrypt API
        offset_for_ss_get_msr = 3; // leave application length of size 2bytes and message type of size 1byte, taken from offset of SPDM header version
        spi_data[2] = MCTP_MSGTYPE_SPDM; // plain text data - application len of 2bytes, msg type 1 byte, followed by the application data
        data_size_for_ss_get_msr = 0;
        state_for_msr_ss = SOM;
        offset_for_msr_ss = 0;
        memcpy(&spi_data[offset_for_ss_get_msr], spdm_encapreq_response, total_size_of_encapreq_resp); // buffer all MSR data for encryption into spi_data[] array and then send it out
        spdmContext->current_resp_cmd = SPDM_ENCAP_REQ_RSP;
        if(spdm_pkt_process_encrypt_spdm_data(spdmContext, false, plaintext_size, spi_data, buf_for_encryption))
        {
            error_handle_secure_session = SPDM_ERROR_UNSPECIFIED;
            return FALSE;
        }
        if (safe_add(total_secured_message_size, MCTP_TOT_HDR_SZ + 1, &packet_sz))
        {
            error_handle_secure_session = SPDM_ERROR_UNSPECIFIED;
            return FALSE;
        }
        remaining_for_ss = total_secured_message_size;
    } else {
        packet_sz = MCTP_TOT_HDR_SZ + 1 + total_size_of_encapreq_resp;
    }

    return TRUE;
}

/******************************************************************************/
/** function to encap get_certificate request for mutual authentication
 * @param get_cert_request - pointer to hold GET_CERT req
 * @param size  - size of get_certificate request
 * @return true/false
 *******************************************************************************/
uint8_t spdm_get_encap_request_get_certificate(void *get_cert_request, uint8_t *size)
{
    if (get_cert_request == NULL)
    {
        return FALSE;
    }

    GET_CERT_REQ_FIELDS *get_cert;
    get_cert = get_cert_request;
    get_cert->version = CURRENT_VERSION;
    get_cert->resp_code = SPDM_GET_CERT;
    get_cert->param1 = requester_cert_chain_slot;
    get_cert->param2 = 0;
    get_cert->offset = get_cert_offset;
    get_cert->length = get_cert_length;

    uint8_t size_of_req = sizeof(GET_CERT_REQ_FIELDS); 

    *size = size_of_req;

    return TRUE;
}

/******************************************************************************/
/** function to parse get_certificate response for mutual authentication
 * @param None
 * @return true/false
 *******************************************************************************/
uint8_t spdm_process_encap_response_certificate()
{
    uint8_t slot = 0;
    uint16_t portion_length = 0;
    uint16_t remainder_length = 0;
    uint8_t ret_val = TRUE;

    slot =  get_mctp_pld[6];
    memcpy(&portion_length, &get_mctp_pld[8], 2);
    memcpy(&remainder_length, &get_mctp_pld[10], 2);
    memcpy(&spi_data[get_cert_offset], &get_mctp_pld[12], portion_length);

    if (safe_add_16(get_cert_offset, get_cert_length, &get_cert_offset))
    {
        return FALSE;
    }
    if ((remainder_length < SPDM_MUT_AUTH_GET_CERTIFICATE_LENGTH)) {
        get_cert_length = remainder_length;
    } else {
        get_cert_length = SPDM_MUT_AUTH_GET_CERTIFICATE_LENGTH;
    }

    if (remainder_length == 0)
    {
        ret_val = spdm_x509_parse_certificate_chain();
    }

    return ret_val;
    
}

/******************************************************************************/
/** function to move to next opcode as part of mutual auth
 * @param None
 * @return None
 *******************************************************************************/
void spdm_encap_move_to_next_opcode()
{
    request_op_code = SPDM_GET_CERT;
}

/******************************************************************************/
/** function to parse the enc response received
 * @param encap_request the response received from host
 * @param size pointer to size
 * @return true/false
 *******************************************************************************/
uint8_t spdm_process_encapsulated_response(void *encap_request, uint8_t *size)
{
    uint8_t status;
    SS_ENCAP_RESPONSE_STRUCT *current_encap_response = NULL;
    SS_ENCAP_RESPONSE_STRUCT encap_response_struct[2] = {
        {SPDM_GET_DIGEST, spdm_get_encap_request_get_digest,
        spdm_process_encap_response_digest},
        {SPDM_GET_CERT, spdm_get_encap_request_get_certificate,
        spdm_process_encap_response_certificate},      
    };
    for (uint8_t index = 0; index < 2; index++)
    {
        if (encap_response_struct[index].request_op_code == request_op_code) {
            current_encap_response = &encap_response_struct[index];
        }
    }

    if (current_encap_response == NULL)
    {
        return false;
    }

    status = current_encap_response->get_encap_request(encap_request, size);

    return status;
}

/******************************************************************************/
/** function to derive IV based on sequence number
 * @param sequence_number seq number for particualr packet
 * @param iv pointer to iv derived
 * @param salt pointer to salt
 * @param aead_iv_size iv size
 * @return true/false
 *******************************************************************************/
static void spdm_generate_iv(uint64_t sequence_number, uint8_t *iv, const uint8_t *salt,
                        size_t aead_iv_size)
{
    uint8_t iv_temp[MAX_AEAD_IV_SIZE] = {0};
    size_t index;

    /* Form the AEAD IV from the salt and the sequence number. */
    memcpy(iv, salt, aead_iv_size);

    /* If little-endian then the sequence number is zero-extended to the higher indices.
     * The sequence number begins at the lowest index (0). */
    memcpy(iv_temp, &sequence_number, sizeof(sequence_number));
    for (index = 0; index < sizeof(sequence_number); index++) {
        iv[index] = iv[index] ^ iv_temp[index];
    }
}

/******************************************************************************/
/** function to encrypt spdm data
 * @param spdmContext pointer to spdm Context
 * @param is_request_message request or response message
 * @param app_message_size size of application message
 * @param app_msg pointer to app msg data
 * @param secured_msg pointer to encrypted msg
 * @return true/false
 *******************************************************************************/
// Before app_message, there is room for spdm_secured_message_cipher_header_t
// After (app_message + app_message_size), there is room for random bytes.
// app_msg is the plaintext data - points to spi_data[0]; 
// spi_data[0,1] = application data len - application data is filled as part of this API
// spi_data[2] = application msg type
// secured_msg = buf_for_encryption[0] - has data from session id
uint8_t spdm_pkt_process_encrypt_spdm_data(SPDM_CONTEXT *spdmContext, uint8_t is_request_message, uint16_t app_message_size, uint8_t *app_msg, uint8_t *secured_msg)
{
    uint8_t *key;
    uint8_t *a_data;
    uint8_t *enc_msg;
    uint8_t *salt;
    uint8_t *tag;
    uint64_t sequence_number;
    uint64_t sequence_number_in_header;
    uint8_t iv[MAX_AEAD_IV_SIZE];
    size_t cipher_text_size;
    size_t aead_tag_size;
    size_t aead_key_size;
    size_t aead_iv_size;
    size_t a_data_size;
    size_t enc_msg_size;
    uint16_t application_data_len;
    uint16_t random_data_len;
    uint8_t ret_val;
    spdm_secured_message_a_data_header_t *record_header;
    spdm_secured_message_cipher_header_t *enc_msg_header;
    uint16_t tag_offset;

    aead_tag_size = MAX_AEAD_TAG_SIZE;
    aead_key_size = MAX_AEAD_KEY_SIZE;
    aead_iv_size = MAX_AEAD_IV_SIZE;

    switch (spdmContext->secure_session_info.session_state) {
    case SPDM_SESSION_STATE_HANDSHAKING:
        if (is_request_message) {
            key = ( uint8_t *)spdmContext->secure_session_info.secure_session_handshake_secret.
                  request_handshake_encryption_key;
            salt = (uint8_t *)spdmContext->secure_session_info.secure_session_handshake_secret.
                   request_handshake_salt;
            sequence_number =
                spdmContext->secure_session_info.secure_session_handshake_secret.request_handshake_sequence_number;
        } else {
            key = ( uint8_t *)spdmContext->secure_session_info.secure_session_handshake_secret.
                  response_handshake_encryption_key;
            salt = (uint8_t *)spdmContext->secure_session_info.secure_session_handshake_secret.
                   response_handshake_salt;
            sequence_number =
                spdmContext->secure_session_info.secure_session_handshake_secret.response_handshake_sequence_number;
        }
        break;
    case SPDM_SESSION_STATE_ESTABLISHED:
        if (is_request_message) {
            key = ( uint8_t *)spdmContext->secure_session_info.secure_session_application_secret.
                  request_data_encryption_key;
            salt = (uint8_t *)spdmContext->secure_session_info.secure_session_application_secret.
                   request_data_salt;
            sequence_number =
                spdmContext->secure_session_info.secure_session_application_secret.request_data_sequence_number;
        } else {
            key = ( uint8_t *)spdmContext->secure_session_info.secure_session_application_secret.
                  response_data_encryption_key;
            salt = (uint8_t *)spdmContext->secure_session_info.secure_session_application_secret.
                   response_data_salt;
            sequence_number =
                spdmContext->secure_session_info.secure_session_application_secret.response_data_sequence_number;
        }
        break;
    default:
        return false;
    }
    spdm_generate_iv(sequence_number, iv, salt, MAX_AEAD_IV_SIZE);

    memcpy(&sequence_number_in_header, &sequence_number, SPDM_MCTP_SEQUENCE_NUMBER_COUNT);

    if (spdmContext->secure_session_info.session_state == SPDM_SESSION_STATE_HANDSHAKING) {
        if (is_request_message) {
           spdmContext->secure_session_info.secure_session_handshake_secret.request_handshake_sequence_number++;
        } else {
           spdmContext->secure_session_info.secure_session_handshake_secret.response_handshake_sequence_number++;
        }
    } else {
        if (is_request_message) {
            spdmContext->secure_session_info.secure_session_application_secret.request_data_sequence_number++;
        } else {
            spdmContext->secure_session_info.secure_session_application_secret.response_data_sequence_number++;
        }
    }

    cipher_text_size = (uint32_t)((sizeof(spdm_secured_message_cipher_header_t) + app_message_size + // spdm_secured_message_cipher_header_t = application data len
                          NOUNCE_DATA_SIZE) & UINT32_MAX); // random data size
    total_secured_message_size = (uint32_t) ((sizeof(spdm_secured_message_a_data_header_t) + cipher_text_size + aead_tag_size) & UINT32_MAX); // spdm_secured_message_a_data_header_t = session id, seq num, len
    void *ptr= (void *)secured_msg; //coverity fix
    record_header = (spdm_secured_message_a_data_header_t *)ptr;
    record_header->session_id = final_session_id;
    record_header->seq_num = (uint16_t)(sequence_number & 0xFFFF);
    record_header->length = (uint16_t)((cipher_text_size + aead_tag_size) & UINT16_MAX);

    enc_msg_header = (void *)(app_msg); // points to spi_data[0] starting from application data len
    enc_msg_header->application_data_length = app_message_size;

    ret_val = spdm_crypto_ops_gen_random_no((uint8_t *)enc_msg_header +
                                            sizeof(spdm_secured_message_cipher_header_t) +
                                            app_message_size, NOUNCE_DATA_SIZE);
    if (ret_val != MCHP_OK)
    {
        memset(&nonce_data[0], 0x00, NOUNCE_DATA_SIZE);
    }

    a_data = (uint8_t *)record_header;
    a_data_size = sizeof(spdm_secured_message_a_data_header_t);

    enc_msg = (uint8_t *)enc_msg_header; // starts from application data len

    tag = &(secured_msg[sizeof(spdm_secured_message_a_data_header_t) + cipher_text_size]);

    ret_val = spdm_aead_ops(AES_GCM_ENCRYPT, key, aead_key_size, iv, aead_iv_size, a_data, a_data_size, enc_msg, cipher_text_size, tag, (uint8_t)aead_tag_size, &(secured_msg[sizeof(spdm_secured_message_a_data_header_t)]), cipher_text_size);

    return ret_val;
}

/******************************************************************************/
/** function to decrypt secure session spdm data
 * @param spdmContext pointer to spdm Context
 * @return true/false
 *******************************************************************************/
uint8_t spdm_pkt_process_decrypt_spdm_data(SPDM_CONTEXT *spdmContext, uint8_t is_request_message)
{
    uint8_t *key;
    uint8_t *a_data;
    uint8_t *enc_msg;
    uint8_t *salt;
    uint8_t *tag;
    uint64_t sequence_number;
    uint16_t sequence_number_in_header;
    uint16_t sequence_number_in_header_rcvd;
    uint8_t iv[MAX_AEAD_IV_SIZE] = {0};
    size_t cipher_text_size = 0;
    size_t aead_tag_size;
    size_t aead_key_size;
    size_t aead_iv_size;
    size_t a_data_size;
    size_t enc_msg_size;
    uint16_t application_data_len;
    uint16_t random_data_len;
    uint8_t ret_val;
    uint32_t session_id_rcvd;
    uint16_t size_to_copy_ciphertext;

    aead_tag_size = MAX_AEAD_TAG_SIZE;
    aead_key_size = MAX_AEAD_KEY_SIZE;
    aead_iv_size = MAX_AEAD_IV_SIZE;

    memcpy(&session_id_rcvd, &get_mctp_pld[SPDM_SM_HEADER_SESSION_ID_POS], 4);
    memcpy(&sequence_number_in_header_rcvd, &get_mctp_pld[SPDM_SM_HEADER_SEQ_NUM_POS], 2);

    if (!(final_session_id == session_id_rcvd)) // session id offset 
    {
        return FALSE;
    }

    switch (spdmContext->secure_session_info.session_state) {
    case SPDM_SESSION_STATE_HANDSHAKING:
        if (is_request_message) {
            key = ( uint8_t *)spdmContext->secure_session_info.secure_session_handshake_secret.
                  request_handshake_encryption_key;
            salt = (uint8_t *)spdmContext->secure_session_info.secure_session_handshake_secret.
                   request_handshake_salt;
            sequence_number =
                spdmContext->secure_session_info.secure_session_handshake_secret.request_handshake_sequence_number;
        } else {
            key = ( uint8_t *)spdmContext->secure_session_info.secure_session_handshake_secret.
                  response_handshake_encryption_key;
            salt = (uint8_t *)spdmContext->secure_session_info.secure_session_handshake_secret.
                   response_handshake_salt;
            sequence_number =
                spdmContext->secure_session_info.secure_session_handshake_secret.response_handshake_sequence_number;
        }
        break;
    case SPDM_SESSION_STATE_ESTABLISHED:
        if (is_request_message) {
            key = ( uint8_t *)spdmContext->secure_session_info.secure_session_application_secret.
                  request_data_encryption_key;
            salt = (uint8_t *)spdmContext->secure_session_info.secure_session_application_secret.
                   request_data_salt;
            sequence_number =
                spdmContext->secure_session_info.secure_session_application_secret.request_data_sequence_number;
        } else {
            key = ( uint8_t *)spdmContext->secure_session_info.secure_session_application_secret.
                  response_data_encryption_key;
            salt = (uint8_t *)spdmContext->secure_session_info.secure_session_application_secret.
                   response_data_salt;
            sequence_number =
                spdmContext->secure_session_info.secure_session_application_secret.response_data_sequence_number;
        }
        break;
    default:
        return false;
    }

    spdm_generate_iv(sequence_number, iv, salt, MAX_AEAD_IV_SIZE);

    memcpy(&sequence_number_in_header, &sequence_number, SPDM_MCTP_SEQUENCE_NUMBER_COUNT);

    if (!(sequence_number_in_header == sequence_number_in_header_rcvd))
    {
        return false;
    }

    if (spdmContext->secure_session_info.session_state == SPDM_SESSION_STATE_HANDSHAKING) {
        if (is_request_message) {
           spdmContext->secure_session_info.secure_session_handshake_secret.request_handshake_sequence_number++;
        } else {
           spdmContext->secure_session_info.secure_session_handshake_secret.response_handshake_sequence_number++;
        }
    } else {
        if (is_request_message) {
            spdmContext->secure_session_info.secure_session_application_secret.request_data_sequence_number++;
        } else {
            spdmContext->secure_session_info.secure_session_application_secret.response_data_sequence_number++;
        }
    }

    uint16_t total_len = 0;
    memcpy(&total_len, &get_mctp_pld[SPDM_SM_HEADER_LENGTH_POS], 2);
    if (safe_sub_16(total_len, MAX_AEAD_TAG_SIZE, (uint16_t *)&cipher_text_size))
    {
        return FALSE;
    }
    a_data = &get_mctp_pld[SPDM_SM_HEADER_SESSION_ID_POS];
    a_data_size = SPDM_SM_HEADER_SESSION_ID_SIZE + SPDM_SM_HEADER_SEQ_NUM_SIZE + SPDM_SM_HEADER_LENGTH_SIZE;

    enc_msg = &get_mctp_pld[SPDM_SM_ENCRYPTED_DATA_POS];
    enc_msg_size = cipher_text_size;
    tag = &(get_mctp_pld[SPDM_SM_ENCRYPTED_DATA_POS + cipher_text_size]);

    ret_val = spdm_aead_ops(AES_GCM_DECRYPT, key, aead_key_size, iv, aead_iv_size, a_data, a_data_size, enc_msg, enc_msg_size, tag, (uint8_t)aead_tag_size, buf_for_encryption, cipher_text_size);

    if (ret_val) // if decrypt results in error, return and send out error response
    {
        return FALSE;
    }
    // buf_for_encryption has the decrypted message starting from application data length, followed by message type
    memcpy(&application_data_len, &buf_for_encryption[0], 2);

    // copy application data into get_mctp_pld array
    if (safe_sub_16(application_data_len, APP_DATA_SIZE_MINUS_MSG_TYPE_SIZE, &size_to_copy_ciphertext))
    {
        return FALSE;
    }

    if (buf_for_encryption[2] != MCTP_IC_MSGTYPE_SPDM)
    {
        return FALSE;
    }

    memcpy(&get_mctp_pld[0], &buf_for_encryption[3], size_to_copy_ciphertext); // copy from msg header in the encap msg, appliacation data len & msg type are left out
    size_for_msr = size_to_copy_ciphertext;

    return TRUE;
}

/******************************************************************************/
/** function to validate spdm data received
 * @param spdmContext - SPDM module context
 * @param spdm_buf_tx  - Fill spdm response in this pointer
 * @param get_cmd                   - RequestResponseCode for individual SPDM Commands Request as per SPDM spec
 *                                    eg: 0x84 - GET_VERSION
 * @return error_handle       : value greater than 0 depending on the error present in payload
 *                                    0 implies No error
 *******************************************************************************/
uint8_t spdm_pkt_validate_and_process_spdm_msg(uint8_t get_cmd, MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext)
{
    uint8_t get_ver = 0x00;
    volatile uint8_t error_handle = 0x00; // SPDM coverity fix

    if (NULL == spdmContext)
    {
        return 0xff;
    }

    get_ver = (get_mctp_pld[0] & 0xff); // offset 0 as per spec

    // all spdm get version request commands shall start with major revision 1.0
    // current implementation is fixed for major versions 1.0 only and only one single version entry
    // codebase is fixed to support 1.0 spdm version protocol alone
    // see macros SPDM_VER_NUM_MAJOR, SPDM_VER_NUM_MINOR
    if (get_ver > CURRENT_VERSION)
    {
        error_handle = SPDM_ERROR_MJR_VRS_MISMATCH;
    }
    switch (get_cmd)
    {
    case SPDM_GET_VERSION:
        // if no version error; then populate the tx buffer with spdm data
        if (error_handle == 0)
        {
            // set all previous states and buffers to NIL
            spdmContext->previous_request_length = 0x00;
            spdmContext->total_request_bytes = 0x00;
            spdmContext->previous_state = 0;

            // This is the first request
            spdmContext->get_requests_state = HASH_INIT_MODE;
            spdmContext->secure_session_get_requests_state = HASH_INIT_MODE; // secure - session key exchange

            spdmContext->request_or_response = 0; // 0 means request
            // init the hash engine
            spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], 0, spdmContext,&ctx_ptr, spdmContext->get_requests_state);

            for (uint8_t i = 0; i < 6; i++)
            {
                spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], length, spdmContext, &ctx_ptr_arr_ss[i], spdmContext->secure_session_get_requests_state);
            }
            req_and_response_sz[0] = spdmContext->current_request_length;
            // change the state to run time hash
            spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
            spdmContext->secure_session_get_requests_state = RUN_TIME_HASH_MODE;
            // calculate hash of request
            spdm_get_len_for_runtime_hash(spdmContext);
            spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], length, spdmContext, &ctx_ptr, spdmContext->get_requests_state);

            for (uint8_t i = 0; i < 6; i++)
            {
                spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], length, spdmContext, &ctx_ptr_arr_ss[i], spdmContext->secure_session_get_requests_state);
            }

            spdm_pkt_process_get_version_cmd(spdm_buf_tx, spdmContext);

            spdm_rqst_cur_state = get_cmd;
        }
        break;
    case SPDM_GET_CAPABILITIES:
        // if no version error; then populate the tx buffer with spdm data
        if (error_handle == 0)
        {
            // check if the request sequence is correct; check if current state is get version
            // check if previous state was get version
            if ((spdm_rqst_cur_state == SPDM_GET_VERSION) || (spdmContext->previous_state == SPDM_GET_VERSION))
            {
                spdmContext->previous_state = SPDM_GET_VERSION;
                spdmContext->request_or_response = 0; // 0 means request
                req_and_response_sz[0] = spdmContext->current_request_length;
                // switch to next state to handle more requests
                spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
                spdmContext->secure_session_get_requests_state = RUN_TIME_HASH_MODE;

                // calculate hash of request
                spdm_get_len_for_runtime_hash(spdmContext);
                spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], length, spdmContext, &ctx_ptr, spdmContext->get_requests_state);

                for (uint8_t i = 0; i < 6; i++)
                {
                    spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], length, spdmContext, &ctx_ptr_arr_ss[i], spdmContext->secure_session_get_requests_state);
                }

                spdm_pkt_process_get_capabilties_cmd(spdm_buf_tx, spdmContext);

                spdm_rqst_cur_state = get_cmd;
            }
            else
            {
                error_handle = SPDM_ERROR_UNXPTD_RQ_CODE;
            }
        }
        break;
    case SPDM_NEG_ALGO:
        if (error_handle == 0)
        {
            // check if the request sequence is correct; check for previous request state
            if ((spdm_rqst_cur_state == SPDM_GET_CAPABILITIES) || (spdmContext->previous_state == SPDM_GET_CAPABILITIES))
            {
                spdmContext->previous_state = SPDM_GET_CAPABILITIES;
                spdmContext->request_or_response = 0; // 0 means request
                req_and_response_sz[0] = spdmContext->current_request_length;
                // switch to next state to handle more requests
                spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
                spdmContext->secure_session_get_requests_state = RUN_TIME_HASH_MODE;
                // calculate hash of request
                spdm_get_len_for_runtime_hash(spdmContext);
                spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], length, spdmContext,&ctx_ptr, spdmContext->get_requests_state);

                for (uint8_t i = 0; i < 6; i++)
                {
                    spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], length, spdmContext, &ctx_ptr_arr_ss[i], spdmContext->secure_session_get_requests_state);
                }

                spdm_pkt_process_neg_alg_cmd(spdm_buf_tx, spdmContext);
                spdm_rqst_cur_state = get_cmd;
            }
            else
            {
                error_handle = SPDM_ERROR_UNXPTD_RQ_CODE;
            }
        }
        break;
    case SPDM_GET_DIGEST:
        if (error_handle == 0)
        {
            // valid command
            // check if previous 3 commands was done or if CACHE_CAP=1 is set already
            if ((spdm_rqst_cur_state == SPDM_NEG_ALGO) || (spdmContext->previous_state == SPDM_NEG_ALGO))
            {
                spdmContext->previous_state = SPDM_NEG_ALGO;
                spdmContext->request_or_response = 0; // 0 means request
                req_and_response_sz[0] = spdmContext->current_request_length;
                // switch to next state to handle more requests
                spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
                // calculate hash of request
                spdm_get_len_for_runtime_hash(spdmContext);
                spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], length, spdmContext,&ctx_ptr, spdmContext->get_requests_state);
                spdm_pkt_process_get_digest_cmd(spdm_buf_tx, spdmContext);

                spdm_rqst_cur_state = get_cmd;
            } else {
                // start from get version
                error_handle = SPDM_ERROR_UNXPTD_RQ_CODE;
            }
        }
        break;
    case SPDM_GET_CERT:
        if (error_handle == 0)
        {
            if (pldm_flash_busy || check_for_i2c_flash_busy_bit())
            {
                error_handle = SPDM_ERROR_BUSY;
            }
            else if ((spdm_rqst_cur_state == SPDM_GET_DIGEST) || (spdmContext->previous_state == SPDM_GET_DIGEST))
            {
                spdmContext->previous_state = SPDM_GET_DIGEST;
                spdmContext->request_or_response = 0; // 0 means request
                req_and_response_sz[0] = spdmContext->current_request_length;
                // switch to next state to handle more requests
                spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
                // calculate hash of request
                spdm_get_len_for_runtime_hash(spdmContext);
                spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], length, spdmContext,&ctx_ptr, spdmContext->get_requests_state);
                uint8_t ret_val = spdm_pkt_process_get_cert_cmd(spdm_buf_tx, spdmContext);
                if (ret_val == SPDM_CMD_FAIL)
                {
                    error_handle = SPDM_ERROR_INVLD_RQ;
                }
                else if (ret_val == SPDM_CMD_CHAIN_INVALID)
                {
                    error_handle = SPDM_ERROR_UNSPECIFIED;
                }
                else
                {
                    spdm_rqst_cur_state = get_cmd;
                }
            }
            else
            {
                error_handle = SPDM_ERROR_UNXPTD_RQ_CODE;
            }
        }
        break;
    case SPDM_CHALLENGE_AUTH_RQ:
        if (error_handle == 0)
        {
            if ((spdm_rqst_cur_state == SPDM_GET_CERT) || (spdmContext->previous_state == SPDM_GET_CERT))
            {
                spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
                spdmContext->request_or_response = 0; // 0 means request
                req_and_response_sz[0] = spdmContext->current_request_length;
                spdm_get_len_for_runtime_hash(spdmContext);
                spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], length, spdmContext,&ctx_ptr,spdmContext->get_requests_state);

                uint8_t ret_val = spdm_pkt_challenge_auth(spdm_buf_tx, spdmContext);
                if (ret_val == SPDM_CMD_FAIL)
                {
                    error_handle = SPDM_ERROR_INVLD_RQ;
                }
                else // Update states Only if Challenge is successful 
                {
                    spdmContext->previous_state = SPDM_GET_CERT;
                    spdm_rqst_cur_state = SPDM_CHALLENGE_AUTH_RQ;
                }
            }
            else
            {
                error_handle = SPDM_ERROR_UNXPTD_RQ_CODE;
            }
        }
        break;
    case SPDM_GET_MEASUREMENT:
        if (error_handle == 0)
        {
            if (((spdm_rqst_cur_state == SPDM_CHALLENGE_AUTH_RQ) && (spdmContext->secure_session_info.session_state == SPDM_SESSION_STATE_NOT_STARTED)) ||
                (spdmContext->secure_session_info.session_state == SPDM_SESSION_STATE_ESTABLISHED)) // Only if Challenge was successful, process Measurements 
            {
                // first call to go for init hash engine
                if (spdmContext->get_requests_state == HASH_INIT_MODE)
                {
                    spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], 0, spdmContext,&ctx_ptr,spdmContext->get_requests_state);
                }
                
                spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
                spdmContext->request_or_response = 0; // 0 means request
                req_and_response_sz[0] = spdmContext->current_request_length;
                spdm_get_len_for_runtime_hash(spdmContext);
                spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], length, spdmContext,&ctx_ptr, spdmContext->get_requests_state);
                uint8_t ret_val = spdm_pkt_get_measurements(spdm_buf_tx, spdmContext);

                if (ret_val == SPDM_CMD_FAIL)
                {
                    error_handle = SPDM_ERROR_INVLD_RQ;
                }

            }
            else
            {
                error_handle = SPDM_ERROR_UNXPTD_RQ_CODE;
            }
        }
        break;
    case SPDM_KEY_EXCHANGE:
        if (error_handle == 0)
        {
            uint8_t ret_val = spdm_secure_session_key_exchange(spdm_buf_tx, spdmContext);
            if (ret_val == SPDM_CMD_FAIL)
            {
                error_handle = error_handle_secure_session;
            }       
        }
        break;
    case SPDM_FINISH:
        if (error_handle == 0)
        {
            uint8_t ret_val = spdm_secure_session_finish(spdm_buf_tx, spdmContext);
            if (ret_val == SPDM_CMD_FAIL)
            {
                error_handle = error_handle_secure_session;
            }       
        }
        break; 
    case SPDM_END_SESSION:
        if (error_handle == 0)
        {
            uint8_t ret_val = spdm_secure_session_end_session(spdm_buf_tx, spdmContext);
            if (ret_val == SPDM_CMD_FAIL)
            {
                error_handle = error_handle_secure_session;
            }        
        }
        break;
    case SPDM_HEARTBEAT:
        if (error_handle == 0)
        {
            uint8_t ret_val = spdm_secure_session_heartbeat(spdm_buf_tx, spdmContext);

            if (ret_val == SPDM_CMD_FAIL)
            {
                error_handle = error_handle_secure_session;
            }       
        }
        break;    
    case SPDM_ENCAP_REQ_REQ:
        if (error_handle == 0)
        {
            uint8_t ret_val = spdm_encap_req_request(spdm_buf_tx, spdmContext);
            if (ret_val == SPDM_CMD_FAIL)
            {
                error_handle = error_handle_secure_session;
            }       
        }
        break;
    case DELIVER_ENCAP_RSP_REQ:
        if (error_handle == 0)
        {
            uint8_t ret_val = spdm_deliver_encap_res_request(spdm_buf_tx, spdmContext);
            if (ret_val == SPDM_CMD_FAIL)
            {
                error_handle = error_handle_secure_session;
            }        
        }
        break;  
    default:
        // invalid request command code
        error_handle = SPDM_ERROR_INVLD_RQ;
        //trace0(0, SPDM_TSK, 0, "spdm_evt_tsk: IVLD CMND rcvd!");
        break;
    }
    // if any reported error scenario; fill tx buffer with error response
    if (error_handle > 0)
    {
        spdm_fill_error_response(error_handle, spdm_buf_tx, get_cmd);
    }

    return error_handle;
}

/******************************************************************************/
/** once spdm msg is ready to transmit, trigger mctp; call this
 * function to configure/initialize tx buffer parameters and handle tx state
 * for scheduling packet transmission over smbus.
 * @param tx_buf Pointer to TX packet buffer
 * @return void
 *******************************************************************************/
void spdm_pkt_msg_ready_trigger_mctp(MCTP_PKT_BUF *tx_buf)
{
    ////trace0(0, MCTP, 0, "mctp_txpktready_init: Enter");

    /* configure/initialize tx packet buffer parameters */
    tx_buf->smbus_nack_retry_count = 0;
    tx_buf->smbus_lab_retry_count = 0;
    tx_buf->smbus_acquire_retry_count = 0;
    tx_buf->request_per_tx_timeout_count = 0;
    tx_buf->request_tx_retry_count = 0;
    tx_buf->buf_full = MCTP_TX_PENDING;

    SPDM_CONTEXT *spdmContext = NULL;
    spdmContext = spdm_ctxt_get();
    if (NULL == spdmContext)
    {
        return;
    }

    /* If tx state is MCTP_IDLE; then switch to MCTP_TX_NEXT. Else if tx state
     * is not MCTP_IDLE; then tx state machine will automatically switch to
     * MCTP_TX_NEXT after completing current tx */
    if (tx_buf->pkt.field.hdr.eom != 1)
    {
        spdm_tx_state = SPDM_TX_IN_PROGRESS;
    }
    else
    {
        spdm_tx_state = SPDM_TX_IDLE;
        packet_sz = 0;
        pkt_seq_mctp = 0;
        first_pkt = true;
        tbl_entry = 0;
        pld_index = 0;
        time_stamp = 0;
        slot_position = 0;
        bytes_sent_over_mctp_for_cert = 0;
        first_get_cert_response_sent = 0;
        remaining_bytes_to_sent = 0;
        first_challenge_resp_sent = CHG_RESP_END_OF_TX;
        measurement_var.msr_bytes_copied = 0;
        measurement_var.msr_operation = 0x00;
        measurement_var.msr_response_byte_iter = MSR_RESP_END_OF_TRANSACTION;
        measurement_var.msr_rcrd_bytes_to_sent = 0;
        signature_offset = 0;
        measurement_var.concat_msr_offset = 0;
        measurement_var.opq_offset = 0;
        measurement_var.msr_rnd_bytes_remaining = 0;
        measurement_var.msr_opq_bytes_remaining = 0;
        measurement_var.msr_hash_rqstd = 0;
        measurement_var.msr_hash_sum_offset = 0;
        nonce_offset = 0;
        spdmContext->current_request_length = 0;
        spdmContext->current_resp_cmd = 0x00;
        memset(get_mctp_pld, 0, MAX_SIZE_CERTIFICATE);
        memset(&spdm_pktbuf_rx, 0, sizeof(MCTP_PKT_BUF));
        spdm_pktbuf_rx.buf_full = MCTP_EMPTY;
    }

	di_send_spdm_reponse_packet((uint8_t *)tx_buf, sizeof(MCTP_PKT_BUF), false,
                                false); // uses mctp_pktbuf[MCTP_BUF2] for tx
} /* End spdm_pkt_spdm_msg_ready_trigger_mctp */

/******************************************************************************/
/** once spdm msg is ready to transmit, spdm state machine is set to transmit mode
 * function to load the spdm data to mctp tx buffer
 * @param none
 * @return void
 *******************************************************************************/
void spdm_pkt_tx_packet()
{
    uint8_t cmd_resp = 0;
    SPDM_CONTEXT *spdmContext = NULL;
    spdmContext = spdm_ctxt_get();
    if (NULL == spdmContext)
    {
        return;
    }
    switch (spdm_tx_state)
    {
    case SPDM_TX_IDLE:
        break;
    case SPDM_TX_IN_PROGRESS:
        mctp_buf_tx = &mctp_pktbuf_tx;
        memset(mctp_buf_tx, 0, sizeof(MCTP_PKT_BUF));
        // handle packet to mctp layer for multiple or single packet response
        spdm_buf_tx = (MCTP_PKT_BUF *)&spdm_pktbuf[0];
        cmd_resp = spdmContext->current_resp_cmd;
        spdm_pkt_populate_mctp_packet_for_resp(spdm_buf_tx, mctp_buf_tx, cmd_resp, spdmContext);
        spdm_pkt_msg_ready_trigger_mctp(mctp_buf_tx);
        break;
    default:
        //trace0(0, MCTP, 0, "spdm_tx_pkt: default");
        break;
    }
}

/******************************************************************************/
/** Function to load the 1Kb spdm input buffer for the spdm request bytes
 * @param spdmContext     - SPDM module context
 * @param spdm_msg_rx_buf - Buffer containing SPDM Request
 * @param len - length of data to be copied
* @param offset - offset at which data starts
 * @return 1 if payload size goes greater than 1024
 *         0 success
 *******************************************************************************/
uint8_t spdm_pkt_fill_buffer(MCTP_PKT_BUF *spdm_msg_rx_buf, SPDM_CONTEXT *spdmContext, uint8_t len, uint8_t offset)
{
    uint8_t i;
    uint8_t ret_sts = 0x00;
    uint16_t get_len = 0x00;

    if (NULL == spdmContext)
    {
        return 0x01;
    }

    if ((pld_index + len) <= MULTIPLE_PKT_MAX_SIZE) // if no of bytes received cross max input buffer size of 1024
    {
        // buffer the spdm payload to 1K input buffer from mctp input buffer
        for (i = 0; i < len; i++)
        {
            get_mctp_pld[pld_index + i] = spdm_msg_rx_buf->pkt.data[offset + i]; // Take only the spdm msg payload
        }

        if (spdm_tx_state == SPDM_PACKETIZING)
        {
            if (safe_add_16(pld_index, len, &pld_index))
            {
                return TRUE;
            }
            spdmContext->current_request_length = pld_index;
        }
        else if (spdm_tx_state == SPDM_NON_PACKETIZING)
        {
            spdmContext->current_request_length = len;
        }
    } else {
        ret_sts = 0x01;
    }

    return ret_sts;
}

/******************************************************************************/
/** once mctp rx buffer has spdm payload received, spdm state machine is set to read mode
 * function to trigger the loading of spdm data to 1Kb buffer
 * trigger validation of spdm data received
 * @param none
 * @return void
 *******************************************************************************/
void spdm_pkt_rcv_packet()
{
    uint8_t get_cmd = 0x00;
    uint8_t ret_sts = 0x00;
    SPDM_CONTEXT *spdmContext = NULL;
    uint8_t get_len = 0, offset_data_pos = 0;

    spdmContext = spdm_ctxt_get();
    if (NULL == spdmContext)
    {
        return;
    }
    MCTP_PKT_BUF *spdm_msg_rx_buf = NULL;
    spdm_msg_rx_buf = (MCTP_PKT_BUF *)&spdm_pktbuf_rx;

    // access mctp buffer for spdm only if mctp receive buffer is available
    if (MCTP_RX_PENDING == spdm_msg_rx_buf->buf_full)
    {
        spdmContext->host_eid = spdm_msg_rx_buf->pkt.field.hdr.src_eid;
        spdmContext->ec_eid = spdm_msg_rx_buf->pkt.field.hdr.dst_eid;
        spdmContext->ec_slv_addr = spdm_msg_rx_buf->pkt.field.hdr.dst_addr;
        spdmContext->host_slv_addr = spdm_msg_rx_buf->pkt.field.hdr.src_addr;
        spdmContext->message_tag = spdm_msg_rx_buf->pkt.field.hdr.msg_tag;
        spdmContext->spdm_cmd_code = spdm_msg_rx_buf->pkt.field.hdr.cmd_code;

        if (spdm_tx_state == SPDM_TX_IDLE || spdm_tx_state == SPDM_PACKETIZING)
        {
            // check if MCTP packet received is single packet request
            if ((spdm_msg_rx_buf->pkt.data[MCTP_MSG_TAG_REF_MASK] & MCTP_SOM_EOM_REF_MSK) == MCTP_SOM_EOM_REF)
            {
                pld_index = 0;
                spdm_tx_state = SPDM_NON_PACKETIZING;
                get_len = (uint8_t)(((spdm_msg_rx_buf->pkt.data[MCTP_BYTE_CNT_OFFSET]) -
                                                NO_OF_MCTP_HDR_BYTES_FRM_BYTE_CNT_OFFSET)&UINT8_MAX); // SPDM payload byte length
                offset_data_pos = SPDM_HEADER_VERSION_POS;
                ret_sts = spdm_pkt_fill_buffer(spdm_msg_rx_buf, spdmContext, get_len, offset_data_pos);
                message_type = spdm_msg_rx_buf->pkt.data[MCTP_PKT_IC_MSGTYPE_POS];
                if (!ret_sts)
                {
                    time_stamp = spdm_msg_rx_buf->rx_timestamp;
                }
            }
            else
            {

                if ((spdm_msg_rx_buf->pkt.data[MCTP_MSG_TAG_REF_MASK] & MCTP_SOM_REF) == MCTP_SOM_REF)
                {
                    pld_index = 0U;
                    message_type = spdm_msg_rx_buf->pkt.data[MCTP_PKT_IC_MSGTYPE_POS];
                    // SPDM payload byte length
                    if (safe_sub_8((spdm_msg_rx_buf->pkt.data[MCTP_BYTE_CNT_OFFSET]), NO_OF_MCTP_HDR_BYTES_FRM_BYTE_CNT_OFFSET, &get_len))
                    {
                        return;
                    }
                    offset_data_pos = SPDM_HEADER_VERSION_POS;
                } else {
                    if (safe_sub_8((spdm_msg_rx_buf->pkt.data[MCTP_BYTE_CNT_OFFSET]), 5, &get_len))
                    {
                        return;
                    }
                    offset_data_pos = SPDM_HEADER_VERSION_POS - 1;
                }
                // som = 1, eom = 0 or som = 0, eom = 0 or som = 0, eom = 1 states
                // this means packetizing in progress
                spdm_tx_state = SPDM_PACKETIZING;
                ret_sts = spdm_pkt_fill_buffer(spdm_msg_rx_buf, spdmContext, get_len, offset_data_pos);
                if (!ret_sts)
                {
                    if ((spdm_msg_rx_buf->pkt.data[MCTP_MSG_TAG_REF_MASK] & MCTP_EOM_REF_MSK) == MCTP_EOM_REF)
                    {
                        spdm_tx_state = SPDM_RX_LAST_PKT;
                        pld_index = 0;
                        time_stamp = spdm_msg_rx_buf->rx_timestamp;
                    }
                    else
                    {
                        spdm_tx_state = SPDM_PACKETIZING;
                    }
                }
                else
                {
                    spdm_buf_tx = (MCTP_PKT_BUF *)&spdm_pktbuf[0];
                    time_stamp = 0;
                    pld_index = 0;
                    spdm_tx_state = SPDM_TX_IDLE;
                    spdmContext->current_request_length = 0;
                    memset(spdm_buf_tx, 0, MCTP_PKT_BUF_DATALEN);
                    memset(get_mctp_pld, 0, MAX_SIZE_CERTIFICATE);
                }
            }
        }

        if (spdm_tx_state == SPDM_NON_PACKETIZING || spdm_tx_state == SPDM_RX_LAST_PKT)
        {
            // reached here means spdm request need to be processed for validation
            spdm_buf_tx = (MCTP_PKT_BUF *)&spdm_pktbuf[0];
            memset(spdm_buf_tx, 0, MCTP_PKT_BUF_DATALEN);
            spdm_buf_tx->rx_timestamp = time_stamp;
            if (message_type == MCTP_IC_MSGTYPE_SECURED_MESSAGE)
            {
                ret_sts = spdm_pkt_process_decrypt_spdm_data(spdmContext, true); // common api for decryption of handshake and data (SPDM GET_MSR) messages
            } else {
                ret_sts = true;
            }

            if (ret_sts == FALSE) {
                spdm_fill_error_response(SPDM_ERROR_DECRYPT_ERROR, spdm_buf_tx, 0);
                spdm_secure_session_cleanup(spdmContext);
            } else {
                get_cmd = (get_mctp_pld[1] & 0xff); // get the spdm command
                ret_sts = spdm_pkt_validate_and_process_spdm_msg(get_cmd, spdm_buf_tx, spdmContext);
            }

            if (ret_sts)
            {
                //trace0(0, SPDM_TSK, 0, "spdm_evt_tsk: Ivld SPDM rqst rcvd!");
                //trace0(0, SPDM_TSK, 0, "spdm_evt_tsk: Sending SPDM err respse!");
            }
            else
            {
                //trace0(0, SPDM_TSK, 0, "spdm_evt_tsk: Vld SPDM rqst rcvd");
            }
            spdm_tx_state = SPDM_TX_IN_PROGRESS;
        }
        else
        {
            memset(&spdm_pktbuf_rx, 0, sizeof(MCTP_PKT_BUF));
            spdm_pktbuf_rx.buf_full = MCTP_EMPTY;
        }

        spdm_di_mctp_done_set();
    }
}

/******************************************************************************/
/** This is called whenever kernel schedules spdm event task.
 * mctp module calls SET_MCTP_EVENT_TASK(spdm) for scheduling spdm event task.
 * This event task is called whenever mctp packet is received with spdm message type over smbus,
 * @param spdmContext - SPDM module context
 * @return void
 *******************************************************************************/
void spdm_event_task(SPDM_CONTEXT *spdmContext)
{
    if (spdmContext->spdm_state_info ==
            SPDM_IDLE) // If SPDM state machine is in idle state, do no set send any SPDM response
    {
        if (MCTP_RX_PENDING == spdm_pktbuf_rx.buf_full)
        {
            spdm_pktbuf_rx.buf_full = MCTP_EMPTY;

            spdm_di_mctp_done_set();
        }
    }
    else
    {
        spdm_pkt_rcv_packet();
        spdm_pkt_tx_packet();
    }
    //trace0(0, SPDM_TSK, 0, "spdm_evt_tsk: End");

} /* End spdm_event_task() */


/******************************************************************************/
/** Function to send request to get CERT chain configured from AP_CFG table
 * @param spdmContext - SPDM module context
 * @return void
 *******************************************************************************/
void spdm_pkt_get_cert_from_apcfg(SPDM_CONTEXT *spdmContext)
{
#if 0
    uint8_t ret_val = false;

    if (NULL == spdmContext)
        return;

    ret_val = sb_apcfg_cert_data_get((CFG_CERT*)&AP_CFG_cert_buffer[0]);

    if(ret_val)
    {
        spdmContext->spdm_state_info = SPDM_COPY_CERT_DATA_TO_BUF;
    }
    else
    {
        spdmContext->spdm_state_info = SPDM_IDLE; //something wrong in ap cfg
    }
    SET_SPDM_EVENT_FLAG();
#endif
    if (NULL == spdmContext)
    {
        return;
    }

    (void)di_sb_apcfg_cert_data_request();
    /* Move to idle state and wait for response */
    spdmContext->spdm_state_info = SPDM_IDLE;
}

/******************************************************************************/
/** Function to copy certificate data from internal flash/ SRAM to buffer
 * @param spdmContext - SPDM module context
 * @return void
 *******************************************************************************/
void spdm_pkt_copy_cert_data_to_buf(SPDM_CONTEXT *spdmContext)
{
    uint8_t cert_cnt_int_flash = 0x01;
    uint8_t slot = 0;
    uint8_t cert_cnt = 0;
    uint8_t index = 0;
    chain_avail_bits_mask = 0x00;
    ptr_cert_buffer = (CFG_CERT *)&AP_CFG_cert_buffer[0];

    if (NULL == spdmContext || NULL == ptr_cert_buffer)
        return;

    for (slot = 0; slot < (MAX_SLOTS - 1); slot += 2, index++)
    {
        if ((ptr_cert_buffer->certificate_chain_slot[index] & NO_CHAIN_VAL_NIBBLE_0_MSK) !=
                NO_CHAIN_VAL_NIBBLE_0_MSK) // 0x1000 means no chain in this slot

        {
            slot_buf[slot].chain_no = ptr_cert_buffer->certificate_chain_slot[index] &
                                      CHAIN_VAL_MSK_NIBBLE0_MSK; //[2:0] bits for chain number
            slot_buf[slot].chain_present = true;
            slot_buf[slot].chain.head_ptr_val = ptr_cert_buffer->head_ptr_chain[slot_buf[slot].chain_no];
            chain_avail_bits_mask = ((1U << slot) | chain_avail_bits_mask) & 0xFFU;
        }

        if ((ptr_cert_buffer->certificate_chain_slot[index] & NO_CHAIN_VAL_NIBBLE_1_MSK) !=
                NO_CHAIN_VAL_NIBBLE_1_MSK) // 0x1000 means no chain in this slot
        {
            slot_buf[slot + 1].chain_no = ((ptr_cert_buffer->certificate_chain_slot[index] & CHAIN_VAL_MSK_NIBBLE1_MSK) >>
                                           4); //[6:4] bits for chain number
            slot_buf[slot + 1].chain_present = true;
            slot_buf[slot + 1].chain.head_ptr_val = ptr_cert_buffer->head_ptr_chain[slot_buf[slot + 1].chain_no];
            if(((1U << (slot + 1U)) | chain_avail_bits_mask) > UINT8_MAX)
            {
                // Handle Error
            }
            else
            {
                chain_avail_bits_mask = (uint8_t)((1U << (slot + 1U)) | chain_avail_bits_mask);
            }
        }
    }

    // initialize certificate tail pointer and memory address (sram/internal spi)
    for (cert_cnt = 0; cert_cnt < MAX_CERTIFICATES; cert_cnt++)
    {
        if (cert_cnt == 0)
        {
            cert_buf[cert_cnt].mem_addr =
                CERTIFICATE_START_ADDRESS; // 0x126800;read from sram mailbox address for AK 0 cert at 0x126600
        }
        else if (cert_cnt == 1)
        {
            cert_buf[cert_cnt].mem_addr = CERTIFICATE_START_ADDRESS +
                                          CERT_MAX_SIZE; // 0x126c00;read from sram mailbox address for AK 1 cert (reserved)
        }
        else if (cert_cnt == 2)
        {
            // devIK (internal flash)
            cert_buf[cert_cnt].mem_addr = cert2_base_addr;
        }
        else
        {
            // other certificates (root etc) in internal flash
            // base address for cert2 + 1K offsets for subsequent cert in flash
            cert_buf[cert_cnt].mem_addr = cert2_base_addr + (CERT_MAX_SIZE * cert_cnt_int_flash);
            cert_cnt_int_flash = cert_cnt_int_flash + 1U; // increment count
            if (is_add_safe(cert_cnt_int_flash, 1U) == 0)
            {
                // Handle Error
            }
        }
        cert_buf[cert_cnt].tail_ptr_val = ptr_cert_buffer->tail_ptr_certificate[cert_cnt];
    }
    spdmContext->spdm_state_info = SPDM_CALC_HASH_CHAIN;
    SET_SPDM_EVENT_FLAG();
}

/******************************************************************************/
/** Function to get the length for runtime hashing
 * @param None
 * @return None 
 *******************************************************************************/
void spdm_get_len_for_runtime_hash(SPDM_CONTEXT *spdmContext)
{
    if(spdmContext->request_or_response)
    {
        //get length of response
        length = req_and_response_sz[1];
    }
    else
    {
        //get length of request
        length = req_and_response_sz[0];
    }
}

/******************************************************************************/
/** function to get tag image loaded
 * @param None
 * @return         : 0U (SB_ECFW_IMG_TAG0)
 *                   1U (SB_ECFW_IMG_TAG1)
 *                   2U (INVALID_TAG_IMAGE)
 *******************************************************************************/
uint8_t get_tag_image_loaded()
{
    uint32_t tag_image = 0x00;
    uint8_t return_loaded_tag_img = INVALID_TAG_IMAGE;

    do
    {
        /*Read the load status*/
        if(SRAM_RLOG_API_rom_event_read((uint8_t *)&tag_image) == DI_CHECK_FAIL)
        {
            return_loaded_tag_img = INVALID_TAG_IMAGE; // Permission Issue
            break;
        }

        if ((tag_image & RLOG_LOAD_FROM_TAG0)) /*Loaded from TAG0*/
        {
            return_loaded_tag_img = SB_ECFW_IMG_TAG0; // If loaded from TAG0 then TAG1 is not loaded
        }
        else /*Loaded from TAG1*/
        {
            return_loaded_tag_img = SB_ECFW_IMG_TAG1;
        }
    }
    while(0);
    return return_loaded_tag_img;
}

/******************************************************************************/
/** function to return signature type to be used
 * @param None
 * @return         : 0U (USE_OTP_GENERATED_PRIVATE_KEY)
 *                   1U (USE_PUF_GENERATED_PRIVATE_KEY)
 *                   2U (INVALID_SIGNATURE_FLAGS)
 *******************************************************************************/
uint8_t signature_type()
{
    uint8_t tag_data[16];
    uint32_t tagx_address = 0x00;
    uint8_t loaded_tag_image;
    uint8_t puf0_lock_feature_otp;
    uint8_t ret_sts = INVALID_SIGNATURE_FLAGS;

    do
    {
        loaded_tag_image = get_tag_image_loaded();
        if(loaded_tag_image == INVALID_TAG_IMAGE)
        {
            ret_sts = INVALID_SIGNATURE_FLAGS;
            break;
        }

        // Get tag address corresponding to the loaded image
        spdm_di_sb_ecfw_tagx_addr_get(&tagx_address, loaded_tag_image);
        tagx_address = tagx_address & 0xFFFFFFF0;
        /*  Read byte 0x006
            Bit [0:3] in spi address is used to get port & component. Due to this constraint, Offsets which are not 16bytes aligned cannot be read byte wise.
            Offset must be 16bytes aligned and minimum read length to get non-16-byte aligned data should be 16*/

        spdm_di_qmspi_clr_port_init_status(SPI_SELECT_INT_COMP_0);
        spdm_di_init_flash_component(SPI_SELECT_INT_COMP_0);
        if (di_spdm_spi_send_read_request(tagx_address, &tag_data[0], 16, SPI_SELECT_INT_COMP_0, true))
        {
            spdm_di_spi_tristate(INT_SPI);
            ret_sts = INVALID_SIGNATURE_FLAGS;
            break;
        }
        spdm_di_spi_tristate(INT_SPI);

        if (GET_SILICON_VER() == SILICON_VER_A0)
        {
            ret_sts = USE_OTP_GENERATED_PRIVATE_KEY;
        }
        else
        {
            if((tag_data[6] & DEVAK_DEVIK_KEY_SELECT_MASK) != 0x0)
            {
                ret_sts = INVALID_SIGNATURE_FLAGS;
                break;
            }

            efuse_read_data(60, &puf0_lock_feature_otp, 1);

            if ((puf0_lock_feature_otp & PUF0_OTP_LOCK_MASK) != 0U)
            {
                ret_sts = INVALID_SIGNATURE_FLAGS;
                break;
            }

            ret_sts = USE_PUF_GENERATED_PRIVATE_KEY;
        }
    }
    while(0);
    return ret_sts;
}

/******************************************************************************/
/** fill_cert_buffer
* Fill the Certificate buffer with slot, chain and certificate details.
* @param void
* @return void
*******************************************************************************/
void fill_cert_buffer()
{
    ptr_cert_buffer = (CFG_CERT *)&AP_CFG_cert_buffer[0];

    ptr_cert_buffer->head_ptr_chain[0] = SPDM_SLOT_CERT_CHAIN01;
    ptr_cert_buffer->head_ptr_chain[1] = SPDM_SLOT_CERT_CHAIN23;
    ptr_cert_buffer->head_ptr_chain[2] = SPDM_SLOT_CERT_CHAIN45;
    ptr_cert_buffer->head_ptr_chain[3] = SPDM_SLOT_CERT_CHAIN67;

    ptr_cert_buffer->head_ptr_chain[0] = SPDM_HEAD_POINTER_0;
    ptr_cert_buffer->head_ptr_chain[1] = SPDM_HEAD_POINTER_1;
    ptr_cert_buffer->head_ptr_chain[2] = SPDM_HEAD_POINTER_2;
    ptr_cert_buffer->head_ptr_chain[3] = SPDM_HEAD_POINTER_3;
    ptr_cert_buffer->head_ptr_chain[4] = SPDM_HEAD_POINTER_4;
    ptr_cert_buffer->head_ptr_chain[5] = SPDM_HEAD_POINTER_5;
    ptr_cert_buffer->head_ptr_chain[6] = SPDM_HEAD_POINTER_6;
    ptr_cert_buffer->head_ptr_chain[7] = SPDM_HEAD_POINTER_7;

    uint32_t temp = TAIL_CERTIFICATE0123;
    memcpy(&(ptr_cert_buffer->tail_ptr_certificate[0]), &temp, 4);

    temp = TAIL_CERTIFICATE4567;
    memcpy(&(ptr_cert_buffer->tail_ptr_certificate[4]), &temp, 4);

    temp = TAIL_CERTIFICATE891011;
    memcpy(&(ptr_cert_buffer->tail_ptr_certificate[8]), &temp, 4);

    temp = TAIL_CERTIFICATE12131415;
    memcpy(&(ptr_cert_buffer->tail_ptr_certificate[12]), &temp, 4); 

    temp = TAIL_CERTIFICATE16171819;
    memcpy(&(ptr_cert_buffer->tail_ptr_certificate[16]), &temp, 4); 

    temp = TAIL_CERTIFICATE20212223;
    memcpy(&(ptr_cert_buffer->tail_ptr_certificate[20]), &temp, 4); 

    temp = TAIL_CERTIFICATE24252627;
    memcpy(&(ptr_cert_buffer->tail_ptr_certificate[24]), &temp, 4); 

    temp = TAIL_CERTIFICATE28293031;
    memcpy(&(ptr_cert_buffer->tail_ptr_certificate[28]), &temp, 4); 

    temp = TAIL_CERTIFICATE32333435;
    memcpy(&(ptr_cert_buffer->tail_ptr_certificate[32]), &temp, 4); 

    temp = TAIL_CERTIFICATE36373839;
    memcpy(&(ptr_cert_buffer->tail_ptr_certificate[36]), &temp, 4); 

    temp = TAIL_CERTIFICATE40414243;
    memcpy(&(ptr_cert_buffer->tail_ptr_certificate[40]), &temp, 4); 

    temp = TAIL_CERTIFICATE44454647;
    memcpy(&(ptr_cert_buffer->tail_ptr_certificate[44]), &temp, 4); 

    temp = TAIL_CERTIFICATE48495051;
    memcpy(&(ptr_cert_buffer->tail_ptr_certificate[48]), &temp, 4); 

    temp = TAIL_CERTIFICATE52535455;
    memcpy(&(ptr_cert_buffer->tail_ptr_certificate[52]), &temp, 4); 

    temp = TAIL_CERTIFICATE56575859;
    memcpy(&(ptr_cert_buffer->tail_ptr_certificate[56]), &temp, 4); 

    temp = TAIL_CERTIFICATE60616263;
    memcpy(&(ptr_cert_buffer->tail_ptr_certificate[60]), &temp, 4); 
}

/******************************************************************************
 ** get_cert2_base_address()
 * This function is to get Certificate 2 Address
 * @param cert_ptr         Pointer to hold certificate 2 address
 * @return                 None
 * @note
 ******************************************************************************/
void get_cert2_base_address(uint32_t *cert_ptr)
{
    uint8_t efuse_data = 0x00;

    if (0 == efuse_read_data(CERT2_BASE_ADDR_OTP_OFFSET, &efuse_data, 1))
    {
        *cert_ptr = efuse_data;
    }
    if (0 == efuse_read_data(CERT2_BASE_ADDR_OTP_OFFSET + 1, &efuse_data, 1))
    {
        *cert_ptr = ((*cert_ptr << 8) | efuse_data);
    }
    if (0 == efuse_read_data(CERT2_BASE_ADDR_OTP_OFFSET + 2, &efuse_data, 1))
    {
        *cert_ptr = ((*cert_ptr << 8) | efuse_data);
    }
    if (0 == efuse_read_data(CERT2_BASE_ADDR_OTP_OFFSET + 3, &efuse_data, 1))
    {
        *cert_ptr = ((*cert_ptr << 8) | efuse_data);
    }
}

/******************************************************************************/
/** spdm_read_certificate
 * This function can be used to read the certificate data and store it in buffer.
 * @param address          Address of certificate
 * @param buff_ptr         Pointer to hold certificate data
 * @param length           Length of the certificate to be read
 * @param certificate_no   Certificate number
 * @return                 None
 * @note
 * ############################################################################
 * *****************************************************************************/
uint8_t spdm_read_certificate(uint32_t address, 
                                  uint8_t *buff_ptr,
                                  uint32_t length,
                                  uint8_t certificate_num)
{
    uint8_t ret_val = 1 ;
    if (certificate_num == 0 || certificate_num == 1)
    {
        if (address <= UINT16_MAX && length <= UINT16_MAX)
        {
            ret_val = SRAM_MBOX_API_devAK_cert_read((uint16_t)address, buff_ptr, certificate_num, (uint16_t)length);
        }
        else
        {
            //handle error
        }
    } 
    else 
    { 
        ret_val = di_spdm_spi_send_read_request(address, buff_ptr, length, SPI_SELECT_INT_COMP_0, true);
    }
    return ret_val;
}

/******************************************************************************/
/** function to update APFW data stored in MSR after reauth is done and calculate
 * concatanate hash
 * @param spdmContext - SPDM module context
 * @return void
 *******************************************************************************/
void spdm_update_ap_msr_data(SPDM_CONTEXT *spdmContext)
{
    pldm_ap_cleaned_on_reauth = 0;
    msr_ap_cleaned_on_reauth = 0;
    spdm_pkt_init_measurement_block(spdmContext);
}

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
                        uint8_t *out_bin_size)
{
    uint8_t final_size;

    #define SPDM_BIN_CONCAT_LABEL "spdm1.1 "

    if (safe_add_8((uint8_t)((label_size)&UINT8_MAX), sizeof(uint16_t) + sizeof(SPDM_BIN_CONCAT_LABEL) - 1, &final_size))
    {
        return;
    }
    if (context != NULL)
    {
       if (safe_add_8(final_size, hash_size, &final_size)) // TH hash size
       {
            return;
       }
    }
    
    *out_bin_size = final_size;

    memcpy(out_bin, &length, sizeof(uint16_t));
    memcpy(out_bin + sizeof(uint16_t), SPDM_BIN_CONCAT_LABEL, sizeof(SPDM_BIN_CONCAT_LABEL) - 1);
    memcpy((out_bin + sizeof(uint16_t) + (sizeof(SPDM_BIN_CONCAT_LABEL) - 1)), label, label_size);
    if (context != NULL) {
        memcpy((out_bin + sizeof(uint16_t) + (sizeof(SPDM_BIN_CONCAT_LABEL) - 1) + label_size), context, hash_size );
    }

}  

/******************************************************************************/
/** function to get length of certificate
 * @param cert_data - pointer to cert data
 * @return void
 *******************************************************************************/
uint16_t spdm_x509_get_cert_len(uint8_t *cert_data)
{
    uint8_t no_of_bytes_for_len;
    uint16_t temp;
    uint16_t length = 0U;

    no_of_bytes_for_len = cert_data[1] & 0x0F;
    memcpy(&length, &cert_data[2], no_of_bytes_for_len);

    if (no_of_bytes_for_len > 1U)
    {
        temp = (uint16_t)((((length & 0xff00) >> 8) | (length << 8))&UINT16_MAX);
        length = temp;
        if (safe_add_16(length, 4U, &length))
        {
            return FALSE;
        }
    } else { 
        if (safe_add_16(length, 3U, &length))
        {
            return FALSE;
        }
    }
    return length;
}

/******************************************************************************/
/** function to parse certificate
 * @param cert_data - pointer to cert data
 * @param len - has the length of certificate
 * @return void
 *******************************************************************************/
uint8_t spdm_x509_parse_certificate(uint8_t *cert_data, uint16_t len)
{
    uint16_t idx;
    uint16_t no_of_bytes_for_length;
    uint16_t r_term_len;
    uint16_t s_term_len;
    uint16_t temp;
    uint16_t raw_length_for_signature;
    uint8_t ret_val = 0;

    // certificate header
    idx = OFFSET_FOR_START;
    memcpy(&X509_struct.tbs_Certificate[0], &cert_data[idx], SERIAL_NUM_LEN);

    // serial number
    idx = idx + SERIAL_NUM_LEN;
    memcpy(&X509_struct.serial_number_start, &cert_data[idx++], CERT_PARSER_DEFAULT_VAL);
    memcpy(&X509_struct.serial_number_len, &cert_data[idx++], CERT_PARSER_DEFAULT_VAL);
    
    X509_struct.serial_number = &cert_data[idx];

    if (safe_add_16(idx, X509_struct.serial_number_len, &idx))
    {
        return FALSE;
    }

    // signature algorithm
    memcpy(&X509_struct.sig_algo[0], &cert_data[idx], SIG_ALGO_LEN);
    if (safe_add_16(idx, SIG_ALGO_LEN, &idx))
    {
        return FALSE;
    }
    // issuer 
    memcpy(&X509_struct.issuer_start, &cert_data[idx++], CERT_PARSER_DEFAULT_VAL);
    if (cert_data[idx] > 0x80)
    {
        no_of_bytes_for_length = cert_data[idx] & 0x0F;
        idx++;
        memcpy(&X509_struct.issuer_len, &cert_data[idx], no_of_bytes_for_length);
        
        if (no_of_bytes_for_length > 1)
        {
            temp = (uint16_t)((((X509_struct.issuer_len) >> BIT_8) | (X509_struct.issuer_len << BIT_8))&UINT16_MAX);
            X509_struct.issuer_len = temp;
        }
        idx = idx + no_of_bytes_for_length;
        X509_struct.issuer_data = &cert_data[idx];

        if (safe_add_16(idx, X509_struct.issuer_len, &idx))
        {
            return FALSE;
        }
    } else {
        memcpy(&X509_struct.issuer_len, &cert_data[idx++], CERT_PARSER_DEFAULT_VAL);
        X509_struct.issuer_data = &cert_data[idx];
        if (safe_add_16(idx, X509_struct.issuer_len, &idx))
        {
            return FALSE;
        }
    }

    // validity
    memcpy(&X509_struct.validity_start, &cert_data[idx++], CERT_PARSER_DEFAULT_VAL);
    memcpy(&X509_struct.validity_len, &cert_data[idx++], CERT_PARSER_DEFAULT_VAL);
    X509_struct.validity_data = &cert_data[idx];

    if (safe_add_16(idx, X509_struct.validity_len, &idx))
    {
        return FALSE;
    }

    // subject
    memcpy(&X509_struct.subject_start, &cert_data[idx++], CERT_PARSER_DEFAULT_VAL);
    if (cert_data[idx] > 0x80)
    {
        no_of_bytes_for_length = cert_data[idx] & 0x0F;
        idx++;
        memcpy(&X509_struct.subject_len, &cert_data[idx], no_of_bytes_for_length);
        if (no_of_bytes_for_length > 1)
        {
            temp = (uint16_t)((((X509_struct.subject_len) >> BIT_8) | (X509_struct.subject_len << BIT_8))&UINT16_MAX);
            X509_struct.subject_len = temp;
        }
        idx = idx + no_of_bytes_for_length;
        X509_struct.subject_data = &cert_data[idx];

        if (safe_add_16(idx, X509_struct.subject_len, &idx))
        {
            return FALSE;
        }
    } else {
        memcpy(&X509_struct.subject_len, &cert_data[idx++], CERT_PARSER_DEFAULT_VAL);
        X509_struct.subject_data = &cert_data[idx];
        if (safe_add_16(idx, X509_struct.subject_len, &idx))
        {
            return FALSE;
        }
    }

    if ((memcmp(X509_struct.issuer_data, X509_struct.subject_data, X509_struct.issuer_len)) == 0)
    {
        is_root_cert = true;
    } else {
        is_root_cert = false;
    }

    // public key info and public key
    memcpy(&X509_struct.spki_start, &cert_data[idx++], CERT_PARSER_DEFAULT_VAL);
    memcpy(&X509_struct.spki_len, &cert_data[idx++], CERT_PARSER_DEFAULT_VAL);
    memcpy(&X509_struct.spki_alg_oid_start, &cert_data[idx++], CERT_PARSER_DEFAULT_VAL);
    memcpy(&X509_struct.spki_alg_oid_len, &cert_data[idx++], CERT_PARSER_DEFAULT_VAL);

    X509_struct.spki_alg_oid_data = &cert_data[idx];
    if (safe_add_16(idx, X509_struct.spki_alg_oid_len, &idx))
    {
        return FALSE;
    }

    memcpy(&X509_struct.public_key_start, &cert_data[idx++], CERT_PARSER_DEFAULT_VAL);
    memcpy(&X509_struct.public_key_len, &cert_data[idx++], CERT_PARSER_DEFAULT_VAL);

    if (cert_data[idx++] == 0x00) // bypass 00 padding at start
    {
        idx++; // bypass 0x04
    }
    X509_struct.public_key = &cert_data[idx];

    idx = (uint16_t)((idx + (X509_struct.public_key_len - CERT_PARSE_PUB_KEY_SHIFT)) & UINT16_MAX);

    // extensions
    memcpy(&X509_struct.ext_start, &cert_data[idx], CERT_PARSER_DEFAULT_VAL);
    if (safe_add_16(idx, 1, &idx))
    {
        return FALSE;
    }
    if (X509_struct.ext_start == CERT_PARSE_EXTENSIONS) {
        if (cert_data[idx] > 0x80)
        {
            no_of_bytes_for_length = cert_data[idx++] & 0x0F;
            memcpy(&X509_struct.ext_len, &cert_data[idx], no_of_bytes_for_length);
            if (no_of_bytes_for_length > 1)
            {
                temp = (uint16_t)((X509_struct.ext_len >> BIT_8) | (X509_struct.ext_len << BIT_8) & UINT16_MAX);

                X509_struct.ext_len = temp;
            }
            idx = idx + no_of_bytes_for_length;
            X509_struct.ext_data = &cert_data[idx];
            if (safe_add_16(idx, X509_struct.ext_len, &idx))
            {
                return FALSE;
            }
        } else {
            memcpy(&X509_struct.ext_len, &cert_data[idx++], CERT_PARSER_DEFAULT_VAL);
            X509_struct.ext_data = &cert_data[idx];
            if (safe_add_16(idx, X509_struct.ext_len, &idx))
            {
                return FALSE;
            }
        }
    } else {
        idx--;
    }
    raw_length_for_signature = idx;
    // signature algorithm
    memcpy(&X509_struct.sig_alg, &cert_data[idx], SIG_ALGO_LEN);

    if (safe_add_16(idx, SIG_ALGO_LEN, &idx))
    {
        return FALSE;
    }

    // signature
    memcpy(&X509_struct.sig_start, &cert_data[idx++], CERT_PARSER_DEFAULT_VAL); // bit string
    memcpy(&X509_struct.sig_len, &cert_data[idx++], CERT_PARSER_DEFAULT_VAL); // length of signature, must be less that 0x7F

    if (safe_add_16(idx, 4u, &idx))
    {
        return FALSE;
    }

    r_term_len = cert_data[idx]; // get len of r-term

    if (safe_add_16(idx, 1u, &idx))
    {
        return FALSE;
    }

    if (cert_data[idx] == 0x00) // if extra 0x00 is padded before r-term, ignore it
    {
        if (safe_sub_16(r_term_len, 1, &r_term_len))
        {
            return FALSE;
        }
        idx++;
    }

    memcpy(&ecdsa_signature.signature_r_term[0], &cert_data[idx], r_term_len); // r term copied

    if (safe_add_16(idx, r_term_len, &idx))
    {
        return FALSE;
    }

    idx++; // bypasss 02

    s_term_len = cert_data[idx++]; // get len of r-term

    if (cert_data[idx] == 0x00) // if extra 0x00 is padded before r-term, ignore it
    {
        if (safe_sub_16(s_term_len, 1, &s_term_len))
        {
            return FALSE;
        }
        idx++;
    }

    memcpy(&ecdsa_signature.signature_s_term[0], &cert_data[idx], s_term_len); // s term copied

    if (is_root_cert == true)
    {
        memcpy(requester_public_key, X509_struct.public_key, PUB_KEY_CODE_LENGTH);
        spdm_crypto_ops_calc_hash(requester_public_key, PUB_KEY_CODE_LENGTH, spdmContext); // check hash of root pub key in APCFG is same as the one in certificate
        if (memcmp (&spdmContext->sha_digest[0], &root_pub_key_hash[0], SB_KEY_HASH_SIZE_MAX))
        {
            return FALSE;
        }
        spdm_crypto_ops_calc_hash(cert_data, len, spdmContext); //check root hash 
        if (memcmp (&spdmContext->sha_digest[0], &spi_data[OFFSET_FOR_ROOT_HASH], SB_KEY_HASH_SIZE_MAX))
        {
            return FALSE;
        }
    } 
    SPDM_CONTEXT *spdmContext = NULL;
    spdmContext = spdm_ctxt_get();
    if (NULL == spdmContext)
    {
        return FALSE;
    }

    if(safe_sub_16(raw_length_for_signature, RAW_LENGTH_OF_DATA_FOR_SIGN, &raw_length_for_signature))
    {
        return FALSE;
    }
    spdm_crypto_ops_calc_hash(&cert_data[RAW_LENGTH_OF_DATA_FOR_SIGN], raw_length_for_signature, spdmContext);

    ret_val = spdm_di_crypto_send_ecdsa_sign_verify_request(requester_public_key, ecdsa_signature.ecdsa_signature, &spdmContext->sha_digest[0], SHA_384_LEN, SB_AUTHALGO_ECDSA_P384, false);

    if (ret_val) 
    {
        return FALSE;
    }
    if (!is_root_cert) {
        memcpy(requester_public_key, X509_struct.public_key, PUB_KEY_CODE_LENGTH);
    }
   
    return TRUE;
}

/******************************************************************************/
/** function to parse certificate chain
 * @param None
 * @return void
 *******************************************************************************/
uint8_t spdm_x509_parse_certificate_chain()
{
    uint16_t length_of_chain = 0;
    uint8_t hash_of_root_cert_chain[SPDM_SHA384_LEN] = {0};
    uint16_t spdm_x509_cert_len = 0;
    uint16_t length_of_chain_to_be_parsed = 0;
    uint16_t offset_of_cert_start = 0;
    uint8_t ret_val = 0;

    memcpy(&length_of_chain, &spi_data[0], 2); // lenght of chain 2 bytes in spec
    memcpy(&hash_of_root_cert_chain[0], &spi_data[4], SPDM_SHA384_LEN);

    // compare the digest of certificate chain by calc hash and the digest recived in get_dgst
    spdm_crypto_ops_calc_hash(&spi_data[0], length_of_chain, spdmContext);
    if (memcmp(&spdmContext->sha_digest[0], &hash_of_req_chains[0], SPDM_SHA384_LEN))
    {
        return FALSE;
    }
    offset_of_cert_start = OFFSET_FOR_CERT_START;
    if (safe_sub_16(length_of_chain, offset_of_cert_start, &length_of_chain_to_be_parsed))
    {
        return FALSE;
    }

    while (length_of_chain_to_be_parsed > 0U) {
        spdm_x509_cert_len = spdm_x509_get_cert_len(&spi_data[offset_of_cert_start]);
        ret_val = spdm_x509_parse_certificate(&spi_data[offset_of_cert_start], spdm_x509_cert_len);
        if (!ret_val)
        {
            return ret_val;
        }
        length_of_chain_to_be_parsed = length_of_chain_to_be_parsed - spdm_x509_cert_len;
        offset_of_cert_start = offset_of_cert_start + spdm_x509_cert_len;
    }

    return ret_val;
}

/******************************************************************************
* Cleans up keys, major secrets derivated
* @param spdmContext pointer to spdmContext
* @return None
*******************************************************************************/
void spdm_secure_session_cleanup(SPDM_CONTEXT *spdmContext)
{
    if (spdmContext == NULL)
    {
        return;
    }
    memset(&(spdmContext->secure_session_info), 0, sizeof(SECURE_SESSION_INFO));
    memset(&ss_pub_key[0], 0, PUB_KEY_CODE_LENGTH);
    memset(&ss_pvt_key[0], 0, PVT_KEY_CODE_LENGTH);
    final_session_id = 0;
}

/******************************************************************************/
/** spdm_secure_session_set_session_state();
* Sets secure session state
* @param spdmContext pointer to spdmContext
* @param val state value to be set
* @return None
*******************************************************************************/
void spdm_secure_session_set_session_state(SPDM_CONTEXT *spdmContext, SPDM_SESSION_STATE val)
{
    if (spdmContext == NULL)
    {
        return;
    }
    spdmContext->secure_session_info.session_state = val;
    return;
}

/******************************************************************************/
/** SPDMHB_timer_callback();
* SPDMHB timer callback
* @param TimerHandle_t pxTimer
* @return None
*******************************************************************************/
void SPDMHB_timer_callback(TimerHandle_t pxTimer)
{
    SPDM_CONTEXT *spdmContext = NULL;
    spdmContext = spdm_ctxt_get();

    if (NULL == spdmContext)
    {
        return;
    }
    if (SPDMHB_timer_started) 
    {
        spdm_hb_response_timeout_stop();
        SPDMHB_timer_started = false;
    }
    spdm_secure_session_cleanup(spdmContext);
}
