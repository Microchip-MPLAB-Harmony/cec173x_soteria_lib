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


#define SPI_DATA_MAX_BUFF 4096U

extern SPDM_BSS1_ATTR uint8_t curr_ec_id;
SPDM_BSS1_ATTR uint8_t hw_config_buff[SRAM_MBOX_HW_CONFIG_SIZE] __attribute__((aligned(8)));

SPDM_BSS1_ATTR uint8_t get_mctp_pld[MAX_SIZE_CERTIFICATE] __attribute__((aligned(8)));
SPDM_BSS1_ATTR MCTP_PKT_BUF mctp_pktbuf_tx;
SPDM_BSS1_ATTR MCTP_PKT_BUF spdm_pktbuf_rx;

SPDM_BSS1_ATTR MCTP_PKT_BUF spdm_pktbuf[1] __attribute__((aligned(8)));
SPDM_BSS1_ATTR uint8_t spdm_tx_state;
SPDM_BSS1_ATTR bool pldm_first_pkt;

SPDM_BSS0_ATTR uint8_t spi_data[SPI_DATA_MAX_BUFF] __attribute__((aligned(8))); // This buffer is externed and used to read/Write certificate
SPDM_BSS1_ATTR VERSION_NUM_ENTRY_TABLE version_tbl_buf[2 * SPDM_VER_NUM_ENTRY_COUNT];
SPDM_BSS1_ATTR REQ_BASE_ASYM_ALG struct_algo;
SPDM_BSS1_ATTR uint8_t spdm_rqst_cur_state;
SPDM_BSS1_ATTR MCTP_PKT_BUF *mctp_buf_tx;
SPDM_BSS1_ATTR MCTP_PKT_BUF *spdm_buf_tx;
SPDM_BSS1_ATTR uint32_t packet_sz;
SPDM_BSS1_ATTR uint8_t pkt_seq_mctp;
SPDM_BSS1_ATTR uint8_t first_pkt;
SPDM_BSS1_ATTR uint8_t tbl_entry;
SPDM_BSS1_ATTR uint16_t pld_index;
SPDM_BSS1_ATTR uint16_t time_stamp;
SPDM_BSS1_ATTR uint32_t cert_len;

SPDM_BSS1_ATTR uint32_t cert2_base_addr;
SPDM_BSS1_ATTR uint8_t chain_avail_bits_mask;
SPDM_BSS1_ATTR uint8_t slot_position;

SPDM_BSS1_ATTR CERT_SLOT slot_buf[MAX_SLOTS]; // 8 slots structure; each slot containing a chain

SPDM_BSS1_ATTR CERTIFICATE cert_buf[MAX_CERTIFICATES]; // 64 certificates structure

// For challenge authentication, required buffers
SPDM_BSS1_ATTR uint8_t pvt_key[PVT_KEY_CODE_LENGTH]__attribute__((aligned(
            4))); // This buffer holds AK_PVT_KEY / AK_PVT_KEY_CODE

SPDM_BSS1_ATTR ecdsa_signature_t ecdsa_signature __attribute__((aligned(8)));

// end of challenge authentication, required buffers
SPDM_BSS0_ATTR uint8_t random_no[CURVE_384_SZ];
//--GET CERTIFICATE RELATED VARIABLES AND BUFFERS ---//
SPDM_BSS1_ATTR uint8_t AP_CFG_cert_buffer[sizeof(CFG_CERT)];
SPDM_BSS1_ATTR CFG_CERT *ptr_cert_buffer;

SPDM_BSS1_ATTR uint8_t requested_slot;
SPDM_BSS1_ATTR uint16_t cert_offset_to_sent;
SPDM_BSS1_ATTR uint16_t RemainderLength;
SPDM_BSS1_ATTR uint16_t PortionLength;
SPDM_BSS1_ATTR uint16_t opaq_size_remaining;
SPDM_BSS1_ATTR volatile uint8_t first_get_cert_response_sent;
SPDM_BSS1_ATTR volatile uint16_t bytes_sent_over_mctp_for_cert;
SPDM_BSS1_ATTR volatile uint16_t remaining_bytes_to_sent;
SPDM_BSS1_ATTR uint16_t pending_bytes;
SPDM_BSS1_ATTR uint16_t global_offset_for_pending_bytes;

// CHALLENGE COMMAND RELATED VARIABLES AND BUFFERS//

SPDM_BSS1_ATTR uint8_t nonce_data[NOUNCE_DATA_SIZE];
SPDM_BSS1_ATTR uint8_t first_challenge_resp_sent;
SPDM_BSS1_ATTR uint8_t digest_state_machine;
SPDM_BSS1_ATTR uint8_t nonce_offset;
SPDM_BSS1_ATTR uint16_t signature_offset;

SPDM_BSS1_ATTR uint8_t hash_of_req_buffer[SPDM_SHA384_LEN] __attribute__((aligned(8)));
SPDM_BSS1_ATTR uint8_t hash_of_chains[MAX_SHA384_BUF_SIZE] __attribute__((aligned(8)));

SPDM_BSS1_ATTR uint16_t req_and_response_sz[2];

// response fields pointers for each command responses
SPDM_BSS1_ATTR GET_VERSION_RESP_FIELDS get_ver_resp_object;
SPDM_BSS1_ATTR uint8_t get_version_response[sizeof(get_ver_resp_object)];

SPDM_BSS1_ATTR GET_CAP_RESP_FIELDS get_cap_resp_object;
SPDM_BSS1_ATTR uint8_t get_cap_response[sizeof(get_cap_resp_object)];

SPDM_BSS1_ATTR NEG_ALGO_RESP_FIELDS nego_algo_resp_object;
SPDM_BSS1_ATTR uint8_t nego_algo_response[sizeof(nego_algo_resp_object)];

SPDM_BSS1_ATTR GET_DIGEST_FIELDS get_digest_resp_object;
SPDM_BSS1_ATTR uint8_t get_digest_response[sizeof(get_digest_resp_object)];

SPDM_BSS1_ATTR GET_CERTIFICATE_FIELDS get_cert_resp_object;
SPDM_BSS1_ATTR uint8_t get_cert_response[sizeof(get_cert_resp_object)];

SPDM_BSS1_ATTR CHALLENGE_FIELDS challenge_resp_object;
SPDM_BSS1_ATTR uint8_t challenge_response_buff[sizeof(challenge_resp_object)];

SPDM_BSS2_ATTR MEASUREMENT_FIELDS measurement_resp_object;
SPDM_BSS2_ATTR uint8_t measurement_response_buff[sizeof(MEASUREMENT_FIELDS) - OPAQUE_DATA_SZ - OPAQUE_DATA_LEN];

//-----MEASUREMENT COMMAND OPERATIONS RELATED BUFFERS AND VARIABLES------//;

// concatenated measurement blocks structure
SPDM_BSS1_ATTR MEASUREMENT_BLOCK msr_block_buffer[NO_OF_MSR_INDICES];

// hash of all msr block
SPDM_BSS1_ATTR uint8_t hash_concat_msrments_val[SPDM_SHA384_LEN];

// hash of all msr block
SPDM_BSS1_ATTR uint8_t hash_concat_tcb_msrments_val[SPDM_SHA384_LEN];

// will have all msr blocks concatenated for sending in response
SPDM_BSS1_ATTR uint8_t concat_msrment_blocks[SIZE_OF_ONE_MSR_BLOCK * NO_OF_MSR_INDICES];

SPDM_BSS1_ATTR uint8_t msr_opaque_buf[OPAQUE_DATA_SZ + OPAQUE_DATA_LEN];

SPDM_BSS1_ATTR MEASUREMENT_VARIABLES measurement_var;

void read_chain_from_offset(uint16_t start_offset, uint16_t end_offset, uint32_t spi_data_offset, uint8_t slot);
SPDM_BSS1_ATTR uint16_t read_bytes_from_chain;
SPDM_BSS1_ATTR uint16_t START_OFFSET_IN_BUFFER;
SPDM_BSS1_ATTR uint16_t BUFFER_END_OFFSET;
SPDM_BSS1_ATTR uint16_t length;

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
                get_tail_ptr = 0;
                offset = ROOT_START_OFFSET;
                get_mem = 0;
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
                    spdm_read_certificate(0, &spi_data[offset], 4, current_cert_ptr);
                        // spi_data[offset + 2] = 0xFF;
                    cert_chain_valid_status |= spdm_pkt_update_cert_data_len(offset, &cert_buf[current_cert_ptr].cert_size);
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

                    spdm_crypto_ops_run_time_hashing(&spi_data[0U], 0, spdmContext);

                    // Step 1 : Compute run-time hash of chain-length(2 bytes), reserved(2 bytes), roothash(48 bytes), Root certificate
                    spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
                    if(offset > UINT16_MAX)
                    {
                        // handle error
                    }
                    else
                    {
                        req_and_response_sz[1] = (uint16_t)(offset); // size of chain-length(2 bytes) + reserved(2 bytes) + roothash(48 bytes) + Root certificate
                    }
                    spdm_get_len_for_runtime_hash(spdmContext);
                    spdm_crypto_ops_run_time_hashing(&spi_data[0U], length, spdmContext);
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
                        spdm_crypto_ops_run_time_hashing(&spi_data[offset], length, spdmContext);

                        get_tail_ptr = current_cert_ptr;
                        current_cert_ptr = cert_buf[get_tail_ptr].tail_ptr_val;
                    }

                        uint16_t size = 0U;
                        if(cert_buf[current_cert_ptr].cert_size > UINT16_MAX)
                        {
                            // Handle Error
                        }
                        else
                        {
                            size = (uint16_t)(cert_buf[current_cert_ptr].cert_size);
                        }
                        spdm_read_certificate(0, &spi_data[offset], size, current_cert_ptr);
                        req_and_response_sz[1] = cert_buf[current_cert_ptr].cert_size;
                        spdm_get_len_for_runtime_hash(spdmContext);
                        spdm_crypto_ops_run_time_hashing(&spi_data[offset], length, spdmContext);


                    spdmContext->get_requests_state = END_OF_HASH;
                    spdm_crypto_ops_run_time_hashing(NULL, 0, spdmContext);
                    memcpy(&hash_of_chains[SPDM_SHA384_LEN * slot], &spdmContext->sha_digest[0], SPDM_SHA384_LEN);
                }
                else
                {
                    /* Ideally, Set hash of cert chain to 0U since one of the cert is invalid
                    However, since variable is global, which is initialized to 0U by default, memset to 0U is not needed */
                }

            }
        }
        spdmContext->spdm_state_info = SPDM_CMD_PROCESS_MODE;
    }
    else
    {
        spdmContext->spdm_state_info = SPDM_IDLE;
    }
    spdm_di_spi_tristate(INT_SPI);
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
            // Handle Error
        }
        // concatenate first block
        memcpy(&concat_msrment_blocks[indx], (uint8_t *)&msr_block_buffer[indx], num_of_bytes_in_current_block);

        measurement_var.msr_record_size = num_of_bytes_in_current_block;

        req_and_response_sz[1] = msr_block_buffer[indx].msr_size[0];
        spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
        spdm_get_len_for_runtime_hash(spdmContext);
        spdm_crypto_ops_run_time_hashing((uint8_t *)&msr_block_buffer[indx].msr_frmt, length, spdmContext);
    }
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
        spdm_crypto_ops_run_time_hashing((uint8_t *)&msr_block_buffer[indx].msr_frmt, length, spdmContext);
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
    spdm_crypto_ops_run_time_hashing((uint8_t *)&msr_block_buffer[indx].msr_frmt, length, spdmContext);
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
    spdm_crypto_ops_run_time_hashing((uint8_t *)&msr_block_buffer[indx].msr_frmt, length, spdmContext);
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
    // calculate hash of all msr blocks
    // switch hash engine to init mode first for run time hashing
    spdmContext->get_requests_state = HASH_INIT_MODE;
    spdm_crypto_ops_run_time_hashing(NULL, 0, spdmContext);
    spdmContext->request_or_response = 1; // hash data is meant for responses

    spdm_pkt_init_immutable_rom_block(spdmContext);
    spdm_pkt_init_mutable_fw(spdmContext);
    spdm_pkt_init_hwconfig(spdmContext);
    spdm_pkt_init_ecfwconfig(spdmContext);

    spdmContext->get_requests_state = END_OF_HASH;
    spdm_crypto_ops_run_time_hashing(NULL, 0, spdmContext);
    memcpy(&hash_concat_msrments_val[0], &spdmContext->sha_digest, SPDM_SHA384_LEN);

    // calculate hash of tcb blocks alone: immutable rom and hw config
    tcb0_len = MSR_BLOCK_DMTF_FLDS_SZ + msr_block_buffer[(INDEX1 - 1)].msr_frmt.dmtf_msr_val_size[1];
    if (is_add_safe(tcb0_len, MSR_BLOCK_DMTF_FLDS_SZ) == 0) // Coverity INT30-C fix
    {
        // Handle Error
    }
    tcb1_len = MSR_BLOCK_DMTF_FLDS_SZ + msr_block_buffer[(INDEX3 - 1)].msr_frmt.dmtf_msr_val_size[1];
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
    spdm_pktbuf[0].rx_smbus_timestamp = 0;
    spdm_tx_state = SPDM_TX_IDLE;
    // initialize table entry buffers
    for (iter = 0; iter < SPDM_VER_NUM_ENTRY_COUNT; iter++)
    {
        version_tbl_buf[iter].major_ver = SPDM_VER_NUM_MAJOR_MINOR + iter;
        version_tbl_buf[iter].update_ver_no = SPDM_VER_NUM_UPDATE_ALPHA;
        get_ver_resp_object.table_entry[iter].table_value[0] = version_tbl_buf[iter].update_ver_no;
        get_ver_resp_object.table_entry[iter].table_value[1] = version_tbl_buf[iter].major_ver;
    }
    packet_sz = 0x00;
    pkt_seq_mctp = 0x00;
    first_pkt = true;
    pldm_first_pkt = true;
    tbl_entry = 0x00;
    bytes_sent_over_mctp_for_cert = 0x00;
    struct_algo.AlgType = ALG_TYPE;
    struct_algo.AlgCount = ALG_CNT;
    struct_algo.AlgSupported = TPM_ALG_ECDSA_ECC_NIST_P384;
    first_get_cert_response_sent = 0;
    first_challenge_resp_sent = END_OF_TX;
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
    spdmContext->pldm_state_info = PLDM_IDLE;
    spdmContext->current_pkt_sequence = 0;

    spdmContext->get_requests_state = HASH_INIT_MODE;

    memset(&measurement_var, 0, sizeof(MEASUREMENT_VARIABLES));
    spdmContext->challenge_success_flag = 0;
    spdm_pkt_init_measurement_block(spdmContext);

    spdmContext->pldm_previous_state = PLDM_IDLE_STATE;
    spdmContext->pldm_current_state = PLDM_IDLE_STATE;
    spdmContext->pldm_status_reason_code = INITIALIZATION_OF_FD;
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
        spdm_crypto_ops_run_time_hashing(&spi_data[0], length, spdmContext);

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
                spdm_crypto_ops_run_time_hashing(&spi_data[0], length, spdmContext);
                if (pending_size > 0) {
                    memcpy(&temp[size_available_in_buffer], &spi_data[0], pending_size); // Get rest of data into temp buffer from spi_data buffer
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
                spdm_crypto_ops_run_time_hashing(&spi_data[0], length, spdmContext);


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

    if (NULL == spdmContext)
    {
        return;
    }

    switch (first_challenge_resp_sent)
    {
    case TXSTATE_1:
        // add 64 - 53 = 11 bytes of nonce_data
        for (uint8_t i = 0; i < (MAX_NUM_BYTES_PLD - (5 + SPDM_SHA384_LEN)); i++)
        {
            spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 2 + SPDM_SHA384_LEN + i] = nonce_data[NOUNCE_DATA_SIZE - 1 - i];
        }

        nonce_offset = (MAX_NUM_BYTES_PLD - (5 + SPDM_SHA384_LEN)); // 11 bytes copied

        // 32 - 11 = 21 bytes of nonce to sent
        remaining_bytes_to_sent = NOUNCE_DATA_SIZE - nonce_offset;
        first_challenge_resp_sent = TXSTATE_2;

        break;
    case TXSTATE_2:
        for (uint16_t i = 0; i < remaining_bytes_to_sent; i++)
        {
            spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + i] = nonce_data[NOUNCE_DATA_SIZE - 1 - nonce_offset - i];
        }
        // 21 bytes of nonce copied
        nonce_offset = 0;

        // if measurement hash summary field present
        if (measurement_var.msr_hash_rqstd)
        {
            if (measurement_var.msr_hash_rqstd == 0xff)
            {
                // measurement hash bytes that can be copied is  64 - 21 = 43
                memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + remaining_bytes_to_sent], &hash_concat_msrments_val[0],
                       MAX_NUM_BYTES_PLD - remaining_bytes_to_sent);
            }
            else
            {
                // measurement hash bytes that can be copied is 64 - 21 = 43
                memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + remaining_bytes_to_sent], &hash_concat_tcb_msrments_val[0],
                       MAX_NUM_BYTES_PLD - remaining_bytes_to_sent);
            }
            // 43 bytes of msr hash summary is copied
            measurement_var.msr_hash_sum_offset = MAX_NUM_BYTES_PLD - remaining_bytes_to_sent;

            // 48-43 = 5 bytes of msr hash summary remaining
            remaining_bytes_to_sent = SPDM_SHA384_LEN - (measurement_var.msr_hash_sum_offset);
            first_challenge_resp_sent = TXSTATE_3;
        }
        else
        {
            opaq_size_remaining = 256U;
            spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + remaining_bytes_to_sent] = 0x00;       // opaquelen1
            spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS + remaining_bytes_to_sent] = 0x01; // opaquelen2
            remaining_bytes_to_sent = remaining_bytes_to_sent + 2U;
            if (is_add_safe(remaining_bytes_to_sent, 2U) == 0)
            {
                // Handle Error
            }
            spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS + remaining_bytes_to_sent + 1] = 0x00; // opaquedata1
            spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS + remaining_bytes_to_sent + 2] = 0x00; // opaquedata2

            memset(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + remaining_bytes_to_sent], 0,
                   (MAX_NUM_BYTES_PLD - remaining_bytes_to_sent));

            if (safe_subraction_16(opaq_size_remaining, (uint16_t)(MAX_NUM_BYTES_PLD - remaining_bytes_to_sent),
                            &opaq_size_remaining)) // Coverity INT31-C
            {
                // Handle Error
                break;
            }
            first_challenge_resp_sent = TXSTATE_5;
        }

        break;
    case TXSTATE_3:
        // if measurement hash summary field present
        if (measurement_var.msr_hash_rqstd)
        {
            if (measurement_var.msr_hash_rqstd == 0xff)
            {
                // remaining 5 bytes of msr hash summary copied
                memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS], &hash_concat_msrments_val[measurement_var.msr_hash_sum_offset],
                       remaining_bytes_to_sent);
            }
            else
            {
                // remaining 5 bytes of msr hash summary copied
                memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS], &hash_concat_tcb_msrments_val[measurement_var.msr_hash_sum_offset],
                       remaining_bytes_to_sent);
            }
            measurement_var.msr_hash_sum_offset = 0;

            opaq_size_remaining = 256U;
            spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + remaining_bytes_to_sent] = 0x00;     // opaquelen1
            spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + remaining_bytes_to_sent + 1] = 0x01; // opaquelen2
            remaining_bytes_to_sent = remaining_bytes_to_sent + 2U;
            if (is_add_safe(remaining_bytes_to_sent, 2U) == 0)
            {
                // Handle Error
            }

            memset(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + remaining_bytes_to_sent], 0,
                   (MAX_NUM_BYTES_PLD - remaining_bytes_to_sent));
            uint16_t temp = (uint16_t)((MAX_NUM_BYTES_PLD - remaining_bytes_to_sent)&UINT16_MAX);
            uint16_t temp1 = (uint16_t)((opaq_size_remaining - temp)&UINT16_MAX);
            if (temp1 > UINT16_MAX) // Coverity INT31-C
            {
                // handle error
            }
            else
            {
                temp = (uint16_t)(((MAX_NUM_BYTES_PLD - remaining_bytes_to_sent))&UINT16_MAX);
                temp1 = (uint16_t)(( opaq_size_remaining - temp)&UINT16_MAX);
                opaq_size_remaining = temp1;
            }
            first_challenge_resp_sent = TXSTATE_5;
        }
        else
        {
            // remaining 9 bytes of r term copied
            memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS], &ecdsa_signature.signature_r_term[signature_offset],
                   remaining_bytes_to_sent);
            signature_offset = 0;

            // remaining_bytes_to_sent - s term alone
            // 48 bytes
            memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + remaining_bytes_to_sent],
                   &ecdsa_signature.signature_s_term[signature_offset], SIGNATURE_S_TERM_SZ);

            packet_sz = remaining_bytes_to_sent + SIGNATURE_S_TERM_SZ + MCTP_TOT_HDR_SZ;
            first_challenge_resp_sent = END_OF_TX;
        }

        break;
    case TXSTATE_5:
        memset(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS], 0, MAX_NUM_BYTES_PLD);
        if (is_sub_safe(opaq_size_remaining, MAX_NUM_BYTES_PLD) == 0)
        {
            // Handle Error
            break;
        }
        else
        {
            opaq_size_remaining = opaq_size_remaining - MAX_NUM_BYTES_PLD;
        }
        remaining_bytes_to_sent = 1;
        if (opaq_size_remaining <= MAX_NUM_BYTES_PLD)
        {
            first_challenge_resp_sent = TXSTATE_6;
        }
        else
        {
            first_challenge_resp_sent = TXSTATE_5;
        }
        break;
    case TXSTATE_6:
        offset = 0U;
        // Copy remaining  bytes of Opaq
        memset(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS], 0, opaq_size_remaining);
        if (opaq_size_remaining > UINT8_MAX) // Coverity INT31-C Fix
        {
            // handle error
        }
        else
        {
            offset = opaq_size_remaining;
        }
        if (measurement_var.msr_hash_rqstd)
        {
            // signature r bytes that can be copied is - 48
            if(sizeof(MCTP_PACKET) > (SPDM_MSG_TYPE_POS + offset) )
            {
                memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + offset], &ecdsa_signature.signature_r_term[0], SIGNATURE_R_TERM_SZ);
                offset += SIGNATURE_R_TERM_SZ;
            }
            else
            {
                break;
            }

            if (is_sub_safe(MAX_NUM_BYTES_PLD, offset) == 0)
            {
                // Handle Error
                break;
            }
            else
            {
                // we can copy 14 bytes of s term
                memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + offset], &ecdsa_signature.signature_s_term[0],
                       MAX_NUM_BYTES_PLD - offset);
                if (is_sub_safe(SIGNATURE_S_TERM_SZ, (MAX_NUM_BYTES_PLD - offset)) == 0)
                {
                    // Handle Error
                    break;
                }
                else
                {
                    remaining_bytes_to_sent = SIGNATURE_S_TERM_SZ - (MAX_NUM_BYTES_PLD - offset); // s term rem bytes
                }
            }
        }
        else
        {
            if (is_sub_safe(MAX_NUM_BYTES_PLD, offset) == 0) // Coverity INT30-C
            {
                // Handle Error
                break;
            }
            else
            {
                memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + offset], &ecdsa_signature.ecdsa_signature[0],
                       MAX_NUM_BYTES_PLD - offset);
                remaining_bytes_to_sent = (CURVE_384_SZ * 2) - (MAX_NUM_BYTES_PLD - offset); // s term rem bytes
            }
        }

        first_challenge_resp_sent = TXSTATE_4;

        break;
    case TXSTATE_4:
        // if measurement hash summary field present
        if (measurement_var.msr_hash_rqstd)
        {
            // remaining 34 bytes of s term copied
            memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS],
                   &ecdsa_signature.signature_s_term[SIGNATURE_S_TERM_SZ - remaining_bytes_to_sent], remaining_bytes_to_sent);
            signature_offset = 0;
            packet_sz = remaining_bytes_to_sent + MCTP_TOT_HDR_SZ;
            first_challenge_resp_sent = END_OF_TX;
        }
        else
        {
            memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS],
                   &ecdsa_signature.ecdsa_signature[(CURVE_384_SZ * 2) - remaining_bytes_to_sent], remaining_bytes_to_sent);
            signature_offset = 0;
            packet_sz = remaining_bytes_to_sent + MCTP_TOT_HDR_SZ;
            first_challenge_resp_sent = END_OF_TX;
        }
        break;
    default:
        first_challenge_resp_sent = END_OF_TX;
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
        spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + spdm_buf_bytes + i] = nonce_data[NOUNCE_DATA_SIZE - 1 - offset - i];
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
/** Function to fill spdm_buffer with Opaq data
 * @param spdm_buf_bytes  - Offset in spdm_buffer to start filling Opaq data
 * @param offset          - start copying from offset specified by this field from opaq_data buffer
 * @param bytes_to_sent   - Number of bytes to copy
 * @return spdm_buf_bytes - Total Number of bytes of Opaq data copied (Inclusive of all transaction, not just the current one)
 *******************************************************************************/
uint16_t spdm_pkt_fill_opaque(uint16_t spdm_buf_bytes, uint16_t offset, uint16_t bytes_to_sent)
{
    for (uint16_t i = 0; i < bytes_to_sent; i++)
    {
        spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + spdm_buf_bytes + i] = msr_opaque_buf[i + offset];
    }

    spdm_buf_bytes = spdm_buf_bytes + bytes_to_sent;

    return spdm_buf_bytes;
}

/******************************************************************************/
/** Function to fill spdm_buffer with r-term of signature
 * @param spdm_buf_bytes  - Offset in spdm_buffer to start filling r-term
 * @param bytes_to_sent   - Number of bytes to copy
 * @return spdm_buf_bytes - Total Number of bytes of r-term data copied (Inclusive of all transaction, not just the current one)
 *******************************************************************************/
uint16_t spdm_pkt_fill_signature_r(uint16_t spdm_buf_bytes, uint16_t bytes_to_sent)
{
    if (is_sub_safe(MAX_NUM_BYTES_PLD, spdm_buf_bytes) == 1)
    {
        // calculate remaining bytes present for signature
        remaining_bytes_to_sent = MAX_NUM_BYTES_PLD - spdm_buf_bytes;
    }
    else
    {
        // Handle error
    }

    if (remaining_bytes_to_sent >= bytes_to_sent) // if remaining bytes that can be sent can fill in signature bytes
    {
        memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + spdm_buf_bytes], &ecdsa_signature.signature_r_term[signature_offset],
               bytes_to_sent);
        signature_offset = signature_offset + bytes_to_sent; // increment offset for the bytes sent
        spdm_buf_bytes = spdm_buf_bytes + bytes_to_sent;
    }
    else
    {
        // less than signature bytes
        memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + spdm_buf_bytes], &ecdsa_signature.signature_r_term[signature_offset],
               remaining_bytes_to_sent);
        signature_offset = signature_offset + remaining_bytes_to_sent; // increment offset for the bytes sent
        spdm_buf_bytes = spdm_buf_bytes + remaining_bytes_to_sent;
    }
    return spdm_buf_bytes;
}

/******************************************************************************/
/** Function to fill spdm_buffer with r-term of signature
 * @param spdm_buf_bytes  - Offset in spdm_buffer to start filling r-term
 * @param bytes_to_sent   - Number of bytes to copy
 * @return spdm_buf_bytes - Total Number of bytes of s-term copied (Inclusive of all transaction, not just the current one)
 *******************************************************************************/
uint16_t spdm_pkt_fill_signature_s(uint16_t spdm_buf_bytes, uint16_t bytes_to_sent)
{
    if (is_sub_safe(MAX_NUM_BYTES_PLD, spdm_buf_bytes) == 1)
    {
        // calculate remaining bytes present for signature
        remaining_bytes_to_sent = MAX_NUM_BYTES_PLD - spdm_buf_bytes;
    }
    else
    {
        // Handle error
    }
    if (remaining_bytes_to_sent >= bytes_to_sent) // if remaining bytes that can be sent can fill in signature bytes
    {
        memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + spdm_buf_bytes], &ecdsa_signature.signature_s_term[signature_offset],
               bytes_to_sent);
        if((signature_offset + bytes_to_sent) > UINT16_MAX) // Coverity INT31-C Fix
        {
            // handle error
        }
        else
        {
            signature_offset = signature_offset + bytes_to_sent; // increment offset for the bytes sent
        }

        if((spdm_buf_bytes + bytes_to_sent) > UINT16_MAX) // Coverity INT31-C Fix
        {
            // handle error
        }
        else
        {
            spdm_buf_bytes = (uint16_t)(spdm_buf_bytes + bytes_to_sent);
        }
    }
    else
    {
        // less than signature bytes
        memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + spdm_buf_bytes], &ecdsa_signature.signature_s_term[signature_offset],
               remaining_bytes_to_sent);
        if((signature_offset + remaining_bytes_to_sent) > UINT16_MAX) // Coverity INT31-C Fix
        {
            // handle error
        }
        else
        {
            signature_offset = (uint16_t)(signature_offset + remaining_bytes_to_sent); // increment offset for the bytes sent
        }

        if((spdm_buf_bytes + remaining_bytes_to_sent) > UINT16_MAX) // Coverity INT31-C Fix
        {
            // handle error
        }
        else
        {
            spdm_buf_bytes = (uint16_t)(spdm_buf_bytes + remaining_bytes_to_sent);
        }
    }
    return spdm_buf_bytes;
}

/******************************************************************************/
/** Handle tx packet buffering when responding to GET_MEASUREMENT request for either of states - all blocks or requested block
 * @param buf_ptr- buffer containing concatenated msr blocks or specific msr block as per index
 * @return void
 *******************************************************************************/
void spdm_pkt_fill_msr_record_stage1(uint8_t *buf_ptr)
{
    if (buf_ptr == NULL)
    {
        return;
    }
    measurement_var.concat_msr_offset = 0;
    measurement_var.msr_rcrd_bytes_to_sent = 0;
    uint8_t read_from_offset = 0U;
    remaining_bytes_to_sent = MAX_NUM_BYTES_PLD - (MSR_BYTES_SZ_TILL_RECRD_LEN + SPDM_MSG_PLD_SPECIFIC_BYTES);
    if(measurement_var.msr_operation <= NO_OF_MSR_INDICES)
    {
        if(((measurement_var.msr_operation - 1U) * SIZE_OF_ONE_MSR_BLOCK) > UINT8_MAX) // Coverity INT31-C Fix
        {
            // handle error
        }
        else
        {
            read_from_offset = (uint8_t)((measurement_var.msr_operation - 1U) * SIZE_OF_ONE_MSR_BLOCK);
        }
    }
    if (remaining_bytes_to_sent >= measurement_var.msr_rcrd_len)
    {
        memcpy(&spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 6], &buf_ptr[read_from_offset], measurement_var.msr_rcrd_len);
        measurement_var.concat_msr_offset = measurement_var.msr_rcrd_len;
        // remaining msr_rcrd bytes
        measurement_var.msr_rcrd_bytes_to_sent = 0;
    }
    else
    {
        memcpy(&spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 6], &buf_ptr[read_from_offset], remaining_bytes_to_sent);
        measurement_var.concat_msr_offset = remaining_bytes_to_sent;
        measurement_var.msr_rcrd_bytes_to_sent = measurement_var.msr_rcrd_len - remaining_bytes_to_sent;
    }
    if (measurement_var.msr_rcrd_bytes_to_sent)
    {
        measurement_var.msr_response_byte_iter = FILL_REM_RCRD;
    }
    else
    {
        measurement_var.msr_response_byte_iter = FILL_MSR_RND;
    }
}

/******************************************************************************/
/** Handle tx packet buffering when responding to GET_MEASUREMENT request - all blocks or requested block
 * handles random , opaque and signature buffers to be updated to spdm tx buffer and change state if buffer size reaches max 64
 * @param rnd_size : size of random bytes pending to be sent
 * @param opq_size : size of opaq data pending to be sent
 * @return void
 *******************************************************************************/
void spdm_pkt_fill_msr_record_stage2(uint16_t rnd_size, uint16_t opq_size, uint8_t bytes_to_start)
{
    uint8_t bytes_to_be_transferred;
    uint8_t num_of_bytes = bytes_to_start; // This is the counter which tracks number of bytes occupied in spdm response
    if (rnd_size)
    {
        measurement_var.msr_bytes_copied = spdm_pkt_fill_from_rnd_no(num_of_bytes, nonce_offset, rnd_size);
        num_of_bytes = num_of_bytes + rnd_size;
        if (is_add_safe(num_of_bytes, rnd_size) == 0)
        {
            // Handle Error
            return;
        }
    }
    if (opq_size)
    {
        if (is_sub_safe(MAX_NUM_BYTES_PLD, num_of_bytes) == 1)
        {
            remaining_bytes_to_sent = MAX_NUM_BYTES_PLD - num_of_bytes;
        }
        else
        {
            // Handle error
        }
        if (opq_size > MAX_NUM_BYTES_PLD)
        {
            bytes_to_be_transferred = MAX_NUM_BYTES_PLD;
        }
        else
        {
            bytes_to_be_transferred = opq_size;
        }
        // if bytes_to_be_transferred is greater than remaining_bytes than can be sent; Send max possible value
        bytes_to_be_transferred = bytes_to_be_transferred > remaining_bytes_to_sent ? remaining_bytes_to_sent :
                                  bytes_to_be_transferred;

        measurement_var.msr_bytes_copied = spdm_pkt_fill_opaque(num_of_bytes, measurement_var.opq_offset,
                                           bytes_to_be_transferred);
        measurement_var.opq_offset = measurement_var.opq_offset + bytes_to_be_transferred;
        if (is_add_safe(measurement_var.opq_offset, bytes_to_be_transferred) == 0)
        {
            // Handle Error
            return;
        }
        num_of_bytes = num_of_bytes + bytes_to_be_transferred;
        if(is_add_safe(num_of_bytes, bytes_to_be_transferred) == 0)
        {
            // Handle Error
            return;
        }

        if (num_of_bytes == MAX_NUM_BYTES_PLD) // Reached max possible value
        {
            measurement_var.msr_response_byte_iter = FILL_MSR_OPQ_PEND;
            return;
        }
    }

    if (measurement_var.sign_needed)
    {
        if (is_sub_safe(MAX_NUM_BYTES_PLD, num_of_bytes) == 1)
        {
            remaining_bytes_to_sent = MAX_NUM_BYTES_PLD - num_of_bytes;
        }
        else
        {
            // Handle error
        }
        bytes_to_be_transferred = SIGNATURE_R_TERM_SZ;

        // if bytes_to_be_transferred is greater than remaining_bytes than can be sent; Send max possible value
        bytes_to_be_transferred = bytes_to_be_transferred > remaining_bytes_to_sent ? remaining_bytes_to_sent :
                                  bytes_to_be_transferred;
        measurement_var.msr_bytes_copied = spdm_pkt_fill_signature_r(num_of_bytes, bytes_to_be_transferred);
        num_of_bytes = num_of_bytes + bytes_to_be_transferred;
        if (is_add_safe(num_of_bytes, bytes_to_be_transferred) == 0)
        {
            // Handle Error
        }

        if (is_sub_safe(MAX_NUM_BYTES_PLD, num_of_bytes) == 1)
        {
            remaining_bytes_to_sent = MAX_NUM_BYTES_PLD - num_of_bytes;
        }
        else
        {
            // Handle error
        }

        if (signature_offset < SIGNATURE_R_TERM_SZ)
        {
            measurement_var.msr_response_byte_iter = FILL_MSR_SIGN_R_PEND;
        }
        else if (signature_offset == SIGNATURE_R_TERM_SZ)
        {
            signature_offset = 0;
            measurement_var.msr_response_byte_iter = FILL_MSR_SIGN_S;
        }

        if (remaining_bytes_to_sent)
        {

            if (remaining_bytes_to_sent >= SIGNATURE_S_TERM_SZ)
            {
                measurement_var.msr_bytes_copied = spdm_pkt_fill_signature_s(num_of_bytes, SIGNATURE_S_TERM_SZ);
                measurement_var.msr_response_byte_iter = END_OF_TRANSACTION;
                signature_offset = 0;
                packet_sz = measurement_var.msr_bytes_copied + MCTP_TOT_HDR_SZ;
            } // copy signature s bytes

            else
            {
                measurement_var.msr_bytes_copied = spdm_pkt_fill_signature_s(num_of_bytes, remaining_bytes_to_sent);
                measurement_var.msr_response_byte_iter = FILL_MSR_SIGN_S_PEND;
            }

            if (signature_offset < SIGNATURE_S_TERM_SZ)
            {
                measurement_var.msr_response_byte_iter = FILL_MSR_SIGN_S_PEND;
            }
            else if (signature_offset == SIGNATURE_S_TERM_SZ)
            {
                measurement_var.msr_response_byte_iter = END_OF_TRANSACTION;
                signature_offset = 0;
                packet_sz = measurement_var.msr_bytes_copied + MCTP_TOT_HDR_SZ;
            }
        }
    }
    else
    {
        measurement_var.msr_response_byte_iter = END_OF_TRANSACTION;
        packet_sz = measurement_var.msr_bytes_copied + MCTP_TOT_HDR_SZ;
    }
}

/******************************************************************************/
/** Handle tx packet buffering when responding to GET_MEASUREMENT request
 * @param spdm_buf_tx- tx buffer of spdm with MCTP packet definition fields
 * @param spdmContext - Context of SPDM module
 * @return void
 *******************************************************************************/
void spdm_pkt_fill_spdm_buf_measurements_resp(MCTP_PKT_BUF *spdm_buf_tx, SPDM_CONTEXT *spdmContext)
{
    if (NULL == spdmContext)
    {
        return;
    }

    uint8_t bytes_to_start = 0U;
    switch (measurement_var.msr_response_byte_iter)
    {
    case FILL_MSR_RCRD:
        spdm_pkt_fill_msr_record_stage1(&concat_msrment_blocks[0]);
        break;

    case FILL_REM_RCRD:
        // check if msr_record bytes is > max packet pld size (64)
        if (measurement_var.msr_rcrd_bytes_to_sent > MAX_NUM_BYTES_PLD)
        {
            measurement_var.msr_bytes_copied = MAX_NUM_BYTES_PLD;

            if (measurement_var.msr_operation)
            {

                memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS], &concat_msrment_blocks[measurement_var.concat_msr_offset],
                       measurement_var.msr_bytes_copied);
            }
            measurement_var.concat_msr_offset = (measurement_var.concat_msr_offset) + measurement_var.msr_bytes_copied;
            measurement_var.msr_rcrd_bytes_to_sent = (measurement_var.msr_rcrd_bytes_to_sent) - measurement_var.msr_bytes_copied;
            measurement_var.msr_response_byte_iter = FILL_REM_RCRD;
        }
        else
        {
            // leftover msr record bytes copied
            measurement_var.msr_bytes_copied = measurement_var.msr_rcrd_bytes_to_sent;

            if (measurement_var.msr_operation)
            {
                memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS], &concat_msrment_blocks[measurement_var.concat_msr_offset],
                       measurement_var.msr_bytes_copied);
            }

            measurement_var.msr_rcrd_bytes_to_sent = 0; // all msr records sent

            // check if remaining bytes still exist in buffer
            remaining_bytes_to_sent = MAX_NUM_BYTES_PLD - measurement_var.msr_bytes_copied;
            if (remaining_bytes_to_sent)
            {
                if ((SPDM_MSG_TYPE_POS + measurement_var.msr_bytes_copied) < MAX_PKT_SIZE)
                {
                    nonce_offset = 0;
                    // if bytes remaining exist
                    // fill random value bytes
                    if (remaining_bytes_to_sent >= NOUNCE_DATA_SIZE)
                    {
                        for (uint8_t i = 0; i < NOUNCE_DATA_SIZE; i++)
                        {
                            if ((SPDM_MSG_TYPE_POS + measurement_var.msr_bytes_copied + i) < sizeof(MCTP_PACKET)) // Coverity Security Fixes
                            {
                                spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + measurement_var.msr_bytes_copied + i] = nonce_data[NOUNCE_DATA_SIZE - 1 -
                                        nonce_offset - i];
                            }
                        }
                        nonce_offset = NOUNCE_DATA_SIZE;
                        measurement_var.msr_bytes_copied = measurement_var.msr_bytes_copied + NOUNCE_DATA_SIZE;
                        remaining_bytes_to_sent = remaining_bytes_to_sent - NOUNCE_DATA_SIZE;
                    }
                    else
                    {
                        for (uint8_t i = 0; i < remaining_bytes_to_sent; i++)
                        {
                            spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + measurement_var.msr_bytes_copied + i] = nonce_data[NOUNCE_DATA_SIZE - 1 -
                                    nonce_offset - i];
                        }
                        nonce_offset = remaining_bytes_to_sent;
                        measurement_var.msr_bytes_copied = measurement_var.msr_bytes_copied + remaining_bytes_to_sent;
                        remaining_bytes_to_sent = 0;
                    }
                    measurement_var.msr_rnd_bytes_remaining = NOUNCE_DATA_SIZE - nonce_offset;

                    // remaining in next state
                    if (measurement_var.msr_rnd_bytes_remaining)
                    {
                        measurement_var.msr_response_byte_iter = FILL_REM_RND; // next state to fill remaining rand bytes and so on
                    }
                    else
                    {
                        if (remaining_bytes_to_sent)
                        {
                            if (SPDM_MSG_TYPE_POS + measurement_var.msr_bytes_copied < MAX_PKT_SIZE)
                            {
                                measurement_var.opq_offset = 0;
                                if (remaining_bytes_to_sent >= OPAQUE_DATA_SZ)
                                {
                                    memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + measurement_var.msr_bytes_copied],
                                           &msr_opaque_buf[measurement_var.opq_offset], OPAQUE_DATA_SZ);
                                    measurement_var.msr_bytes_copied = measurement_var.msr_bytes_copied + OPAQUE_DATA_SZ;
                                    remaining_bytes_to_sent = remaining_bytes_to_sent - OPAQUE_DATA_SZ;
                                }
                                else
                                {
                                    memcpy(&spdm_buf_tx->pkt.data[SPDM_MSG_TYPE_POS + measurement_var.msr_bytes_copied],
                                           &msr_opaque_buf[measurement_var.opq_offset], remaining_bytes_to_sent);
                                    measurement_var.opq_offset = remaining_bytes_to_sent;
                                    measurement_var.msr_bytes_copied = measurement_var.msr_bytes_copied + remaining_bytes_to_sent;
                                    remaining_bytes_to_sent = 0;
                                    measurement_var.msr_response_byte_iter = FILL_MSR_OPQ_PEND;
                                    break;
                                }
                            }
                            else
                            {
                                measurement_var.msr_response_byte_iter = END_OF_TRANSACTION;
                                packet_sz = 0;
                                remaining_bytes_to_sent = 0;
                            }

                            if (measurement_var.sign_needed)
                            {
                                if (remaining_bytes_to_sent)
                                {

                                    if (SPDM_MSG_TYPE_POS + measurement_var.msr_bytes_copied < MAX_PKT_SIZE)
                                    {
                                        if (remaining_bytes_to_sent >= SIGNATURE_R_TERM_SZ)
                                        {
                                            measurement_var.msr_bytes_copied = spdm_pkt_fill_signature_r(measurement_var.msr_bytes_copied, SIGNATURE_R_TERM_SZ);
                                            remaining_bytes_to_sent = remaining_bytes_to_sent - SIGNATURE_R_TERM_SZ;
                                            signature_offset = 0;
                                        }
                                        else
                                        {
                                            measurement_var.msr_bytes_copied = spdm_pkt_fill_signature_r(measurement_var.msr_bytes_copied, remaining_bytes_to_sent);
                                            remaining_bytes_to_sent = 0;
                                            measurement_var.msr_response_byte_iter = FILL_MSR_SIGN_R_PEND;
                                        }
                                    }
                                    else
                                    {
                                        measurement_var.msr_response_byte_iter = END_OF_TRANSACTION;
                                        packet_sz = 0;
                                        remaining_bytes_to_sent = 0;
                                    }

                                    if (remaining_bytes_to_sent)
                                    {
                                        if (SPDM_MSG_TYPE_POS + measurement_var.msr_bytes_copied < MAX_PKT_SIZE)
                                        {
                                            if (remaining_bytes_to_sent >= SIGNATURE_S_TERM_SZ)
                                            {
                                                measurement_var.msr_bytes_copied = spdm_pkt_fill_signature_s(measurement_var.msr_bytes_copied, SIGNATURE_S_TERM_SZ);
                                                measurement_var.msr_response_byte_iter = END_OF_TRANSACTION;
                                                signature_offset = 0;
                                                packet_sz = measurement_var.msr_bytes_copied + MCTP_TOT_HDR_SZ;
                                            }
                                            else
                                            {
                                                measurement_var.msr_bytes_copied = spdm_pkt_fill_signature_s(measurement_var.msr_bytes_copied, remaining_bytes_to_sent);
                                                remaining_bytes_to_sent = 0;
                                                measurement_var.msr_response_byte_iter = FILL_MSR_SIGN_S_PEND;
                                            }
                                        }
                                        else
                                        {
                                            measurement_var.msr_response_byte_iter = END_OF_TRANSACTION;
                                            packet_sz = 0;
                                            remaining_bytes_to_sent = 0;
                                        }
                                    }
                                    else
                                    {
                                        measurement_var.msr_response_byte_iter = FILL_MSR_SIGN_S;
                                    }
                                }
                                else
                                {
                                    measurement_var.msr_response_byte_iter = FILL_MSR_SIGN_R;
                                }
                            }
                            else
                            {
                                measurement_var.msr_response_byte_iter = END_OF_TRANSACTION;
                                packet_sz = measurement_var.msr_bytes_copied + MCTP_TOT_HDR_SZ;
                            }
                        }
                        else
                        {
                            measurement_var.msr_response_byte_iter = FILL_MSR_OPQ;
                        }
                    }
                }
                else
                {
                    measurement_var.msr_response_byte_iter = END_OF_TRANSACTION;
                    packet_sz = 0;
                    remaining_bytes_to_sent = 0;
                }
            }
            else
            {
                measurement_var.msr_response_byte_iter = FILL_MSR_RND;
            }
        }
        break;

    case FILL_MSR_RND:
        // if we come here, it means all msr rec sent, next is random no
        nonce_offset = 0;
        measurement_var.opq_offset = 0;
        signature_offset = 0;
        measurement_var.msr_rnd_bytes_remaining = NOUNCE_DATA_SIZE;
        measurement_var.msr_opq_bytes_remaining = OPAQUE_DATA_SZ + OPAQUE_DATA_LEN;
        if (measurement_var.msr_operation == 0)
        {
            bytes_to_start = SPDM_MSG_PLD_SPECIFIC_BYTES + 8U;
        }
        spdm_pkt_fill_msr_record_stage2(measurement_var.msr_rnd_bytes_remaining, measurement_var.msr_opq_bytes_remaining,
                                        bytes_to_start);

        break;
    case FILL_REM_RND:
        measurement_var.msr_bytes_copied = 0;
        measurement_var.opq_offset = 0;
        signature_offset = 0;
        measurement_var.msr_opq_bytes_remaining = OPAQUE_DATA_SZ + OPAQUE_DATA_LEN;
        spdm_pkt_fill_msr_record_stage2(measurement_var.msr_rnd_bytes_remaining, measurement_var.msr_opq_bytes_remaining, 0U);
        break;

    case FILL_MSR_OPQ:
        measurement_var.msr_bytes_copied = 0;
        measurement_var.opq_offset = 0;
        signature_offset = 0;
        measurement_var.msr_rnd_bytes_remaining = 0;
        measurement_var.msr_opq_bytes_remaining = OPAQUE_DATA_SZ + OPAQUE_DATA_LEN;
        spdm_pkt_fill_msr_record_stage2(measurement_var.msr_rnd_bytes_remaining, measurement_var.msr_opq_bytes_remaining, 0U);
        break;
    case FILL_MSR_OPQ_PEND:
    {
        uint16_t transfer_opaq_bytes;
        signature_offset = 0;
        measurement_var.msr_rnd_bytes_remaining = 0;
        measurement_var.msr_opq_bytes_remaining = OPAQUE_DATA_SZ + OPAQUE_DATA_LEN - (measurement_var.opq_offset);
        if (measurement_var.msr_opq_bytes_remaining > MAX_NUM_BYTES_PLD)
        {
            transfer_opaq_bytes = MAX_NUM_BYTES_PLD;
        }
        else
        {
            transfer_opaq_bytes = measurement_var.msr_opq_bytes_remaining;
        }

        spdm_pkt_fill_msr_record_stage2(measurement_var.msr_rnd_bytes_remaining, transfer_opaq_bytes, 0U);
        break;
    }
    case FILL_MSR_SIGN_R:
        measurement_var.msr_bytes_copied = 0;
        measurement_var.opq_offset = 0;
        measurement_var.msr_rnd_bytes_remaining = 0;
        measurement_var.msr_opq_bytes_remaining = 0;

        measurement_var.msr_bytes_copied = spdm_pkt_fill_signature_r(measurement_var.msr_bytes_copied, SIGNATURE_R_TERM_SZ);
        remaining_bytes_to_sent = MAX_NUM_BYTES_PLD - measurement_var.msr_bytes_copied;

        if (remaining_bytes_to_sent)
        {
            if (SPDM_HEADER_VERSION_POS + measurement_var.msr_bytes_copied < MAX_PKT_SIZE)
            {
                // if we are here it means, signature r is completely copied
                signature_offset = 0;

                // copy signature s bytes
                if (remaining_bytes_to_sent >= SIGNATURE_S_TERM_SZ)
                {
                    measurement_var.msr_bytes_copied = spdm_pkt_fill_signature_s(measurement_var.msr_bytes_copied, SIGNATURE_S_TERM_SZ);
                }
                else
                {
                    measurement_var.msr_bytes_copied = spdm_pkt_fill_signature_s(measurement_var.msr_bytes_copied, remaining_bytes_to_sent);
                }
                if (signature_offset < SIGNATURE_S_TERM_SZ)
                {
                    measurement_var.msr_response_byte_iter = FILL_MSR_SIGN_S_PEND;
                }
                else if (signature_offset == SIGNATURE_S_TERM_SZ)
                {
                    measurement_var.msr_response_byte_iter = END_OF_TRANSACTION;
                    signature_offset = 0;
                    packet_sz = measurement_var.msr_bytes_copied + MCTP_TOT_HDR_SZ;
                }
            }
            else
            {
                measurement_var.msr_response_byte_iter = END_OF_TRANSACTION;
                packet_sz = 0;
                remaining_bytes_to_sent = 0;
            }
        }

        break;

    case FILL_MSR_SIGN_R_PEND:
        measurement_var.msr_bytes_copied = 0;
        measurement_var.opq_offset = 0;
        measurement_var.msr_rnd_bytes_remaining = 0;
        measurement_var.msr_opq_bytes_remaining = 0;

        measurement_var.msr_bytes_copied = spdm_pkt_fill_signature_r(measurement_var.msr_bytes_copied,
                                           (SIGNATURE_R_TERM_SZ - signature_offset));
        remaining_bytes_to_sent = MAX_NUM_BYTES_PLD - measurement_var.msr_bytes_copied;

        if (remaining_bytes_to_sent)
        {
            if (SPDM_MSG_TYPE_POS + measurement_var.msr_bytes_copied < MAX_PKT_SIZE)
            {
                // if we are here it means, signature r is completely copied
                signature_offset = 0;

                // copy signature s bytes
                if (remaining_bytes_to_sent >= SIGNATURE_S_TERM_SZ)
                {
                    measurement_var.msr_bytes_copied = spdm_pkt_fill_signature_s(measurement_var.msr_bytes_copied, SIGNATURE_S_TERM_SZ);
                }
                else
                {
                    measurement_var.msr_bytes_copied = spdm_pkt_fill_signature_s(measurement_var.msr_bytes_copied, remaining_bytes_to_sent);
                }
                if (signature_offset < SIGNATURE_S_TERM_SZ)
                {
                    measurement_var.msr_response_byte_iter = FILL_MSR_SIGN_S_PEND;
                }
                else if (signature_offset == SIGNATURE_S_TERM_SZ)
                {
                    measurement_var.msr_response_byte_iter = END_OF_TRANSACTION;
                    signature_offset = 0;
                    packet_sz = measurement_var.msr_bytes_copied + MCTP_TOT_HDR_SZ;
                }
            }
            else
            {
                measurement_var.msr_response_byte_iter = END_OF_TRANSACTION;
                packet_sz = 0;
                remaining_bytes_to_sent = 0;
            }
        }

        break;
    case FILL_MSR_SIGN_S:
        measurement_var.msr_bytes_copied = 0;
        measurement_var.opq_offset = 0;
        measurement_var.msr_rnd_bytes_remaining = 0;
        measurement_var.msr_opq_bytes_remaining = 0;

        measurement_var.msr_bytes_copied = spdm_pkt_fill_signature_s(measurement_var.msr_bytes_copied, SIGNATURE_S_TERM_SZ);
        measurement_var.msr_response_byte_iter = END_OF_TRANSACTION;
        signature_offset = 0;
        packet_sz = measurement_var.msr_bytes_copied + MCTP_TOT_HDR_SZ;
        break;

    case FILL_MSR_SIGN_S_PEND:
        measurement_var.msr_bytes_copied = 0;
        measurement_var.msr_bytes_copied = spdm_pkt_fill_signature_s(measurement_var.msr_bytes_copied,
                                           (SIGNATURE_S_TERM_SZ - signature_offset));
        measurement_var.msr_response_byte_iter = END_OF_TRANSACTION;
        signature_offset = 0;
        packet_sz = measurement_var.msr_bytes_copied + MCTP_TOT_HDR_SZ;

        break;
    default:
        packet_sz = 0;
        break;

    } // end of switch case
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
    mctp_buf->pkt.field.hdr.cmd_code = MCTP_SMBUS_HDR_CMD_CODE;
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
    /* for response packet */
    mctp_buf->pkt.field.hdr.tag_owner = 0;
    /* Packet sequence number */
    mctp_buf->pkt.field.hdr.pkt_seq = pkt_seq_mctp;
    mctp_buf->pkt.field.hdr.msg_type = MCTP_MSGTYPE_SPDM;

    /* integrity check */
    mctp_buf->pkt.field.hdr.integrity_check = 0;

    mctp_buf->rx_smbus_timestamp = spdm_buf_tx->rx_smbus_timestamp;

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
        spdm_pkt_fill_spdm_buf_measurements_resp(spdm_buf_tx, spdmContext);
        if (packet_sz > MCTP_BYTECNT_MAX)
        {
            packet_sz = packet_sz + MCTP_TOT_HDR_SZ;
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
    spdm_get_len_for_runtime_hash(spdmContext);
    spdm_crypto_ops_run_time_hashing(&get_version_response[0], length, spdmContext);
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
    spdm_get_len_for_runtime_hash(spdmContext);
    spdm_crypto_ops_run_time_hashing(&get_cap_response[0], length, spdmContext);
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

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 34] = struct_algo.AlgType;
    nego_algo_resp_object.req_algo_struct.algo_struct[0] = struct_algo.AlgType;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 35] = struct_algo.AlgCount;
    nego_algo_resp_object.req_algo_struct.algo_struct[1] = struct_algo.AlgCount;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 36] = struct_algo.AlgSupported;
    nego_algo_resp_object.req_algo_struct.algo_struct[2] = struct_algo.AlgSupported;

    spdm_buf_tx->pkt.field.hdr.byte_cnt = SPDM_NEG_ALG_RESP_SIZE;
    packet_sz = SPDM_NEG_ALG_RESP_SIZE;
    spdmContext->current_resp_cmd = SPDM_NEG_ALGO_RESP;

    memcpy(nego_algo_response, (uint8_t *)&nego_algo_resp_object, sizeof(NEG_ALGO_RESP_FIELDS));

    spdmContext->request_or_response = 1;
    req_and_response_sz[1] = sizeof(NEG_ALGO_RESP_FIELDS);
    // calculate run time hash of response
    spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
    spdm_get_len_for_runtime_hash(spdmContext);
    spdm_crypto_ops_run_time_hashing(&nego_algo_response[0], length, spdmContext);
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
    spdm_crypto_ops_run_time_hashing(&get_digest_response[0], length, spdmContext);

    for (iter = 0; iter < MAX_SLOTS; iter++)
    {

        if (((chain_avail_bits_mask & (1U << iter)) >> iter) > UINT8_MAX) // Coverity INT31-C Fix
        {
            // handle error
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
            spdm_crypto_ops_run_time_hashing(&hash_of_chains[(iter * SPDM_SHA384_LEN)], length, spdmContext);
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
            first_cert = 0U;
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
            spdm_crypto_ops_run_time_hashing((uint8_t *)&get_cert_resp_object, length, spdmContext);

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

    if (NULL == spdmContext)
    {
        return 0xff;
    }

    spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS] = CURRENT_VERSION; // V1.1
    challenge_resp_object.version = CURRENT_VERSION;

    spdm_buf_tx->pkt.data[SPDM_HEADER_COMMAND_POS] = SPDM_CHALLENGE_AUTH_RSP;
    challenge_resp_object.resp_code = SPDM_CHALLENGE_AUTH_RSP;

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
            spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS] = requested_slot; // No support for BasicMutAuthReq
            challenge_resp_object.resp_attr = requested_slot;

            // Slot mask
            spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 1] = (1 << requested_slot);

            challenge_resp_object.slot_mask = (1 << requested_slot);

            // at offset 4:
            // Hash of the certificate chain
            for (iter = 0; iter < SPDM_SHA384_LEN; iter++)
            {
                spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 2 + iter] = hash_of_chains[(SPDM_SHA384_LEN * requested_slot) + iter];
            }

            memcpy(challenge_resp_object.hash_of_cert_chain, &hash_of_chains[(requested_slot * SPDM_SHA384_LEN)], SPDM_SHA384_LEN);

            // at offset 4 + SPDM_SHA384_LEN
            // random value of 256 bit as nonce generated by TRNG each time a response sent:
            ret_val = spdm_crypto_ops_gen_random_no(&nonce_data[0], NOUNCE_DATA_SIZE);

            if (ret_val != STATUS_OK)
            {
                memset(&nonce_data[0], 0x00, NOUNCE_DATA_SIZE);
            }

            memcpy(challenge_resp_object.random_val, &nonce_data[0], NOUNCE_DATA_SIZE);

            // Reverse array; NONCE should be in BE format
            reverse(challenge_resp_object.random_val, NOUNCE_DATA_SIZE);

            memcpy(challenge_response_buff, (uint8_t *)&challenge_resp_object, (sizeof(CHALLENGE_FIELDS) - 4));
            spdmContext->get_requests_state = RUN_TIME_HASH_MODE;

            // get the length of challenge response struct (excluding opaque data fields)
            spdmContext->request_or_response = 1;
            req_and_response_sz[1] = (sizeof(CHALLENGE_FIELDS) - 4);

            // get the hash of challenge response data of requested size till random value fields
            spdm_get_len_for_runtime_hash(spdmContext);
            spdm_crypto_ops_run_time_hashing(&challenge_response_buff[0], length, spdmContext);

            if (measurement_var.msr_hash_rqstd)
            {
                // get the length of hash of measurement summary sz
                req_and_response_sz[1] = SPDM_MEASUREMENT_HASH_SUMMARY_SZ;

                if (measurement_var.msr_hash_rqstd == 0xff)
                {
                    // get the hash of challenge response data of requested size till random value fields
                    spdm_get_len_for_runtime_hash(spdmContext);
                    spdm_crypto_ops_run_time_hashing(&hash_concat_msrments_val[0], length, spdmContext);
                }
                else
                {
                    // get the hash of challenge response data of requested size till random value fields
                    spdm_get_len_for_runtime_hash(spdmContext);
                    spdm_crypto_ops_run_time_hashing(&hash_concat_tcb_msrments_val[0], length, spdmContext);
                }
            }

            challenge_resp_object.opaq_len[0] = 0x0U;
            challenge_resp_object.opaq_len[1] = 0x1U; // 256 bytes in BE format

            memset(challenge_resp_object.opaq_data, 0, 2);

            spdmContext->get_requests_state = RUN_TIME_HASH_MODE;

            // get the size of opaque length field
            req_and_response_sz[1] = OPAQUE_FIELDS_SZ;

            // get the hash of challenge response data of requested size
            spdm_get_len_for_runtime_hash(spdmContext);
            spdm_crypto_ops_run_time_hashing(&challenge_resp_object.opaq_len[0], length, spdmContext);
            uint8_t challenge_opaq_data[256] = {0}; // Since opaq data is zero, Use local array to save space

            // get the size of opaque data field
            req_and_response_sz[1] = 256U;
            spdmContext->get_requests_state = RUN_TIME_HASH_MODE;

            // get the hash of challenge response data of requested size
            spdm_get_len_for_runtime_hash(spdmContext);
            spdm_crypto_ops_run_time_hashing(&challenge_opaq_data[0], length, spdmContext);

            spdmContext->get_requests_state = END_OF_HASH;
            spdm_crypto_ops_run_time_hashing(NULL, 0, spdmContext);
            memcpy(&hash_of_req_buffer[0], &spdmContext->sha_digest, SPDM_SHA384_LEN);

            // generate signature for the response using AK pvt key
            spdm_crypto_ops_gen_signature(); 

            if (measurement_var.msr_hash_rqstd == 0xff)
            {
                packet_sz = SPDM_CHALLENGE_AUTH_RESPONSE_BYTES_SZ + SPDM_MEASUREMENT_HASH_SUMMARY_SZ;
            }
            else
            {
                packet_sz = SPDM_CHALLENGE_AUTH_RESPONSE_BYTES_SZ;
            }

            spdmContext->current_resp_cmd = SPDM_CHALLENGE_AUTH_RSP;
            spdmContext->challenge_success_flag = 0x01;
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
    if (NULL == spdmContext)
    {
        return 0xff;
    }

    uint8_t no_of_indices = 0x00;

    uint8_t no_of_blocks = NO_OF_MSR_INDICES;
    uint8_t get_len = 0x0;
    // GET_MEASUREMENTS request attributes
    measurement_var.sign_needed = (get_mctp_pld[2] & 0xff); // Bit to check that signature will be present in responder

    measurement_var.msr_operation = (get_mctp_pld[3] & 0xff); // param2 for the measurement operation

    MCTP_PKT_BUF *spdm_msg_rx_buf = (MCTP_PKT_BUF *)&spdm_pktbuf_rx;

    if ((spdm_msg_rx_buf->pkt.data[MCTP_BYTE_CNT_OFFSET] -
            NO_OF_MCTP_HDR_BYTES_FRM_BYTE_CNT_OFFSET) > UINT8_MAX) // Coverity INT31-C Fix
    {
        // handle error
    }
    else
    {
        get_len = (uint8_t)(spdm_msg_rx_buf->pkt.data[MCTP_BYTE_CNT_OFFSET] -
                            NO_OF_MCTP_HDR_BYTES_FRM_BYTE_CNT_OFFSET);
    }
    // SPDM payload byte length
    // SlotIDParam
    if(measurement_var.sign_needed)
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
        no_of_indices = NO_OF_MSR_INDICES;
        no_of_blocks = 0x00;
        measurement_var.msr_rcrd_len = 0x00;
        measurement_var.msr_response_byte_iter = FILL_MSR_RND;
    }
    else if (measurement_var.msr_operation == 0xff) // request all blocks
    {
        no_of_blocks = NO_OF_MSR_INDICES;
        measurement_var.msr_rcrd_len = measurement_var.msr_record_size; // get the size of all measurement blocks supported
        measurement_var.msr_response_byte_iter = FILL_MSR_RCRD;
    }
    else if ((measurement_var.msr_operation >= START_INDEX) && (measurement_var.msr_operation <= NO_OF_MSR_INDICES))
    {
        no_of_blocks = 1U;
        uint32_t temp = (uint32_t)((msr_block_buffer[(measurement_var.msr_operation) - 1u].msr_size[0u]) & UINT32_MAX);
        uint32_t msr_offset = (uint32_t)((temp + MSR_BLOCK_SPEC_FLDS_SZ)& UINT32_MAX);
        if((msr_offset > UINT32_MAX) || (is_add_safe(msr_offset,
                                         msr_block_buffer[(measurement_var.msr_operation) - 1].msr_size[0]) == false)) // Coverity INT31-C Fix
        {
            // handle error
        }
        else
        {
            measurement_var.msr_rcrd_len = msr_offset; // get the size of only requested block
        }
        measurement_var.msr_response_byte_iter = FILL_MSR_RCRD;
    }
    else
    {
        return SPDM_CMD_FAIL;
    }

    spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS] = CURRENT_VERSION; // V1.1
    measurement_resp_object.version = CURRENT_VERSION;

    spdm_buf_tx->pkt.data[SPDM_HEADER_COMMAND_POS] = SPDM_GET_MEASUREMENT_RSP; // response code
    measurement_resp_object.resp_code = SPDM_GET_MEASUREMENT_RSP;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS] = no_of_indices; // Param1
    measurement_resp_object.param1 = no_of_indices;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 1] = requested_slot; // Param2
    measurement_resp_object.param2 = requested_slot;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 2] = no_of_blocks; // NumberOfBlocks
    measurement_resp_object.num_of_blocks = no_of_blocks;

    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 5u] = (uint8_t)((measurement_var.msr_rcrd_len >> 16u)&UINT8_MAX); // MeasurementRecordLength
    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 4u] = (uint8_t)((measurement_var.msr_rcrd_len >> 8u)&UINT8_MAX);  // MeasurementRecordLength
    spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 3u] = (uint8_t)(measurement_var.msr_rcrd_len&UINT8_MAX);         // MeasurementRecordLength

    memcpy(&measurement_resp_object.msr_rec_len[0], &spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 3], 3);
    memcpy(measurement_response_buff, (uint8_t *)&measurement_resp_object,
           (sizeof(MEASUREMENT_FIELDS) - OPAQUE_DATA_SZ - OPAQUE_DATA_LEN));

    spdmContext->get_requests_state = RUN_TIME_HASH_MODE;

    spdmContext->request_or_response = 1;
    // get the size of measurement response till measurerecord len
    req_and_response_sz[1] = sizeof(MEASUREMENT_FIELDS) - OPAQUE_DATA_SZ - OPAQUE_DATA_LEN;

    // run the hash engine with response data of requested size
    spdm_get_len_for_runtime_hash(spdmContext);
    spdm_crypto_ops_run_time_hashing(&measurement_response_buff[0], length, spdmContext);

    // concatenated msr blocks data fed to hash engine
    // run the hash engine with response data of requested size
    if (measurement_var.msr_rcrd_len > UINT16_MAX) // Coverity INT31-C Fix
    {
        // handle error
    }
    else
    {
        req_and_response_sz[1] = (uint16_t)(measurement_var.msr_rcrd_len);
    }

    if (measurement_var.msr_operation == 0xff) // request all blocks
    {
        spdm_get_len_for_runtime_hash(spdmContext);
        spdm_crypto_ops_run_time_hashing(&concat_msrment_blocks[0], length, spdmContext);
    }
    else if ((measurement_var.msr_operation >= START_INDEX) && (measurement_var.msr_operation <= NO_OF_MSR_INDICES))
    {
        spdm_get_len_for_runtime_hash(spdmContext);
        spdm_crypto_ops_run_time_hashing(&concat_msrment_blocks[(measurement_var.msr_operation - 1) * SIZE_OF_ONE_MSR_BLOCK],
                                         length, spdmContext);
    }

    // at offset 8 + MeasurementRecordLength, nonce data
    memset(nonce_data, 0, NOUNCE_DATA_SIZE);
    ret_val = spdm_crypto_ops_gen_random_no(&nonce_data[0], NOUNCE_DATA_SIZE);

    if (ret_val != STATUS_OK)
    {
        memset(&nonce_data[0], 0x00, NOUNCE_DATA_SIZE);
    }

    uint8_t nonce_array[NOUNCE_DATA_SIZE];
    memcpy(&nonce_array[0], &nonce_data[0], NOUNCE_DATA_SIZE);

    // Reverse array; NONCE should be in BE format
    reverse(nonce_array, NOUNCE_DATA_SIZE);

    req_and_response_sz[1] = NOUNCE_DATA_SIZE;
    spdm_get_len_for_runtime_hash(spdmContext);
    spdm_crypto_ops_run_time_hashing(&nonce_array[0], length, spdmContext);

    memset(measurement_resp_object.opaq_len, 0, OPAQUE_DATA_LEN);

    memset(msr_opaque_buf, 0, OPAQUE_DATA_SZ + OPAQUE_DATA_LEN);
    measurement_resp_object.opaq_len[0] = 0x0U;
    measurement_resp_object.opaq_len[1] = 0x1U;
    msr_opaque_buf[0] = measurement_resp_object.opaq_len[0];
    msr_opaque_buf[1] = measurement_resp_object.opaq_len[1];

    // get the size of opaque data field
    req_and_response_sz[1] = OPAQUE_DATA_SZ + OPAQUE_DATA_LEN;

    spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
    spdm_get_len_for_runtime_hash(spdmContext);
    spdm_crypto_ops_run_time_hashing(&msr_opaque_buf[0], length, spdmContext);

    if (measurement_var.sign_needed)
    {
        spdmContext->get_requests_state = END_OF_HASH;
        spdm_crypto_ops_run_time_hashing(NULL, 0, spdmContext);
        memcpy(&hash_of_req_buffer[0], &spdmContext->sha_digest, SPDM_SHA384_LEN);

        // generate signature for the response using AK pvt key
        spdm_crypto_ops_gen_signature(); // return value is purposely ignored, Failure case is handled inside the function
        packet_sz = SPDM_MEASUREMENT_RESPONSE_BYTES_SZ + measurement_var.msr_rcrd_len + (2 * CURVE_384_SZ);
    }
    else
    {
        packet_sz = SPDM_MEASUREMENT_RESPONSE_BYTES_SZ + measurement_var.msr_rcrd_len;
    }

    spdmContext->current_resp_cmd = SPDM_GET_MEASUREMENT_RSP;

    return cmd_ret_val;
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
            spdmContext->request_or_response = 0; // 0 means request
            // init the hash engine
            spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], 0, spdmContext);

            req_and_response_sz[0] = spdmContext->current_request_length;
            // change the state to run time hash
            spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
            // calculate hash of request
            spdm_get_len_for_runtime_hash(spdmContext);
            spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], length, spdmContext);
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

                // calculate hash of request
                spdm_get_len_for_runtime_hash(spdmContext);
                spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], length, spdmContext);
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
                // calculate hash of request
                spdm_get_len_for_runtime_hash(spdmContext);
                spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], length, spdmContext);
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
                spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], length, spdmContext);
                spdm_pkt_process_get_digest_cmd(spdm_buf_tx, spdmContext);

                spdm_rqst_cur_state = get_cmd;
            }
            else
            {
                // start from get version
                error_handle = SPDM_ERROR_UNXPTD_RQ_CODE;
            }
        }
        break;
    case SPDM_GET_CERT:
        if (error_handle == 0)
        {
            if ((spdm_rqst_cur_state == SPDM_GET_DIGEST) || (spdmContext->previous_state == SPDM_GET_DIGEST))
            {
                spdmContext->previous_state = SPDM_GET_DIGEST;
                spdmContext->request_or_response = 0; // 0 means request
                req_and_response_sz[0] = spdmContext->current_request_length;
                // switch to next state to handle more requests
                spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
                // calculate hash of request
                spdm_get_len_for_runtime_hash(spdmContext);
                spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], length, spdmContext);
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
                spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], length, spdmContext);
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
            if (spdm_rqst_cur_state == SPDM_CHALLENGE_AUTH_RQ) // Only if Challenge was successful, process Measurements 
            {
                // first call to go for init hash engine
                if (spdmContext->get_requests_state == HASH_INIT_MODE)
                {
                    spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], 0, spdmContext);
                }

                spdmContext->get_requests_state = RUN_TIME_HASH_MODE;
                spdmContext->request_or_response = 0; // 0 means request
                req_and_response_sz[0] = spdmContext->current_request_length;
                spdm_get_len_for_runtime_hash(spdmContext);
                spdm_crypto_ops_run_time_hashing(&get_mctp_pld[0], length, spdmContext);
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
    default:
        // invalid request command code
        error_handle = SPDM_ERROR_INVLD_RQ;
        break;
    }
    // if any reported error scenario; fill tx buffer with error response
    if (error_handle > 0)
    {
        spdm_buf_tx->pkt.data[SPDM_HEADER_VERSION_POS] = CURRENT_VERSION; // V1.1
        spdm_buf_tx->pkt.data[SPDM_HEADER_COMMAND_POS] = SPDM_ERROR_RESP;
        spdm_buf_tx->pkt.field.hdr.byte_cnt = SPDM_ERROR_RESPONSE_BYTES_SIZE;
        packet_sz = SPDM_ERROR_RESPONSE_BYTES_SIZE;

        spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS] = error_handle;

        if ((error_handle == SPDM_ERROR_INVLD_RQ) || (error_handle == SPDM_ERROR_UNXPTD_RQ_CODE) ||
                (error_handle == SPDM_ERROR_MJR_VRS_MISMATCH))
        {
            spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 1] = 0x0; // No extended error data is provided
        }
        else if ((error_handle == SPDM_ERROR_USPRTD_RQ_CODE) || (error_handle == SPDM_ERROR_RESYNCH))
        {
            spdm_buf_tx->pkt.data[SPDM_HEADER_DATA_POS + 1] = get_cmd; // error data - request response code
        }
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
        first_challenge_resp_sent = END_OF_TX;
        measurement_var.msr_bytes_copied = 0;
        measurement_var.msr_operation = 0x00;
        measurement_var.msr_response_byte_iter = END_OF_TRANSACTION;
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
        break;
    }
}

/******************************************************************************/
/** Function to load the 1Kb spdm input buffer for the spdm request bytes
 * @param spdmContext     - SPDM module context
 * @param spdm_msg_rx_buf - Buffer containing SPDM Request
 * @return 1 if payload size goes greater than 1024
 *         0 success
 *******************************************************************************/
uint8_t spdm_pkt_fill_buffer(MCTP_PKT_BUF *spdm_msg_rx_buf, SPDM_CONTEXT *spdmContext)
{
    uint8_t i;
    uint8_t ret_sts = 0x00;
    uint16_t get_len = 0x00;

    if (NULL == spdmContext)
    {
        return 0x01;
    }

    get_len = ((spdm_msg_rx_buf->pkt.data[MCTP_BYTE_CNT_OFFSET]) -
               NO_OF_MCTP_HDR_BYTES_FRM_BYTE_CNT_OFFSET); // SPDM payload byte length

    // buffer the spdm payload to 1K input buffer from mctp input buffer
    for (i = 0; i < get_len; i++)
    {
        get_mctp_pld[pld_index + i] = spdm_msg_rx_buf->pkt.data[SPDM_HEADER_VERSION_POS + i]; // Take only the spdm msg payload
    }

    if (spdm_tx_state == SPDM_PACKETIZING)
    {
        pld_index = pld_index + get_len;
        spdmContext->current_request_length = pld_index;
    }
    else if (spdm_tx_state == SPDM_NON_PACKETIZING)
    {
        spdmContext->current_request_length = get_len;
    }

    if (pld_index > INPUT_BUF_MAX_BYTES) // if no of bytes received cross max input buffer size of 1023
    {
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
    spdmContext = spdm_ctxt_get();
    if (NULL == spdmContext)
    {
        return;
    }
    MCTP_PKT_BUF *spdm_msg_rx_buf = NULL;
    //    spdm_msg_rx_buf = (MCTP_PKT_BUF *) &mctp_pktbuf[MCTP_BUF3];
    spdm_msg_rx_buf = (MCTP_PKT_BUF *)&spdm_pktbuf_rx;

    // access mctp buffer for spdm only if mctp receive buffer is available
    if (MCTP_RX_PENDING == spdm_msg_rx_buf->buf_full)
    {
        spdmContext->host_eid = spdm_msg_rx_buf->pkt.field.hdr.src_eid;
        spdmContext->ec_eid = spdm_msg_rx_buf->pkt.field.hdr.dst_eid;
        spdmContext->ec_slv_addr = spdm_msg_rx_buf->pkt.field.hdr.dst_addr;
        spdmContext->host_slv_addr = spdm_msg_rx_buf->pkt.field.hdr.src_addr;

        if (spdm_tx_state == SPDM_TX_IDLE || spdm_tx_state == SPDM_PACKETIZING)
        {
            // check if MCTP packet received is single packet request
            if ((spdm_msg_rx_buf->pkt.data[MCTP_MSG_TAG_REF_MASK] & MCTP_SOM_EOM_REF_MSK) == MCTP_SOM_EOM_REF)
            {
                pld_index = 0;
                spdm_tx_state = SPDM_NON_PACKETIZING;
                ret_sts = spdm_pkt_fill_buffer(spdm_msg_rx_buf, spdmContext);
                if (!ret_sts)
                {
                    time_stamp = spdm_msg_rx_buf->rx_smbus_timestamp;
                }
            }
            else
            {
                if ((spdm_msg_rx_buf->pkt.data[MCTP_MSG_TAG_REF_MASK] & MCTP_SOM_REF) == MCTP_SOM_REF)
                {
                    pld_index = 0U;
                }
                // som = 1, eom = 0 or som = 0, eom = 0 or som = 0, eom = 1 states
                // this means packetizing in progress
                spdm_tx_state = SPDM_PACKETIZING;
                ret_sts = spdm_pkt_fill_buffer(spdm_msg_rx_buf, spdmContext);
                if (!ret_sts)
                {
                    if ((spdm_msg_rx_buf->pkt.data[MCTP_MSG_TAG_REF_MASK] & MCTP_EOM_REF_MSK) == MCTP_EOM_REF)
                    {
                        spdm_tx_state = SPDM_RX_LAST_PKT;
                        pld_index = 0;
                        time_stamp = spdm_msg_rx_buf->rx_smbus_timestamp;
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
            spdm_buf_tx->rx_smbus_timestamp = time_stamp;
            get_cmd = (get_mctp_pld[1] & 0xff); // get the spdm command

            ret_sts = spdm_pkt_validate_and_process_spdm_msg(get_cmd, spdm_buf_tx, spdmContext);
            if (ret_sts)
            {
               /* Invalid SPDM req rcvd */
            }
            else
            {
               /* Valid SPDM req rcvd */
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
 * @return         : 0U (ECFW_IMG_TAG0)
 *                   1U (ECFW_IMG_TAG1)
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
            return_loaded_tag_img = ECFW_IMG_TAG0; // If loaded from TAG0 then TAG1 is not loaded
        }
        else /*Loaded from TAG1*/
        {
            return_loaded_tag_img = ECFW_IMG_TAG1;
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

    ptr_cert_buffer->certificate_chain_slot[0] = SPDM_SLOT_CERT_CHAIN01;
    ptr_cert_buffer->certificate_chain_slot[1] = SPDM_SLOT_CERT_CHAIN23;
    ptr_cert_buffer->certificate_chain_slot[2] = SPDM_SLOT_CERT_CHAIN45;
    ptr_cert_buffer->certificate_chain_slot[3] = SPDM_SLOT_CERT_CHAIN67;

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
    uint8_t ret_val = 1;
    if (certificate_num == 0 || certificate_num == 1)
    {
        ret_val = SRAM_MBOX_API_devAK_cert_read(address, buff_ptr, certificate_num, length);
    } else {
        ret_val = di_spdm_spi_send_read_request(address, buff_ptr, length, SPI_SELECT_INT_COMP_0, true);
    }
    return ret_val;
}
