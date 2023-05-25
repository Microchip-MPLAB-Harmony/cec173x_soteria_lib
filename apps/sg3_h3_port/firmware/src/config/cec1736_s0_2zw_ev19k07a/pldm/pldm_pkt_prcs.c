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

#include <stdlib.h>
#include "pldm.h"
#include "pldm_common.h"
#include "mctp/mctp_smbus.h"
#include "mctp/mctp_base.h"
#include "mctp/mctp_control.h"
#include "pldm_pkt_prcs.h"

#define PAGE_SIZE                   256ul

PLDM_BSS1_ATTR MCTP_PKT_BUF pldm_buf_tx[1];
PLDM_BSS2_ATTR uint8_t pldm_request_firmware_update;
PLDM_BSS2_ATTR MCTP_PKT_BUF *mctp_buf_tx1 = MCTP_NULL;
PLDM_BSS2_ATTR uint32_t offset;
PLDM_BSS2_ATTR uint32_t len;
extern PLDM_BSS1_ATTR uint8_t get_mctp_pld[PLDM_MAX_PAYLOAD_BUFF_SIZE]__attribute__((aligned(8)));
PLDM_BSS1_ATTR uint32_t pldm_packet_size;
PLDM_BSS2_ATTR bool received_1024b_for_flash_write;
PLDM_BSS1_ATTR uint32_t offset_for_flash;
PLDM_BSS1_ATTR uint16_t number_of_1024b_requests;
PLDM_BSS1_ATTR REQUEST_REQUEST_UPDATE request_update;
PLDM_BSS2_ATTR bool in_update_mode;
PLDM_BSS2_ATTR REQUEST_UPDATE_COMPONENT request_update_component;
PLDM_BSS2_ATTR REQUEST_ACTIVATE_FIRMWARE request_activate_firmware;
PLDM_BSS2_ATTR REQUEST_PASS_COMPONENT_TABLE request_pass_component_table;
PLDM_BSS0_ATTR uint8_t buffer[1024];
PLDM_BSS1_ATTR uint8_t pldm_pkt_seq_mctp;
PLDM_BSS1_ATTR bool pldm_first_pkt;
extern PLDM_BSS1_ATTR uint8_t curr_ec_id;
PLDM_BSS1_ATTR uint16_t total_length;
PLDM_BSS1_ATTR uint16_t offset_for_data;

PLDM_BSS1_ATTR MCTP_PKT_BUF pldm_pktbuf_rx;
PLDM_BSS1_ATTR MCTP_PKT_BUF pldm_mctp_pktbuf_tx;

PLDM_BSS1_ATTR bool can_activate;
PLDM_BSS1_ATTR bool can_update;
PLDM_BSS1_ATTR bool self_contained_activation;
PLDM_BSS1_ATTR bool failure_recovery_cap;
PLDM_BSS1_ATTR bool retry_update_comp_cap;
PLDM_BSS1_ATTR bool verify_fail;
PLDM_BSS1_ATTR bool apply_completed;
PLDM_BSS1_ATTR uint32_t no_of_requests;
PLDM_BSS2_ATTR uint8_t descriptors[200];
PLDM_BSS1_ATTR REQUEST_UPDATE_COMPONENT_RESPONSE req_update_comp_resp_data;
PLDM_BSS2_ATTR GET_FIRMWARE_PARAMETERS_RES_FIELDS get_firmware_parameters_res;

extern void timer_delay_ms(uint32_t num_ms);
PLDM_BSS2_ATTR bool host_functionality_reduced;

PLDM_BSS1_ATTR uint32_t remaining_data_size_for_transfer;
PLDM_BSS1_ATTR bool background_update_in_progress;
PLDM_BSS1_ATTR bool retry_request_firmware_data;
PLDM_BSS1_ATTR uint32_t size_supported_by_UA;

PLDM_BSS1_ATTR uint16_t pkt_cnt;
PLDM_BSS1_ATTR uint8_t completion_code;

PLDM_BSS1_ATTR uint8_t req_cmd;

PLDM_BSS1_ATTR uint8_t pldm_type;

PLDM_BSS1_ATTR uint8_t offset_data_pos;

PLDM_BSS1_ATTR bool PLDMResp_timer_started;

// support for update during crisis update
PLDM_BSS1_ATTR PLDM_AP_CFG pldm_ap_cfg;

/******************************************************************************/
/** function for handling query device identifiers message
* @param pldm_buf_tx trasmit buffer,
* @param pldmContext pldm task context
* @return None
*******************************************************************************/
void pldm_pkt_handle_query_device_identifiers(MCTP_PKT_BUF *pldm_buf_tx, PLDM_CONTEXT *pldmContext)
{
    bool override = 0;
    uint16_t size=0x0u;

    uint16_t des_type1 = PLDM_QUERY_DEVICE_DES1_TYPE;
    uint16_t des_length1 = PLDM_QUERY_DEVICE_DES1_LEN;
    uint16_t dec_value1 = PLDM_QUERY_DEVICE_DES1_VAL;

    uint16_t des_type2 = PLDM_QUERY_DEVICE_DES2_TYPE;
    uint16_t des_length2 = PLDM_QUERY_DEVICE_DES2_LEN;
    uint64_t dec_value2_serial_number = 0x00; //coverity fix
    uint32_t dec_value2_part_number = 0x00; //coverity fix

    pldm_buf_tx->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS] = PLDM_HDR_VERSION_PLDM_FW_UPDATE_TYPE;
    pldm_buf_tx->pkt.data[PLDM_HEADER_COMMAND_CODE_POS] = PLDM_QUERY_DEVICE_IDENTIFIERS_REQ;

    QUERY_DEVICE_IDENTIFIERS_REQ_MESSAGE_RES_FIELDS device_desp_resp;
    override = get_pldm_override_device_desc();
    device_desp_resp.completion_code = PLDM_SUCCESS;

    if (!override)
    {
        device_desp_resp.device_identifiers_length = PLDM_QUERY_DEVICE_DES_TOTAL_LEN;
        device_desp_resp.descriptor_count = PLDM_QUERY_DEVICE_DES_COUNT;

        memcpy(&device_desp_resp.descriptor[0], &des_type1, 2); // PCI vendor
        memcpy(&device_desp_resp.descriptor[2], &des_length1, 2); // 2 bytes
        memcpy(&device_desp_resp.descriptor[4], &dec_value1, 2); // mchp pci vendor

        memcpy(&device_desp_resp.descriptor[6], &des_type2, 2); // PCI vendor
        memcpy(&device_desp_resp.descriptor[8], &des_length2, 2); // 2 bytes

        if (0 == efuse_read_data(DEVICE_SERIAL_NUMBER_OFFSET, (uint8_t*)&dec_value2_serial_number, 8))
        {
            memcpy(&device_desp_resp.descriptor[14], &dec_value2_serial_number, 8);
        }

        if (0 == efuse_read_data(DEVICE_PART_NUMBER_OFFSET, (uint8_t*)&dec_value2_part_number, 4))
        {
            memcpy(&device_desp_resp.descriptor[10], &dec_value2_part_number, 4); // mchp pci vendor // otp 980 serial number
        }

        size = 6 + PLDM_QUERY_DEVICE_DES_TOTAL_LEN;
        total_length = size;
        
    }
    else
    {
        device_desp_resp.device_identifiers_length = pldm_apcfg_device_identifier_length();

        device_desp_resp.descriptor_count = pldm_apcfg_descriptor_count();

        pldm_apcfg_descriptor(descriptors);

        memcpy(device_desp_resp.descriptor, descriptors, device_desp_resp.device_identifiers_length);
        uint64_t local = (uint64_t)((device_desp_resp.device_identifiers_length + 6u )& UINT64_MAX);
        if(local > UINT64_MAX)
        {
            /* Handle Error */
        }
        else{
            total_length = (uint32_t)local;
            size = (uint32_t)local;
        }
    }

    memcpy(buffer, &device_desp_resp, size);

    pldm_packet_size = size + 9;

    pldmContext->pldm_current_response_cmd = PLDM_QUERY_DEVICE_IDENTIFIERS_REQ;
}

/******************************************************************************/
/** function for handling get firmware parameters message
* @param pldm_buf_tx trasmit buffer,
* @param pldmContext pldm task context
* @return None
*******************************************************************************/
void pldm_pkt_handle_get_firmware_parameters(MCTP_PKT_BUF *pldm_buf_tx, PLDM_CONTEXT *pldmContext)
{
    pldm_buf_tx->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS] = PLDM_HDR_VERSION_PLDM_FW_UPDATE_TYPE;
    pldm_buf_tx->pkt.data[PLDM_HEADER_COMMAND_CODE_POS] = PLDM_GET_FIRMWARE_PARAMETERS_REQ;
    uint16_t size = 0;
    uint16_t cap;

    get_firmware_parameters_res.completion_code = PLDM_SUCCESS;
    if (pldm_apcfg_override_cap_upgrade()) {
        cap = pldm_apcfg_capabilities_upgrade();
        get_firmware_parameters_res.capabilities_during_update = cap;
        if (get_firmware_parameters_res.capabilities_during_update & PLDM_COMP_UPDATE_FAILURE_RECOVERY_CAP) {
            failure_recovery_cap = false;
        } else {
            failure_recovery_cap = true;
        }

        if (get_firmware_parameters_res.capabilities_during_update & PLDM_COMP_UPDATE_RETRY_CAP) {
            retry_update_comp_cap = false;
        } else {
            retry_update_comp_cap = true;
        }
        if (get_firmware_parameters_res.capabilities_during_update & PLDM_COMP_UPDATE_HOST_FUNCTIONALITY) {
            host_functionality_reduced = true;
        } else {
            host_functionality_reduced = false;
        }
    } else {
        get_firmware_parameters_res.capabilities_during_update = PLDM_DEFAULT_CAP_DURING_UPDATE;
        failure_recovery_cap = true;
        retry_update_comp_cap = true;
        host_functionality_reduced = true;
    }

    pldm_get_firmware_param_resp_feilds(&get_firmware_parameters_res);

    size = (PLDM_REQ_GET_FIRMWARE_PARAMETERS_RESPONSE_COMMON_DATA_BYTES +
            ((sizeof(COMPONENT_PARAMETER_TABLE)) * get_firmware_parameters_res.component_count));
    
    if(size > sizeof(buffer)) // Misra fix - make sure size does not exceed size of destination buffer
    {
        size = sizeof(buffer);
    }
    else
    {
        // do nothing
    }

    memcpy(buffer, &get_firmware_parameters_res, size);
    
    total_length = size;

    pldmContext->pldm_current_response_cmd = PLDM_GET_FIRMWARE_PARAMETERS_REQ;
}

/******************************************************************************/
/** function for handling request update message
* @param pldm_buf_tx trasmit buffer,
* @param pldmContext pldm task context
* @return None
*******************************************************************************/
void pldm_pkt_handle_request_update(MCTP_PKT_BUF *pldm_buf_tx, PLDM_CONTEXT *pldmContext)
{
    MCTP_PKT_BUF *pldm_msg_rx_buf = NULL;
    pldm_msg_rx_buf = (MCTP_PKT_BUF *) &pldm_pktbuf_rx;

    memcpy(&request_update, &(get_mctp_pld[0]), PLDM_REQUEST_UPDATE_DATA_LEN);
    if (request_update.max_transfer_size < ONE_KB)
    {
        size_supported_by_UA = request_update.max_transfer_size;
    }
    else
    {
        size_supported_by_UA = ONE_KB;
    }

    pldm_buf_tx->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS] = PLDM_HDR_VERSION_PLDM_FW_UPDATE_TYPE;
    pldm_buf_tx->pkt.data[PLDM_HEADER_COMMAND_CODE_POS] = PLDM_REQUEST_UPDATE_REQ;

    REQUEST_UPDATE_RESPONSE req_update_resp_data;

    pldmContext->pldm_previous_state = pldmContext->pldm_current_state;
    if (!in_update_mode || (verify_fail && retry_update_comp_cap))
    {
        req_update_resp_data.completion_code = PLDM_SUCCESS;
        pldmContext->pldm_current_state = PLDM_LEARN_COMPONENTS;
    }
    else
    {
        req_update_resp_data.completion_code = ALREADY_IN_UPDATE_MODE;
    }

    req_update_resp_data.fw_dev_metadata_len = 0;
    req_update_resp_data.get_pkg_data_cmd = 0;

    memcpy(&pldm_buf_tx->pkt.data[PLDM_HEADER_COMPLETION_CODE_POS], &req_update_resp_data,
           PLDM_REQ_UPDATE_RESPONSE_DATA_BYTES);

    pldm_packet_size = MCTP_TOT_HDR_SZ + PLDM_SPECIFIC_HEADER_BYTES + PLDM_REQ_UPDATE_RESPONSE_DATA_BYTES;

    pldmContext->pldm_current_response_cmd = PLDM_REQUEST_UPDATE_REQ;

    if (in_update_mode == 1)
    {
        // PLDM:already in update mode
        // trace0(PLDM_// trace, SPDM_TSK, 3, "PlaU");
    }

    if (in_update_mode == 0)
    {
        in_update_mode = 1;
    }
}

/******************************************************************************/
/** function for handling pass component table message
* @param pldm_buf_tx trasmit buffer,
* @param pldmContext pldm task context
* @return None
*******************************************************************************/
void pldm_pkt_handle_pass_component_table(MCTP_PKT_BUF *pldm_buf_tx, PLDM_CONTEXT *pldmContext)
{
    MCTP_PKT_BUF *pldm_msg_rx_buf = NULL;
    pldm_msg_rx_buf = (MCTP_PKT_BUF *) &pldm_pktbuf_rx;

    memcpy(&request_pass_component_table, &(get_mctp_pld[0]), sizeof(REQUEST_PASS_COMPONENT_TABLE));
    

    pldm_buf_tx->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS] = PLDM_HDR_VERSION_PLDM_FW_UPDATE_TYPE;
    pldm_buf_tx->pkt.data[PLDM_HEADER_COMMAND_CODE_POS] = PLDM_PASS_COMPONENT_TABLE_REQ;

    REQUEST_PASS_COMPONENT_TABLE_RESPONSE req_pass_component_table_response_data;

    if (!in_update_mode)
    {
        req_pass_component_table_response_data.completion_code = NOT_IN_UPDATE_MODE;
    }
    else if (pldmContext->pldm_current_state != PLDM_LEARN_COMPONENTS)
    {
        req_pass_component_table_response_data.completion_code = INVALID_STATE_FOR_COMMAND;
    }
    else
    {
        req_pass_component_table_response_data.completion_code = PLDM_SUCCESS;
    }

    if (req_pass_component_table_response_data.completion_code == PLDM_SUCCESS) {
        if (!is_comp_iden_supported(request_pass_component_table.comp_identifier))
        {
            req_pass_component_table_response_data.component_response = PLDM_PASS_COMPONENT_TABLE_MAY_BE_UPDATED;
            req_pass_component_table_response_data.component_response_code = PLDM_PASS_COMPONENT_RESPCODE_COMP_NOT_SUPPORTED;
        }
        else
        {
            req_pass_component_table_response_data.component_response = PLDM_PASS_COMPONENT_TABLE_CAN_BE_UPDATED;
            req_pass_component_table_response_data.component_response_code = PLDM_PASS_COMPONENT_TABLE_CAN_BE_UPDATED;
        }

        if (req_pass_component_table_response_data.component_response == PLDM_PASS_COMPONENT_TABLE_CAN_BE_UPDATED)
        {
            pldmContext->pldm_previous_state = pldmContext->pldm_current_state;
            if (request_pass_component_table.transfer_flag == PLDM_TRANSFER_START ||
                    request_pass_component_table.transfer_flag == PLDM_TRANSFER_MIDDLE)
            {
                pldmContext->pldm_current_state = PLDM_LEARN_COMPONENTS;
            }
            else if (request_pass_component_table.transfer_flag == PLDM_TRANSFER_END ||
                     request_pass_component_table.transfer_flag == PLDM_TRANSFER_START_AND_END)
            {
                pldmContext->pldm_current_state = PLDM_READY_TRANSFER;
            }
        }
        else
        {
            pldmContext->pldm_previous_state = pldmContext->pldm_current_state;
            pldmContext->pldm_current_state = PLDM_LEARN_COMPONENTS;
        }
    }
    else
    {
        req_pass_component_table_response_data.component_response = 0;
        req_pass_component_table_response_data.component_response_code = 0;
    }

    memcpy(&pldm_buf_tx->pkt.data[PLDM_HEADER_COMPLETION_CODE_POS], &req_pass_component_table_response_data,
           PLDM_REQ_PASS_COMPONENT_TABLE_RESPONSE_DATA_BYTES);

    pldm_packet_size = MCTP_TOT_HDR_SZ + PLDM_SPECIFIC_HEADER_BYTES + PLDM_REQ_PASS_COMPONENT_TABLE_RESPONSE_DATA_BYTES;

    pldmContext->pldm_current_response_cmd = PLDM_PASS_COMPONENT_TABLE_REQ;
}

/******************************************************************************/
/** function for handling update component message
* @param pldm_buf_tx trasmit buffer,
* @param pldmContext pldm task context
* @return None
*******************************************************************************/
void pldm_pkt_handle_update_component(MCTP_PKT_BUF *pldm_buf_tx, PLDM_CONTEXT *pldmContext)
{
    MCTP_PKT_BUF *pldm_msg_rx_buf = NULL;
    pldm_msg_rx_buf = (MCTP_PKT_BUF *) &pldm_pktbuf_rx;
    uint8_t image_id, ap, comp, ht_num, ht_id;
    memcpy(&request_update_component, &(get_mctp_pld[0]), PLDM_UPDATE_COMP_REQUEST_SIZE);
    

    pldm_buf_tx->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS] = PLDM_HDR_VERSION_PLDM_FW_UPDATE_TYPE;
    pldm_buf_tx->pkt.data[PLDM_HEADER_COMMAND_CODE_POS] = PLDM_UPDATE_COMPONENT_REQ;

    no_of_requests = request_update_component.comp_image_size / size_supported_by_UA;
    remaining_data_size_for_transfer = request_update_component.comp_image_size;
    if (remaining_data_size_for_transfer % size_supported_by_UA)
    {
        no_of_requests = no_of_requests + 1;
        if (is_add_safe(no_of_requests, 1) == 0) // Coverity INT30-C
        {
            // Handle Error
        }
    }
    REQUEST_UPDATE_COMPONENT_RESPONSE req_update_comp_resp_data;

    if (!in_update_mode)
    {
        req_update_comp_resp_data.completion_code = NOT_IN_UPDATE_MODE;
    }
    else if (pldmContext->pldm_current_state != PLDM_READY_TRANSFER)
    {
        req_update_comp_resp_data.completion_code = INVALID_STATE_FOR_COMMAND;
    }
    else
    {
        req_update_comp_resp_data.completion_code = PLDM_SUCCESS;
    }

    if (req_update_comp_resp_data.completion_code == PLDM_SUCCESS) {
        if (!is_comp_iden_supported(request_update_component.comp_identifier))
        {
            req_update_comp_resp_data.comp_capability_resp = PLDM_UPDATE_COMP_REQ_RES_COMP_CANNOT_BE_UPDATED;
            req_update_comp_resp_data.comp_capability_resp_code = COMP_NOT_SUPPORTED;
            can_update = false;
        }
        else
        {
            req_update_comp_resp_data.comp_capability_resp = PLDM_UPDATE_COMP_REQ_RES_COMP_CAN_BE_UPDATED;
            req_update_comp_resp_data.comp_capability_resp_code = COMP_CAN_BE_UPDATED;
            can_update = true;
        }
        if (can_update  == true)
        {
            req_update_comp_resp_data.update_option_flag_enabled = 0;
            req_update_comp_resp_data.est_time_sending_reqfwdata = 0x0002;

            pldmContext->pldm_previous_state = pldmContext->pldm_current_state;
            pldmContext->pldm_current_state = PLDM_DOWNLOAD;
        }
        else
        {
            req_update_comp_resp_data.update_option_flag_enabled = 0;
            req_update_comp_resp_data.est_time_sending_reqfwdata = 0x0000;

            pldmContext->pldm_previous_state = pldmContext->pldm_current_state;
            pldmContext->pldm_current_state = PLDM_READY_TRANSFER;
        }
    }
    else
    {
        req_update_comp_resp_data.comp_capability_resp = 0;
        req_update_comp_resp_data.comp_capability_resp_code = 0;
        req_update_comp_resp_data.update_option_flag_enabled = 0;
        req_update_comp_resp_data.est_time_sending_reqfwdata = 0x0000;
        can_update = false;
    }

    memcpy(&pldm_buf_tx->pkt.data[PLDM_HEADER_COMPLETION_CODE_POS], &req_update_comp_resp_data,
           PLDM_REQ_UPDATE_COMP_RESPONSE_DATA_BYTES);

    pldm_packet_size = MCTP_TOT_HDR_SZ + PLDM_SPECIFIC_HEADER_BYTES + PLDM_REQ_UPDATE_COMP_RESPONSE_DATA_BYTES;

    pldmContext->pldm_current_response_cmd = PLDM_UPDATE_COMPONENT_REQ;

    if (can_update) {
        pldm_init_peripheral_for_update(request_update_component.comp_identifier);
    }
}

/******************************************************************************/
/** function for handling request firmware data message
* @param pldm_buf_tx trasmit buffer,
* @param pldmContext pldm task context
* @return None
*******************************************************************************/
void pldm_pkt_create_request_firmware_data(MCTP_PKT_BUF *pldm_buf_tx, PLDM_CONTEXT *pldmContext)
{
    pldm_buf_tx->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS] = PLDM_HDR_VERSION_PLDM_FW_UPDATE_TYPE;
    pldm_buf_tx->pkt.data[PLDM_HEADER_COMMAND_CODE_POS] = PLDM_REQUEST_FIRMWARE_DATA_REQ;

    REQUEST_FIRMWARE_DATA req_firmware_data;

    req_firmware_data.offset = offset;
    if (retry_request_firmware_data)
    {
        if (is_sub_safe(req_firmware_data.offset, size_supported_by_UA) == 0) // Coverity INT30-C
        {
            // Handle Error
        }
        else
        {
            req_firmware_data.offset = req_firmware_data.offset - size_supported_by_UA;
        }
    }
    else
    {
        offset = offset + size_supported_by_UA;
        if (is_add_safe(offset, size_supported_by_UA) == 0) // Coverity INT30-C
        {
            // Handle Error
        }
    }

    if (!retry_request_firmware_data)
    {
        if (remaining_data_size_for_transfer < size_supported_by_UA)
        {
            len = remaining_data_size_for_transfer;
        }
        else
        {
            len = size_supported_by_UA;
        }
    }

    req_firmware_data.length = len;
    if (!retry_request_firmware_data)
    {
        remaining_data_size_for_transfer = remaining_data_size_for_transfer - len;
    }

    if (retry_request_firmware_data)
    {
        retry_request_firmware_data = false;
    }

    memcpy(&pldm_buf_tx->pkt.data[PLDM_HEADER_DATA_POS_FOR_REQ], &req_firmware_data, PLDM_REQ_FIRMWARE_DATA_REQ_DATA_BYTES);

    pldm_packet_size = MCTP_TOT_HDR_SZ + PLDM_SPECIFIC_HEADER_BYTES + PLDM_REQ_FIRMWARE_DATA_REQ_DATA_BYTES;

    pldmContext->pldm_current_response_cmd = PLDM_REQUEST_FIRMWARE_DATA_REQ;

    pldm_response_timeout_start();
    PLDMResp_timer_started = true;
}

/******************************************************************************/
/** function for decoding the response of reqfirmwaredata and copying to flash
 * once 256bytes of data is received.
* @param data_ptr - pointer to data array
* @param size - size to copy into data array
* @return NULL
*******************************************************************************/
void pldm_pkt_process_request_firmware_update_response(void)
{
    MCTP_PKT_BUF *pldm_msg_rx_buf = NULL;
    pldm_msg_rx_buf = (MCTP_PKT_BUF *) &pldm_pktbuf_rx;

    uint8_t fn_ret=0;
    uint8_t retry_count = 0;

    if (PLDMResp_timer_started)
    {
        pldm_response_timeout_stop();
        PLDMResp_timer_started = false;
    }
    received_1024b_for_flash_write = true;

    if (!completion_code)
    {
        if (received_1024b_for_flash_write == true)
        {
            fn_ret =  pldm_write_firmware_data(request_update_component.comp_identifier, &get_mctp_pld[0], offset_for_flash);
            if(fn_ret)
            {
                return;
            }

            memset(get_mctp_pld, 0, size_supported_by_UA);
            received_1024b_for_flash_write = false;
            offset_for_flash = offset_for_flash + size_supported_by_UA;
            number_of_1024b_requests = number_of_1024b_requests + 1;
            pldm_request_firmware_update = true;
        }
    }
    else
    {
        pldm_request_firmware_update = false;
        received_1024b_for_flash_write = 0;
        offset_for_flash = 0;
        number_of_1024b_requests = 0;
        offset = 0;
        len = 0;
        pldm_reset_firmware_update_flags();
        // PLDM:req fw data resp completion code not success
    }
    return;
}

/******************************************************************************/
/** function for handling transfer complete message
* @param pldm_buf_tx trasmit buffer,
* @param pldmContext pldm task context
* @return None
*******************************************************************************/
void pldm_pkt_create_transfer_complete_req(MCTP_PKT_BUF *pldm_buf_tx, PLDM_CONTEXT *pldmContext)
{
    pldm_buf_tx->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS] = PLDM_HDR_VERSION_PLDM_FW_UPDATE_TYPE;
    pldm_buf_tx->pkt.data[PLDM_HEADER_COMMAND_CODE_POS] = PLDM_TRANSFER_COMPLETE_REQ;

    REQUEST_TRANSFER_COMPLETE req_transfer_complete;

    req_transfer_complete.transfer_result = PLDM_SUCCESS;

    memcpy(&pldm_buf_tx->pkt.data[PLDM_HEADER_DATA_POS_FOR_REQ], &req_transfer_complete,
           PLDM_REQ_TRANSFER_COMPLETE_REQ_DATA_BYTES);

    pldm_packet_size = MCTP_TOT_HDR_SZ + PLDM_SPECIFIC_HEADER_BYTES + PLDM_REQ_TRANSFER_COMPLETE_REQ_DATA_BYTES;

    pldmContext->pldm_current_response_cmd = PLDM_TRANSFER_COMPLETE_REQ;

    pldmContext->pldm_previous_state = pldmContext->pldm_current_state;
    pldmContext->pldm_current_state = PLDM_VERIFY;

    pldmContext->pldm_tx_state = PLDM_TX_IN_PROGRESS;
    pldm_pkt_tx_packet();
}

/******************************************************************************/
/** function for handling verify complete message
* @param pldm_buf_tx trasmit buffer,
* @param pldmContext pldm task context
* @return None
*******************************************************************************/
void pldm_pkt_create_verify_complete_req(MCTP_PKT_BUF *pldm_buf_tx, PLDM_CONTEXT *pldmContext)
{
    pldm_buf_tx->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS] = PLDM_HDR_VERSION_PLDM_FW_UPDATE_TYPE;
    pldm_buf_tx->pkt.data[PLDM_HEADER_COMMAND_CODE_POS] = PLDM_VERIFY_COMPLETE_REQ;

    REQUEST_VERIFY_COMPLETE req_verify_complete;

    req_verify_complete.verify_result = pldmContext->pldm_verify_state;

    memcpy(&pldm_buf_tx->pkt.data[PLDM_HEADER_DATA_POS_FOR_REQ], &req_verify_complete,
           PLDM_REQ_VERIFY_COMPLETE_REQ_DATA_BYTES);

    pldm_packet_size = MCTP_TOT_HDR_SZ + PLDM_SPECIFIC_HEADER_BYTES + PLDM_REQ_VERIFY_COMPLETE_REQ_DATA_BYTES;

    pldmContext->pldm_current_response_cmd = PLDM_VERIFY_COMPLETE_REQ;

    pldmContext->pldm_previous_state = pldmContext->pldm_current_state;

    if (!req_verify_complete.verify_result)
    {
        pldmContext->pldm_current_state = PLDM_APPLY;
    }
    else
    {
        pldmContext->pldm_current_state = PLDM_VERIFY;
    }
    if (!req_verify_complete.verify_result)
    {
        background_update_in_progress = true;
    }
    apply_completed = false;
}

/******************************************************************************/
/** function for handling apply complete message
* @param pldm_buf_tx trasmit buffer,
* @param pldmContext pldm task context
* @return None
*******************************************************************************/
void pldm_pkt_create_apply_complete_req(MCTP_PKT_BUF *pldm_buf_tx, PLDM_CONTEXT *pldmContext)
{
    pldm_buf_tx->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS] = PLDM_HDR_VERSION_PLDM_FW_UPDATE_TYPE;
    pldm_buf_tx->pkt.data[PLDM_HEADER_COMMAND_CODE_POS] = PLDM_APPLY_COMPLETE_REQ;

    REQUEST_APPLY_COMPLETE req_apply_complete;

    req_apply_complete.apply_status = pldmContext->pldm_apply_state;
    req_apply_complete.comp_activation_mthd_modification = 0x00;

    memcpy(&pldm_buf_tx->pkt.data[PLDM_HEADER_DATA_POS_FOR_REQ], &req_apply_complete,
           PLDM_REQ_APPLY_COMPLETE_REQ_DATA_BYTES);

    pldm_packet_size = MCTP_TOT_HDR_SZ + PLDM_SPECIFIC_HEADER_BYTES + PLDM_REQ_APPLY_COMPLETE_REQ_DATA_BYTES;

    pldmContext->pldm_current_response_cmd = PLDM_APPLY_COMPLETE_REQ;

    pldmContext->pldm_previous_state = pldmContext->pldm_current_state;
    if (req_apply_complete.apply_status)
    {
        pldmContext->pldm_current_state = PLDM_APPLY;
    }
    else
    {
        pldmContext->pldm_current_state = PLDM_READY_TRANSFER;
    }

    apply_completed = true;
    background_update_in_progress = false;
}

/******************************************************************************/
/** function for handling get status message
* @param pldm_buf_tx trasmit buffer,
* @param pldmContext pldm task context
* @return None
*******************************************************************************/
void pldm_pkt_handle_get_status(MCTP_PKT_BUF *pldm_buf_tx, PLDM_CONTEXT *pldmContext)
{
    pldm_buf_tx->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS] = PLDM_HDR_VERSION_PLDM_FW_UPDATE_TYPE;
    pldm_buf_tx->pkt.data[PLDM_HEADER_COMMAND_CODE_POS] = PLDM_REQUEST_GET_STATUS;

    GET_STATUS_REQ get_status_req_response_data;

    get_status_req_response_data.completion_code = 0x00;
    get_status_req_response_data.current_state = pldmContext->pldm_current_state;
    get_status_req_response_data.previous_state = pldmContext->pldm_previous_state;
    get_status_req_response_data.aux_state = 0x00;
    get_status_req_response_data.aux_state_status = 0x00;
    get_status_req_response_data.progress_percent = PROGRESS_PERCENT;
    get_status_req_response_data.reason_code = pldmContext->pldm_status_reason_code;
    get_status_req_response_data.update_option_flags_enabled = 0x00000000;

    memcpy(&pldm_buf_tx->pkt.data[PLDM_HEADER_COMPLETION_CODE_POS], &get_status_req_response_data,
           PLDM_REQ_GET_STATUS_RESPONSE_DATA_BYTES);

    pldm_packet_size = MCTP_TOT_HDR_SZ + PLDM_SPECIFIC_HEADER_BYTES + PLDM_REQ_GET_STATUS_RESPONSE_DATA_BYTES;

    pldmContext->pldm_current_response_cmd = PLDM_REQUEST_GET_STATUS;
}

/******************************************************************************/
/** function for handling activate firmware message
* @param pldm_buf_tx trasmit buffer,
* @param pldmContext pldm task context
* @return None
*******************************************************************************/
void pldm_pkt_handle_activate_firmware(MCTP_PKT_BUF *pldm_buf_tx, PLDM_CONTEXT *pldmContext)
{
    MCTP_PKT_BUF *pldm_msg_rx_buf = NULL;
    pldm_msg_rx_buf = (MCTP_PKT_BUF *) &pldm_pktbuf_rx;

    memcpy(&request_activate_firmware, &get_mctp_pld[0], PLDM_REQUEST_ACTIVATE_FIRMWARE_LEN);


    pldm_buf_tx->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS] = PLDM_HDR_VERSION_PLDM_FW_UPDATE_TYPE;
    pldm_buf_tx->pkt.data[PLDM_HEADER_COMMAND_CODE_POS] = PLDM_ACTIVATE_FIRMWARE_REQ;

    REQUEST_ACTIVATE_FIRMWARE_RESP req_activate_firmware_resp;

    pldmContext->pldm_previous_state = pldmContext->pldm_current_state;

    if (pldmContext->pldm_current_state != PLDM_READY_TRANSFER)
    {
        req_activate_firmware_resp.completion_code = INCOMPLETE_UPDATE;
        can_activate = false;
        req_activate_firmware_resp.est_time_for_self_contained_activation = 0x0000;
    }
    else if (in_update_mode == false)
    {
        req_activate_firmware_resp.completion_code = NOT_IN_UPDATE_MODE;
        can_activate = false;
        req_activate_firmware_resp.est_time_for_self_contained_activation = 0x0000;
    }
    else if(request_activate_firmware.self_contained_activation_request == false)
    {
        req_activate_firmware_resp.completion_code = PLDM_SUCCESS;
        can_activate = false;
        pldmContext->pldm_current_state = PLDM_IDLE;
        pldmContext->pldm_status_reason_code = ACTIVATE_FIRMWARE_RECEIVED;
        req_activate_firmware_resp.est_time_for_self_contained_activation = 0x0000;
        in_update_mode = false;
        pldm_restore_configs(request_update_component.comp_identifier, host_functionality_reduced);
    }
    else
    {
        req_activate_firmware_resp.completion_code = PLDM_SUCCESS;
        pldmContext->pldm_current_state = PLDM_ACTIVATE;
        can_activate = true;
        in_update_mode = false;
        req_activate_firmware_resp.est_time_for_self_contained_activation = 0x0005;
    }

    memcpy(&pldm_buf_tx->pkt.data[PLDM_HEADER_COMPLETION_CODE_POS], &req_activate_firmware_resp,
           PLDM_REQ_ACTIVATE_FIRMWARE_RESPONSE_DATA_BYTES);

    pldm_packet_size = MCTP_TOT_HDR_SZ + PLDM_SPECIFIC_HEADER_BYTES + PLDM_REQ_ACTIVATE_FIRMWARE_RESPONSE_DATA_BYTES;

    pldmContext->pldm_current_response_cmd = PLDM_ACTIVATE_FIRMWARE_REQ;
}

/******************************************************************************/
/** function for handling cancel update component message
* @param pldm_buf_tx trasmit buffer,
* @param pldmContext pldm task context
* @return None
*******************************************************************************/
void pldm_pkt_create_response_cancel_update_component(MCTP_PKT_BUF *pldm_buf_tx, PLDM_CONTEXT *pldmContext)
{
    uint8_t ret_Val;
    pldm_buf_tx->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS] = PLDM_HDR_VERSION_PLDM_FW_UPDATE_TYPE;
    pldm_buf_tx->pkt.data[PLDM_HEADER_COMMAND_CODE_POS] = PLDM_CANCEL_UPDATE_COMPONENT_REQ;

    REQUEST_CANCEL_UPDATE_COMPONENT_RESPONSE req_cancel_update_comp_res;

    if (!in_update_mode)
    {
        req_cancel_update_comp_res.completion_code = NOT_IN_UPDATE_MODE;
    }
    else if (background_update_in_progress)
    {
        req_cancel_update_comp_res.completion_code = BUSY_IN_BACKGROUND;
    }
    else
    {
        req_cancel_update_comp_res.completion_code = PLDM_SUCCESS;
    }

    memcpy(&pldm_buf_tx->pkt.data[PLDM_HEADER_COMPLETION_CODE_POS], &req_cancel_update_comp_res,
           PLDM_REQ_CANCEL_UPDATE_COMP_RESPONSE_DATA_BYTES);

    pldm_packet_size = MCTP_TOT_HDR_SZ + PLDM_SPECIFIC_HEADER_BYTES + PLDM_REQ_CANCEL_UPDATE_COMP_RESPONSE_DATA_BYTES;

    pldmContext->pldm_current_response_cmd = PLDM_CANCEL_UPDATE_COMPONENT_REQ;

    pldm_request_firmware_update = false;
    received_1024b_for_flash_write = 0;
    offset_for_flash = 0;
    number_of_1024b_requests = 0;
    offset = 0;
    len = 0;
    if (PLDMResp_timer_started)
    {
        pldm_response_timeout_stop();
        PLDMResp_timer_started = false;
    }

    pldmContext->pldm_previous_state = pldmContext->pldm_current_state;

    if (apply_completed && failure_recovery_cap)
    {
        ret_Val = pldm_cancel_update(request_update_component.comp_identifier, 0);
        apply_completed = false;
    }

    pldmContext->pldm_current_state = PLDM_READY_TRANSFER;
}

/******************************************************************************/
/** function for handling cancel update message
* @param pldm_buf_tx trasmit buffer,
* @param pldmContext pldm task context
* @return None
*******************************************************************************/
void pldm_pkt_create_response_cancel_update(MCTP_PKT_BUF *pldm_buf_tx, PLDM_CONTEXT *pldmContext)
{
    uint8_t ret_Val;
    pldm_buf_tx->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS] = PLDM_HDR_VERSION_PLDM_FW_UPDATE_TYPE;
    pldm_buf_tx->pkt.data[PLDM_HEADER_COMMAND_CODE_POS] = PLDM_CANCEL_UPDATE_REQ;

    REQUEST_CANCEL_UPDATE_RESPONSE req_cancel_update_res;
    if (!in_update_mode)
    {
        req_cancel_update_res.completion_code = NOT_IN_UPDATE_MODE;
    }
    else if (background_update_in_progress)
    {
        req_cancel_update_res.completion_code = BUSY_IN_BACKGROUND;
    }
    else
    {
        req_cancel_update_res.completion_code = PLDM_SUCCESS;
    }

    pldmContext->pldm_current_response_cmd = PLDM_CANCEL_UPDATE_REQ;

    pldm_request_firmware_update = false;
    received_1024b_for_flash_write = 0;
    offset_for_flash = 0;
    number_of_1024b_requests = 0;
    offset = 0;
    len = 0;
    if (PLDMResp_timer_started)
    {
        pldm_response_timeout_stop();
        PLDMResp_timer_started = false;
    }

    if (apply_completed && failure_recovery_cap)
    {
        ret_Val = pldm_cancel_update(request_update_component.comp_identifier, 1);
        req_cancel_update_res.non_func_comp_indication = ret_Val;
        if (!req_cancel_update_res.non_func_comp_indication)
        {
            if (request_update_component.comp_identifier == PLDM_COMP_IDENTIFIER_TAG0)
            {
                req_cancel_update_res.non_func_comp_bitmap |= (1 << 0);
            }
            else if (request_update_component.comp_identifier == PLDM_COMP_IDENTIFIER_TAG1)
            {
                req_cancel_update_res.non_func_comp_bitmap |= (1 << 1);
            }
        }
        else
        {
            req_cancel_update_res.non_func_comp_bitmap = 0x0;
        }
        apply_completed = false;
    }
    else
    {
        req_cancel_update_res.non_func_comp_indication = false;
        req_cancel_update_res.non_func_comp_bitmap = 0x0;
    }
    pldmContext->pldm_previous_state = pldmContext->pldm_current_state;
    pldmContext->pldm_current_state = PLDM_IDLE_STATE;
    pldmContext->pldm_status_reason_code = CANCEL_UPDATE_RECEIVED;
    in_update_mode = false;

    memcpy(&pldm_buf_tx->pkt.data[PLDM_HEADER_COMPLETION_CODE_POS], &req_cancel_update_res,
           PLDM_REQ_CANCEL_UPDATE_RESPONSE_DATA_BYTES);

    pldm_packet_size = MCTP_TOT_HDR_SZ + PLDM_SPECIFIC_HEADER_BYTES + PLDM_REQ_CANCEL_UPDATE_RESPONSE_DATA_BYTES;

    if (req_cancel_update_res.completion_code == PLDM_SUCCESS)
    {
        pldm_restore_configs(request_update_component.comp_identifier, host_functionality_reduced);
    }
}

/******************************************************************************/
/** function for handling response of transfer request
* @param pldm_buf_tx trasmit buffer,
* @param pldmContext pldm task context
* @return None
*******************************************************************************/
void pldm_pkt_handle_transfer_complete_response(MCTP_PKT_BUF *pldm_buf_tx, PLDM_CONTEXT *pldmContext)
{
    MCTP_PKT_BUF *pldm_msg_rx_buf = NULL;
    pldm_msg_rx_buf = (MCTP_PKT_BUF *) &pldm_pktbuf_rx;

    REQUEST_TRANSFER_COMPLETE_RESPONSE transfer_complete_response;

    transfer_complete_response.completion_code = pldm_msg_rx_buf->pkt.data[PLDM_HEADER_DATA_POS_FOR_RESP];

    pldm_start_firmware_update(request_update_component.comp_identifier);
}

/******************************************************************************/
/** function for handling response of verify request
* @param pldm_buf_tx trasmit buffer,
* @param pldmContext pldm task context
* @return None
*******************************************************************************/
void pldm_pkt_handle_verify_complete_response(MCTP_PKT_BUF *pldm_buf_tx, PLDM_CONTEXT *pldmContext)
{
    MCTP_PKT_BUF *pldm_msg_rx_buf = NULL;
    pldm_msg_rx_buf = (MCTP_PKT_BUF *) &pldm_pktbuf_rx;

    REQUEST_VERIFY_COMPLETE_RESPONSE verify_complete_response;

    verify_complete_response.completion_code = pldm_msg_rx_buf->pkt.data[PLDM_HEADER_DATA_POS_FOR_RESP];

    pldm_start_firmware_apply();
}

/******************************************************************************/
/** function for handling response of apply request
* @param pldm_buf_tx trasmit buffer,
* @param pldmContext pldm task context
* @return None
*******************************************************************************/
void pldm_pkt_handle_apply_complete_response(MCTP_PKT_BUF *pldm_buf_tx, PLDM_CONTEXT *pldmContext)
{
    MCTP_PKT_BUF *pldm_msg_rx_buf = NULL;
    pldm_msg_rx_buf = (MCTP_PKT_BUF *) &pldm_pktbuf_rx;

    REQUEST_APPLY_COMPLETE_RESPONSE apply_complete_response;

    apply_complete_response.completion_code = pldm_msg_rx_buf->pkt.data[PLDM_HEADER_DATA_POS_FOR_RESP];
    pldmContext->pldm_tx_state = PLDM_TX_IDLE;
    pldm_apply_complete_response(request_update_component.comp_identifier);
}

/******************************************************************************/
/** function for encoding get tid (terminal id) response to UA
* @param msg trasmit buffer
* @param tid terminal id
* @param instance_id instance id
* @param completion_code completion code for get tid request
* @return success or invalid data
*******************************************************************************/
int encode_get_tid_resp(uint8_t instance_id, uint8_t completion_code,
                        uint8_t tid, MCTP_PKT_BUF *msg)
{
    if (msg == NULL)
    {
        return PLDM_ERROR_INVALID_DATA;
    }

    msg->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS] = PLDM_HDR_VERSION_PLDM_DIS_CONTROL_TYPE;
    msg->pkt.data[PLDM_HEADER_COMMAND_CODE_POS] = PLDM_GET_ID_REQ;

    void* ptr = (void*)&msg->pkt.data[PLDM_HEADER_COMPLETION_CODE_POS];
    struct pldm_get_tid_resp *response = (struct pldm_get_tid_resp *)ptr; // Coverity EXP39-C
    response->completion_code = completion_code;
    response->tid = tid;

    pldm_packet_size = MCTP_TOT_HDR_SZ + PLDM_SPECIFIC_HEADER_BYTES + PLDM_REQ_GET_TID_RESPONSE_DATA_BYTES;
    total_length = PLDM_REQ_GET_TID_RESPONSE_DATA_BYTES;
    memcpy(buffer, response, PLDM_REQ_GET_TID_RESPONSE_DATA_BYTES);
    return PLDM_SUCCESS;
}

/******************************************************************************/
/** function for handling request firmware data message
* @param pldm_buf_tx trasmit buffer,
* @param pldmContext pldm task context
* @return None
*******************************************************************************/
void pldm_handle_get_id(MCTP_PKT_BUF *pldm_buf, PLDM_CONTEXT *pldmContext)
{
    uint8_t instance_id = 0;
    uint8_t comp_code = 0;
    uint8_t tid;

    tid = pldm_apcfg_tid();

    pldmContext->pldm_current_response_cmd = PLDM_GET_ID_REQ;
    encode_get_tid_resp(instance_id, comp_code, tid, pldm_buf);
}

/******************************************************************************/
/** function for decoding get version message request from UA
* @param msg trasmit buffer
* @param payload_length length of version
* @param instance_id instance id
* @return success or invalid data
*******************************************************************************/
int decode_get_version_req(MCTP_PKT_BUF *msg, size_t payload_length,
                           uint32_t *transfer_handle, uint8_t *transfer_opflag,
                           uint8_t *type)
{

    MCTP_PKT_BUF *pldm_msg_rx_buf = NULL;
    pldm_msg_rx_buf = (MCTP_PKT_BUF *) &pldm_pktbuf_rx;

    if (payload_length != PLDM_GET_VERSION_REQ_BYTES)
    {
        return PLDM_ERROR_INVALID_LENGTH;
    }

    void *ptr = (void*)&(pldm_msg_rx_buf->pkt.data[PLDM_HEADER_DATA_POS_FOR_REQ]);
    struct pldm_get_version_req *request = (struct pldm_get_version_req *)ptr;
    *transfer_handle = (request->transfer_handle);
    *transfer_opflag = request->transfer_opflag;
    *type = request->type;

    return PLDM_SUCCESS;
}

/******************************************************************************/
/** function for encoding get version message response to UA
* @param msg trasmit buffer
* @return success or invalid data
*******************************************************************************/
int encode_get_version_resp(uint8_t instance_id, uint8_t completion_code,
                            uint32_t next_transfer_handle,
                            uint8_t transfer_flag, ver32_t *version_data,
                            size_t version_size, MCTP_PKT_BUF *msg)
{
    if (NULL == msg)
    {
        return PLDM_ERROR_INVALID_DATA;
    }

    msg->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS] = PLDM_HDR_VERSION_PLDM_DIS_CONTROL_TYPE;
    msg->pkt.data[PLDM_HEADER_COMMAND_CODE_POS] = PLDM_GET_PLDM_VERSION_REQ;

    void* ptr = (void*)&msg->pkt.data[PLDM_HEADER_COMPLETION_CODE_POS];
    struct pldm_get_version_resp *response =(struct pldm_get_version_resp *)ptr; // Coverity EXP39-C
    response->completion_code = completion_code;
    if (response->completion_code == PLDM_SUCCESS)
    {
        response->next_transfer_handle = (next_transfer_handle);
        response->transfer_flag = transfer_flag;
        memcpy(response->version_data, (void *)version_data,
               version_size);
    }

    pldm_packet_size = MCTP_TOT_HDR_SZ + PLDM_SPECIFIC_HEADER_BYTES + PLDM_REQ_GET_PLDM_VERSION_DATA_BYTES;
    return PLDM_SUCCESS;
}

/******************************************************************************/
/** function for handling get version message
* @param pldm_buf_tx trasmit buffer,
* @param pldmContext pldm task context
* @return None
*******************************************************************************/
void pldm_handle_get_version(MCTP_PKT_BUF *pldm_buf, PLDM_CONTEXT *pldmContext)
{
    MCTP_PKT_BUF *pldm_msg_rx_buf = NULL;
    pldm_msg_rx_buf = (MCTP_PKT_BUF *) &pldm_pktbuf_rx;

    uint8_t completion_code = 0;

    uint8_t payload_length = 6;
    ver32_t version = {0xFF, 0xFF, 0xFF, 0xFF};

    struct pldm_get_version_req pldm_get_version_req;

    decode_get_version_req(pldm_msg_rx_buf, payload_length,
                           &pldm_get_version_req.transfer_handle, &pldm_get_version_req.transfer_opflag,
                           &pldm_get_version_req.type);

    pldmContext->pldm_current_response_cmd = PLDM_GET_PLDM_VERSION_REQ;

    if (pldm_get_version_req.type == PLDM_FIRMWARE_UPDATE) {
        completion_code = PLDM_SUCCESS;
        version.alpha = 0x00;
        version.update = 0xF0;
        version.minor = 0xF1;
        version.major = 0xF1;
    } else if (pldm_get_version_req.type == PLDM_CONTROL_AND_DISCOVERY) {
        completion_code = PLDM_SUCCESS;
        version.alpha = 0x00;
        version.update = 0xF0;
        version.minor = 0xF1;
        version.major = 0xF1;
    } else {
        completion_code = PLDM_ERROR_INVALID_PLDM_TYPE;
        version.alpha = 0xFF;
        version.update = 0xFF;
        version.minor = 0xFF;
        version.major = 0xFF;
    }
    encode_get_version_resp(0, completion_code, 0, 0, &version, sizeof(ver32_t), pldm_buf);
}

/******************************************************************************/
/** function for encoding get PLDM types response
* @param pldm_buf_tx trasmit buffer,
* @param completion_code completion code
* @return None
*******************************************************************************/
int encode_get_types_resp(uint8_t instance_id, uint8_t completion_code,
                          uint8_t *types, struct MCTP_PKT_BUF *msg)
{
    if (msg == NULL)
    {
        return PLDM_ERROR_INVALID_DATA;
    }

    msg->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS] = PLDM_HDR_VERSION_PLDM_DIS_CONTROL_TYPE;
    msg->pkt.data[PLDM_HEADER_COMMAND_CODE_POS] = PLDM_GET_PLDM_TYPES;

    void* ptr = (void*)&msg->pkt.data[PLDM_HEADER_COMPLETION_CODE_POS];
    struct pldm_get_types_resp *response =(struct pldm_get_types_resp *)ptr;
    response->completion_code = completion_code;
    if (response->completion_code == PLDM_SUCCESS)
    {
        if (types == NULL)
        {
            return PLDM_ERROR_INVALID_DATA;
        }
        memcpy(response->types, (types), PLDM_MAX_TYPES / 8);
    }

    pldm_packet_size = MCTP_TOT_HDR_SZ + PLDM_SPECIFIC_HEADER_BYTES + PLDM_REQ_GET_PLDM_TYPES_DATA_BYTES;
    return PLDM_SUCCESS;
}

/******************************************************************************/
/** function for handling get PLDM types message
* @param pldm_buf_tx trasmit buffer,
* @param pldmContext pldm task context
* @return None
*******************************************************************************/
void pldm_handle_get_pldm_types(MCTP_PKT_BUF *pldm_buf, PLDM_CONTEXT *pldmContext)
{
    uint8_t types[8];
    memset(types, 0, 8);
    types[0] = 0x21;
    pldmContext->pldm_current_response_cmd = PLDM_GET_PLDM_TYPES;
    encode_get_types_resp(INSTANCE_ID, 0, types, pldm_buf);
}

/******************************************************************************/
/** function for encoding get commands response
* @param msg trasmit buffer
* @return None
*******************************************************************************/
int encode_get_commands_resp(uint8_t instance_id, uint8_t completion_code,
                             uint8_t *commands, MCTP_PKT_BUF *msg)
{
    if (msg == NULL)
    {
        return PLDM_ERROR_INVALID_DATA;
    }

    msg->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS] = PLDM_HDR_VERSION_PLDM_DIS_CONTROL_TYPE;
    msg->pkt.data[PLDM_HEADER_COMMAND_CODE_POS] = PLDM_GET_PLDM_COMMANDS;

    // Coverity EXP39-C
    void* ptr = (void* )&msg->pkt.data[PLDM_HEADER_COMPLETION_CODE_POS];
    struct pldm_get_commands_resp *response =(struct pldm_get_commands_resp *)ptr;
    response->completion_code = completion_code;
    if (response->completion_code == PLDM_SUCCESS)
    {
        if (commands == NULL)
        {
            return PLDM_ERROR_INVALID_DATA;
        }
        memcpy(response->commands, (commands),
               PLDM_MAX_CMDS_PER_TYPE / 8);
    }

    pldm_packet_size = MCTP_TOT_HDR_SZ + PLDM_SPECIFIC_HEADER_BYTES + PLDM_REQ_GET_PLDM_COMMANDS_DATA_BYTES;

    return PLDM_SUCCESS;
}

/******************************************************************************/
/** function for handling get pldm commands message
* @param pldm_buf_tx trasmit buffer,
* @param pldmContext pldm task context
* @return None
*******************************************************************************/
void pldm_handle_get_pldm_commands(MCTP_PKT_BUF *pldm_buf, PLDM_CONTEXT *pldmContext)
{
    uint8_t commands[32];
    memset(commands, 0, 32);
    commands[0] = 0x03;
    commands[1] = 0x00;
    commands[2] = 0xF9;
    commands[3] = 0x3D;

    pldmContext->pldm_current_response_cmd = PLDM_GET_PLDM_COMMANDS;
    encode_get_commands_resp(INSTANCE_ID, PLDM_SUCCESS, commands, pldm_buf);
}

/******************************************************************************/
/** function for routing packets based on PLDM type (0 or 5) and command code
* @param pldm_type type 0 discovery messages, type 5 fw update messages
* @param get_cmd command code
* @param pldm_buf_tx buffer for transmitting message
* @param pldmConext PLDM task context
* @return error_handle if pldm context is NULL
*******************************************************************************/
uint8_t pldm_pkt_validate_and_process_pldm_msg(uint8_t pldm_type, uint8_t get_cmd, MCTP_PKT_BUF *pldm_buf_tx,
        PLDM_CONTEXT *pldmContext)
{
    uint8_t get_ver = 0x00;
    uint8_t error_handle = 0x00;
    uint8_t ret_val =0;

    if(NULL == pldmContext)
    {
        return 0xff;
    }

    if (pldm_type == PLDM_FIRMWARE_UPDATE)
    {
        // PLDM:cmd rcvd is 
        // trace1(PLDM_// trace, SPDM_TSK, 2, "PcRx:%x", get_cmd);
        switch(get_cmd)
        {
        case PLDM_QUERY_DEVICE_IDENTIFIERS_REQ:
            // query device identfs
            // trace0(PLDM_// trace, SPDM_TSK, 2, "Pqdi");
            pldm_pkt_handle_query_device_identifiers(pldm_buf_tx, pldmContext);
            break;

        case PLDM_GET_FIRMWARE_PARAMETERS_REQ:
            // get fw parms
            // trace0(PLDM_// trace, SPDM_TSK, 2, "PGfpq");
            pldm_pkt_handle_get_firmware_parameters(pldm_buf_tx, pldmContext);
            break;

        case PLDM_REQUEST_UPDATE_REQ:
            // rqst update
            // trace0(PLDM_// trace, SPDM_TSK, 2, "PrUq");
            pldm_pkt_handle_request_update(pldm_buf_tx, pldmContext);
            break;

        case PLDM_REQUEST_FIRMWARE_DATA_REQ: // response from UA, having the data
            // rqst fw data resp
            // trace0(PLDM_// trace, SPDM_TSK, 2, "Prfdr");
            pldm_pkt_process_request_firmware_update_response();
            break;

        case PLDM_PASS_COMPONENT_TABLE_REQ:
            pldm_pkt_handle_pass_component_table(pldm_buf_tx, pldmContext);
            // pass comp table
            // trace0(PLDM_// trace, SPDM_TSK, 3, "PPcTr");
            break;

        case PLDM_UPDATE_COMPONENT_REQ:
            // update comp
            // trace0(PLDM_// trace, SPDM_TSK, 2, "PUco");
            pldm_pkt_handle_update_component(pldm_buf_tx, pldmContext);
            break;

        case PLDM_ACTIVATE_FIRMWARE_REQ:
            // activate firmware
            // trace0(PLDM_// trace, SPDM_TSK, 2, "Pafw");
            pldm_pkt_handle_activate_firmware(pldm_buf_tx, pldmContext);
            break;

        case PLDM_CANCEL_UPDATE_COMPONENT_REQ:
            // cacl update comp
            // trace0(PLDM_// trace, SPDM_TSK, 2, "PcUq");
            pldm_pkt_create_response_cancel_update_component(pldm_buf_tx, pldmContext);
            break;

        case PLDM_CANCEL_UPDATE_REQ:
            // cacl update
            // trace0(PLDM_// trace, SPDM_TSK, 2, "PclU");
            pldm_pkt_create_response_cancel_update(pldm_buf_tx, pldmContext);
            break;

        case PLDM_REQUEST_GET_STATUS:
            // get status
            // trace0(PLDM_// trace, SPDM_TSK, 2, "PqGs");
            pldm_pkt_handle_get_status(pldm_buf_tx, pldmContext);
            break;

        case PLDM_TRANSFER_COMPLETE_REQ:
            // xfr cmplt resp
            // trace0(PLDM_// trace, SPDM_TSK, 2, "PxCr");
            pldm_pkt_handle_transfer_complete_response(pldm_buf_tx, pldmContext);
            break;

        case PLDM_VERIFY_COMPLETE_REQ:
            // verify cmpt resp
            // trace0(PLDM_// trace, SPDM_TSK, 2, "PvCq");
            pldm_pkt_handle_verify_complete_response(pldm_buf_tx, pldmContext);
            break;

        case PLDM_APPLY_COMPLETE_REQ:
            // apply cmpt resp
            // trace0(PLDM_// trace, SPDM_TSK, 2, "PaCq");
            pldm_pkt_handle_apply_complete_response(pldm_buf_tx, pldmContext);
            break;

        default:
            // PLDM:invalid type 5 cmd code
            // trace0(PLDM_// trace, SPDM_TSK, 3, "PIvcc");
            return 0xff;
        }
    }
    else if (pldm_type == PLDM_CONTROL_AND_DISCOVERY)
    {
        switch(get_cmd)
        {
        case PLDM_GET_ID_REQ:
            pldm_handle_get_id(pldm_buf_tx, pldmContext);
            break;

        case PLDM_GET_PLDM_VERSION_REQ:
            pldm_handle_get_version(pldm_buf_tx, pldmContext);
            break;

        case PLDM_GET_PLDM_TYPES:
            pldm_handle_get_pldm_types(pldm_buf_tx, pldmContext);
            break;

        case PLDM_GET_PLDM_COMMANDS:
            pldm_handle_get_pldm_commands(pldm_buf_tx, pldmContext);
            break;

        default:
            // PLDM:ivld type 0 cmd code
            // trace0(PLDM_// trace, SPDM_TSK, 3, "PIvt");
            return 0xff;
        }
    }

    if (get_cmd != PLDM_TRANSFER_COMPLETE_REQ && get_cmd != PLDM_VERIFY_COMPLETE_REQ && get_cmd != PLDM_APPLY_COMPLETE_REQ)
    {
        pldmContext->pldm_tx_state = PLDM_TX_IN_PROGRESS;
    }
    return 0x00;
}

/******************************************************************************/
/** function for populating MCTP packets for PLDM message
* @param pldm_buf_tx - transmit buffer
* @param mctp_buf - final MCTP buffer
* @param cmd_resp - command code
* @param pldmContext - PLDM task context
* @param req_bit - request or response message
* @return NULL
*******************************************************************************/
void pldm_pkt_populate_mctp_packet_for_resp(MCTP_PKT_BUF *pldm_buf_tx, MCTP_PKT_BUF *mctp_buf, uint8_t cmd_resp,
        PLDM_CONTEXT *pldmContext, bool req_bit)
{
    uint8_t local=0x0u;
    if (pldmContext == NULL)
    {
        return;
    }

    if (cmd_resp == PLDM_REQUEST_FIRMWARE_DATA_REQ)
    {
        pldm_pkt_seq_mctp = 0;
        pldm_first_pkt = true; // specific scenario where after update comp response is sent, FD sends req fw data
        // request, this is a new msg and shouldn't be sent as multiple pkt
    }

    if (pldm_first_pkt)
    {
        pldm_first_pkt = false;
        mctp_buf->pkt.field.hdr.som      = 1;
    }
    else
    {
        mctp_buf->pkt.field.hdr.som      = 0;
    }

    mctp_buf->pkt.field.hdr.msg_type   = MCTP_MSGTYPE_PLDM;

    /* integrity check */
    mctp_buf->pkt.field.hdr.integrity_check = 0;

    /* for req/response packet */
    if(req_bit==false)
    {
        local =0x0u;
    }
    else{
        local =0x1u;
    }
    mctp_buf->pkt.field.hdr.req_bit    = (uint8_t)(local&UINT8_MAX);;
    mctp_buf->pkt.field.hdr.dgram_bit  = 0;
    /* mctp reserved */
    mctp_buf->pkt.field.hdr.rsvd       = 0;
    /* instance id */
    if (req_bit)
    {
        mctp_buf->pkt.field.hdr.inst_id = INSTANCE_ID;
    }
    else
    {
        mctp_buf->pkt.field.hdr.inst_id = pldmContext->pldm_instance_id & 0x1F;
    }

    mctp_buf->rx_smbus_timestamp  = pldm_buf_tx->rx_smbus_timestamp;

    if (cmd_resp == PLDM_QUERY_DEVICE_IDENTIFIERS_REQ || cmd_resp == PLDM_GET_FIRMWARE_PARAMETERS_REQ)
    {
        if (total_length > MAX_NO_OF_PLDM_PAYLOAD)
        {
            mctp_buf->pkt.field.hdr.byte_cnt = MCTP_BYTECNT_MAX;
            mctp_buf->pkt.field.hdr.eom      = 0;
            if (mctp_buf->pkt.field.hdr.som)
            {
                memcpy(&pldm_buf_tx->pkt.data[PLDM_HEADER_COMPLETION_CODE_POS], &buffer[offset_for_data], MAX_NO_OF_PLDM_PAYLOAD);
                memcpy(&mctp_buf->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS],
                       &pldm_buf_tx->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS], NO_OF_BYTES_PLDM_HEADER_TYPE_CMD_CODE_AND_PAYLOAD);
                total_length = total_length - MAX_NO_OF_PLDM_PAYLOAD;
                offset_for_data = (uint16_t)((offset_for_data + MAX_NO_OF_PLDM_PAYLOAD)&UINT16_MAX);
            }
            else
            {
                memcpy(&pldm_buf_tx->pkt.data[PLDM_PAYLOAD_START_MUTLIPLE_PKT_POS],  &buffer[offset_for_data],
                       MAX_NO_OF_PLDM_PAYLOAD_NOT_FIRST_PKT);
                memcpy(&mctp_buf->pkt.data[PLDM_PAYLOAD_START_MUTLIPLE_PKT_POS], &buffer[offset_for_data],
                       MAX_NO_OF_PLDM_PAYLOAD_NOT_FIRST_PKT);
                total_length = (uint16_t)((total_length - MAX_NO_OF_PLDM_PAYLOAD_NOT_FIRST_PKT)&UINT16_MAX);
                offset_for_data = (uint16_t)((offset_for_data + MAX_NO_OF_PLDM_PAYLOAD_NOT_FIRST_PKT)&UINT16_MAX);
            }
        }
        else
        {
            mctp_buf->pkt.field.hdr.eom      = 1;
            if (mctp_buf->pkt.field.hdr.som)
            {
                memcpy(&pldm_buf_tx->pkt.data[PLDM_HEADER_COMPLETION_CODE_POS], &buffer[offset_for_data], total_length);
                mctp_buf->pkt.field.hdr.byte_cnt = total_length + BYTE_CNT_FOR_ONE_PKT;
                memcpy(&mctp_buf->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS],
                       &pldm_buf_tx->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS], total_length + 2);
            }
            else
            {
                memcpy(&pldm_buf_tx->pkt.data[PLDM_PAYLOAD_START_MUTLIPLE_PKT_POS],  &buffer[offset_for_data], total_length);
                mctp_buf->pkt.field.hdr.byte_cnt = total_length + BYTE_CNT_FOR_LAST_PKT_MSG;
                memcpy(&mctp_buf->pkt.data[PLDM_PAYLOAD_START_MUTLIPLE_PKT_POS], &buffer[offset_for_data], total_length);
            }
        }
    }

    /* destination slave address */
    mctp_buf->pkt.field.hdr.dst_addr  = pldmContext->pldm_host_slv_addr;
    mctp_buf->pkt.field.hdr.rw_dst    = 0;
    /*  Command Code */
    mctp_buf->pkt.field.hdr.cmd_code  = MCTP_SMBUS_HDR_CMD_CODE;
    /* Source Slave address*/
    mctp_buf->pkt.field.hdr.src_addr  = pldmContext->pldm_ec_slv_addr;
    mctp_buf->pkt.field.hdr.ipmi_src  = 1;
    /* mctp reserved */
    mctp_buf->pkt.field.hdr.mctp_rsvd = 0;
    /* header version supported by this implementation */
    mctp_buf->pkt.field.hdr.hdr_ver   = 1;
    /* destination eid */
    mctp_buf->pkt.field.hdr.dst_eid   = pldmContext->pldm_host_eid;
    /* source eid = eid of self/EC */
    mctp_buf->pkt.field.hdr.src_eid = curr_ec_id;
    /* message tag */
    /* for req/response packet */
    if (req_bit)
    {
        mctp_buf->pkt.field.hdr.tag_owner = 1;
    }
    else
    {
        mctp_buf->pkt.field.hdr.tag_owner = 0;
    }
    /* Packet sequence number */
    mctp_buf->pkt.field.hdr.pkt_seq  = pldm_pkt_seq_mctp;

    if (cmd_resp != PLDM_QUERY_DEVICE_IDENTIFIERS_REQ && cmd_resp != PLDM_GET_FIRMWARE_PARAMETERS_REQ)
    {
        mctp_buf->pkt.field.hdr.eom      = 1;
        mctp_buf->pkt.field.hdr.byte_cnt = (uint8_t)(pldm_packet_size&UINT8_MAX);
        memcpy(&mctp_buf->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS],
               &pldm_buf_tx->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS], (MCTP_BYTECNT_MAX - MCTP_BYTECNT_MIN));
        pldm_packet_size = 0;
    }
    pldm_pkt_seq_mctp += 1;
    if (pldm_pkt_seq_mctp > 3) pldm_pkt_seq_mctp = 0;
}

/******************************************************************************/
/** once pldm msg is ready to transmit, trigger MCTP
* @param buf - transmit buffer
* @param pldmContext - PLDM task context
* @return void
*******************************************************************************/
void pldm_pkt_msg_ready_trigger_mctp(MCTP_PKT_BUF *buf, PLDM_CONTEXT *pldmContext)
{
    if(NULL == pldmContext)
    {
        return;
    }


    buf->buf_full = MCTP_TX_PENDING;

    if((buf->pkt.field.hdr.eom != 1) || (pldm_request_firmware_update))
    {
        pldmContext->pldm_tx_state = PLDM_TX_IN_PROGRESS;
    }
    else
    {
        pldm_first_pkt = true;
        pldm_pkt_seq_mctp = 0;
        pldmContext->pldm_tx_state = MCTP_TX_IDLE;
        pldm_pktbuf_rx.buf_full = MCTP_EMPTY;
        offset_for_data = 0;
    }
    di_send_spdm_reponse_packet((uint8_t*)buf, sizeof(MCTP_PKT_BUF), true, pldm_request_firmware_update);
}

/******************************************************************************/
/** once pldm msg is ready to transmit, pldm state machine is set to transmit mode
* function to load the pldm data to mctp tx buffer
* @param none
* @return void
*******************************************************************************/
void pldm_pkt_tx_packet(void)
{
    uint8_t cmd_resp = 0;
    uint8_t error = false;
    PLDM_CONTEXT *pldmContext = NULL;
    pldmContext = pldm_ctxt_get();
    bool req_bit = 0;
    MCTP_PKT_BUF *pldm_rx_buf;
    pldm_rx_buf = (MCTP_PKT_BUF *) &pldm_pktbuf_rx;
    if(NULL == pldmContext)
    {
        return;
    }

    switch(pldmContext->pldm_tx_state)
    {
    case PLDM_TX_IDLE:
        break;
    case PLDM_TX_IN_PROGRESS:
        mctp_buf_tx1 = &pldm_mctp_pktbuf_tx;
        memset(mctp_buf_tx1, 0, sizeof(MCTP_PKT_BUF));

        if (pldm_request_firmware_update)
        {
            if (number_of_1024b_requests < no_of_requests)
            {
                pldm_pkt_create_request_firmware_data(pldm_buf_tx, pldmContext);
                pldm_request_firmware_update = false;
            } else {
                pldm_write_firmware_data_complete(request_update_component.comp_identifier);
                // PLDM:rcvd all binary data
                // trace0(PLDM_// trace, SPDM_TSK, 2, "Prab");
                pldm_request_firmware_update = false;
                received_1024b_for_flash_write = 0;
                offset_for_flash = 0;
                number_of_1024b_requests = 0;
                offset = 0;
                len = 0;
                pldm_pkt_create_transfer_complete_req(pldm_buf_tx, pldmContext);
                return;
            }
        }

        cmd_resp = pldmContext->pldm_current_response_cmd;

        if (cmd_resp == PLDM_REQUEST_FIRMWARE_DATA_REQ || cmd_resp == PLDM_TRANSFER_COMPLETE_REQ
                || cmd_resp == PLDM_APPLY_COMPLETE_REQ
                || cmd_resp == PLDM_VERIFY_COMPLETE_REQ)
        {
            req_bit = true;
        }

        pldm_pkt_populate_mctp_packet_for_resp(pldm_buf_tx, mctp_buf_tx1, cmd_resp, pldmContext, req_bit);

        if (cmd_resp == PLDM_UPDATE_COMPONENT_REQ && can_update)
        {
            pldm_request_firmware_update = true;
        }

        pldm_rx_buf->buf_full = MCTP_EMPTY;

        pldm_pkt_msg_ready_trigger_mctp(mctp_buf_tx1, pldmContext);

        if (cmd_resp == PLDM_ACTIVATE_FIRMWARE_REQ && can_activate)
        {
            timer_delay_ms(10);
            pldm_activate_firmware();
        }

        if (cmd_resp == PLDM_VERIFY_COMPLETE_REQ)
        {
            // PLDM : Verfiy cmpt
            // trace0(PLDM_// trace, SPDM_TSK, 2, "Pvcpt");
        }
        break;

    default:
        // pldm_tx_pkt: default
        // trace0(PLDM_// trace, SPDM_TSK, 2, "Pxpd");
        break;
    }
}

/******************************************************************************/
/** once mctp rx buffer has pldm payload received
* @param none
* @return void
*******************************************************************************/
void pldm_pkt_rcv_packet()
{
    uint8_t ret_sts = 0x00;
    uint8_t req_field;
    uint8_t byte_cnt_for_one_pkt = BYTE_CNT_FOR_ONE_PKT;

    PLDM_CONTEXT *pldmContext = NULL;
    pldmContext = pldm_ctxt_get();
    uint16_t len = 0;
    if(NULL == pldmContext)
    {
        return;
    }
    MCTP_PKT_BUF *pldm_msg_rx_buf = NULL;
    pldm_msg_rx_buf = (MCTP_PKT_BUF *) &pldm_pktbuf_rx;

    if(MCTP_RX_PENDING == pldm_msg_rx_buf->buf_full)
    {
        pldmContext->pldm_host_eid = pldm_msg_rx_buf->pkt.field.hdr.src_eid;
        pldmContext->pldm_ec_eid = pldm_msg_rx_buf->pkt.field.hdr.dst_eid;
        pldmContext->pldm_ec_slv_addr = pldm_msg_rx_buf->pkt.field.hdr.dst_addr;
        pldmContext->pldm_host_slv_addr = pldm_msg_rx_buf->pkt.field.hdr.src_addr;
        pldmContext->pldm_instance_id = pldm_msg_rx_buf->pkt.field.hdr.inst_id;

        if(pldmContext->pldm_tx_state == PLDM_TX_IDLE || pldmContext->pldm_tx_state == PLDM_PACKETIZING)
        {
            //check if MCTP packet received is single packet request
            if((pldm_msg_rx_buf->pkt.data[MCTP_MSG_TAG_REF_MASK] & MCTP_SOM_EOM_REF_MSK) == MCTP_SOM_EOM_REF)
            {
                pkt_cnt = 0;
                pldmContext->pldm_tx_state = PLDM_NON_PACKETIZING;
                pldm_type = (pldm_msg_rx_buf->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS]) & 0x3F;
                req_cmd = pldm_msg_rx_buf->pkt.data[PLDM_HEADER_COMMAND_CODE_POS];
                completion_code = pldm_msg_rx_buf->pkt.data[PLDM_HEADER_COMPLETION_CODE_POS];
                req_field = (pldm_msg_rx_buf->pkt.data[MCTP_PKT_RQ_D_POS] & MCTP_HDR_MASK_RQ) >> 7;

                if (req_field)
                {
                    offset_data_pos = PLDM_HEADER_DATA_POS_FOR_REQ;
                }
                else
                {
                    offset_data_pos = PLDM_HEADER_DATA_POS_FOR_RESP;
                }

                if(is_sub_safe(pldm_msg_rx_buf->pkt.data[MCTP_BYTE_CNT_OFFSET], byte_cnt_for_one_pkt) == 0)
                {
                    ;//Handle error
                }
                len = pldm_msg_rx_buf->pkt.data[MCTP_BYTE_CNT_OFFSET] - byte_cnt_for_one_pkt;
                ret_sts = pldm_pkt_fill_buffer(pldm_msg_rx_buf, pldmContext, len, offset_data_pos);
                if (ret_sts)
                {
                    // PLDM:Fill bufr fail for single pkt
                    // trace0(PLDM_// trace, SPDM_TSK, 2, "PFbp");
                }
            }
            else if(!((pldm_msg_rx_buf->pkt.data[MCTP_MSG_TAG_REF_MASK] & MCTP_SOM_EOM_REF_MSK) == MCTP_SOM_EOM_REF))
            {
                if ((pldm_msg_rx_buf->pkt.data[MCTP_PKT_TO_MSGTAG_POS] & MCTP_SOM_REF_MSK) == MCTP_SOM_REF)
                {
                    pldm_type = (pldm_msg_rx_buf->pkt.data[PLDM_HEADER_VERSION_PLDM_TYPE_POS]) & 0x3F;
                    req_cmd = pldm_msg_rx_buf->pkt.data[PLDM_HEADER_COMMAND_CODE_POS];
                    completion_code = pldm_msg_rx_buf->pkt.data[PLDM_HEADER_COMPLETION_CODE_POS];
                    len = (uint16_t)((pldm_msg_rx_buf->pkt.data[MCTP_BYTE_CNT_OFFSET] - NO_OF_HDR_BYTES_FRM_BYTE_CNT_OFFSET_SOM_BIT_SET) & UINT16_MAX);
                    req_field = (pldm_msg_rx_buf->pkt.data[MCTP_PKT_RQ_D_POS] & MCTP_HDR_MASK_RQ) >> 7;

                    if (req_field)
                    {
                        offset_data_pos = PLDM_HEADER_DATA_POS_FOR_REQ;
                    }
                    else
                    {
                        offset_data_pos = PLDM_HEADER_DATA_POS_FOR_RESP;
                    }

                }
                else
                {
                    offset_data_pos = PLDM_RESPONSE_DATA_POS;
                    len = (uint16_t)((pldm_msg_rx_buf->pkt.data[MCTP_BYTE_CNT_OFFSET] - NO_OF_HDR_BYTES_FRM_BYTE_CNT_OFFSET)&UINT16_MAX);
                }
                //som = 1, eom = 0 or som = 0, eom = 0 or som = 0, eom = 1 states
                //this means packetizing in progress
                pldmContext->pldm_tx_state = PLDM_PACKETIZING;
                ret_sts = pldm_pkt_fill_buffer(pldm_msg_rx_buf, pldmContext, len, offset_data_pos);
                if(!ret_sts)
                {
                    if((pldm_msg_rx_buf->pkt.data[MCTP_MSG_TAG_REF_MASK] & MCTP_EOM_REF_MSK) == MCTP_EOM_REF)
                    {
                        pldmContext->pldm_tx_state = PLDM_RX_LAST_PKT;
                        pkt_cnt = 0;
                    }
                    else
                    {
                        pldmContext->pldm_tx_state = PLDM_PACKETIZING;
                    }
                }
                else
                {
                    pkt_cnt = 0;
                    pldmContext->pldm_tx_state = PLDM_TX_IDLE;
                    pldmContext->pldm_current_request_length = 0;
                    memset(pldm_buf_tx, 0, MCTP_PKT_BUF_DATALEN);
                    memset(get_mctp_pld, 0, PLDM_MAX_PAYLOAD_BUFF_SIZE);

                }
            }
        }
    }

    if(pldmContext->pldm_tx_state == PLDM_NON_PACKETIZING || pldmContext->pldm_tx_state == PLDM_RX_LAST_PKT)
    {
        ret_sts = pldm_pkt_validate_and_process_pldm_msg(pldm_type, req_cmd, pldm_buf_tx, pldmContext);
        if(ret_sts)
        {
            // PLDM:Ivld msg rcvd
            // trace0(PLDM_// trace, SPDM_TSK, 2, "PImrx");
        }
        else
        {
            // PLDM:Vld msg rcvd
            // trace0(PLDM_// trace, SPDM_TSK, 2, "PVmRx");
        }
    }
    else
    {
        pldm_msg_rx_buf->buf_full = MCTP_EMPTY;
    }

    pldm_di_mctp_done_set();
    pldm_msg_rx_buf->pkt.data[PLDM_HEADER_COMMAND_CODE_POS] = 0;
}


/******************************************************************************/
/** Function to load the 1Kb pldm input buffer for the pldm response bytes
* @param none
* @return 1 if payload size goes greater than 1024 else success
*******************************************************************************/
uint8_t pldm_pkt_fill_buffer(MCTP_PKT_BUF *pldm_msg_rx_buf, PLDM_CONTEXT *pldmContext, uint16_t len, uint8_t offset)
{
    uint32_t i;
    uint8_t ret_sts = 0x00;
    uint16_t get_len = 0x00;

    if(NULL == pldmContext)
    {
        return 0x01;
    }

    //buffer the pldm payload to 1K input buffer from mctp input buffer
    for(i = 0; i < len; i++)
    {
        get_mctp_pld[pkt_cnt + i] = pldm_msg_rx_buf->pkt.data[offset + i]; //Take only the pldm msg payload

    }

    if(pldmContext->pldm_tx_state == PLDM_PACKETIZING)
    {
        pkt_cnt = (uint16_t)((pkt_cnt + len)&UINT16_MAX);
        pldmContext->pldm_current_request_length = pkt_cnt;
    }
    else if(pldmContext->pldm_tx_state == PLDM_NON_PACKETIZING)
    {
        pldmContext->pldm_current_request_length = len;
    }

    if (pkt_cnt > INPUT_BUF_MAX_BYTES)//if no of bytes received cross max input buffer size of 1224
    {
        ret_sts = 0x01;
    }

    return ret_sts;
}

/******************************************************************************/
/** This is called whenever kernel schedules pldm event task.
* mctp module calls SET_PLDM_EVENT_FLAG(pldm) for scheduling pldm event task.
* This event task is called whenever mctp packet is received with pldm message type over smbus,
* @param void
* @return void
*******************************************************************************/
void pldm1_event_task(void)
{
    // pldm_evt_tsk: Start
    // trace0(PLDM_// trace, SPDM_TSK, 2, "PEtS");
    pldm_pkt_rcv_packet();
    pldm_pkt_tx_packet();
    // pldm_evt_tsk: End
    // trace0(PLDM_// trace, SPDM_TSK, 2, "PetE");
}

/******************************************************************************/
/** This function is for sending verify complete to UA
* @param verify_state verify success or failure
* @return void
*******************************************************************************/
void pldm_initiate_verify_req_to_update_agent(uint8_t verify_state)
{
    PLDM_CONTEXT *pldmContext = NULL;
    uint8_t ret_val = 0;

    pldmContext = pldm_ctxt_get();
    if(NULL == pldmContext)
    {
        return;
    }

    pldmContext->pldm_verify_state = verify_state;
    pldm_pkt_create_verify_complete_req(pldm_buf_tx, pldmContext);
    pldmContext->pldm_tx_state = PLDM_TX_IN_PROGRESS;
    pldm_pkt_tx_packet();
    pldm_initiate_verify_req_tx_end();
}

/******************************************************************************/
/** This function is for sending apply complete to UA
* @param apply_state apply success or failure
* @return void
*******************************************************************************/
void pldm_initiate_apply_req_to_update_agent(uint8_t apply_state)
{
    PLDM_CONTEXT *pldmContext = NULL;
    pldmContext = pldm_ctxt_get();
    if(NULL == pldmContext)
    {
        return;
    }
    pldmContext->pldm_apply_state = apply_state;
    pldm_pkt_create_apply_complete_req(pldm_buf_tx, pldmContext);
    pldmContext->pldm_tx_state = PLDM_TX_IN_PROGRESS;

    pldm_pkt_tx_packet();
}

/******************************************************************************/
/** PLDMResp_timer_callback();
* PLDMResp timer callback
* @param TimerHandle_t pxTimer
* @return None
*******************************************************************************/
void PLDMResp_timer_callback(TimerHandle_t pxTimer)
{
    PLDM_CONTEXT *pldmContext = NULL;
    pldmContext = pldm_ctxt_get();

    if (NULL == pldmContext)
    {
        return;
    }
    pkt_cnt = 0;
    memset(get_mctp_pld, 0, PLDM_MAX_PAYLOAD_BUFF_SIZE);
    
    if (PLDMResp_timer_started)
    {
        pldm_response_timeout_stop();
        PLDMResp_timer_started = false;
    }
    retry_request_firmware_data = true;
    pldm_pkt_create_request_firmware_data(pldm_buf_tx, pldmContext);
    pldm_request_firmware_update = false;
    pldmContext->pldm_tx_state = PLDM_TX_IN_PROGRESS;
    pldm_pkt_tx_packet();
}

/******************************************************************************/
/** is_comp_iden_supported();
* check if comp identifier in pass component and update component messages are
* supported
* @param uint16_t comp_iden
* @return true or false
*******************************************************************************/
bool is_comp_iden_supported(uint16_t comp_iden)
{
    uint8_t i = 0;

    for (i = 0; i < NO_OF_COMP_TBL; i++)
    {
        if (get_firmware_parameters_res.comp_parameter[i].comp_identifier == comp_iden)
        {
            return true;
        }
    }

    return false;
}

/******************************************************************************/
/** pldm_init_flags()
* Init Pldm flags
* @param None
* @return None
*******************************************************************************/
void pldm_init_flags()
{
    pldm_first_pkt = true;
    PLDM_CONTEXT *pldmContext = NULL;
    pldmContext = pldm_ctxt_get();
    if(NULL == pldmContext)
    {
        return;
    }
    pldmContext->pldm_state_info = PLDM_IDLE;
    pldmContext->pldm_previous_state = PLDM_IDLE_STATE;
    pldmContext->pldm_current_state = PLDM_IDLE_STATE;
    pldmContext->pldm_status_reason_code = INITIALIZATION_OF_FD;  
    pldmContext->current_pkt_sequence = 0;  
}

/******************************************************************************/
/** pldm_apcfg_device_identifier_length();
* Returns device descriptor length
* @param None
* @return length of device descriptor
*******************************************************************************/
uint32_t pldm_apcfg_device_identifier_length(void)
{
    uint32_t length = 0;

    length = pldm_ap_cfg.pldm_device_identifier_len;

    return length;
}

/******************************************************************************/
/** pldm_apcfg_descriptor_count();
* Returns device descriptor count
* @param None
* @return count no of device descriptor available
*******************************************************************************/
uint8_t pldm_apcfg_descriptor_count(void)
{
    uint8_t count = 0;

    if(pldm_ap_cfg.pldm_des_cnt > UINT8_MAX) // Coverity INT31-C Fix
    {
        // Handle Error
    }
    else
    {
        count = (uint8_t)(pldm_ap_cfg.pldm_des_cnt);
    }

    return count;
}

/******************************************************************************/
/** pldm_apcfg_descriptor();
* Returns device descriptor
* @param descriptor array
* @return None
*******************************************************************************/
void pldm_apcfg_descriptor(uint8_t *descriptor)
{
    uint32_t length = 0;

    length = pldm_ap_cfg.pldm_device_identifier_len;
    memcpy(descriptor, pldm_ap_cfg.descriptor, length);

    return;
}

/******************************************************************************/
/** pldm_apcfg_override_cap_upgrade();
* Returns flag of pldm override capabilities during upgrade
* @param None
* @return true = override, false = no override
*******************************************************************************/
bool pldm_apcfg_override_cap_upgrade(void)
{
    bool override = false;

    override = pldm_ap_cfg.PLDM_override_capability_upgrade;

    return override;
}

/******************************************************************************/
/** pldm_apcfg_capabilities_upgrade();
* Returns pldm capabilities during upgrade
* @param None
* @return cap capapbilites configured
*******************************************************************************/
uint16_t pldm_apcfg_capabilities_upgrade(void)
{
    uint16_t cap = 0;

    cap = pldm_ap_cfg.cap_during_update;

    return cap;
}

/******************************************************************************/
/** pldm_apcfg_override_comp_classification();
* Returns flag of pldm override component classification
* @param None
* @return true = override, false = no override
*******************************************************************************/
bool pldm_apcfg_override_comp_classification(void)
{
    bool override = false;

    override = pldm_ap_cfg.PLDM_override_comp_classification;

    return override;
}

/******************************************************************************/
/** pldm_pkt_get_config_from_apcfg();
* Initialize the PLDM config params
* @param pldmContext
* @return None
*******************************************************************************/
void pldm_pkt_get_config_from_apcfg(PLDM_CONTEXT *pldmContext)
{  
    pldm_get_AP_custom_configs_from_apcfg();
    /* Move to idle state and wait for response */
    pldmContext->pldm_state_info = PLDM_IDLE;
}

/******************************************************************************/
/** pldm_apcfg_tid();
* Returns pldm terminal id
* @param None
* @return tid
*******************************************************************************/
uint8_t pldm_apcfg_tid(void)
{
    uint8_t tid = 0;

    tid = pldm_ap_cfg.tid;

    return tid;
}

/******************************************************************************/
/** pldm_apcfg_component_classification();
* Returns pldm component classification
* @param None
* @return comp_classification
*******************************************************************************/
uint16_t pldm_apcfg_component_classification(void)
{
    uint16_t comp_classification = 0;

    comp_classification = pldm_ap_cfg.comp_classification;

    return comp_classification;
}