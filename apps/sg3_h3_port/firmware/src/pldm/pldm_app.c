#include "pldm/pldm.h"
#include "pldm/pldm_pkt_prcs.h"
#include "pldm/pldm_common.h"
#include "pldm_app.h"
#include "common.h"
#include "pmci.h"
#include "app.h"

PLDM_BSS1_ATTR uint8_t pending_comp_string_ECFW0[COMP_STRING_TYPE_SIZE];
PLDM_BSS1_ATTR uint8_t pending_comp_string_ECFW1[COMP_STRING_TYPE_SIZE];
PLDM_BSS1_ATTR uint8_t pending_comp_string[COMP_STRING_TYPE_SIZE];

PLDM_BSS1_ATTR uint32_t staged_address;
PLDM_BSS1_ATTR uint8_t spi_select;
PLDM_BSS1_ATTR uint8_t sg3_state;
PLDM_BSS1_ATTR uint32_t APKHB_address;
PLDM_BSS1_ATTR uint32_t APCFG_address;
PLDM_BSS1_ATTR uint32_t HT_address;
PLDM_BSS1_ATTR uint32_t APFW_address;
PLDM_BSS1_ATTR uint32_t spi_address_flash_write;
PLDM_BSS1_ATTR uint32_t update_comp_image_size;
PLDM_BSS1_ATTR uint16_t tag0FWNum;
PLDM_BSS1_ATTR uint16_t tag1FWNum;
PLDM_BSS1_ATTR UINT8 spi_select_configured;
PLDM_BSS1_ATTR uint8_t FW_type;

extern PLDM_BSS1_ATTR bool failure_recovery_cap;
extern PLDM_BSS1_ATTR bool retry_update_comp_cap;
extern PLDM_BSS2_ATTR bool host_functionality_reduced;
extern PLDM_BSS1_ATTR REQUEST_REQUEST_UPDATE request_update;
extern PLDM_BSS1_ATTR PLDM_AP_CFG pldm_ap_cfg;
extern PLDM_BSS2_ATTR REQUEST_UPDATE_COMPONENT request_update_component;
extern PLDM_BSS1_ATTR uint32_t size_supported_by_UA;
extern BUILD_INFO_T const sb_build_info __attribute__((section("build_info")));

extern void timer_delay_ms(uint32_t num_ms);

PLDM_BSS1_ATTR uint32_t copy_data_offset;
extern PLDM_BSS1_ATTR uint8_t pldm_flash_busy;

/******************************************************************************/
/** pldm_get_AP_custom_configs_from_apcfg();
* This function gets the PLDM_AP_CFG feilds required for AP image update
* Refer PLDM_AP_CFG
* @param None
* @return overrite - true, false
*******************************************************************************/
void pldm_get_AP_custom_configs_from_apcfg()
{
    (void)di_sb_apcfg_config_data_request();
}

/******************************************************************************/
/** pldm_apply_complete_response
* This function is for any action required internal to UA after sending 
* apply complete response 
* @param apply_state apply success or failure
* @return void
*******************************************************************************/
void pldm_apply_complete_response(uint16_t comp_identifier)
{
    if (sg3_state == SG3_CRISIS) 
    {
        pldm_get_staged_address_for_crisis_recovery(comp_identifier);
    }
}

/******************************************************************************/
/** get_pldm_override_device_desc();
* Returns pldm override device descriptor
* @param None
* @return overrite - true, false
*******************************************************************************/
bool get_pldm_override_device_desc()
{
    bool override = false;
    if (sg3_state == SG3_POSTAUTH) {
        override = di_sb_apcfg_pldm_override_device_desc();
    } else if (sg3_state == SG3_CRISIS) {
        override = false;
    }    
    return override;
}

/******************************************************************************/
/** pldm_get_staged_address_for_crisis_recovery();
* Get staged/active locations for APFW crisis recovery
* @param TimerHandle_t pxTimer
* @return None
*******************************************************************************/
void pldm_get_staged_address_for_crisis_recovery(uint16_t comp_identifier)
{
    uint8_t buffer[32];
        sb_parser_get_spi_info(&staged_address, &spi_select);  
        spdm_di_qmspi_clr_port_init_status(spi_select);
        spdm_di_init_flash_component((SPI_FLASH_SELECT)spi_select);
    if (comp_identifier == PLDM_COMP_IDENTIFIER_AP_BA_PTR0) {
        sb_parser_get_spi_info(&staged_address, &spi_select);        
        if (di_spdm_spi_send_read_request(staged_address, buffer, 8, (SPI_FLASH_SELECT)spi_select, true)) {
            // sb_parse_flash_map: Error in reading AP Descriptor 
            //trace0(PLDM_TRACE, SPDM_TSK, 2, "PAdEr");
        }
        memcpy(&APCFG_address, buffer, 4);
        memcpy(&APKHB_address, (buffer + 4), 4);
    } else if (comp_identifier == PLDM_COMP_IDENTIFIER_APCFG0) {
        sb_parser_get_spi_info(&APCFG_address, &spi_select);
        if ((is_add_safe(APCFG_address, 0x5C0) == 0)) // Coverity INT30-C
        {
            return;
        }
        else
        {
            if (di_spdm_spi_send_read_request((APCFG_address + 0x5C0), buffer, 8, (SPI_FLASH_SELECT)spi_select, true)) {
                // sb_parse_flash_map: Error in reading APCFG 
                //trace0(PLDM_TRACE, SPDM_TSK, 2, "PACEr");
            }
            memcpy(&HT_address, buffer, 4);
        }
    } else if (comp_identifier == PLDM_COMP_IDENTIFIER_HT0_AP0C0) {
        sb_parser_get_spi_info(&HT_address, &spi_select);
        if (di_spdm_spi_send_read_request((HT_address), buffer, 32, (SPI_FLASH_SELECT)spi_select, true)) {
            // sb_parse_flash_map: Error in reading HT
            //trace0(PLDM_TRACE, SPDM_TSK, 2, "PHTer");
        }
        memcpy(&APFW_address, &buffer[28], 4);
    }
    spdm_di_spi_tristate(sb_spi_port_get(spi_select));
}

/******************************************************************************/
/** pldm_get_firmware_param_resp_feilds
 * This function is used to the Firmware parameters 
 * @param buf_ptr         Pointer. Refer GET_FIRMWARE_PARAMETERS_RES_FIELDS
 * @return                None
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by PLDM module when the Get Firmware parameter 
 * command is received from Host.
 * The completion_code, capabilities_during_update parameters will we filled
 * by the PLDM module. User is expected to fill all other 
 * parameters in  GET_FIRMWARE_PARAMETERS_RES_FIELDS. 
 * The string type and length supported are ASCII and  UTF16
 * 
 * -----------------------
 * Example:
 * -----------------------
 * pldm_get_firmware_param_resp_feilds(
 * GET_FIRMWARE_PARAMETERS_RES_FIELDS *buf_ptr)
 * {
 *     buf_ptr->component_count = 1;
 *     buf_ptr->active_comp_image_set_version_string_type = ASCII;
 *     buf_ptr->active_comp_image_set_version_string_length = ASCII_SIZE;
 *     buf_ptr->pending_comp_image_set_version_string_type = ASCII;
 *     buf_ptr->pending_comp_image_set_version_string_length = ASCII_SIZE;  
 *     memcpy(buf_ptr->active_comp_image_set_version_string, ver, ASCII_SIZE);
 *     memcpy(buf_ptr->pending_comp_image_set_version_string, ver_pend, ASCII_SIZE);
 *     buf_ptr->comp_parameter[0].comp_classification = 2;
 *     buf_ptr->comp_parameter[0].comp_identifier = 0x1121
 *     buf_ptr->comp_parameter[0].comp_classification_index = 0x00;    
 *     buf_ptr->comp_parameter[0].pending_comp_version_string_type = UTF16;
 *     buf_ptr->comp_parameter[0].pending_comp_version_string_length = COMP_STRING_TYPE_SIZE;
 *     buf_ptr->comp_parameter[0].comp_activation_methods = PLDM_COMP_ACTIVATION_SUPPORTED;
 * 
 * }
 * ############################################################################
*******************************************************************************/
void pldm_get_firmware_param_resp_feilds(GET_FIRMWARE_PARAMETERS_RES_FIELDS *buf_ptr)
{
     uint8_t crisis_int_en = 0U;
    uint16_t cap = 0U;
    uint16_t ap0_rev = 0U, ap1_rev = 0U;
    uint8_t container_state = 0U, no_of_ECFW_KHB = 0U, no_of_HT_support_AP0C0 = 0U, no_of_HT_support_AP0C1 = 0U, no_of_HT_support_AP1C0 = 0U, no_of_HT_support_AP1C1 = 0U, i = 0U, j = 0U, k = 0U, l = 0U, m = 0U, n = 0U, b = 0U;
    uint16_t ht_version[SB_HASH_TABLE_ID_MAX] = {0};
    uint16_t crisis_rec_version = PLDM_CRISIS_BA_VERSION;
    uint8_t no_of_byte_match_int_spi_support = 0U, supported_idx = 0U; //byte match int flash update
    uint32_t ascii_conv;
    uint8_t no_of_apfw_images_supported = 0;

    if (sg3_state == SG3_POSTAUTH) { 
        no_of_HT_support_AP0C0 = di_pldm_sb_get_number_of_ht(0);
        no_of_HT_support_AP0C1 = di_pldm_sb_get_number_of_ht(1);
        no_of_HT_support_AP1C0 = di_pldm_sb_get_number_of_ht(2);
        no_of_HT_support_AP1C1 = di_pldm_sb_get_number_of_ht(3);
        no_of_byte_match_int_spi_support = di_pldm_sb_get_number_of_byte_match_int();
        no_of_apfw_images_supported = di_pldm_sb_get_number_of_apfw_images();

        SRAM_RLOG_API_tag0_buildnum_read((uint8_t*)&tag0FWNum);
        SRAM_RLOG_API_tag0_buildnum_read((uint8_t*)&tag1FWNum);
        if(efuse_read_data(TOO_CONTAINER_STATE, &container_state, 1)) {
            return;
        }
        SRAM_MBOX_API_read_container_crisis_interface_en(&crisis_int_en);

        if ((container_state & 0x1) && (crisis_int_en & 0x4)) {
            no_of_ECFW_KHB = 1;
        } 

        buf_ptr->component_count = PLDM_NUMBER_OF_COMPONENTS_SUPPORTED + 
                            no_of_HT_support_AP0C0 + no_of_HT_support_AP0C1 + no_of_HT_support_AP1C0 + 
                            no_of_HT_support_AP1C1 + no_of_ECFW_KHB + no_of_byte_match_int_spi_support + no_of_apfw_images_supported;
        buf_ptr->active_comp_image_set_version_string_type = ASCII;
        buf_ptr->active_comp_image_set_version_string_length = ASCII_SIZE;
        buf_ptr->pending_comp_image_set_version_string_type = ASCII;
        buf_ptr->pending_comp_image_set_version_string_length = ASCII_SIZE;
        memcpy(buf_ptr->active_comp_image_set_version_string, sb_build_info.build_label, ASCII_SIZE);
        memcpy(buf_ptr->pending_comp_image_set_version_string, request_update.comp_image_set_version_string, ASCII_SIZE);

        // EC_FW
        if (pldm_apcfg_override_comp_classification()) {
            buf_ptr->comp_parameter[0].comp_classification = pldm_apcfg_component_classification();
        }
        else
        {
            buf_ptr->comp_parameter[0].comp_classification = PLDM_DEFAULT_COMP_CLASSIFICATION;
        }

        buf_ptr->comp_parameter[0].comp_identifier = PLDM_COMP_IDENTIFIER_TAG0;
        buf_ptr->comp_parameter[0].comp_classification_index = 0x00;
        buf_ptr->comp_parameter[0].active_comp_comparison_stamp = 0x00000000;
        buf_ptr->comp_parameter[0].active_comp_version_string_type = UTF16;
        buf_ptr->comp_parameter[0].active_comp_version_string_length = COMP_STRING_TYPE_SIZE;
        memset(buf_ptr->comp_parameter[0].active_comp_release_date, 0, 8);
        buf_ptr->comp_parameter[0].pending_comp_comparison_stamp = 0x00000000;
        buf_ptr->comp_parameter[0].pending_comp_version_string_type = UTF16;
        buf_ptr->comp_parameter[0].pending_comp_version_string_length = COMP_STRING_TYPE_SIZE;
        memset(buf_ptr->comp_parameter[0].pending_comp_release_date, 0, 8);

        buf_ptr->comp_parameter[0].comp_activation_methods = PLDM_COMP_ACTIVATION_SUPPORTED;

        buf_ptr->comp_parameter[0].cap_during_update = 0x00000000;
        convert16BitHexToAscii(tag0FWNum, &ascii_conv);
        memcpy(buf_ptr->comp_parameter[0].active_comp_version_string, &ascii_conv, COMP_STRING_TYPE_SIZE);
        memcpy(buf_ptr->comp_parameter[0].pending_comp_version_string, &pending_comp_string_ECFW0[0], COMP_STRING_TYPE_SIZE);


        if (pldm_apcfg_override_comp_classification()) {
            buf_ptr->comp_parameter[1].comp_classification = pldm_apcfg_component_classification();
        } else {
            buf_ptr->comp_parameter[1].comp_classification = PLDM_DEFAULT_COMP_CLASSIFICATION;
        }

        buf_ptr->comp_parameter[1].comp_identifier = PLDM_COMP_IDENTIFIER_TAG1;
        buf_ptr->comp_parameter[1].comp_classification_index = 0x00;
        buf_ptr->comp_parameter[1].active_comp_comparison_stamp = 0x00000000;
        buf_ptr->comp_parameter[1].active_comp_version_string_type = UTF16;
        buf_ptr->comp_parameter[1].active_comp_version_string_length = COMP_STRING_TYPE_SIZE;
        memset(buf_ptr->comp_parameter[1].active_comp_release_date, 0, 8);
        buf_ptr->comp_parameter[1].pending_comp_comparison_stamp = 0x00000000;
        buf_ptr->comp_parameter[1].pending_comp_version_string_type = UTF16;
        buf_ptr->comp_parameter[1].pending_comp_version_string_length = COMP_STRING_TYPE_SIZE;
        memset(buf_ptr->comp_parameter[1].pending_comp_release_date, 0, 8);

        buf_ptr->comp_parameter[1].comp_activation_methods = PLDM_COMP_ACTIVATION_SUPPORTED;

        buf_ptr->comp_parameter[1].cap_during_update = 0x00000000;
        convert16BitHexToAscii(tag1FWNum, &ascii_conv);
        memcpy(buf_ptr->comp_parameter[1].active_comp_version_string, &ascii_conv, COMP_STRING_TYPE_SIZE);
        memcpy(buf_ptr->comp_parameter[1].pending_comp_version_string, &pending_comp_string_ECFW1[0], COMP_STRING_TYPE_SIZE);

        // AP_CFG
        ap0_rev = di_pldm_sb_apcfg_apcfg_version(true);
        ap1_rev = di_pldm_sb_apcfg_apcfg_version(false);

        if (pldm_apcfg_override_comp_classification()) {
            buf_ptr->comp_parameter[2].comp_classification = pldm_apcfg_component_classification();
        }
        else
        {
            buf_ptr->comp_parameter[2].comp_classification = PLDM_DEFAULT_COMP_CLASSIFICATION;
        }

        buf_ptr->comp_parameter[2].comp_identifier = PLDM_COMP_IDENTIFIER_APCFG0;
        buf_ptr->comp_parameter[2].comp_classification_index = 0x00;
        buf_ptr->comp_parameter[2].active_comp_comparison_stamp = 0x00000000;
        buf_ptr->comp_parameter[2].active_comp_version_string_type = UTF16;
        buf_ptr->comp_parameter[2].active_comp_version_string_length = COMP_STRING_TYPE_SIZE;
        memset(buf_ptr->comp_parameter[2].active_comp_release_date, 0, 8);
        buf_ptr->comp_parameter[2].pending_comp_comparison_stamp = 0x00000000;
        buf_ptr->comp_parameter[2].pending_comp_version_string_type = UTF16;
        buf_ptr->comp_parameter[2].pending_comp_version_string_length = COMP_STRING_TYPE_SIZE;
        memset(buf_ptr->comp_parameter[2].pending_comp_release_date, 0, 8);

        buf_ptr->comp_parameter[2].comp_activation_methods = PLDM_COMP_ACTIVATION_SUPPORTED;

        buf_ptr->comp_parameter[2].cap_during_update = 0x00000000;
        convert16BitHexToAscii(ap0_rev, &ascii_conv);
        memcpy(buf_ptr->comp_parameter[2].active_comp_version_string, &ascii_conv, COMP_STRING_TYPE_SIZE);
        memcpy(buf_ptr->comp_parameter[2].pending_comp_version_string, pending_comp_string, COMP_STRING_TYPE_SIZE);

        if (pldm_apcfg_override_comp_classification()) {
            buf_ptr->comp_parameter[3].comp_classification = pldm_apcfg_component_classification();
        }
        else
        {
            buf_ptr->comp_parameter[3].comp_classification = PLDM_DEFAULT_COMP_CLASSIFICATION;
        }

        buf_ptr->comp_parameter[3].comp_identifier = PLDM_COMP_IDENTIFIER_APCFG1;
        buf_ptr->comp_parameter[3].comp_classification_index = 0x00;
        buf_ptr->comp_parameter[3].active_comp_comparison_stamp = 0x00000000;
        buf_ptr->comp_parameter[3].active_comp_version_string_type = UTF16;
        buf_ptr->comp_parameter[3].active_comp_version_string_length = COMP_STRING_TYPE_SIZE;
        memset(buf_ptr->comp_parameter[3].active_comp_release_date, 0, 8);
        buf_ptr->comp_parameter[3].pending_comp_comparison_stamp = 0x00000000;
        buf_ptr->comp_parameter[3].pending_comp_version_string_type = UTF16;
        buf_ptr->comp_parameter[3].pending_comp_version_string_length = COMP_STRING_TYPE_SIZE;
        memset(buf_ptr->comp_parameter[3].pending_comp_release_date, 0, 8);

        buf_ptr->comp_parameter[3].comp_activation_methods = PLDM_COMP_ACTIVATION_SUPPORTED;

        buf_ptr->comp_parameter[3].cap_during_update = 0x00000000;
        convert16BitHexToAscii(ap1_rev, &ascii_conv);
        memcpy(buf_ptr->comp_parameter[3].active_comp_version_string, &ascii_conv, COMP_STRING_TYPE_SIZE);
        memcpy(buf_ptr->comp_parameter[3].pending_comp_version_string, pending_comp_string, COMP_STRING_TYPE_SIZE);

        // HASH TABLE
        di_pldm_sb_get_ht_version(&ht_version[0]);

        n = 4;
        for (i = 0; i < no_of_HT_support_AP0C0; i++)
        {
            if (i > SB_HASH_TABLE2 && di_pldm_sb_get_use_c1_ht_for_c0(AP_0))
            {
                buf_ptr->comp_parameter[n].comp_identifier = PLDM_COMP_IDENTIFIER_HT0_AP0C1 + m;
                if(safe_add_8(m, 1U, &m) == 0U) // INT30-C, INT31-C
                {
                }
                else
                {
                    // handle error
                }
            
            }
            else
            {
                buf_ptr->comp_parameter[n].comp_identifier = PLDM_COMP_IDENTIFIER_HT0_AP0C0 + i;
            }
            if (pldm_apcfg_override_comp_classification()) {
                buf_ptr->comp_parameter[n].comp_classification = pldm_apcfg_component_classification();
            } else {
                buf_ptr->comp_parameter[n].comp_classification = PLDM_DEFAULT_COMP_CLASSIFICATION;
            }
            buf_ptr->comp_parameter[n].comp_classification_index = 0x00;
            buf_ptr->comp_parameter[n].active_comp_comparison_stamp = 0x00000000;
            buf_ptr->comp_parameter[n].active_comp_version_string_type = UTF16;
            buf_ptr->comp_parameter[n].active_comp_version_string_length = COMP_STRING_TYPE_SIZE;
            memset(buf_ptr->comp_parameter[n].active_comp_release_date, 0, 8);
            buf_ptr->comp_parameter[n].pending_comp_comparison_stamp = 0x00000000;
            buf_ptr->comp_parameter[n].pending_comp_version_string_type = UTF16;
            buf_ptr->comp_parameter[n].pending_comp_version_string_length = COMP_STRING_TYPE_SIZE;
            memset(buf_ptr->comp_parameter[n].pending_comp_release_date, 0, 8);

            buf_ptr->comp_parameter[n].comp_activation_methods = PLDM_COMP_ACTIVATION_SUPPORTED;

            buf_ptr->comp_parameter[n].cap_during_update = 0x00000000;  
            convert16BitHexToAscii(ht_version[i], &ascii_conv);
            memcpy(buf_ptr->comp_parameter[n].active_comp_version_string, &ascii_conv, COMP_STRING_TYPE_SIZE);
            memcpy(buf_ptr->comp_parameter[n].pending_comp_version_string, pending_comp_string, COMP_STRING_TYPE_SIZE);   
            if(safe_add_8(n, 1U, &n) == 0U) // INT30-C, INT31-C
            {
            }
            else
            {
                // handle error
            }
        }
        m= 0;
        for (j = 0; j < no_of_HT_support_AP0C1; j++)
        {
            if (pldm_apcfg_override_comp_classification()) {
                buf_ptr->comp_parameter[n].comp_classification = pldm_apcfg_component_classification();
            } else {
                buf_ptr->comp_parameter[n].comp_classification = PLDM_DEFAULT_COMP_CLASSIFICATION;
            }

            buf_ptr->comp_parameter[n].comp_identifier = PLDM_COMP_IDENTIFIER_HT0_AP0C1 + j;
            buf_ptr->comp_parameter[n].comp_classification_index = 0x00;
            buf_ptr->comp_parameter[n].active_comp_comparison_stamp = 0x00000000;
            buf_ptr->comp_parameter[n].active_comp_version_string_type = UTF16;
            buf_ptr->comp_parameter[n].active_comp_version_string_length = COMP_STRING_TYPE_SIZE;
            memset(buf_ptr->comp_parameter[n].active_comp_release_date, 0, 8);
            buf_ptr->comp_parameter[n].pending_comp_comparison_stamp = 0x00000000;
            buf_ptr->comp_parameter[n].pending_comp_version_string_type = UTF16;
            buf_ptr->comp_parameter[n].pending_comp_version_string_length = COMP_STRING_TYPE_SIZE;
            memset(buf_ptr->comp_parameter[n].pending_comp_release_date, 0, 8);

            buf_ptr->comp_parameter[n].comp_activation_methods = PLDM_COMP_ACTIVATION_SUPPORTED;

            buf_ptr->comp_parameter[n].cap_during_update = 0x00000000;
            convert16BitHexToAscii(ht_version[i], &ascii_conv);
            memcpy(buf_ptr->comp_parameter[n].active_comp_version_string, &ascii_conv, COMP_STRING_TYPE_SIZE);
            memcpy(buf_ptr->comp_parameter[n].pending_comp_version_string, pending_comp_string, COMP_STRING_TYPE_SIZE); 
            if(safe_add_8(n, 1U, &n) == 0U) // INT30-C, INT31-C
            {
            }
            else
            {
                // handle error
            }
        }
       
        for (k = 0; k < no_of_HT_support_AP1C0; k++)
        {
            if (pldm_apcfg_override_comp_classification()) {
                buf_ptr->comp_parameter[n].comp_classification = pldm_apcfg_component_classification();
            } else {
                buf_ptr->comp_parameter[n].comp_classification = PLDM_DEFAULT_COMP_CLASSIFICATION;
            }

            if (k > SB_HASH_TABLE2 && di_pldm_sb_get_use_c1_ht_for_c0(AP_1))
            {
                buf_ptr->comp_parameter[n].comp_identifier = PLDM_COMP_IDENTIFIER_HT0_AP1C1 + m;

                if (safe_add_8(m, 1U, &m) == 0U) // INT30-C, INT31-C
                {

                }
                else
                {
                    // handle error
                }

            }
            else
            {
                buf_ptr->comp_parameter[n].comp_identifier = PLDM_COMP_IDENTIFIER_HT0_AP1C0 + k;
            }
            buf_ptr->comp_parameter[n].comp_classification_index = 0x00;
            buf_ptr->comp_parameter[n].active_comp_comparison_stamp = 0x00000000;
            buf_ptr->comp_parameter[n].active_comp_version_string_type = UTF16;
            buf_ptr->comp_parameter[n].active_comp_version_string_length = COMP_STRING_TYPE_SIZE;
            memset(buf_ptr->comp_parameter[n].active_comp_release_date, 0, 8);
            buf_ptr->comp_parameter[n].pending_comp_comparison_stamp = 0x00000000;
            buf_ptr->comp_parameter[n].pending_comp_version_string_type = UTF16;
            buf_ptr->comp_parameter[n].pending_comp_version_string_length = COMP_STRING_TYPE_SIZE;
            memset(buf_ptr->comp_parameter[n].pending_comp_release_date, 0, 8);

            buf_ptr->comp_parameter[n].comp_activation_methods = PLDM_COMP_ACTIVATION_SUPPORTED;

            buf_ptr->comp_parameter[n].cap_during_update = 0x00000000;
            convert16BitHexToAscii(ht_version[i], &ascii_conv);
            memcpy(buf_ptr->comp_parameter[n].active_comp_version_string, &ascii_conv, COMP_STRING_TYPE_SIZE);
            memcpy(buf_ptr->comp_parameter[n].pending_comp_version_string, pending_comp_string, COMP_STRING_TYPE_SIZE);
            if(safe_add_8(n, 1U, &n) == 0U) // INT30-C, INT31-C
            {
            }
            else
            {
                // handle error
            }
        }

        for (l = 0; l < no_of_HT_support_AP1C1; l++)
        {
            if (pldm_apcfg_override_comp_classification()) {
                buf_ptr->comp_parameter[n].comp_classification = pldm_apcfg_component_classification();
            } else {
                buf_ptr->comp_parameter[n].comp_classification = PLDM_DEFAULT_COMP_CLASSIFICATION;
            }

            buf_ptr->comp_parameter[n].comp_identifier = PLDM_COMP_IDENTIFIER_HT0_AP1C1 + l;
            buf_ptr->comp_parameter[n].comp_classification_index = 0x00;
            buf_ptr->comp_parameter[n].active_comp_comparison_stamp = 0x00000000;
            buf_ptr->comp_parameter[n].active_comp_version_string_type = UTF16;
            buf_ptr->comp_parameter[n].active_comp_version_string_length = COMP_STRING_TYPE_SIZE;
            memset(buf_ptr->comp_parameter[n].active_comp_release_date, 0, 8);
            buf_ptr->comp_parameter[n].pending_comp_comparison_stamp = 0x00000000;
            buf_ptr->comp_parameter[n].pending_comp_version_string_type = UTF16;
            buf_ptr->comp_parameter[n].pending_comp_version_string_length = COMP_STRING_TYPE_SIZE;
            memset(buf_ptr->comp_parameter[n].pending_comp_release_date, 0, 8);

            buf_ptr->comp_parameter[n].comp_activation_methods = PLDM_COMP_ACTIVATION_SUPPORTED;

            buf_ptr->comp_parameter[n].cap_during_update = 0x00000000;
            convert16BitHexToAscii(ht_version[i], &ascii_conv);
            memcpy(buf_ptr->comp_parameter[n].active_comp_version_string, &ascii_conv, COMP_STRING_TYPE_SIZE);
            memcpy(buf_ptr->comp_parameter[n].pending_comp_version_string, pending_comp_string, COMP_STRING_TYPE_SIZE);
            if(safe_add_8(n, 1U, &n) == 0U) // INT30-C, INT31-C
            {
            }
            else
            {
                // handle error
            }
        }

        if (no_of_ECFW_KHB == 1) {
            // KHB1 TAG1
            if (pldm_apcfg_override_comp_classification()) {
                buf_ptr->comp_parameter[n].comp_classification = pldm_apcfg_component_classification();
            } else {
                buf_ptr->comp_parameter[n].comp_classification = PLDM_DEFAULT_COMP_CLASSIFICATION;
            }

            buf_ptr->comp_parameter[n].comp_identifier = PLDM_COMP_IDENTIFIER_KHB1_TAG1;
            buf_ptr->comp_parameter[n].comp_classification_index = 0x00;
            buf_ptr->comp_parameter[n].active_comp_comparison_stamp = 0x00000000;
            buf_ptr->comp_parameter[n].active_comp_version_string_type = UTF16;
            buf_ptr->comp_parameter[n].active_comp_version_string_length = COMP_STRING_TYPE_SIZE;
            memset(buf_ptr->comp_parameter[n].active_comp_release_date, 0, 8);
            buf_ptr->comp_parameter[n].pending_comp_comparison_stamp = 0x00000000;
            buf_ptr->comp_parameter[n].pending_comp_version_string_type = UTF16;
            buf_ptr->comp_parameter[n].pending_comp_version_string_length = COMP_STRING_TYPE_SIZE;
            memset(buf_ptr->comp_parameter[n].pending_comp_release_date, 0, 8);

            buf_ptr->comp_parameter[n].comp_activation_methods = PLDM_COMP_ACTIVATION_SUPPORTED;

            buf_ptr->comp_parameter[n].cap_during_update = 0x00000000;
            convert16BitHexToAscii(tag0FWNum, &ascii_conv);
            memcpy(buf_ptr->comp_parameter[n].active_comp_version_string, &ascii_conv, COMP_STRING_TYPE_SIZE);
            memcpy(buf_ptr->comp_parameter[n].pending_comp_version_string, &pending_comp_string_ECFW0[0], COMP_STRING_TYPE_SIZE);
            if(safe_add_8(n, 1U, &n) == 0U) // INT30-C, INT31-C
            {
            }
            else
            {
                // handle error
            }
        }

        for (b = 0; b < no_of_byte_match_int_spi_support; b++) 
        {
            if (pldm_apcfg_override_comp_classification()) {
                buf_ptr->comp_parameter[n].comp_classification = pldm_apcfg_component_classification();
            } else {
                buf_ptr->comp_parameter[n].comp_classification = PLDM_DEFAULT_COMP_CLASSIFICATION;
            }
            if (no_of_byte_match_int_spi_support == 1) {

                if (pldm_ap_cfg.byte_match_details[0].is_present) {
                    supported_idx = 0;
                } else {
                    supported_idx = 1;
                }
                buf_ptr->comp_parameter[n].comp_identifier = (PLDM_COMP_IDENTIFIER_BYTE_MATCH_INT_SPI_BASE | (pldm_ap_cfg.byte_match_details[supported_idx].comp_img_id) |
                                                                            (pldm_ap_cfg.byte_match_details[supported_idx].comp_id << 4) | (pldm_ap_cfg.byte_match_details[supported_idx].ap << 8));

            } else {
                buf_ptr->comp_parameter[n].comp_identifier = (PLDM_COMP_IDENTIFIER_BYTE_MATCH_INT_SPI_BASE | (pldm_ap_cfg.byte_match_details[b].comp_img_id) |
                                                                            (pldm_ap_cfg.byte_match_details[b].comp_id << 4) | (pldm_ap_cfg.byte_match_details[b].ap << 8));
            }
            buf_ptr->comp_parameter[n].comp_classification_index = 0x00;
            buf_ptr->comp_parameter[n].active_comp_comparison_stamp = 0x00000000;
            buf_ptr->comp_parameter[n].active_comp_version_string_type = UTF16;
            buf_ptr->comp_parameter[n].active_comp_version_string_length = COMP_STRING_TYPE_SIZE;
            memset(buf_ptr->comp_parameter[n].active_comp_release_date, 0, 8);
            buf_ptr->comp_parameter[n].pending_comp_comparison_stamp = 0x00000000;
            buf_ptr->comp_parameter[n].pending_comp_version_string_type = UTF16;
            buf_ptr->comp_parameter[n].pending_comp_version_string_length = COMP_STRING_TYPE_SIZE;
            memset(buf_ptr->comp_parameter[n].pending_comp_release_date, 0, 8);

            buf_ptr->comp_parameter[n].comp_activation_methods = PLDM_COMP_ACTIVATION_SUPPORTED;

            buf_ptr->comp_parameter[n].cap_during_update = 0x00000000;
            memcpy(buf_ptr->comp_parameter[n].pending_comp_version_string, pending_comp_string, COMP_STRING_TYPE_SIZE);
            if(safe_add_8(n, 1U, &n) == 0U) // INT30-C, INT31-C
            {
            }
            else
            {
                // handle error
            }
        }
        b = 0;
        // AP_FW image  
        for (b = 0; b < no_of_apfw_images_supported; b++) 
        {
            if (pldm_apcfg_override_comp_classification()) {
                buf_ptr->comp_parameter[n].comp_classification = pldm_apcfg_component_classification();
            } else {
                buf_ptr->comp_parameter[n].comp_classification = PLDM_DEFAULT_COMP_CLASSIFICATION;
            }

            if (pldm_ap_cfg.apfw_image_details[b].is_present) {
                buf_ptr->comp_parameter[n].comp_identifier = (PLDM_COMP_IDENTIFIER_AP_FW_SPI_BASE  | (pldm_ap_cfg.apfw_image_details[b].comp_img_id) |
                                                                        (pldm_ap_cfg.apfw_image_details[b].comp_id << 4) | (pldm_ap_cfg.apfw_image_details[b].ap << 8));

            }
            buf_ptr->comp_parameter[n].comp_classification_index = 0x00;
            buf_ptr->comp_parameter[n].active_comp_comparison_stamp = 0x00000000;
            buf_ptr->comp_parameter[n].active_comp_version_string_type = ASCII;
            buf_ptr->comp_parameter[n].active_comp_version_string_length = COMP_STRING_TYPE_SIZE;
            memset(buf_ptr->comp_parameter[n].active_comp_release_date, 0, 8);
            buf_ptr->comp_parameter[n].pending_comp_comparison_stamp = 0x00000000;
            buf_ptr->comp_parameter[n].pending_comp_version_string_type = ASCII;
            buf_ptr->comp_parameter[n].pending_comp_version_string_length = COMP_STRING_TYPE_SIZE;
            memset(buf_ptr->comp_parameter[n].pending_comp_release_date, 0, 8);

            buf_ptr->comp_parameter[n].comp_activation_methods = PLDM_COMP_ACTIVATION_SUPPORTED;

            buf_ptr->comp_parameter[n].cap_during_update = 0x00000000;
            memcpy(buf_ptr->comp_parameter[n].pending_comp_version_string, pending_comp_string, COMP_STRING_TYPE_SIZE);
            if(safe_add_8(n, 1U, &n) == 0U) // INT30-C, INT31-C
            {
            }
            else
            {
                // handle error
            }
        }
    } else if (sg3_state == SG3_CRISIS) {
        buf_ptr->completion_code = PLDM_SUCCESS;
        buf_ptr->capabilities_during_update = PLDM_DEFAULT_CAP_DURING_UPDATE;
        failure_recovery_cap = true;
        retry_update_comp_cap = true;
        host_functionality_reduced = true;

        buf_ptr->component_count = PLDM_NUMBER_OF_COMP_SUPPORTED_DURING_CRISIS;
        buf_ptr->active_comp_image_set_version_string_type = ASCII;
        buf_ptr->active_comp_image_set_version_string_length = ASCII_SIZE;
        buf_ptr->pending_comp_image_set_version_string_type = ASCII;
        buf_ptr->pending_comp_image_set_version_string_length = ASCII_SIZE;
        memcpy(buf_ptr->active_comp_image_set_version_string, sb_build_info.build_label, ASCII_SIZE);
        memcpy(buf_ptr->pending_comp_image_set_version_string, request_update.comp_image_set_version_string, ASCII_SIZE);

        // AP BA 0
        buf_ptr->comp_parameter[0].comp_classification = PLDM_DEFAULT_COMP_CLASSIFICATION;
        buf_ptr->comp_parameter[0].comp_identifier = PLDM_COMP_IDENTIFIER_AP_BA_PTR0;
        buf_ptr->comp_parameter[0].comp_classification_index = 0x00;
        buf_ptr->comp_parameter[0].active_comp_comparison_stamp = 0x00000000;
        buf_ptr->comp_parameter[0].active_comp_version_string_type = UTF16;
        buf_ptr->comp_parameter[0].active_comp_version_string_length = COMP_STRING_TYPE_SIZE;
        memset(buf_ptr->comp_parameter[0].active_comp_release_date, 0, 8);
        buf_ptr->comp_parameter[0].pending_comp_comparison_stamp = 0x00000000;
        buf_ptr->comp_parameter[0].pending_comp_version_string_type = UTF16;
        buf_ptr->comp_parameter[0].pending_comp_version_string_length = COMP_STRING_TYPE_SIZE;
        memset(buf_ptr->comp_parameter[0].pending_comp_release_date, 0, 8);

        buf_ptr->comp_parameter[0].comp_activation_methods = PLDM_COMP_ACTIVATION_SUPPORTED;

        buf_ptr->comp_parameter[0].cap_during_update = 0x00000000;
        convert16BitHexToAscii(crisis_rec_version, &ascii_conv);
        memcpy(buf_ptr->comp_parameter[0].active_comp_version_string, &ascii_conv, COMP_STRING_TYPE_SIZE);
        memcpy(buf_ptr->comp_parameter[0].pending_comp_version_string, &pending_comp_string_ECFW0[0], COMP_STRING_TYPE_SIZE);

        // AP KHB 0
        buf_ptr->comp_parameter[1].comp_classification = PLDM_DEFAULT_COMP_CLASSIFICATION;
        buf_ptr->comp_parameter[1].comp_identifier = PLDM_COMP_IDENTIFIER_AP_KHB0;
        buf_ptr->comp_parameter[1].comp_classification_index = 0x00;
        buf_ptr->comp_parameter[1].active_comp_comparison_stamp = 0x00000000;
        buf_ptr->comp_parameter[1].active_comp_version_string_type = UTF16;
        buf_ptr->comp_parameter[1].active_comp_version_string_length = COMP_STRING_TYPE_SIZE;
        memset(buf_ptr->comp_parameter[1].active_comp_release_date, 0, 8);
        buf_ptr->comp_parameter[1].pending_comp_comparison_stamp = 0x00000000;
        buf_ptr->comp_parameter[1].pending_comp_version_string_type = UTF16;
        buf_ptr->comp_parameter[1].pending_comp_version_string_length = COMP_STRING_TYPE_SIZE;
        memset(buf_ptr->comp_parameter[1].pending_comp_release_date, 0, 8);

        buf_ptr->comp_parameter[1].comp_activation_methods = PLDM_COMP_ACTIVATION_SUPPORTED;

        buf_ptr->comp_parameter[1].cap_during_update = 0x00000000;
        convert16BitHexToAscii(crisis_rec_version, &ascii_conv);
        memcpy(buf_ptr->comp_parameter[1].active_comp_version_string, &ascii_conv, COMP_STRING_TYPE_SIZE);
        memcpy(buf_ptr->comp_parameter[1].pending_comp_version_string, &pending_comp_string_ECFW1[0], COMP_STRING_TYPE_SIZE);

        // AP CFG0
        buf_ptr->comp_parameter[2].comp_classification = PLDM_DEFAULT_COMP_CLASSIFICATION;
        buf_ptr->comp_parameter[2].comp_identifier = PLDM_COMP_IDENTIFIER_APCFG0;
        buf_ptr->comp_parameter[2].comp_classification_index = 0x00;
        buf_ptr->comp_parameter[2].active_comp_comparison_stamp = 0x00000000;
        buf_ptr->comp_parameter[2].active_comp_version_string_type = UTF16;
        buf_ptr->comp_parameter[2].active_comp_version_string_length = COMP_STRING_TYPE_SIZE;
        memset(buf_ptr->comp_parameter[2].active_comp_release_date, 0, 8);
        buf_ptr->comp_parameter[2].pending_comp_comparison_stamp = 0x00000000;
        buf_ptr->comp_parameter[2].pending_comp_version_string_type = UTF16;
        buf_ptr->comp_parameter[2].pending_comp_version_string_length = COMP_STRING_TYPE_SIZE;
        memset(buf_ptr->comp_parameter[2].pending_comp_release_date, 0, 8);

        buf_ptr->comp_parameter[2].comp_activation_methods = PLDM_COMP_ACTIVATION_SUPPORTED;

        buf_ptr->comp_parameter[2].cap_during_update = 0x00000000;
        convert16BitHexToAscii(crisis_rec_version, &ascii_conv);
        memcpy(buf_ptr->comp_parameter[2].active_comp_version_string, &ascii_conv, COMP_STRING_TYPE_SIZE);
        memcpy(buf_ptr->comp_parameter[2].pending_comp_version_string, &pending_comp_string_ECFW0[0], COMP_STRING_TYPE_SIZE);

        // HASH TABLE0
        buf_ptr->comp_parameter[3].comp_classification = PLDM_DEFAULT_COMP_CLASSIFICATION;
        buf_ptr->comp_parameter[3].comp_identifier = PLDM_COMP_IDENTIFIER_HT0_AP0C0;
        buf_ptr->comp_parameter[3].comp_classification_index = 0x00;
        buf_ptr->comp_parameter[3].active_comp_comparison_stamp = 0x00000000;
        buf_ptr->comp_parameter[3].active_comp_version_string_type = UTF16;
        buf_ptr->comp_parameter[3].active_comp_version_string_length = COMP_STRING_TYPE_SIZE;
        memset(buf_ptr->comp_parameter[3].active_comp_release_date, 0, 8);
        buf_ptr->comp_parameter[3].pending_comp_comparison_stamp = 0x00000000;
        buf_ptr->comp_parameter[3].pending_comp_version_string_type = UTF16;
        buf_ptr->comp_parameter[3].pending_comp_version_string_length = COMP_STRING_TYPE_SIZE;
        memset(buf_ptr->comp_parameter[1].pending_comp_release_date, 0, 8);

        buf_ptr->comp_parameter[3].comp_activation_methods = PLDM_COMP_ACTIVATION_SUPPORTED;

        buf_ptr->comp_parameter[3].cap_during_update = 0x00000000;
        convert16BitHexToAscii(crisis_rec_version, &ascii_conv);
        memcpy(buf_ptr->comp_parameter[3].active_comp_version_string, &ascii_conv, COMP_STRING_TYPE_SIZE);
        memcpy(buf_ptr->comp_parameter[3].pending_comp_version_string, &pending_comp_string_ECFW1[0], COMP_STRING_TYPE_SIZE);

        // APFW 0
        buf_ptr->comp_parameter[4].comp_classification = PLDM_DEFAULT_COMP_CLASSIFICATION;
        buf_ptr->comp_parameter[4].comp_identifier = PLDM_COMP_IDENTIFIER_APFW_0;
        buf_ptr->comp_parameter[4].comp_classification_index = 0x00;
        buf_ptr->comp_parameter[4].active_comp_comparison_stamp = 0x00000000;
        buf_ptr->comp_parameter[4].active_comp_version_string_type = UTF16;
        buf_ptr->comp_parameter[4].active_comp_version_string_length = COMP_STRING_TYPE_SIZE;
        memset(buf_ptr->comp_parameter[4].active_comp_release_date, 0, 8);
        buf_ptr->comp_parameter[4].pending_comp_comparison_stamp = 0x00000000;
        buf_ptr->comp_parameter[4].pending_comp_version_string_type = UTF16;
        buf_ptr->comp_parameter[4].pending_comp_version_string_length = COMP_STRING_TYPE_SIZE;
        memset(buf_ptr->comp_parameter[4].pending_comp_release_date, 0, 8);      
        buf_ptr->comp_parameter[4].comp_activation_methods = PLDM_COMP_ACTIVATION_SUPPORTED;   
        buf_ptr->comp_parameter[4].cap_during_update = 0x00000000;
        convert16BitHexToAscii(crisis_rec_version, &ascii_conv);
        memcpy(buf_ptr->comp_parameter[4].active_comp_version_string, &ascii_conv, COMP_STRING_TYPE_SIZE);
        memcpy(buf_ptr->comp_parameter[4].pending_comp_version_string, &pending_comp_string_ECFW0[0], COMP_STRING_TYPE_SIZE);        
    }
}

/******************************************************************************/
/** pldm_init_peripheral_for_update()
 * This function is used to initialize the peripheral to get ready  to receive
 * the Update firmware data from Host 
 * @param component_id   Component identifier
 * @return                 None
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the PLDM module when the update component
 * message is received from the host and if UA can update the requested  
 * component_id. The User is expected to initialize the peripherals required
 * to receive the Firmware image
 * -----------------------
 * Example:
 * -----------------------
 * void pldm_init_peripheral_for_update(uint8_t *buffer_ptr, uint8_t index)
 * {
 *      spi_tristate();
 *      spi_init_shd_spi();  // Initialize the external flash 
 * }
 * ############################################################################
*******************************************************************************/
void pldm_init_peripheral_for_update(uint16_t component_id)
{
    uint8_t image_id = ECFW_IMG_INVALID;
    uint8_t ap = 0;
    uint8_t comp = 0;
    uint8_t ht_num = 0;
    uint8_t ht_id = 0;
    if ((component_id == PLDM_COMP_IDENTIFIER_TAG0) || 
        (component_id == PLDM_COMP_IDENTIFIER_KHB_TAG0)) {
        image_id = ECFW_IMG_TAG0;
        memcpy(&pending_comp_string_ECFW0[0], &request_update_component.comp_version_string[0], COMP_STRING_TYPE_SIZE);
    } else if ((component_id == PLDM_COMP_IDENTIFIER_TAG1) || 
        (component_id == PLDM_COMP_IDENTIFIER_KHB1_TAG1) || 
        (component_id == PLDM_COMP_IDENTIFIER_KHB_TAG1)) {
        image_id = ECFW_IMG_TAG1;
        memcpy(&pending_comp_string_ECFW1[0], &request_update_component.comp_version_string[0], COMP_STRING_TYPE_SIZE);
    }
    else if (component_id == PLDM_COMP_IDENTIFIER_APCFG0)
    {
        image_id = ECFW_IMG_APCFG0;
    }
    else if (component_id == PLDM_COMP_IDENTIFIER_APCFG1)
    {
        image_id = ECFW_IMG_APCFG1;
    }

    if (component_id == PLDM_COMP_IDENTIFIER_TAG0 || 
        component_id == PLDM_COMP_IDENTIFIER_TAG1 ||
        !(INVALID_ECFWKHB_IDENTIFIER(component_id)))
    {
        di_pldm_sb_apcfg_ecfw_staged_add_get(image_id, &staged_address);
    }
    else if (component_id == PLDM_COMP_IDENTIFIER_APCFG0 ||
                component_id == PLDM_COMP_IDENTIFIER_APCFG1)
    {
        if (sg3_state == SG3_POSTAUTH) {
            di_pldm_sb_apcfg_apcfg_staged_add_get(image_id, &staged_address);
        } else if (sg3_state == SG3_CRISIS) {
            staged_address = APCFG_address;
        }
    }
    else if (!INVALID_HASH_COMP_IDENTIFIER(component_id))
    {
        if (sg3_state == SG3_POSTAUTH) {
            sb_get_apx_compx_htnum_based_on_comp_identifier(component_id, &ap, &comp, &ht_num);
            staged_address = di_pldm_sb_apcfg_ht_staged_add_get(ap, comp, ht_num);
        } else if (sg3_state == SG3_CRISIS) {
            staged_address = HT_address;
        }
    } else if (component_id == PLDM_COMP_IDENTIFIER_AP_BA_PTR0) {
        (void)efuse_read_data(AP_BA_PTR0_BASE_ADDR_EFUSE_OFFSET, (uint8_t*)&staged_address, 4);
    } else if (component_id == PLDM_COMP_IDENTIFIER_AP_KHB0) {
        staged_address = APKHB_address;
    } else if (component_id == PLDM_COMP_IDENTIFIER_APFW_0) {
        staged_address = APFW_address;
    } else if ((request_update_component.comp_identifier & PLDM_COMP_IDENTIFIER_AP_FW_SPI_BASE) ==
                    PLDM_COMP_IDENTIFIER_AP_FW_SPI_BASE)
    {
        di_pldm_sb_apcfg_apfw_img_staged_addr_get(request_update_component.comp_identifier, &staged_address);
    } else if ((request_update_component.comp_identifier & PLDM_COMP_IDENTIFIER_BYTE_MATCH_INT_SPI_BASE) ==
            PLDM_COMP_IDENTIFIER_BYTE_MATCH_INT_SPI_BASE)
    {
        di_pldm_sb_apcfg_byte_match_staged_addr_get(request_update_component.comp_identifier, &staged_address);
    }
    
    spi_select = staged_address & SPI_SELECT_MASK;
    spi_select_configured |= (uint8_t)((1u << (sb_spi_port_get(spi_select)))&UINT8_MAX);

    pldm_flash_busy = true;

    spdm_di_sb_i2c_ec_fw_port_prepare(host_functionality_reduced, spi_select, spi_select, spi_select);
    spdm_di_i2c_spi_init(spi_select, spi_select, spi_select);

    spdm_di_qmspi_clr_port_init_status(spi_select);
    spdm_di_init_flash_component((SPI_FLASH_SELECT)spi_select);   
}

/******************************************************************************/
/** pldm_write_firmware_data
 * This function can be used to read the certificate data and store it in buffer. 
 * @param component_id     Component identifier
 * @param buff_ptr         Pointer to firmware data from host
 * @param offset           Address of firmware date requested by UA
 * @return                 uint8_t
 *                         0 - No error . UA can continue to get next 
 *                             firmware data
 *                         1 - Error occured . UA does not continue to
 *                         create next request for firmware data.
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the PLDM module when the firmware data is
 * received from the host. UA requests host to get the firmware image in
 * 1KB chunck size. 
 * The user is expected to process the firmware data stored in buff ptr.
 * User should return success to continue the PLDM update process.
 * Return error will terminate the update process
 * UA supports firmware data reques of Max 1KB 
 * -----------------------
 * Example:
 * -----------------------
 * Example If image size  = 2048 bytes , This function is called 
 * twice for each 1KB request. 
 * offset = 0 for 1st 1KB data
 * offset = 1024 for 2nd 1KB data 
 * flash_wr_addr = 0x480000
 * 
 * uint8_t pldm_write_firmware_data(uint16_t component_id, uint8_t *buffer_ptr, 
 *                                  uint32_t offset)
 * {
 * 
 *      uint8_t ret_val = 0;
 *      flash_wr_addr += offset
 *      if(!(offset % SERIAL_FLASH_SECTOR_SIZE))
 *      {
 *          if(sector_erase(flash_wr_addr))
 *          {
 *              return 1; // erase fail
 *          }
 *      }
 *      if(flash_write(flash_wr_addr, buffer_ptr, 1024))
 *      {
 *          return 1; // flash write fail
 *      }
 *
 *      return 0;    // flash write success
 * }
 * ############################################################################
*******************************************************************************/
uint8_t pldm_write_firmware_data(uint16_t component_id, uint8_t *buff_ptr, uint32_t offset)
{
    uint8_t image_id = 0x00;
    uint8_t i = 0;
    uint8_t ap = 0x00;
    uint8_t comp = 0x00;
    uint8_t ht_num = 0x00;
    uint8_t no_of_looping = 0x00;
    uint8_t fn_ret=0;
    
    copy_data_offset = 0;
    no_of_looping = (uint8_t)((size_supported_by_UA / PAGE_SIZE) & UINT8_MAX);

    if ((component_id == PLDM_COMP_IDENTIFIER_TAG0) ||
                (component_id == PLDM_COMP_IDENTIFIER_KHB_TAG0))
    {
        image_id = ECFW_IMG_TAG0;
    } else if ((component_id == PLDM_COMP_IDENTIFIER_TAG1) ||
        (component_id == PLDM_COMP_IDENTIFIER_KHB_TAG1) || 
        (component_id == PLDM_COMP_IDENTIFIER_KHB1_TAG1))
    {
        image_id = ECFW_IMG_TAG1;
    }
    else if (component_id == PLDM_COMP_IDENTIFIER_APCFG0)
    {
        image_id = ECFW_IMG_APCFG0;
    }
    else if (component_id == PLDM_COMP_IDENTIFIER_APCFG1)
    {
        image_id = ECFW_IMG_APCFG1;
    }

    if (image_id == ECFW_IMG_TAG0 || image_id == ECFW_IMG_TAG1)
    {
        di_pldm_sb_apcfg_ecfw_staged_add_get(image_id, &staged_address);
    } else if (image_id == ECFW_IMG_APCFG0 || image_id == ECFW_IMG_APCFG1) {
        if (sg3_state == SG3_POSTAUTH) {
            di_pldm_sb_apcfg_apcfg_staged_add_get(image_id, &staged_address);
        } else {
            staged_address = APCFG_address;
        }
    }

    if (!INVALID_HASH_COMP_IDENTIFIER(component_id))
    {
        if (sg3_state == SG3_POSTAUTH) {
        sb_get_apx_compx_htnum_based_on_comp_identifier(component_id, &ap, &comp, &ht_num);
        staged_address = di_pldm_sb_apcfg_ht_staged_add_get(ap, comp, ht_num);
        } else if (sg3_state == SG3_CRISIS) {
            staged_address = HT_address;
        }
    }

    if (component_id == PLDM_COMP_IDENTIFIER_AP_BA_PTR0) {
        (void)efuse_read_data(AP_BA_PTR0_BASE_ADDR_EFUSE_OFFSET, (uint8_t *)&staged_address, 4);
    } else if (component_id == PLDM_COMP_IDENTIFIER_AP_KHB0) {
        staged_address = APKHB_address;
    } else if (component_id == PLDM_COMP_IDENTIFIER_APFW_0) {
        staged_address = APFW_address;
    }


    if ((component_id & PLDM_COMP_IDENTIFIER_BYTE_MATCH_INT_SPI_BASE) ==
        PLDM_COMP_IDENTIFIER_BYTE_MATCH_INT_SPI_BASE)
    {
        if (di_pldm_sb_apcfg_byte_match_staged_addr_get(component_id, &staged_address)) {
            // continue
        } else {
            // invalid case
            return 1;
        }
    }

    if ((request_update_component.comp_identifier & PLDM_COMP_IDENTIFIER_AP_FW_SPI_BASE) ==
        PLDM_COMP_IDENTIFIER_AP_FW_SPI_BASE)
    {
        if (di_pldm_sb_apcfg_apfw_img_staged_addr_get(request_update_component.comp_identifier, &staged_address)) {
            // continue
        } else {
            // invalid case
            return 1;
        }
    }

    staged_address = staged_address + offset;
    if (is_addition_safe(staged_address, offset) == 0) // Coverity INT30-C
    {
        // Handle Error
        // Do not erase
        // return error
        return 1;
    }
    if (!(offset % FLASH_SECTOR_SIZE))   //sector erase for every 4096 bytes
    {

        if(spdm_di_sb_spi_sector_erase(staged_address, spi_select))
        {
            // PLDM:sector erase fail
            //trace0(PLDM_TRACE, SPDM_TSK, 3, "Psef");
            return 1;
        }
        timer_delay_ms(5);
    }

    do
    {
        timer_delay_ms(5);
        uint32_t write_address = staged_address + copy_data_offset;
        if ((is_addition_safe(write_address, staged_address) == 0)) // Coverity INT30-C
        {
            // Do not write to invalid address.Cannot write to invalid address
            //trace0(PLDM_TRACE, SPDM_TSK, 3, "PiCw");
            return 1;
        }
        else
        {
            fn_ret = spdm_di_sb_spi_write(write_address, spi_select, (buff_ptr + copy_data_offset),
                                            PAGE_SIZE);
        }
        if (fn_ret)
        {
            // PLDM:page write err
            //trace0(PLDM_TRACE, SPDM_TSK, 3, "PpWe");
            return 1;
        }
        timer_delay_ms(5);
        copy_data_offset = copy_data_offset + PAGE_SIZE;
        i++;
    }
    while (i < no_of_looping);

    return 0;
}

/******************************************************************************/
/** pldm_start_firmware_update
 * This function is used to start the update process after the firmware
 * data transfer is complete.
 * @param component_id     Component identifier
 * @param buff_ptr         Pointer to firmware data from host
 * @param offset           Address of firmware date requested by UA
 * @return                 none
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the PLDM module when the transfer complete
 * response is received from Host
 *  User is expected to verify the firmware data in the update process
 *  and send verify complete or success
 * -----------------------
 * Example:
 * -----------------------
 *
 * 
 * uint8_t pldm_start_firmware_update(uint16_t component_id)
 * {
 *      uint8_t flash_rd_addr  = 0x480000;
 *      uint8_t buff[1024], verify_state = PLDM_VERIFY_SUCCESS
 *      while(i < 2) {
 *         // read the firmware data 
 *         if(flash_read(flash_rd_addr, buffer_ptr, 1024))
 *         {
 *            // read fail
 *            verify_state = PLDM_VERIFY_FAILURE;
 *            break;
 *         }
 *         // calculate hash 
 *         if(do_sha(buffer_ptr, digest_buff)) 
 *         {
 *            //hash error
 *            verify_state = PLDM_VERIFY_FAILURE;
 *         }
 *      }
 *      if(validate_hash(digest_buff, compare_buff))
 *      {
 *            // hash verify fail
 *            verify_state = PLDM_VERIFY_FAILURE;
 *      }
 * 
 * }
 * ############################################################################
*******************************************************************************/
void pldm_start_firmware_update(uint16_t component_id)
{
    if (sg3_state == SG3_POSTAUTH) {
        pldm_start_update(component_id);
    } else if (sg3_state == SG3_CRISIS) {
        pldm_initiate_verify_req_to_update_agent(0);
    }
}

/******************************************************************************/
/** pldm_start_firmware_apply
 * This function is called after verifu complete response is received.
 * @param                  none
 * @return                 none
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * PLDM module calls this function after verify complete 
 * is received. 
 * User is expected to initiate apply update process is complete
 * 
 * -----------------------
 * Example:
 * -----------------------
 * 
 * uint8_t pldm_start_firmware_apply()
 * {
 *    flash_write addr = 0x20000
 *    flash_write(flash_write, data, 2048)
 *    pldm_initiate_apply_req_to_update_agent(0) // initiate apply success
 * }
 * ############################################################################
*******************************************************************************/
void pldm_start_firmware_apply()
{
    if (sg3_state == SG3_POSTAUTH) {
        di_sb_core_request_switch_mode_verify_response();
    } else if (sg3_state == SG3_CRISIS) {
        pldm_initiate_apply_req_to_update_agent(0);
    }    
}

/******************************************************************************/
/** pldm_cancel_update
 * THis function is called by PLDM module when cancel 
 * update component or cancel update  request is received
 * @param   component_id
 * @param   cancel_update_flag    0 - cancel update component received
 *                                1 - Cancel update message received
 * @return  uint8_t   0 - success , 1 - fail
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * User is expected to restore flash states and restore the flash image 
 * if required
 * -----------------------
 * Example:
 * -----------------------
 * pldm_cancel_update()
 * {
 *      flash_erase(0x20000)
 *      return 0;
 *      
 * }
 * ############################################################################
*******************************************************************************/
uint8_t pldm_cancel_update(uint16_t component_id, uint8_t cancel_update_flag)
{
    uint8_t ret_Val = 1;
    if(FALSE == cancel_update_flag)
    {
        // cancel update component
        di_sb_core_request_switch_mode(FW_type);
    }
    else
    {
        // cancel update
        ret_Val = di_sb_core_request_switch_mode(FW_type);
    }
    
    pldm_flash_busy = false;
    return ret_Val;
}

/******************************************************************************/
/** pldm_restore_configs
 * THis function is called by PLDM module when cancel update is received or
 *  when activate firmware request is false
 * @param   component_id
 * @param   host_funct_reduced    0 - host reduced functionality false
 *                                1 - host reduced functionality true
 * @return  none  
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * User is expected to store any global variables or states used during 
 * update process
 * ############################################################################
*******************************************************************************/
void  pldm_restore_configs(uint16_t component_id, uint8_t host_funct_reduced)
{
    di_sb_core_restore_configs(spi_select_configured, host_funct_reduced);
    spi_select_configured = 0x00;
}


/******************************************************************************/
/** pldm_reset_firmware_update_flags
 * This function is used to reaset firmware update user states when 
 * the completion code of firmware update response is failure  
 * @param                  None
 * @return                 None
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the PLDM module to reset user states
 * when the firmware update response is failure.
 * ############################################################################
*******************************************************************************/
void pldm_reset_firmware_update_flags(void)
{

}

/******************************************************************************/
/** pldm_activate_firmware
 * This function is called by the PLDM module when the activate firmware
 * request is received from host 
 * @param  none         
 * @param  none          
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * The user is expected to add his/her own  activation method for 
 * firmware update complete process
 * single shot.
 * The PLDM module is designed for SHA384 hash calculations.
 * The user is expected to store the resultant hash in pldmContext->sha_digest.
 * -----------------------
 * Example:
 * -----------------------
 * uint8_t pldm_activate_firmware(void)
 * {
 *      system_soft_reset();
 * }
 * ############################################################################
*******************************************************************************/
void pldm_activate_firmware(void)
{
    sb_boot_sys_soft_rst();
}


/******************************************************************************/
/** pldm_initiate_verify_req_tx_end
 * This function is called by the PLDM module when the verify complete 
 * request is sent to UA 
 * @param  none         
 * @param  none          
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * The user is expected to use this function if there is requirment to 
 * move the user defined internal states.
 * firmware update complete process
 * -----------------------
 * Example:
 * -----------------------
 * uint8_t pldm_initiate_verify_req_tx_end(void)
 * {
 *      update_state = IDLE 
 * }
 * ############################################################################
*******************************************************************************/
void pldm_initiate_verify_req_tx_end()
{
    uint8_t ret_val;
    if (sg3_state == SG3_POSTAUTH) {
        ret_val = spdm_di_mctp_done_wait(MCTP_DI_PACKET_DONE);
    
        if (ret_val)
        {
            spdm_di_core_done_set();
        }
    }    
}

/******************************************************************************/
/** pldm_write_firmware_data_complete
 * This function is called when the firmware data received by the UA is done
 * @param  component_id    Component identifier
 * @return                 None
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the User PLDM modur=le when all the 1KB chuncks
 * of data is received from Host
 * The user is expected to release the pripherals accuried during 
 * firmware write data
 * -----------------------
 * Example:
 * -----------------------
 * uint8_t pldm_write_firmware_data_complete()
 * {
 *    spi_tristate();
 * }
 * ############################################################################
*******************************************************************************/
void pldm_write_firmware_data_complete(uint16_t component_id)
{
    spdm_di_spi_tristate(sb_spi_port_get(spi_select));
}

/******************************************************************************/
/** This function returns the AP, COMP, HT_NUM based on PLDM component identifier
* @param component_identifier - PLDM Component identifier
* @param ap - to return the AP
* @param comp - to return the Component
* @param ht_num - to return the HT_NUM
* @return void
*******************************************************************************/
void sb_get_apx_compx_htnum_based_on_comp_identifier(uint16_t component_identifier, uint8_t *ap, uint8_t *comp,
        uint8_t *ht_num)
{
    switch(component_identifier)
    {
    case PLDM_COMP_IDENTIFIER_HT0_AP0C0:
        *ap = AP_0;
        *comp = COMPONENT_0;
        *ht_num = SB_HASH_TABLE0;
        break;

    case PLDM_COMP_IDENTIFIER_HT1_AP0C0:
        *ap = AP_0;
        *comp = COMPONENT_0;
        *ht_num = SB_HASH_TABLE1;
        break;

    case PLDM_COMP_IDENTIFIER_HT2_AP0C0:
        *ap = AP_0;
        *comp = COMPONENT_0;
        *ht_num = SB_HASH_TABLE2;
        break;

    case PLDM_COMP_IDENTIFIER_HT0_AP0C1:
        *ap = AP_0;
        *comp = COMPONENT_1;
        *ht_num = SB_HASH_TABLE0;
        break;

    case PLDM_COMP_IDENTIFIER_HT1_AP0C1:
        *ap = AP_0;
        *comp = COMPONENT_1;
        *ht_num = SB_HASH_TABLE1;
        break;

    case PLDM_COMP_IDENTIFIER_HT2_AP0C1:
        *ap = AP_0;
        *comp = COMPONENT_1;
        *ht_num = SB_HASH_TABLE2;
        break;

    case PLDM_COMP_IDENTIFIER_HT0_AP1C0:
        *ap = AP_1;
        *comp = COMPONENT_0;
        *ht_num = SB_HASH_TABLE0;
        break;

    case PLDM_COMP_IDENTIFIER_HT1_AP1C0:
        *ap = AP_1;
        *comp = COMPONENT_0;
        *ht_num = SB_HASH_TABLE1;
        break;

    case PLDM_COMP_IDENTIFIER_HT2_AP1C0:
        *ap = AP_1;
        *comp = COMPONENT_0;
        *ht_num = SB_HASH_TABLE2;
        break;

    case PLDM_COMP_IDENTIFIER_HT0_AP1C1:
        *ap = AP_1;
        *comp = COMPONENT_1;
        *ht_num = SB_HASH_TABLE0;
        break;
    case PLDM_COMP_IDENTIFIER_HT1_AP1C1:
        *ap = AP_1;
        *comp = COMPONENT_1;
        *ht_num = SB_HASH_TABLE1;
        break;

    case PLDM_COMP_IDENTIFIER_HT2_AP1C1:
        *ap = AP_1;
        *comp = COMPONENT_1;
        *ht_num = SB_HASH_TABLE2;
        break;
    }
}

/******************************************************************************/
/** This is called for starting EC_FW/AP_CFG update process
* EC_FW/AP_CFG is copied to staged location before this function is called, and
* reusing EC_FW update and reboot I2C command code.
* @param comp_identifier
* @return void
*******************************************************************************/
void pldm_start_update(uint16_t comp_identifier)
{
    //coverity fix
    uint32_t tagx_address = 0x00;
    uint32_t staged_address = 0x00;
    uint32_t restore_address = 0x00;
    uint8_t image_id = ECFW_IMG_INVALID;
    uint32_t image_size = 0x00;
    uint8_t tagx_spi_select = 0x00;
    uint8_t staged_spi_select = 0x00;
    uint8_t restore_spi_select = 0x00;
    uint8_t apcfg_id = 0x00;
    uint8_t ht_id = SB_HASH_TABLE_ID_MAX;
    uint8_t ap = 0x00;
    uint8_t comp = 0x00;
    uint8_t ht_num =0x00;
    uint8_t img_id = 0x00;

    if ((comp_identifier == PLDM_COMP_IDENTIFIER_TAG0) ||
        (comp_identifier == PLDM_COMP_IDENTIFIER_KHB_TAG0)) 
    {
        image_id = ECFW_IMG_TAG0;
    } else if ((comp_identifier == PLDM_COMP_IDENTIFIER_TAG1) ||
        (comp_identifier == PLDM_COMP_IDENTIFIER_KHB_TAG1) ||
        (comp_identifier == PLDM_COMP_IDENTIFIER_KHB1_TAG1)) 
    {
        image_id = ECFW_IMG_TAG1;
    }
    else if (comp_identifier == PLDM_COMP_IDENTIFIER_APCFG0)
    {
        image_id = ECFW_IMG_APCFG0;
        apcfg_id = 0;
    }
    else if (comp_identifier == PLDM_COMP_IDENTIFIER_APCFG1)
    {
        image_id = ECFW_IMG_APCFG1;
        apcfg_id = 1;
    }

    if (image_id == ECFW_IMG_TAG0 || image_id == ECFW_IMG_TAG1)
    {
        spdm_di_sb_ecfw_tagx_addr_get(&tagx_address, image_id);
        if ((comp_identifier == PLDM_COMP_IDENTIFIER_TAG0) || (comp_identifier == PLDM_COMP_IDENTIFIER_TAG1))
        {
            FW_type = PLDM_FW_TYPE_EC_FW;
        } else if (comp_identifier == PLDM_COMP_IDENTIFIER_KHB1_TAG1) {
            FW_type = PLDM_FW_TYPE_ECFWKHB_TOO;
        }  else if (comp_identifier == PLDM_COMP_IDENTIFIER_KHB_TAG0) {
            FW_type = PLDM_FW_TYPE_ECFW0_KHB;
        } else if (comp_identifier == PLDM_COMP_IDENTIFIER_KHB_TAG1) {
            FW_type = PLDM_FW_TYPE_ECFW1_KHB;
        }
    }
    else if (image_id == ECFW_IMG_APCFG0 || image_id == ECFW_IMG_APCFG1)
    {
        tagx_address = di_pldm_sb_apcfg_apcfg_base_address_get(apcfg_id);
        FW_type = PLDM_FW_TYPE_AP_CFG;
    }

    if (!INVALID_HASH_COMP_IDENTIFIER(comp_identifier))
    {
        FW_type = PLDM_FW_TYPE_HASH_TABLE;
        sb_get_apx_compx_htnum_based_on_comp_identifier(comp_identifier, &ap, &comp, &ht_num);
        ht_id = sb_get_hash_table_id_apx_compx(ap, comp, ht_num);
    }

    if ((comp_identifier & PLDM_COMP_IDENTIFIER_BYTE_MATCH_INT_SPI_BASE) ==
        PLDM_COMP_IDENTIFIER_BYTE_MATCH_INT_SPI_BASE)
    {
        FW_type = PLDM_FW_BYTE_MATCH_INT_SPI;
        if (di_pldm_sb_apcfg_byte_match_info_get(comp_identifier, &staged_address, 
                                                &restore_address, &tagx_address, &img_id)) {
            ht_id = img_id;
            // continue
        } else {
            // invalid case
            return;
        }
    }

    if (FW_type == PLDM_FW_TYPE_EC_FW || FW_type == PLDM_FW_TYPE_ECFWKHB_TOO || 
        FW_type == PLDM_FW_TYPE_ECFW0_KHB || FW_type == PLDM_FW_TYPE_ECFW1_KHB)
    {
        di_pldm_sb_apcfg_ecfw_update_info_get(image_id, &staged_address, &restore_address);
    }
    
    if (FW_type == PLDM_FW_TYPE_AP_CFG)
    {
        di_pldm_sb_apcfg_apcfg_update_info_get(image_id, &staged_address, &restore_address);
    }

    if (FW_type == PLDM_FW_TYPE_EC_FW || FW_type == PLDM_FW_TYPE_AP_CFG || 
        FW_type == PLDM_FW_TYPE_ECFW0_KHB || FW_type == PLDM_FW_TYPE_ECFW1_KHB ||
        FW_type == PLDM_FW_TYPE_ECFWKHB_TOO)
    {
        staged_spi_select = staged_address & 0xF;
        restore_spi_select = restore_address & 0xF;
        tagx_spi_select = (uint8_t)(tagx_address & 0xF); //coverity fix

        spdm_di_i2c_spi_init(restore_spi_select, restore_spi_select, restore_spi_select);
        spdm_di_i2c_spi_init(tagx_spi_select, tagx_spi_select, tagx_spi_select);
        // PLDM ECFW/APCFG:staged ,restore,active
        //trace3(PLDM_TRACE, SPDM_TSK, 2, "PFA:%x,%x,%x", staged_address, restore_address, tagx_address);
    }

    if ((request_update_component.comp_identifier & PLDM_COMP_IDENTIFIER_AP_FW_SPI_BASE) ==
        PLDM_COMP_IDENTIFIER_AP_FW_SPI_BASE)
    {
        FW_type = PLDM_FW_APFW_IMG;
        if (di_pldm_sb_apcfg_apfw_img_info_get(request_update_component.comp_identifier, &staged_address, 
                                                &restore_address, &tagx_address, &img_id)) {
            ht_id = img_id;
            // continue
        } else {
            // invalid case
            return;
        }
    }

    image_size = request_update_component.comp_image_size;

    if (sg3_state != SG3_CRISIS) {
        spdm_di_sb_core_i2c_ec_fw_update_start(staged_address, FW_type,
            restore_address, image_size, 0, tagx_address, true, failure_recovery_cap, ht_id);
    }
}