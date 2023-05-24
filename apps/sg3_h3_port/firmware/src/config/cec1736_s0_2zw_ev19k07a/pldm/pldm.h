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

/** @file pldm.h
 * Interface header file for applications of PLDM
 */
/** @defgroup PLDM interface
 */

#ifndef PLDM_H
#define PLDM_H

#include "definitions.h"
#include "pldm_common.h"
#include "pldm_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/** COMPONENT_PARAMETER_TABLE 
 * Comparameter parameters
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * The values is the parameters supported for any update component
 * User can update these parameters as part of get firmware parameters
 * command
 * ############################################################################
 *******************************************************************************/
typedef struct COMPONENT_PARAMETER_TABLE
{
    uint16_t comp_classification;
    uint16_t comp_identifier;
    uint8_t comp_classification_index;
    uint32_t active_comp_comparison_stamp;
    uint8_t active_comp_version_string_type;
    uint8_t active_comp_version_string_length;
    uint8_t active_comp_release_date[8];
    uint32_t pending_comp_comparison_stamp;
    uint8_t pending_comp_version_string_type;
    uint8_t pending_comp_version_string_length;
    uint8_t pending_comp_release_date[8];
    uint16_t comp_activation_methods;
    uint32_t cap_during_update;
    uint8_t active_comp_version_string[COMP_STRING_TYPE_SIZE];
    uint8_t pending_comp_version_string[COMP_STRING_TYPE_SIZE];
} __attribute__((packed)) COMPONENT_PARAMETER_TABLE;

typedef struct GET_FIRMWARE_PARAMETERS_RES_FIELDS
{
    uint8_t completion_code;
    uint32_t capabilities_during_update;
    uint16_t component_count;
    uint8_t active_comp_image_set_version_string_type;
    uint8_t active_comp_image_set_version_string_length;
    uint8_t pending_comp_image_set_version_string_type;
    uint8_t pending_comp_image_set_version_string_length;
    uint8_t active_comp_image_set_version_string[ASCII_SIZE];
    uint8_t pending_comp_image_set_version_string[ASCII_SIZE];
    COMPONENT_PARAMETER_TABLE comp_parameter[NO_OF_COMP_TBL];
} __attribute__((packed)) GET_FIRMWARE_PARAMETERS_RES_FIELDS;

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
extern void  pldm_get_firmware_param_resp_feilds(GET_FIRMWARE_PARAMETERS_RES_FIELDS *buf_ptr);


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
extern void pldm_init_peripheral_for_update(uint16_t component_id);

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
extern uint8_t pldm_write_firmware_data(uint16_t component_id, uint8_t *buff_ptr, uint32_t offset);

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
extern void pldm_start_firmware_update(uint16_t component_id);

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
extern void pldm_start_firmware_apply();

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
extern uint8_t pldm_cancel_update(uint16_t component_id, uint8_t cancel_update_flag);

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
extern void  pldm_restore_configs(uint16_t component_id, uint8_t host_funct_reduced);

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
extern void pldm_reset_firmware_update_flags(void);

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
extern void pldm_activate_firmware(void);

/******************************************************************************/
/** pldm_initiate_verify_req_to_update_agent
 * This function is for sending verify complete to UA.
 * @param  verify_state    0 - success , 1 - verify fail
 * @return                 None
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the User to inintiate a verify complete 
 * request to UA
 * -----------------------
 * Example:
 * -----------------------
 * uint8_t pldm_start_firmware_update(uint16_t component_id)
 * {
 *      update_state = HASH_VERIFY;
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
 *      pldm_initiate_verify_req_to_update_agent(verify_state);
 * }
 * ############################################################################
*******************************************************************************/
void pldm_initiate_verify_req_to_update_agent(uint8_t verify_state);

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
extern void pldm_initiate_verify_req_tx_end();

/******************************************************************************/
/** pldm_initiate_apply_req_to_update_agent
 * This function is for sending apply complete to UA.
 * @param  apply_state    0 - success , 1 - verify fail
 * @return                 None
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the User to inintiate a apply complete 
 * request to UA
 * -----------------------
 * Example:
 * -----------------------
 * uint8_t pldm_start_firmware_apply()
 * {
 *    flash_write addr = 0x20000
 *    flash_write(flash_write, data, 2048)
 *    pldm_initiate_apply_req_to_update_agent(0) // initiate apply success
 * }
 * ############################################################################
*******************************************************************************/
void pldm_initiate_apply_req_to_update_agent(uint8_t apply_state);

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
extern void pldm_write_firmware_data_complete(uint16_t component_id);

/******************************************************************************/
/** pldm_app_task_create(void)
 * Create PLDM FreeRTOS task
 * @param pvParams  This parameter is not used
 * @return -1 :Fail, 0: Pass
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function creates and prepares the PLDM task to be run by the FreeRTOS
 * task scheduler. The user is expected to call this function in their main
 * function along with any other application task creation routines and start
 * the FreeRTOS scheduler. Make sure that all necessary peripheral initializations
 * have been completed before calling this function
 * -----------------------
 * Example:
 * -----------------------
 * int main ( void )
 * {
 *    SYS_Initialize ( NULL );
 *    
 *    if(pldm_app_task_create((void*)NULL) < 0)
 *    {
 *        while(1);
 *    }
 * 
 *    if(mctp_app_task_create((void*)NULL) < 0)
 *    {
 *        while(1);
 *    }
 *    
 *    if(smb_drv_task_create((void*)NULL) < 0)
 *    {
 *        while(1);
 *    }
 *    
 *    vTaskStartScheduler();
 *    
 *    return ( EXIT_FAILURE );
 * }
 * ############################################################################
*******************************************************************************/
int pldm_app_task_create(void *pvParams);

/******************************************************************************/
/** pldmContext
 * Global structure to save PLDM context information.
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * -----------------------
 * pldmContext
 * -----------------------
 * pldmContext is used for saving PLDM context information. Used by Internal 
 * PLDM state machine. 
 * ############################################################################
*******************************************************************************/
extern PLDM_BSS2_ATTR PLDM_CONTEXT *pldmContext;


#ifdef __cplusplus
}
#endif

#endif /* PLDM_H */

/**   @}
 */
