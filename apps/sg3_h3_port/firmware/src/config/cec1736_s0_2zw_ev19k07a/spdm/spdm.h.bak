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

/** @file spdm.h
 * Interface header file for applications of SPDM
 */
/** @defgroup SPDM interface
 */

#ifndef SPDM_H
#define SPDM_H

#include "definitions.h"
#include "spdm_common.h"
#include "spdm_task.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/** get_cert2_base_address
 * This function is to get SPDM Certificate 2 Address
 * @param cert_ptr         Pointer to hold certificate 2 address
 * @return                 None
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * Currently the SPDM module supports only 64 certificates. 
 * This function is called by SPDM module to get the Certificate 2 base address.
 * Certificates 3 to 63 have to be placed at an offset 0x400 from previous
 * certificate. User can have these certificates 3 to 63 in any peripheral as
 * per the system design.
 * Certificate 0 and 1 addresses are configurable via macro
 * CERTIFICATE_START_ADDRESS, and this can be placed in same peripheral of
 * certificate 3 to 63, or can be any other peripheral as per design.
 * _______________________________________________________________________
 * |           |            |           |            |     |             |
 * | Cert 0    | Cert 1     | Cert 2    | Cert 3     |     | Cert 63     |
 * | offset 0  |offset 0x400| offset 0  |offst 0x400 |     |offset 0xF400|
 * |___________|____________|___________|____________|_____|_____________|
 * ^                        ^
 * |                        |____________________________
 * CERTIFICATE_START_ADDRESS                           get_cert2_base_address()
 * 
 * -----------------------
 * Example:
 * -----------------------
 * void spdm_pkt_initialize_cert_params_to_default(SPDM_CONTEXT *spdmContext)
 * {
 *     get_cert2_base_address(&cert2_base_addr);
 * }
 *
 * void get_cert2_base_address(uint8_t *cert_ptr)
 * {
 *     read_cert2_addr(*cert_ptr);
 * }
 * ############################################################################
*******************************************************************************/
extern void get_cert2_base_address(uint32_t *cert_ptr);

/******************************************************************************/
/** CERTIFICATE_START_ADDRESS
 * Macro to define Certificate 0 Base Address
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * -----------------------
 * Certificate 0 Base Address
 * -----------------------
 * Configure SPDM Certificate 0 Base Address here, Certificate 1 has to be at an 
 * offset 0x400 from Certificate 0.
 * ############################################################################
*******************************************************************************/
#define CERTIFICATE_START_ADDRESS 0x126800

/******************************************************************************/
/** spdm_get_measurements
 * This function can be used to get hash of measurement data. 
 * @param buff_ptr         Pointer to hold the measurement data
 * @param index            measurement id i.e. 1,2,3,4
 * @return                 None
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the SPDM module to get hash of measurement data.
 * Currently SPDM module supports 4 measurements of data hash. User is 
 * expected to fill unused measurements with zeros.
 * -----------------------
 * Example:
 * -----------------------
 * void spdm_get_measurements(uint8_t *buffer_ptr, uint8_t index)
 * {
 *      switch(index)
 *      {
 *          case 1:
 *              get_measurement_1_data(buffer_ptr);
 *              break;
 *          case 2:
 *              get_measurement_2_data(buffer_ptr);
 *              break;
 *          case 3:
 *              get_measurement_3_data(buffer_ptr);
 *              break;
 *          case 4:
 *              get_measurement_4_data(buffer_ptr);
 *              break;
 *          default:
 *              break;
 *      }
 * }
 * ############################################################################
*******************************************************************************/
extern void spdm_get_measurements(uint8_t *buffer_ptr,
                                    uint8_t index);

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
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the SPDM module to read certificate data.
 * Root certificate of size 1KB is read, other certificates read length are
 * decided by the size specified in the certificate header. Certificate number
 * passed can be used to read certificate data from corresponding peripherals.
 * -----------------------
 * Example:
 * -----------------------
 * uint8_t spdm_read_certificate(uint32_t address, uint8_t *buffer_ptr, uint8_t length
 *                              uint8_t certificate_no)
 * {
 *      uint8_t ret_val = FAILURE;
 * 
 *      if (certificate_no == 0 || certificate_no == 1) {
 *          ret_val = read_certificate_from_flash(address, buffer_ptr, length);
 *      } else {
 *          ret_val = read_certificate_from_ram(address, buffer_ptr, length);
 *      }
 * 
 *      return ret_val;
 * }
 * ############################################################################
*******************************************************************************/
extern uint8_t spdm_read_certificate(uint32_t address, 
                                  uint8_t *buff_ptr,
                                  uint32_t length,
                                  uint8_t certificate_num);

/******************************************************************************/
/** pvt_key
 * Global variable used to store the private key for signature generation.
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * User has to fill this global variable with the private key used in signature 
 * generation spdm_crypto_ops_gen_signature().
 * ############################################################################
*******************************************************************************/
extern SPDM_BSS1_ATTR uint8_t pvt_key[PVT_KEY_CODE_LENGTH];

/******************************************************************************/
/** hash_of_req_buffer
 * Global variable that has the hash of data for signature generation.
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * SPDM module fills this buffer with hash of data.
 * User can use this global variable in signature generation API. 
 * Note: SPDM module supports only hash of data for signature generation.
 * ############################################################################
*******************************************************************************/
extern SPDM_BSS0_ATTR uint8_t hash_of_req_buffer[SPDM_SHA384_LEN];

/******************************************************************************/
/** ecdsa_signature
 * Global variable used to store the generated signature.
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * User is expected to store the generated signature in this variable.
 * SPDM module uses this variable to add signature as part of SPDM challenge and 
 * measurement response messages.
 * ############################################################################
*******************************************************************************/
extern SPDM_BSS1_ATTR ecdsa_signature_t ecdsa_signature __attribute__((aligned(8)));

/******************************************************************************/
/** random_no
 * Global variable to store the random number for signature generation.
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * User is expected to store the generated random number for signature generation
 * if design demands it.
 * ############################################################################
*******************************************************************************/
extern SPDM_BSS0_ATTR uint8_t random_no[CURVE_384_SZ];

/******************************************************************************/
/** spdm_crypto_ops_gen_signature
 * This function can be used to generate signature. 
 * @param                  None
 * @return                 None
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the SPDM module to generate signature.
 * The SPDM module is designed for ECDSA384 calculations.
 * pvt_key array - User is expected to fill the private key.
 * hash_of_req_buffer - will have the hash of data, filled by SPDM stack
 *                      (no action needed from user).
 * random_no - User is expected to fill the random number for signature generation.
 * ecdsa_signature.ecdsa_signature - User is expected to fill with generated
 * signature.
 *
 *  typedef union
 *  {
 *     struct
 *      {
 *          uint8_t signature_r_term[CURVE_384_SZ];
 *          uint8_t signature_s_term[CURVE_384_SZ];
 *      };
 *      uint8_t ecdsa_signature[CURVE_384_SZ*2];
 *  } ecdsa_signature_t;
 *
 *  #define CURVE_384_SZ 48
 * -----------------------
 * Example:
 * -----------------------
 * uint32_t spdm_crypto_ops_gen_signature(void)
 * {
 *      uint32_t ret_val = FAILURE;
 * 
 *      ret_val = fill_pvt_key(&pvt_key[0]);
 * 
 *      if (ret_val == SUCCESS) {
 *          ret_val = generate_random_number(&random_no);
 *      }
 * 
 *      if (ret_val == SUCCESS) {
 *          ret_val = ecdsa_gen_sig(&pvt_key[0], &hash_of_req_buffer[0], &random_no[0],
 *                     &ecdsa_signature.ecdsa_signature[0]);
 *      }
 * 
 *      return ret_val;
 * }
 * ############################################################################
*******************************************************************************/
extern uint32_t spdm_crypto_ops_gen_signature(void);

/******************************************************************************/
/** spdm_crypto_ops_calc_hash
 * This function can be used to calculate hash of data at single shot.
 * @param  buf_ptr         Pointer to data to be hashed
 * @param  length          Length of data to be hashed
 * @param  spdmContext     Holds the stage of hashing in case of intermediate
 *                         hashing
 * @return                 success or failure
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the SPDM module to calculate hash of data at
 * single shot.
 * The SPDM module is designed for SHA384 hash calculations.
 * The user is expected to store the resultant hash in spdmContext->sha_digest.
 * -----------------------
 * Example:
 * -----------------------
 * uint8_t spdm_crypto_ops_calc_hash(uint8_t *buff_ptr, uint32_t length, 
 *                                  SPDM_CONTEXT *spdmContext)
 * {
 *      uint8_t ret_val = FAILURE;
 * 
 *      ret_val = crypto_calc_hash(buff_ptr, length, &spdmContext->sha_digest[0]);
 * 
 *      return ret_val;
 * }
 * ############################################################################
*******************************************************************************/
extern uint8_t spdm_crypto_ops_calc_hash(uint8_t *buff_ptr, uint32_t length,
                                    SPDM_CONTEXT *spdmContext);


/******************************************************************************/
/** spdm_crypto_ops_run_time_hashing
 * This function can be used to calculate hash of data at runtime.
 * @param  buf_ptr         Pointer to data to be hashed
 * @param  length          Length of data to be hashed
 * @param  spdmContext     Holds the stage of hashing in case of intermediate
 *                         hashing
 * @return                 None
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the SPDM module to calculate hash of data at
 * runtime.
 * Data to be hashed has to be fed into crypto engine in chunks with engine
 * saving the intermediate result till hash finalize is issued.
 * spdmContext->get_requests_state will hold the hashing state.
 * HASH_INIT_MODE - initialize the memory for hash context saving
 * RUN_TIME_HASH_MODE - feed chunks of data and hash the same
 * END_OF_HASH - get the finalized hash of all chunks passed to engine for
 *               hashing
 * The resultant hash has to be stored in spdmContext->sha_digest.
 * -----------------------
 * Example:
 * -----------------------
 * uint8_t spdm_crypto_ops_run_time_hashing(uint8_t *buff_ptr, uint32_t length, 
 *                                  SPDM_CONTEXT *spdmContext)
 * {
 *   uint8_t rc = 0x00;
 *
 *   if(NULL == spdmContext)
 *   {
 *       return 0xff;
 *   }
 *
 *   switch(spdmContext->get_requests_state)
 *   {
 *   case HASH_INIT_MODE:
 *       memset(&ctx_ptr, 0, sizeof(ctx_ptr));
 *       // initialize hash structure pointer to HW instance
 *       // allocate a descriptor to load hash configuration word
 *       rc = spdm_crypto_ops_hash_ctx_init(&ctx_ptr);
 *       if (rc != OK)
 *       {
 *           return rc;
 *       }
 *       break;
 *   case RUN_TIME_HASH_MODE:
 *       //populate internal buffer (128 bytes) upto 128 (block size)
 *       //feed to hash engine and calculate intermediate hash if internal
 *       //buffer reaches max 128 bytes
 *       rc = spdm_crypto_ops_hash_ctx_update_buf(buff, &ctx_ptr, length);
 *       if (rc != OK)
 *       {
 *           return rc;
 *       }
 *       break;
 *   case END_OF_HASH:
 *       rc = spdm_crypto_ops_hash_ctx_final(&ctx_ptr,
 *                                           &spdmContext->sha_digest[0]);
 *       if (rc != OK)
 *       {
 *           return rc;
 *       }
 *       //switch back to init state as we got the final digest
 *       spdmContext->get_requests_state = HASH_INIT_MODE;
 *       break;
 *   default:
 *       break;
 *
 *   }
 *   return rc;
 *
 * }
 * ############################################################################
*******************************************************************************/
extern uint8_t spdm_crypto_ops_run_time_hashing(uint8_t *buff_ptr,
                                                uint32_t length,
                                                SPDM_CONTEXT *spdmContext);

/******************************************************************************/
/** spdm_crypto_ops_gen_random_no
 * This function can be used to generate random number.
 * @param  buff           Pointer to hold the generated random number
 * @param  bytes          Length of random number
 * @return                None
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the SPDM module to generate random number for nonce
 * data. User is expected to store the generated random number in the address pointed
 * by buff.
 * Same API can be used for generating random number during signature generation.
 * -----------------------
 * Example:
 * -----------------------
 * uint8_t spdm_crypto_ops_gen_random_no(uint8_t *buff, uint32_t length)
 * {
 *    uint8_t ret = FAILURE;
 *    ret = generate_random_num(buff, length); 
 *    return ret;  
 * }
 * ############################################################################
*******************************************************************************/
extern uint8_t spdm_crypto_ops_gen_random_no(uint8_t *buff, uint8_t bytes);

/******************************************************************************/
/** spdm_app_task_create(void)
 * Create SPDM FreeRTOS task
 * @param pvParams  This parameter is not used
 * @return -1 :Fail, 0: Pass
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function creates and prepares the SPDM task to be run by the FreeRTOS
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
 *    if(spdm_app_task_create((void*)NULL) < 0)
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
int spdm_app_task_create(void *pvParams);

/******************************************************************************/
/** spdmContext
 * Global structure to save SPDM context information.
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * -----------------------
 * spdmContext
 * -----------------------
 * spdmContext is used for saving SPDM context information. User is expected to 
 * store the resultant hash computaiton value in spdmContext->sha_digest[48].
 * spdmContext->get_requests_state has the hash intermediate state which can 
 * be used for corresponding hash crypto engine functionality.
 * ############################################################################
*******************************************************************************/
extern SPDM_BSS2_ATTR SPDM_CONTEXT *spdmContext;

/******************************************************************************/
/** SPDM_RQS_STATE 
 * Intermediate hashing states
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * The values in this enum are used by SPDM module to store the intermediate
 * hashing state. User can make use these for defining intermediate hashing crypto
 * implementation.
 * ############################################################################
 *******************************************************************************/
enum SPDM_RQS_STATE
{
    HASH_INIT_MODE,
    RUN_TIME_HASH_MODE,
    END_OF_HASH
};


#ifdef __cplusplus
}
#endif

#endif /* SPDM_H */

/**   @}
 */

