#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "../config/cec1736_s0_2zw_ev19k07a/spdm/spdm_task.h"

#ifndef APP_H
#define APP_H

#define NVIC_DFLT_PRIORITY 1u
#define TOO_CONTAINER_STATE 347
#define AP_BA_PTR0_BASE_ADDR_EFUSE_OFFSET                   824

#define SPI_SELECT_MASK                 0x0000000Fu
#define PAGE_SIZE                   256ul
#define FLASH_SECTOR_SIZE                   4096
#define FALSE       (0u)

#define is_addition_safe(sum, aug_or_add) ((sum) < (aug_or_add) ? 0 : 1) 

typedef struct
{
    uint32_t firmware_id;
    uint32_t build_number;
    uint8_t  build_label[40];
    uint8_t  copyright[40];
    uint8_t  otpmap_rev;
    uint8_t  firmware_revision;
} BUILD_INFO_T;

typedef enum
{
    SB_HASH_TABLE0_AP0_0,
    SB_HASH_TABLE0_AP0_1,
    SB_HASH_TABLE0_AP1_0,
    SB_HASH_TABLE0_AP1_1,
    SB_HASH_TABLE1_AP0_0,
    SB_HASH_TABLE1_AP0_1,
    SB_HASH_TABLE1_AP1_0,
    SB_HASH_TABLE1_AP1_1,
    SB_HASH_TABLE2_AP0_0,
    SB_HASH_TABLE2_AP0_1,
    SB_HASH_TABLE2_AP1_0,
    SB_HASH_TABLE2_AP1_1,
    SB_HASH_TABLE_ID_MAX,
} SB_HASH_TABLE_IDS;

typedef enum
{
    SB_HASH_TABLE0,
    SB_HASH_TABLE1,
    SB_HASH_TABLE2,
    SB_HASH_TABLE_NUM_MAX,
    SB_HASH_TABLE3 = SB_HASH_TABLE_NUM_MAX,
    SB_HASH_TABLE4,
    SB_HASH_TABLE5,
} SB_HT_NUM_PER_COMP;

extern int sg3_init(void);
extern void sg3_start(void);
extern uint8_t sb_spi_port_get(uint8_t spi_select);
extern uint8_t safe_add_8(uint8_t augend, uint8_t addend, uint8_t * rslt);
extern void sb_boot_sys_soft_rst(void);
extern SB_HASH_TABLE_IDS sb_get_hash_table_id_apx_compx(uint8_t ap, uint8_t comp, uint8_t ht_num);

extern uint8_t safe_sub_16_volatile(volatile uint16_t minuend, uint16_t subtrahend, volatile uint16_t * rslt);
extern uint8_t safe_sub(uint32_t minuend, uint32_t subtrahend, uint32_t * rslt);
extern uint8_t safe_add(uint32_t augend, uint32_t addend, uint32_t * rslt);
extern uint8_t safe_add_8(uint8_t augend, uint8_t addend, uint8_t * rslt);
extern uint8_t safe_add_16(uint16_t augend, uint16_t addend, uint16_t * rslt);
extern uint8_t safe_sub_8(uint8_t minuend, uint8_t subtrahend, uint8_t * rslt);
extern uint8_t safe_sub_16(uint16_t minuend, uint16_t subtrahend, uint16_t * rslt);

extern uint8_t spdm_crypto_ops_generate_dhe_secret(uint8_t *pub_key, uint8_t *pvt_key, uint8_t *dhe_secret);
extern int8_t spdm_crypto_ops_hkdf(uint8_t req_type, uint8_t algo_type, uint8_t *info, uint8_t infosz, uint8_t *key, uint8_t keysz, uint8_t *out, uint8_t outsz);
extern uint8_t spdm_di_crypto_send_ecdsa_sign_verify_request(uint8_t *public_key, uint8_t *signature, uint8_t *hash, uint16_t hash_size,
                                        uint8_t key_type, bool use_puf);
extern uint8_t spdm_crypto_ops_run_time_hmac(SPDM_CONTEXT *spdmContext, uint8_t *hash_input, uint8_t *key);
extern uint8_t spdm_aead_ops(uint8_t req_type, uint8_t *key, size_t key_size, uint8_t *iv, size_t iv_size, uint8_t *a_data, size_t a_data_size, uint8_t *in_msg, size_t in_msg_size, uint8_t *tag, uint8_t tag_size,
                            uint8_t *out_msg, size_t out_msg_size);

extern void config_task_parameters(TaskParameters_t* task_param, TaskFunction_t task_code, 
                            configSTACK_DEPTH_TYPE stk_depth, void* params, 
                            UBaseType_t priority, StackType_t *stack_buff,
                            CEC_AHB_PRIV_REGS_VALUES * cec_priv);

extern void config_task_memory_regions(TaskParameters_t* task_param, uint32_t region_num, 
                                uint32_t base_addr, uint32_t len_bytes,
                                uint32_t params);

#endif /* #ifndef APP_H */
