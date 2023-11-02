
//#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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

#endif /* #ifndef APP_H */
