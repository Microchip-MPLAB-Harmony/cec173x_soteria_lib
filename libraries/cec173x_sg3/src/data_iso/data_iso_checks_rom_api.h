/*****************************************************************************
* Copyright 2021 Microchip Technology Inc. and its subsidiaries.
* You may use this software and any derivatives exclusively with
* Microchip products.
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP 'AS IS'.
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

/** @file data_iso_checks_rom_api.h
 * data_isolation cross checks
 */
/** @defgroup data_isolation
 */

#ifndef DATA_ISO_CHECKS_ROM_API_H
#define DATA_ISO_CHECKS_ROM_API_H

// DI Permissions
enum di_permissions_rom_apis_tbl0
{
    DI_PERMISSION_ROM_API_NONE_TBL0        = 0x40000000U,   //Table0 - NO PERM

    DI_PERMISSION_ROM_API_ROM_VER_TBL0       = 0x40000001U,   //Table 0 - Bit 1
    DI_PERMISSION_ROM_API_QMSPI_INIT_TBL0      = 0x40000002U,   //Table 0 - Bit 2
    DI_PERMISSION_ROM_API_QMSPI_PORT_CTRL_TBL0     = 0x40000004U,   //Table 0 - Bit 3
    DI_PERMISSION_ROM_API_QMSPI_FLASH_CMD_TBL0     = 0x40000008U,   //Table 0 - Bit 4

    DI_PERMISSION_ROM_API_QMSPI_IS_DONE_STATUS_TBL0    = 0x40000010U,   //Table 0 - Bit 5
    DI_PERMISSION_ROM_API_QMSPI_FLASH_PGM_DMA_TBL0    = 0x40000020U,   //Table 0 - Bit 6
    DI_PERMISSION_ROM_API_QMSPI_PORT_DRV_SLEW_TBL0    = 0x40000040U,   //Table 0 - Bit 7
    DI_PERMISSION_ROM_API_QMSPI_START_TBL0      = 0x40000080U,   //Table 0 - Bit 8

    DI_PERMISSION_ROM_API_DMA_DEV_XFR_CFG_TBL0     = 0x40000100U,   //Table 0 - Bit 9
    DI_PERMISSION_ROM_API_QMSPI_FLASH_READ24_DMA_TBL0   = 0x40000200U,   //Table 0 - Bit 10
    DI_PERMISSION_ROM_API_QMSPI_FLASH_READ_24_32_DMA_TBL0  = 0x40000400U,   //Table 0 - Bit 11
    DI_PERMISSION_ROM_API_QMSPI_FLASH_READ_24_32_LDMA_TBL0  = 0x40000800U,   //Table 0 - Bit 12

    DI_PERMISSION_ROM_API_MCHP_HASH_CREATE_SHA1_TBL0   = 0x40001000U,   //Table 0 - Bit 13
    DI_PERMISSION_ROM_API_MCHP_HASH_CREATE_SHA224_TBL0   = 0x40002000U,   //Table 0 - Bit 14
    DI_PERMISSION_ROM_API_MCHP_HASH_CREATE_SHA256_TBL0   = 0x40004000U,   //Table 0 - Bit 15
    DI_PERMISSION_ROM_API_MCHP_HASH_CREATE_SHA384_TBL0   = 0x40008000U,   //Table 0 - Bit 16

    DI_PERMISSION_ROM_API_MCHP_HASH_CREATE_SHA512_TBL0   = 0x40010000U,   //Table 0 - Bit 17
    DI_PERMISSION_ROM_API_MCHP_HASH_FEED_TBL0     = 0x40020000U,   //Table 0 - Bit 18
    DI_PERMISSION_ROM_API_MCHP_HASH_DIGEST_TBL0     = 0x40040000U,   //Table 0 - Bit 19
    DI_PERMISSION_ROM_API_MCHP_HASH_WAIT_TBL0     = 0x40080000U,   //Table 0 - Bit 20

    DI_PERMISSION_ROM_API_MCHP_PK_GET_CURVE_NISTP384_TBL0  = 0x40100000U,   //Table 0 - Bit 21
    DI_PERMISSION_ROM_API_MCHP_PK_GET_CURVE_NISTP256_TBL0  = 0x40200000U,   //Table 0 - Bit 22
    DI_PERMISSION_ROM_API_MCHP_PK_INIT_TBL0      = 0x40400000U,   //Table 0 - Bit 23
    DI_PERMISSION_ROM_API_MCHP_ECDSA_VERIFY_TBL0    = 0x40800000U,   //Table 0 - Bit 24

    DI_PERMISSION_ROM_API_MCHP_ASYNC_ECDSA_GENERATE_GO_TBL0  = 0x41000000U,   //Table 0 - Bit 25
    DI_PERMISSION_ROM_API_BK_ECDSA_SIGN_TBL0     = 0x42000000U,   //Table 0 - Bit 26
    DI_PERMISSION_ROM_API_MCHP_PK_WAIT_TBL0      = 0x44000000U,   //Table 0 - Bit 27
    DI_PERMISSION_ROM_API_MCHP_NDRNG_INIT_TBL0     = 0x48000000U,   //Table 0 - Bit 28

    /* Last two bits used as table ID */
};

enum di_permissions_rom_apis_tbl_1
{
    DI_PERMISSION_ROM_API_NONE_TBL1        = 0x80000000U,   //Table1 - NO PERM

    DI_PERMISSION_ROM_API_MCHP_NDRNG_ENABLE_TBL1    = 0x80000001U,   //Table 0 - Bit 1
    DI_PERMISSION_ROM_API_MCHP_NDRNG_SOFT_RESET_TBL1   = 0x80000002U,   //Table 0 - Bit 2
    DI_PERMISSION_ROM_API_MCHP_CRYPT_SLEEP_CONTROL_TBL1   = 0x80000004U,   //Table 1 - Bit 3
    DI_PERMISSION_ROM_API_MCHP_NDRNG_INTR_EN_TBL1    = 0x80000008U,   //Table 1 - Bit 4

    DI_PERMISSION_ROM_API_MCHP_NDRNG_READ_BYTES_TBL1   = 0x80000010U,   //Table 1 - Bit 5
    DI_PERMISSION_ROM_API_MCHP_ASYNC_ECDSA_GENERATE_END_TBL1 = 0x80000020U,   //Table 1 - Bit 6
    DI_PERMISSION_ROM_API_MCHP_HASH_INIT_STATE_TBL1    = 0x80000040U,   //Table 1 - Bit 7
    DI_PERMISSION_ROM_API_MCHP_HASH_RESUME_STATE_TBL1   = 0x80000080U,   //Table 1 - Bit 8

    DI_PERMISSION_ROM_API_MCHP_HASH_SAVE_STATE_TBL1    = 0x80000100U,   //Table 1 - Bit 9
    DI_PERMISSION_ROM_API_EFUSE_BYTE_READ_TBL1     = 0x80000200U,   //Table 1 - Bit 10
    DI_PERMISSION_ROM_API_EFUSE_BYTE_WRITE_TBL1     = 0x80000400U,   //Table 1 - Bit 11
};

/******************************************************************************/
/** oem_task1_rom_api_permission_add
* Add an entry into the ROM API permission table for OEM task1
* @param reqd_permission  Permission that is required
* @return None
*******************************************************************************/
uint8_t oem_task1_rom_api_permission_add(uint32_t reqd_permission);

/******************************************************************************/
/** oem_task2_rom_api_permission_add
* Add an entry into the ROM API permission table for OEM task2
* @param reqd_permission  Permission that is required
* @return None
*******************************************************************************/
uint8_t oem_task2_rom_api_permission_add(uint32_t reqd_permission);

/******************************************************************************/
/** oem_task3_rom_api_permission_add
* Add an entry into the ROM API permission table for OEM task3
* @param reqd_permission  Permission that is required
* @return None
*******************************************************************************/
uint8_t oem_task3_rom_api_permission_add(uint32_t reqd_permission);

#endif
/* end DATA_ISO_CHECKS_SRAM_MBOX_H */
/**   @}
 */