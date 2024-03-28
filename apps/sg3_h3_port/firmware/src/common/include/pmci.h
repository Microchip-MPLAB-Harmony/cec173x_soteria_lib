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
#ifndef INCLUDE_PMCI_H_
#define INCLUDE_PMCI_H_

#include "../../config/cec1736_s0_2zw_ev19k07a/mctp/mctp.h"
#include "FreeRTOS.h"
#include "common.h"

#define SMB_DI_EVENT_RESPONSE         (1u << 22)
#define MCTP_DI_EVENT_REQUEST         (1u << 23)
#define MCTP_DI_EVENT_RESPONSE        (1u << 22)
#define SB_CORE_DI_EVENT_RESPONSE	  (1 << 20u)
#define SB_CORE_DI_EVENT_APPLY_RESPONSE (1 << 19u)
#define tskIDLE_PRIORITY    ( ( UBaseType_t ) 0U )
#define SPT_DI_EVENT_RESPONSE           (1u << 17)
#define configMAX_PRIORITIES (8)
#define SPT_DI_EVENT_REQUEST            (1u << 23)
#define MCTP_DI_PACKET_DONE            (1u << 20)

typedef uint8_t * DI_QUEUEITEM_SMB;

// GPIO configs
#define ATTESTATION_I2C00_SLAVE_CLK_LINE             GPIO_PIN_GPIO004           //I2C00_SCL
#define ATTESTATION_I2C00_SLAVE_DAT_LINE             GPIO_PIN_GPIO003           //I2C00_SDA

#define ATTESTATION_I2C04_SLAVE_CLK_LINE             GPIO_PIN_GPIO144
#define ATTESTATION_I2C04_SLAVE_DAT_LINE             GPIO_PIN_GPIO143

#define ATTESTATION_I2C06_SLAVE_CLK_LINE             GPIO_PIN_GPIO140
#define ATTESTATION_I2C06_SLAVE_DAT_LINE             GPIO_PIN_GPIO132

#define ATTESTATION_I2C09_SLAVE_CLK_LINE             GPIO_PIN_GPIO146
#define ATTESTATION_I2C09_SLAVE_DAT_LINE             GPIO_PIN_GPIO145

#define ATTESTATION_I2C10_SLAVE_CLK_LINE             GPIO_PIN_GPIO107
#define ATTESTATION_I2C10_SLAVE_DAT_LINE             GPIO_PIN_GPIO030

#define ATTESTATION_I2C15_SLAVE_CLK_LINE             GPIO_PIN_GPIO150
#define ATTESTATION_I2C15_SLAVE_DAT_LINE             GPIO_PIN_GPIO147

// APP ids
#define DI_APP_ID_MCTP 4
#define DI_APP_ID_SPDM 5
#define DI_APP_ID_SPT  15
#define DI_APP_ID_OEM 10

/* Max number of MCTP applications */
#define DI_MAX_MCTP_APPS		2

#define IS_PWR2(x) (((x) != 0) && (((x) & ((x) - 1)) == 0))

// PLDM configs
#define FD_T1 120000 // FD_T1 timeout 120000ms for Req fw data responses

// SPDM related configs
/** SB Public Key Algo */
enum SB_AUTH_ALGO
{
    SB_AUTHALGO_ECDSA_P256,
    SB_AUTHALGO_ECDSA_P384,
    SB_AUTHALGO_UNDEFINED1,
    SB_AUTHALGO_UNDEFINED2,
    SB_AUTHALGO_RSA_PKCS_1_5,
};
typedef enum
{
    AES_GCM_ENCRYPT,
    AES_GCM_DECRYPT,
} aes_gcm_enum;
/** SPI Flash to Use to read the image */
typedef enum
{
    SPI_SELECT_SHARED_COMP_0 = 0x00, //00_00
    SPI_SELECT_SHARED_COMP_1, //00_01
    SPI_SELECT_RESERVED1,
    SPI_SELECT_RESERVED2,
    SPI_SELECT_PVT_COMP_0,    //01_00
    SPI_SELECT_PVT_COMP_1,    //01_01
    SPI_SELECT_RESERVED3,
    SPI_SELECT_RESERVED4,
    SPI_SELECT_INT_COMP_0,    //10_00
    SPI_SELECT_MAX,
} SPI_FLASH_SELECT;

#define DI_CHECK_FAIL      1
#define RLOG_LOAD_FROM_TAG0             (0x08000000u)
#define SILICON_VER_A0 0x55
#define SRAM_MBOX_HW_CONFIG_SIZE                      (0x04)

typedef enum
{
    SB_ECFW_IMG_TAG0,
    SB_ECFW_IMG_TAG1,
    SB_ECFW_IMG_APCFG0,
    SB_ECFW_IMG_APCFG1,
    SB_ECFW_IMG_INVALID
} EC_FW_IMAGE_ID;

// SMB configs
// Channel enums
enum smb_channels
{
    SMB_CHANNEL_0 = 0
    , SMB_CHANNEL_1
    , SMB_CHANNEL_2
    , SMB_CHANNEL_3
    , SMB_CHANNEL_4
};

enum MasterBusyStatus
{
    MASTER_BUSY =0      /**< Master is busy, retry after sometime */
    , MASTER_AVAILABLE  /**< Master is available, go ahead with transaction */
};

/* Port Enums */
enum smb_ports
{
    SMB_PORT_0 = 0
    , SMB_PORT_1
    , SMB_PORT_2
    , SMB_PORT_3
    , SMB_PORT_4
    , SMB_PORT_5
    , SMB_PORT_6
    , SMB_PORT_7
    , SMB_PORT_8
    , SMB_PORT_9
    , SMB_PORT_10
    , SMB_PORT_11
    , SMB_PORT_12
    , SMB_PORT_13
    , SMB_PORT_14
    , SMB_PORT_15
};

typedef struct SMB_MAPP_CBK_NEW_TX_
{
    uint8_t *buffer_ptr;      /**< Application buffer */
    uint8_t smb_protocol;     /**< SMB Protocol */
    uint8_t WriteCount;       /**< Write Count */
    uint8_t pecEnable;        /**< PEC Enable/Disable Flag */
}SMB_MAPP_CBK_NEW_TX;

typedef struct byte_match_details {
        bool is_present;
        uint8_t ap;
        uint8_t comp_id;
        uint8_t comp_img_id;
        uint32_t staged_address;
        uint32_t restore_address;
        uint32_t spi_addr;
        uint8_t image_id;
} BYTE_MATCH_DETAILS;
#define I2C_NOP    0
#define I2C_WRITE  1
#define I2C_READ   2
#define I2C_REPEAT 3

#define SMB_I2C_WRITE                               ((I2C_WRITE  << 6) + 0x00)
#define SMB_I2C_READ                                ((I2C_READ   << 6) + 0x00)
#define SMB_I2C_COMBINED                            ((I2C_REPEAT << 6) + 0x00)

#define SMB_QUICK_COMMAND                           ((I2C_WRITE  << 6) + 0x00)
#define SMB_SEND_BYTE                               ((I2C_WRITE  << 6) + 0x00)
#define SMB_RECEIVE_BYTE                            ((I2C_READ   << 6) + 0x01)
#define SMB_WRITE_BYTE                              ((I2C_WRITE  << 6) + 0x00)
#define SMB_WRITE_WORD                              ((I2C_WRITE  << 6) + 0x00)

#define SMB_READ_BYTE                               ((I2C_REPEAT << 6) + 0x01)
#define SMB_READ_WORD                               ((I2C_REPEAT << 6) + 0x02)
#define SMB_PROCESS_CALL                            ((I2C_REPEAT << 6) + 0x02)
#define SMB_WRITE_BLOCK                             ((I2C_WRITE  << 6) + 0x00)
#define SMB_READ_BLOCK                              ((I2C_REPEAT << 6) + 0x3F)
#define SMB_BLOCK_WRITE_BLOCK_READ_PROCESS_CALL     ((I2C_REPEAT << 6) + 0x3F)
/******************************************************************************/
/** struct BUFFER_INFO \n
 * This structure is used to store information of a slave receive buffer
 * XmitCount is used along with slaveXmitDoneFlag for chaining slave transmit data
*******************************************************************************/
typedef struct BUFFER_INFO
{
    UINT8 *     buffer_ptr;     /**< Pointer to buffer memory */
    UINT16      TimeStamp;      /**< Packet received timestamp */
    UINT8       DataLen;        /**< Data Length of packet received */
    UINT8       XmitCount;       /**< Number of times slave has transmitted using this buffer for this transaction */
    UINT8       RxCount;       /**< Number of times slave has received using this buffer for this transaction */
    UINT8        pecFlag;        /**< PEC valid/invalid flag */
    UINT8        slaveXmitDoneFlag;   /**< Flag indicating if xmit is completed by slave application */
    UINT8       channel;        /**< Channel on which this packet is received */
    BOOL        sdoneFlag;      /**< Flag to indicate if SDONE occured for this buffer */
    BOOL        repeatWriteFlag; /**< Flag to indicate if transaction involved a repeated write */
} BUFFER_INFO;

/******************************************************************************/
/** Slave application function pointer \n
 * The first argument is pointer to BUFFER_INFO structure which will contain
 * details of the packet received. The second parameter is flag to indicate
 * slave transmit phase. In case of slave transmit phase, the application
 * should provide the data to be transmitted in the same buffer and indicate
 * whether PEC should be enabled for transmit phase.
*******************************************************************************/
typedef UINT8 (*SLAVE_FUNC_PTR)(BUFFER_INFO *, UINT8);

// Efuse address
#define ATTESTATION_PORT_EFUSE_START_ADDR              367

// I2C Crisis mode port select bits
#define CR_MODE_I2C_PORT_BITS                               (0x07)

//Open drain, signal function 1
#define SMBus_CLK_LINE_CFG_1            ((1u<<12)| (1u<<8))
#define SMBus_DAT_LINE_CFG_1            ((1u<<12)| (1u<<8))

// Efuse calls
uint8_t efuse_read_data(uint16_t byte_index, uint8_t* efuse_buffer, uint16_t length);

// SMB function calls
uint8_t smb_protocol_execute(const uint8_t channel, uint8_t *buffer_ptr, const uint8_t smb_protocol,
                             const uint8_t writeCount,
                             const uint8_t pecEnable, void *smb_queue_item, const uint8_t readChainedFlag, const uint8_t writeChainedFlag);
extern uint8_t smb_register_slave(const uint8_t channel, SLAVE_FUNC_PTR slaveFuncPtr);
void smbus_configure_and_enable(uint8_t channel, uint16_t own_address, uint8_t speed, uint8_t port, uint8_t configFlag);
uint8_t di_request_smb_channel_busystatus(uint8_t channel_num);
extern UINT16 smb_get_current_timestamp(void);

uint8_t mctp_di_smb_configure_and_enable(uint8_t channel, uint16_t own_address, uint8_t speed, uint8_t port,
        uint8_t configFlag, uint8_t i2c_enable);
uint8_t mctp_di_smb_protocol_execute(const uint8_t channel, uint8_t *buffer_ptr, const uint8_t smb_protocol,
                                     const uint8_t writecount,
                                     const uint8_t pecEnable);

// MCTP function call
void mctp_di_send_slave_response_packet(uint8_t *mctp_rxbuf_ptr, uint8_t len, bool is_pldm);
void di_mctp_done_set(void);
void *di_mctp_context_get(void);
static void mctp_main(void* pvParameters);
void mctp_di_init(void *di_init_context);
typedef uint8_t (*MASTER_FUNC_PTR)(uint8_t, uint8_t, uint8_t *, SMB_MAPP_CBK_NEW_TX *);
void mctp_di_get_next_request(void);
void mctp_i2c_enable();
void mctp_di_get_smb_response(void);
extern void trigger_spdm_event(void);
void trigger_pldm_event(void);
void trigger_pldm_res_event(void);

// SRAM sections
uintptr_t mctp_bss_base(void);
size_t mctp_bss_size(void);
uint32_t mctp_data_mpu_attr(void);

uintptr_t spdm_bss0_base(void);
size_t spdm_bss0_size(void);
uintptr_t spdm_bss1_base(void);
size_t spdm_bss1_size(void);
uintptr_t spdm_bss2_base(void);
size_t spdm_bss2_size(void);
uint32_t spdm_data_mpu_attr(void);

uintptr_t spt_bss_base(void);
size_t spt_bss_size(void);
uint32_t spt_data_mpu_attr(void);

// SPDM function calls
void spdm_di_init(void *di_init_context);
void PLDMResp_timer_callback(TimerHandle_t pxTimer);
void spdm_di_get_i2c_response(void);
void spdm_di_get_mctp_response(void);
void spdm_di_get_sb_core_response(void);
void pldm1_event_task(void);
void pldm_pkt_tx_packet(void);
void *di_spdm_context_get(void);
uint8_t SRAM_MBOX_API_rom_hash_read(uint8_t *buffer);
uint8_t SRAM_MBOX_API_ec_fw_hash_read(uint8_t *buffer);
uint8_t SRAM_MBOX_API_hw_config_read(uint8_t *buffer);
uint8_t spdm_di_qmspi_clr_port_init_status(uint8_t spi_select);
uint8_t spdm_di_init_flash_component(SPI_FLASH_SELECT spi_select);
uint8_t spdm_di_spi_tristate(uint8_t component);
uint8_t di_send_spdm_reponse_packet(uint8_t *buffer_ptr, uint8_t len, bool is_pldm,
                                    bool is_pldm_request_firmware_update);
void spdm_di_mctp_done_set(void);
void pldm_di_mctp_done_set(void);
uint8_t di_sb_apcfg_cert_data_request(void);
uint8_t SRAM_RLOG_API_rom_event_read(uint8_t *buffer);    
uint8_t spdm_di_sb_ecfw_tagx_addr_get(uint32_t *tagx_ret, uint8_t ecfw_id);  
uint8_t di_spdm_spi_send_read_request(uint32_t spi_addr,
                                      uint8_t *mem_addr,
                                      uint32_t data_len,
                                      SPI_FLASH_SELECT spi_select,
                                      bool buffer_fill_flag);
void spdm_di_sb_i2c_ec_fw_port_prepare(bool APx_reset, uint8_t spi_select_1, uint8_t spi_select_2,
                                       uint8_t spi_select_3);
uint8_t spdm_di_i2c_spi_init(uint8_t x, uint8_t y, uint8_t z);
bool di_pldm_sb_apcfg_byte_match_staged_addr_get(uint16_t comp_iden, uint32_t *staged_addr);
uint8_t spdm_di_sb_spi_sector_erase(uint32_t spi_addr, uint8_t spi_select);
uint8_t spdm_di_sb_spi_write(uint32_t spi_addr, uint8_t spi_select, uint8_t* mem_addr, uint32_t data_len);

uint8_t GET_SILICON_VER();
uint8_t SRAM_MBOX_API_devAK_cert_read(uint16_t start_offset, uint8_t *buffer, uint8_t cert_num, uint16_t num_of_bytes);
void sb_parser_get_spi_info(uint32_t *spi_address, uint8_t *spi_select);

uint8_t mctp_di_spt_transmit_request(const uint8_t channel, uint8_t *buffer_ptr,
                                     const uint8_t writecount,
                                     const uint8_t pecEnable);
void mctp_di_get_spt_response(void);
uint8_t di_request_spt_channel_busystatus(uint8_t channel_num);
uint8_t check_for_i2c_flash_busy_bit(void);
uint8_t pldm_di_update_crisis_update_state(uint8_t status);

void spt_di_init(void *di_init_context);
void spt_di_get_next_request(void);
uint8_t mctp_di_spt_configure_and_enable(uint8_t channel, uint8_t io_mode, 
                    uint8_t spt_wait_time, uint8_t tar_time);
uint8_t di_sb_apcfg_config_data_request(void);
bool di_sb_apcfg_pldm_override_device_desc(void);
uint8_t di_pldm_sb_get_number_of_ht(uint8_t id);
uint8_t di_pldm_sb_get_number_of_byte_match_int(void);
bool di_pldm_sb_apcfg_ecfw_staged_add_get(uint8_t image_id, uint32_t *staged_addr);
bool di_pldm_sb_apcfg_apcfg_staged_add_get(uint8_t image_id, uint32_t *staged_addr);
uint32_t di_pldm_sb_apcfg_ht_staged_add_get(uint8_t ap, uint8_t comp, uint8_t ht_num);
uint16_t di_pldm_sb_apcfg_apcfg_version(bool apcfg_id);
void di_pldm_sb_get_ht_version(uint16_t *dest);
uint8_t di_pldm_sb_get_use_c1_ht_for_c0(uint8_t ap);
uint8_t di_sb_core_request_switch_mode_verify_response();
uint8_t di_sb_core_request_switch_mode(uint8_t FW_type);
uint8_t di_sb_core_restore_configs(uint8_t spi_select, bool host_functionality_reduced, uint8_t resp_command);
uint8_t spdm_di_mctp_done_wait(EventBits_t bits_to_wait);
void spdm_di_core_done_set(void);
uint32_t di_pldm_sb_apcfg_apcfg_base_address_get(uint8_t apcfg_id);
bool di_pldm_sb_apcfg_ecfw_update_info_get(uint8_t image_id, uint32_t* staged_addr, uint32_t* restore_addr);
bool di_pldm_sb_apcfg_apcfg_update_info_get(uint8_t image_id, 
        uint32_t* staged_addr, uint32_t* restore_addr);
bool di_pldm_sb_apcfg_byte_match_info_get(uint16_t comp_iden, uint32_t *staged_addr, uint32_t *restore_addr, 
            uint32_t *tagx_addr, uint8_t *img_id);
bool di_pldm_sb_apcfg_apfw_img_info_get(uint16_t comp_iden, uint32_t *staged_addr, uint32_t *restore_addr, 
            uint32_t *tagx_addr, uint8_t *img_id);
bool di_pldm_sb_apcfg_apfw_img_staged_addr_get(uint16_t comp_iden, uint32_t *staged_addr);
uint8_t di_pldm_sb_get_number_of_apfw_images(void);

uint8_t spdm_di_sb_core_i2c_ec_fw_update_start(uint32_t staged_addr, uint8_t FW_type, uint32_t restore_addr,
        uint32_t max_img_size,
        uint8_t ecfw_image_id, uint32_t tagx_addr, bool is_pldm, bool failure_recovery_cap, uint8_t ht_id);

void *di_spt_context_get(void);
//extern void spt_di_tx_callback(DI_QUEUEITEM_SPT *spt_queue_item); 
#endif /* INCLUDE_PMCI_H_ */