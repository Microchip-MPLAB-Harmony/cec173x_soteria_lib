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

// FreeRTOS configs
#define config_CEC_AHB_PROTECTION_ENABLE 1
#define config_CEC_DATA_ISOLATION_CHECKS 1
#define SMB_DI_EVENT_RESPONSE         (1u << 22)
#define MCTP_DI_EVENT_REQUEST         (1u << 23)
#define MCTP_DI_EVENT_RESPONSE        (1u << 22)
#define SB_CORE_DI_EVENT_RESPONSE	  (1 << 20u)
#define SB_CORE_DI_EVENT_APPLY_RESPONSE (1 << 19u)
#define tskIDLE_PRIORITY    ( ( UBaseType_t ) 0U )
#define configMAX_PRIORITIES (8)
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

/* Max number of MCTP applications */
#define DI_MAX_MCTP_APPS		2

#define IS_PWR2(x) (((x) != 0) && (((x) & ((x) - 1)) == 0))

// PLDM configs
#define FD_T1 120000 // FD_T1 timeout 120000ms for Req fw data responses

// SPDM related configs
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
    SPI_SELECT_MAX
} SPI_FLASH_SELECT;

#define PLDM_IDLE_STATE 0
#define DI_CHECK_FAIL      1
#define RLOG_LOAD_FROM_TAG0             (0x08000000u)
#define ECFW_IMG_TAG0 0
#define ECFW_IMG_TAG1 1
#define SILICON_VER_A0 0x55
#define SRAM_MBOX_HW_CONFIG_SIZE                      (0x04)
#define INITIALIZATION_OF_FD 0

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

/**  SPDM Context Information
*******************************************************************************/
typedef struct SPDM_CONTEXT
{
    uint8_t spdm_state_info;

    uint8_t current_resp_cmd;

    uint8_t challenge_success_flag;

    uint8_t host_eid;

    uint8_t ec_eid;

    uint8_t ec_slv_addr;

    uint8_t host_slv_addr;

    /* Event group handle */
    EventGroupHandle_t xSPDMEventGroupHandle;

    /* Event group buffer*/
    StaticEventGroup_t xSPDMCreatedEventGroup;

    uint8_t no_of_chains_avail;

    uint8_t previous_state;

    uint8_t get_requests_state;

    uint8_t requested_slot[8];

    uint8_t sha_digest[48];

    uint8_t request_or_response;

    uint16_t current_request_length;

    uint16_t previous_request_length;

    uint16_t total_request_bytes;

    uint16_t cert_offset_to_read[8];

    uint16_t cert_bytes_to_sent_curr[8];

    uint16_t cert_bytes_pending_to_sent[8];

    uint16_t cert_bytes_requested[8];

    uint8_t pldm_state_info; // added this to track PLDM packets (if tracking with spdm_state_info, if spdm packet comes in between
    // pldm packets, the current state would go into toss)

    uint8_t pldm_tx_state;

    uint8_t pldm_host_eid;

    uint8_t pldm_ec_eid;

    uint8_t pldm_ec_slv_addr;

    uint8_t pldm_host_slv_addr;

    uint8_t pldm_instance_id;

    uint8_t pldm_current_response_cmd;

    uint8_t pldm_current_state;

    uint8_t pldm_previous_state;  // maintaining this for sending previous state in GetStatus command

    uint8_t pldm_next_state;

    uint8_t pldm_verify_state;

    uint8_t pldm_apply_state;

    uint8_t pldm_status_reason_code;

    uint8_t current_pkt_sequence; // PLDM; used to find for packet loss and retry

    uint8_t expected_pkt_sequence;

    /* PLDM timeout response Timer Handle*/
    TimerHandle_t xPLDMRespTimer;

    /* PLDM timeout response Timer buffer*/
    StaticTimer_t PLDMResp_TimerBuffer __attribute__((aligned(8)));

} __attribute__((packed)) SPDM_CONTEXT;

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
uint16_t tx_time_get(void);
uint8_t smb_protocol_execute(const uint8_t channel, uint8_t *buffer_ptr, const uint8_t smb_protocol,
                             const uint8_t writeCount,
                             const uint8_t pecEnable, void *smb_queue_item, const uint8_t readChainedFlag, const uint8_t writeChainedFlag);
uint8_t smb_register_slave(const uint8_t channel, I2C_SLAVE_FUNC_PTR slaveFuncPtr);
void smbus_configure_and_enable(uint8_t channel, uint16_t own_address, uint8_t speed, uint8_t port, uint8_t configFlag);
uint8_t di_request_smb_channel_busystatus(uint8_t channel_num);
extern uint16_t smb_get_current_timestamp(void);
#define tx_time_get()   smb_get_current_timestamp()
uint8_t mctp_di_smb_configure_and_enable(uint8_t channel, uint16_t own_address, uint8_t speed, uint8_t port,
        uint8_t configFlag);
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
void trigger_spdm_event(void);
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


// SPDM function calls
void spdm_di_init(void *di_init_context);
void PLDMResp_timer_callback(TimerHandle_t pxTimer);
void spdm_di_get_i2c_response(void);
void pldm_pkt_get_config_from_apcfg(SPDM_CONTEXT *spdmContext);
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
uint8_t di_sb_apcfg_cert_data_request(void);
uint8_t SRAM_RLOG_API_rom_event_read(uint8_t *buffer);    
uint8_t spdm_di_sb_ecfw_tagx_addr_get(uint32_t *tagx_ret, uint8_t ecfw_id);  
uint8_t di_spdm_spi_send_read_request(uint32_t spi_addr,
                                      uint8_t *mem_addr,
                                      uint32_t data_len,
                                      SPI_FLASH_SELECT spi_select,
                                      bool buffer_fill_flag);
uint8_t GET_SILICON_VER();
uint8_t SRAM_MBOX_API_devAK_cert_read(uint16_t start_offset, uint8_t *buffer, uint8_t cert_num, uint16_t num_of_bytes);
#endif /* INCLUDE_PMCI_H_ */