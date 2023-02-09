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

#define config_CEC_AHB_PROTECTION_ENABLE 1
#define config_CEC_DATA_ISOLATION_CHECKS 1
typedef uint8_t * DI_QUEUEITEM_SMB;

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

#define DI_APP_ID_MCTP 4

/* Max number of MCTP applications */
#define DI_MAX_MCTP_APPS		2

/* For CEC application  */
#define IS_PWR2(x) (((x) != 0) && (((x) & ((x) - 1)) == 0))

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

/******************************************************************************/
/** struct SMB_MAPP_CBK_NEW_TX \n
 * This structure is used to get information for new Tx from Application Callback
*******************************************************************************/
typedef struct SMB_MAPP_CBK_NEW_TX_
{
    uint8_t *buffer_ptr;      /**< Application buffer */
    uint8_t smb_protocol;     /**< SMB Protocol */
    uint8_t WriteCount;       /**< Write Count */
    uint8_t pecEnable;        /**< PEC Enable/Disable Flag */
}SMB_MAPP_CBK_NEW_TX;

#define configTICK_RATE_HZ (200)
#define ATTESTATION_PORT_EFUSE_START_ADDR              367
/* I2C Crisis mode port select bits */
#define CR_MODE_I2C_PORT_BITS                               (0x07)

//Open drain, signal function 1
#define SMBus_CLK_LINE_CFG_1            ((1u<<12)| (1u<<8))
#define SMBus_DAT_LINE_CFG_1            ((1u<<12)| (1u<<8))
#define SMB_DI_EVENT_RESPONSE                 (1u << 22)
#define MCTP_DI_EVENT_REQUEST         (1u << 23)

#define tskIDLE_PRIORITY    ( ( UBaseType_t ) 0U )
#define configMAX_PRIORITIES (8)

uint16_t tx_time_get(void);
uint8_t smb_protocol_execute(const uint8_t channel, uint8_t *buffer_ptr, const uint8_t smb_protocol,
                             const uint8_t writeCount,
                             const uint8_t pecEnable, void *smb_queue_item, const uint8_t readChainedFlag, const uint8_t writeChainedFlag);
uint8_t smb_register_slave(const uint8_t channel, I2C_SLAVE_FUNC_PTR slaveFuncPtr);
void smbus_configure_and_enable(uint8_t channel, uint16_t own_address, uint8_t speed, uint8_t port, uint8_t configFlag);
typedef uint32_t * EventGroupHandle_t;
typedef long             BaseType_t;

void mctp_di_send_slave_response_packet(uint8_t *mctp_rxbuf_ptr, uint8_t len, bool is_pldm);
uint8_t efuse_read_data(uint16_t byte_index, uint8_t* efuse_buffer, uint16_t length);
void di_mctp_done_set(void);
uint8_t di_request_smb_channel_busystatus(uint8_t channel_num);
void *di_mctp_context_get(void);
static void mctp_main(void* pvParameters);
uintptr_t mctp_bss_base(void);
size_t mctp_bss_size(void);
uint32_t mctp_data_mpu_attr(void);
void mctp_di_init(void *di_init_context);
typedef uint8_t (*MASTER_FUNC_PTR)(uint8_t, uint8_t, uint8_t *, SMB_MAPP_CBK_NEW_TX *);
void mctp_di_get_next_request(void);
void mctp_i2c_enable();
void mctp_di_get_smb_response(void);
extern uint16_t smb_get_current_timestamp(void);
#define tx_time_get()   smb_get_current_timestamp()
uint8_t mctp_di_smb_configure_and_enable(uint8_t channel, uint16_t own_address, uint8_t speed, uint8_t port,
        uint8_t configFlag);
uint8_t mctp_di_smb_protocol_execute(const uint8_t channel, uint8_t *buffer_ptr, const uint8_t smb_protocol,
                                     const uint8_t writecount,
                                     const uint8_t pecEnable);
#endif /* INCLUDE_PMCI_H_ */