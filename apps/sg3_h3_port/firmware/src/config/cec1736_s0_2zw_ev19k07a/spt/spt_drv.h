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

#ifndef SPT_DRV_H
#define SPT_DRV_H

#include <stdint.h>
#include <stddef.h>
#include "spt_common.h"

#define SPT_RX_BUF_SIZE             80U
#define SPT_TX_BUF_SIZE             80U

/* No. of clock cycles required to xfer 1 byte */
#define SINGLE_CYCLES_PER_BYTE      8U
#define QUAD_CYCLES_PER_BYTE        2U

#define FINAL_STATUS_NUM_BYTES      2U

enum SPT_INSTANCE
{
    SPT0,
    SPT1,
    SPT_MAX,
};

enum SPT_IO_MODE
{
    SPT_IO_SINGLE,
    SPT_IO_QUAD
};

enum SPT_READ_STATES
{
    SPT_READ_IDLE_STATE,
    SPT_READ_IN_PROGRESS_STATE,  // Host write done set 
    SPT_PROCESS_DATA_READ_STATE, // Send received data to the application 
    SPT_READ_DONE_STATE,         // Host write done and EC Read done set 
    SPT_WAIT_READ_DONE_ACK_STATE, // Host polls EC read done and sets host write done 
    SPT_READ_COMPLETE_STATE,     // Host write done and EC Read done clear
    SPT_READ_ENTER_ERROR_STATE,
    SPT_READ_WAIT_ERROR_STATE,
    SPT_READ_EXIT_ERROR_STATE,
};

enum SPT_WRITE_STATES
{
    SPT_WRITE_IDLE_STATE,
    SPT_WRITE_DONE_STATE,        // EC Write done 
    SPT_WAIT_WRITE_COMPLETE_STATE,
    SPT_WRITE_COMPLETE_STATE,    // Host read done 
};

typedef union HOST2EC_MBX_SPT_STATUS_UNION
{
    struct HOST2EC_MBX_SPT_STATUS_BITS_STRUCT
    {
        uint32_t host_wr_done : 1;
        uint32_t host_rd_done : 1;
        uint32_t host_err     : 1;
        uint32_t host_rsvd    : 29;
    }HOST2EC_MBX_SPT_STATUS_BITS;
    
    uint32_t host2ec_mbx_sts;
    
}HOST2EC_MBX_SPT_STATUS;

typedef union SPT_STATUS_EC2HOST_MBX_UNION
{
    struct SPT_STATUS_BITS_EC2HOST_MBX_STRUCT
    {
        uint32_t ec_rd_done : 1;
        uint32_t ec_wr_done : 1;
        uint32_t ec_err     : 1;
        uint32_t ec_rsvd    : 29;
    }SPT_STATUS_BITS_EC2HOST_MBX;
    
    uint32_t ec2host_mbx_sts;
    
}SPT_STATUS_EC2HOST_MBX;

typedef union SPT_STATUS_REG_UNION
{
    struct SPT_STATUS_REG_STRUCT
    {
        uint32_t mem_wr_done     : 1;
        uint32_t mem_rd_done     : 1;
        uint32_t rsvd1           : 1;
        uint32_t mem_wr_bsy      : 1;
        uint32_t mem_rd_bsy      : 1;
        uint32_t sreg_trns_bsy   : 1;
        uint32_t poll_hi_rq      : 1;
        uint32_t rsvd2           : 1;
        uint32_t rxfifo_emty     : 1;
        uint32_t rxfifo_full     : 1;
        uint32_t txfifo_emty     : 1;
        uint32_t txfifo_full     : 1;
        uint32_t rsvd3           : 2;
        uint32_t ibf             : 1;
        uint32_t obf             : 1;
        uint32_t rst_req         : 1;
        uint32_t rxfifo_rst_done : 1;
        uint32_t txfifo_rst_done : 1;
        uint32_t lmt0_err        : 1;
        uint32_t lmt1_err        : 1;
        uint32_t arm_bus_err     : 1;
        uint32_t undef_cmd       : 1; 
        uint32_t dv_bsy          : 1; 
        uint32_t rxfifi_sz_err   : 1;
        uint32_t txfifo_undrflw  : 1;
        uint32_t txfifo_ovrflw   : 1; 
        uint32_t rxfifo_undrflw  : 1;
        uint32_t rxfifo_ovrflw   : 1; 
        uint32_t rsvd4           : 3;
    }SPT_STATUS_REG_BITS;
    
    uint32_t spt_sts;
    
}SPT_SPT_STATUS_REG;

typedef struct SPT_IO_CFG_STRUCT
{
    uint8_t io_mode;
    uint8_t wait_time;
    uint8_t tar_time;     
}SPT_IO_CFG;

typedef struct SPT_MEM_CFG_STRUCT
{
    uint8_t* tx_buf_ptr;
    uint8_t* rx_buf_ptr;
    uint32_t tx_buf_max_limit;
    uint32_t rx_buf_max_limit;
    uint32_t write_count;
    uint32_t read_count;  
}SPT_MEM_CFG;

typedef struct SPT_STATE_HANDLE_STRUCT
{
    uint8_t write_state;
    uint8_t read_state; 
}SPT_STATE_HANDLE;


typedef struct SPT_BUFFER_INFO
{
    uint8_t *     buffer_ptr;     /**< Pointer to buffer memory */
    uint16_t      TimeStamp;      /**< Packet received timestamp */
    uint32_t       DataLen;        /**< Data Length of packet received */
    uint32_t       XmitCount;       /**< Number of times slave has transmitted using this buffer for this transaction */
    uint32_t       RxCount;       /**< Number of times slave has received using this buffer for this transaction */
    uint8_t       pecFlag;        /**< PEC valid/invalid flag */
    uint8_t       channel;        /**< Channel on which this packet is received */

} SPT_BUFFER_INFO;

typedef struct SPT_DRV_MAPP_CBK_NEW_TX_STRUCT
{
    uint8_t *buffer_ptr;      /**< Application buffer */
    uint8_t WriteCount;       /**< Write Count */
    uint8_t pecEnable;        /**< PEC Enable/Disable Flag */
}SPT_DRV_MAPP_CBK_NEW_TX;

enum SPT_APPLICATION_PACKET_RX_STATUS
{
    SPT_APP_BUFFER_NOT_DONE= 0 ,  /**< Packet not accepted, since application is busy */
    SPT_APP_BUFFER_DONE,         /**< Packet accepted by application */
    SPT_APP_BUFFER_ERROR,        /**< Packet not meant for this application */     
};

enum SPT_APPLICATION_TX_STATUS
{
    SPT_DRV_APP_ERROR_TIMEOUT =0        /**< Bus Error due to Timeout */
    , SPT_DRV_APP_ERROR_PEC                 /**< PEC Error */
    , SPT_DRV_APP_TX_BUSY_ERROR        /** TX already in progress */
    , SPT_DRV_APP_ENABLE_ERROR
    , SPT_DRV_APP_TX_BUFF_ERROR
    , SPT_DRV_APP_SUCCESS_TX                /**< Successful Master Tx */

};

typedef uint8_t (*TX_FUNC_PTR)(uint8_t, uint8_t, uint8_t*, SPT_DRV_MAPP_CBK_NEW_TX *);

typedef uint8_t (*SPT_SLAVE_FUNC_PTR)(SPT_BUFFER_INFO *, uint8_t);

typedef struct SPT_APP_INFO_STRUCT {
    uint8_t pec_enable;
    SPT_SLAVE_FUNC_PTR applRxFuncPtr;
    TX_FUNC_PTR applTxFuncPtr;
    uint8_t* app_tx_buff;
}SPT_APP_INFO;

typedef struct SPT_DRV_CONTEXT_STRUCT {
    uint8_t spt_enable;
    SPT_IO_CFG spt_io_cfg;
    SPT_MEM_CFG spt_mem_cfg;  
    HOST2EC_MBX_SPT_STATUS host_mbx;
    SPT_STATUS_EC2HOST_MBX ec_mbx;
    SPT_SPT_STATUS_REG spt_sts_reg;
    SPT_STATE_HANDLE spt_stat_handler;
    SPT_APP_INFO appl_info;
}SPT_DRV_CONTEXT;



//uint8_t spt_init_advanced_single_mode();
//uint8_t spt_init_advanced_quad_mode();

uint8_t spt_config_and_enable(uint8_t channel, uint8_t io_mode, uint8_t spt_wait_time,
        uint8_t tar_time, uint8_t pec_en);
void spt_read_event_handler();
void spt_write_event_handler();

void spt_isr(uint32_t status, uintptr_t context);

uint8_t spt_check_tx_status(uint8_t channel);

uint8_t spt_register_app_rx_callback(uint8_t channel, SPT_SLAVE_FUNC_PTR slaveFuncPtr);

uint8_t spt_write(uint8_t channel, uint8_t* buff_ptr, uint16_t writecount, uint8_t pecEnable, TX_FUNC_PTR func_ptr);

uint8_t spt_handle_received_data(uint8_t channel);
        
void spt_disable(uint8_t channel);

#endif /* SPT_DRV_H */