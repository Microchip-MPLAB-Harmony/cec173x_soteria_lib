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

/** @file mctp.h
 * Interface header file for MCTP
 */
/** @defgroup MCTP interface
 */

#ifndef MCTP_H    /* Guard against multiple inclusion */
#define MCTP_H

#include "definitions.h"
#include "mctp_common.h"
#include "mctp_base.h"
#include "../../common/include/FreeRTOS.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/** Global buffer to store MCTP. SPDM and PLDM request and response packets
 * mctp_pktbuf[MCTP_BUF1] - MCTP control request and response packet buffer
 * mctp_pktbuf[MCTP_BUF2] - SPDM response packet buffer
 * mctp_pktbuf[MCTP_BUF3] - SPDM request packet buffer
 * mctp_pktbuf[MCTP_BUF4] - PLDM request packet buffer
 * mctp_pktbuf[MCTP_BUF5] - PLDM response packet buffer
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * -----------------------
 * SPDM packet processing:
 * -----------------------
 * MCTP module will store SPDM request packets into mctp_pktbuf[MCTP_BUF3]
 * SPDM module is expected to read mctp_pktbuf[MCTP_BUF3], process the request
 * packet, and store the response packet into mctp_pktbuf[MCTP_BUF2]
 * Example:
 * MCTP_PKT_BUF *spdm_pkt_rx = mctp_pktbuf[MCTP_BUF3];
 * MCTP_PKT_BUF *spdm_pkt_tx = mctp_pktbuf[MCTP_BUF2];
 * Now use spdm_pkt_rx to process the received SPDM packet and use
 * spdm_tx_buf to populate the response packet
 * ############################################################################
 * -----------------------
 * PLDM packet processing:
 * -----------------------
 * MCTP module will store PLDM request packets into mctp_pktbuf[MCTP_BUF4]
 * PLDM module is expected to read mctp_pktbuf[MCTP_BUF4], process the request
 * packet, and store the response packet into mctp_pktbuf[MCTP_BUF5]
 * Example:
 * MCTP_PKT_BUF *pldm_pkt_rx = mctp_pktbuf[MCTP_BUF4];
 * MCTP_PKT_BUF *pldm_pkt_tx = mctp_pktbuf[MCTP_BUF5];
 * Now use spdm_pkt_rx to process the received SPDM packet and use
 * spdm_tx_buf to populate the response packet
 * ############################################################################
*******************************************************************************/
extern MCTP_BSS_ATTR MCTP_PKT_BUF mctp_pktbuf[MCTP_PKT_BUF_NUM]__attribute__ ((aligned(8)));

/******************************************************************************/
/** struct I2C_MAPP_CBK_NEW_TX
 * This structure is used to get information for new Tx from Application Callback
*******************************************************************************/
typedef struct I2C_MAPP_CBK_NEW_TX
{
    uint8_t *buffer_ptr;      /**< Application buffer */
    uint8_t smb_protocol;     /**< SMB Protocol */
    uint8_t WriteCount;       /**< Write Count */
    uint8_t pecEnable;        /**< PEC Enable/Disable Flag */
} I2C_MAPP_CBK_NEW_TX;

/******************************************************************************/
/** struct SPT_MAPP_CBK_NEW_TX
 * This structure is used to get information for new Tx from Application Callback
*******************************************************************************/
typedef struct SPT_MAPP_CBK_NEW_TX
{
    uint8_t *buffer_ptr;      /**< Application buffer */
    uint8_t WriteCount;       /**< Write Count */
    uint8_t pecEnable;        /**< PEC Enable/Disable Flag */
} SPT_MAPP_CBK_NEW_TX;

/******************************************************************************/
/** enum i2c_master_busy_status 
 * Enumeration to denote whether master is busy or ready to send the next packet
 * on the bus
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * The I2C driver is expected to use the values in this enumeration
 * when returning from mctp_i2c_get_chan_busy_status interface function
 * -----------------------
 * Example:
 * -----------------------
 * Refer mctp_i2c_get_chan_busy_status interface function
 * ############################################################################
*******************************************************************************/
enum i2c_master_busy_status
{
    I2C_MASTER_BUSY =0      /**< Master is busy, retry after sometime */
   , I2C_MASTER_AVAILABLE   /**< Master is available, go ahead with transaction */
};

/******************************************************************************/
/** enum i2c_slave_app_reg_status 
 * Enumeration to denote status codes for slave registration by application
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * The I2C driver is expected to use the values in this enumeration
 * when returning from mctp_i2c_rx_register interface function
 * -----------------------
 * Example:
 * -----------------------
 * Refer mctp_i2c_rx_register interface function
 * ############################################################################
*******************************************************************************/
enum i2c_slave_app_reg_status
{
     I2C_SLAVE_APP_STATUS_ALREADY_REGISTERED = 0  /**< Application has already registered */
    , I2C_SLAVE_APP_STATUS_MAX_APP_REGISTERED     /**< Maximum number of application registered */
    , I2C_SLAVE_APP_STATUS_APP_NOT_REGISTERED     /**< Application is not registered */
    , I2C_SLAVE_APP_STATUS_OK                     /**< Success status */
};

/******************************************************************************/
/** enum spt_busy_status 
 * Enumeration to denote whether master is busy or ready to send the next packet
 * on the bus
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * The SPT driver is expected to use the values in this enumeration
 * when returning from mctp_spi_get_chan_busy_status interface function
 * -----------------------
 * Example:
 * -----------------------
 * Refer mctp_spi_get_chan_busy_status interface function
 * ############################################################################
*******************************************************************************/
enum spi_busy_status
{
    SPT_TX_BUSY =0      /**< SPI is busy, retry after sometime */
   , SPT_TX_AVAILABLE   /**< SPI is available, go ahead with transaction */
};

/******************************************************************************/
/** enum spi_slave_app_reg_status 
 * Enumeration to denote status codes for slave registration by application
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * The SPI driver is expected to use the values in this enumeration
 * when returning from mctp_spi_rx_register interface function
 * -----------------------
 * Example:
 * -----------------------
 * Refer mctp_spi_rx_register interface function
 * ############################################################################
*******************************************************************************/
enum spi_slave_app_reg_status
{
     SPT_SLAVE_APP_STATUS_ALREADY_REGISTERED = 0  /**< Application has already registered */
    , SPT_SLAVE_APP_STATUS_APP_NOT_REGISTERED     /**< Application is not registered */
    , SPT_SLAVE_APP_STATUS_OK                     /**< Success status */
};

/******************************************************************************/
/** Status codes for master transaction. The error codes are for Bus Error,
 * MNAKX and PEC Error. The success codes are for successful tx and successful
 * rx (we could have only one success code, though)
 * @note
 * -----------------------
 * Usage notes:
 * -----------------------
 * The  driver is expected to use the values in this enumeration to communicate 
 * to the MCTP module about the packet transmission status 
 * -----------------------
 * Example:
 * -----------------------
 * Refer mctp_i2c_tx / mctp_spt_tx interface function
 *******************************************************************************/
enum mctp_tx_status
{
    I2C_ERROR_BER_TIMEOUT =0        /**< Bus Error due to Timeout */
    , I2C_ERROR_BER_NON_TIMEOUT     /**< Bus Error due to Non Timeout */
    , I2C_ERROR_LAB                 /**< Lost Arbitration Error */
    , I2C_ERROR_MADDR_NAKX          /**< Slave sent Address NACK */
    , I2C_ERROR_MDATA_NAKX          /**< Slave sent Data NACK */
    , I2C_ERROR_SMB_DISABLED        /**< smbus is disabled thru Vreg */
    , I2C_ERROR_CLK_DATA_NOT_HIGH   /**< CLK or Data Not High */
    , I2C_ERROR_PEC                 /**< PEC Error */
    , I2C_SUCCESS_TX                /**< Successful Master Tx */
    , I2C_SUCCESS_RX                /**< Successful Master Rx */
    , I2C_SUCCESS_TX_CHAINED        /**< Successful Master Tx for chained transfer - intermediate status*/
    , I2C_SUCCESS_RX_CHAINED        /**< Successful Master Rx for chained transfer - intermediate status*/

};

enum mctp_status_spt
{
    SPT_ERR_TIMEOUT =0        /**< Bus Error due to Timeout */
    , SPT_PEC_ERROR                 /**< PEC Error */
    , SPT_BUSY_TX        /** TX already in progress */
    , SPT_ENABLE_ERROR
    , SPT_TX_BUFF_ERROR
    , SPT_SUCCESS_TX                /**< Successful Master Tx */

};

/******************************************************************************/
/** Application return values
 * Action codes to the I2C driver about the action to take for the current
 * packet
 * @note
 * -----------------------
 * Usage notes:
 * -----------------------
 * The MCTP module will use the values in this enumeration to communicate to the
 * i2c driver about the action to take with the current packet, when the i2c driver
 * communicates about the transmission status to the MCTP module (refer mctp_i2c_tx)
 * The I2C driver is expected to take relevent action
 *******************************************************************************/
enum i2c_application_return_values
{
    I2C_APP_RETVAL_RELEASE_SMBUS =0     /**< Application releases hold on smbus */
    , I2C_APP_RETVAL_HOLD_SMBUS         /**< Application still acquires smbus */
    , I2C_APP_RETVAL_RETRY              /**< Application wants to retry the current transaction */
    , I2C_APP_RETVAL_NEW_TX             /**< Application wants to start new Master TX immediately */
    , I2C_APP_RETVAL_CHAINED_RX         /**< Application is continuing a chained RX transaction */
    , I2C_APP_RETVAL_CHAINED_RX_LAST    /**< Last request for a chained RX transaction */
    , I2C_APP_RETVAL_CHAINED_TX         /**< Application is continuing a chained TX transaction */
    , I2C_APP_RETVAL_CHAINED_TX_LAST    /**< Last request for a chained TX transaction */
};

/******************************************************************************/
/** Application return values
 * Action codes to the SPT driver about the action to take for the current
 * packet
 * @note
 * -----------------------
 * Usage notes:
 * -----------------------
 * The MCTP module will use the values in this enumeration to communicate to the
 * SPT driver about the action to take with the current packet, when the spt driver
 * communicates about the transmission status to the MCTP module (refer mctp_spt_tx)
 * The SPT driver is expected to take relevent action
 *******************************************************************************/
enum spt_application_return_values
{
    SPT_APP_RETVAL_TX_DONE =0     /**< Application gets ready for next tx packet */
    , SPT_APP_RETVAL_HOLD_SPT         /**< Application still acquires SPT */
    , SPT_APP_RETVAL_RETRY              /**< Application wants to retry the current transaction */
    , SPT_APP_RETVAL_NEW_TX             /**< Application wants to start new Master TX immediately */
};

/******************************************************************************/
/** i2c_bus_speed 
 * I2C bus speed values
 * @note
 * -----------------------
 * Usage notes:
 * -----------------------
 * The values in this enum are used by MCTP module to identify I2C bus speed
 * values
 *******************************************************************************/
enum i2c_bus_speed
{
    I2C_BUS_SPEED_100KHZ
   ,I2C_BUS_SPEED_400KHZ
   ,I2C_BUS_SPEED_1MHZ
};

/******************************************************************************/
/** i2c_slave_packet_status 
 * Values used by MCTP module to return the status of packet acceptance
 * @note
 * -----------------------
 * Usage notes:
 * -----------------------
 * The values in this enum are used by MCTP module to notify the I2C driver 
 * if it accepted the provided packet or not
 *******************************************************************************/
enum i2c_slave_packet_status
{
    I2C_STATUS_BUFFER_NOT_DONE= 0   /**< Packet not accepted, since application is busy */
    ,I2C_STATUS_BUFFER_DONE         /**< Packet accepted by application */
    ,I2C_STATUS_BUFFER_ERROR        /**< Packet not meant for this application */
};

/******************************************************************************/
/** spt_slave_packet_status 
 * Values used by MCTP module to return the status of packet acceptance
 * @note
 * -----------------------
 * Usage notes:
 * -----------------------
 * The values in this enum are used by MCTP module to notify the SPT driver 
 * if it accepted the provided packet or not
 *******************************************************************************/
enum spt_slave_packet_status
{
    SPT_STATUS_BUFFER_NOT_DONE= 0 ,  /**< Packet not accepted, since application is busy */
    SPT_STATUS_BUFFER_DONE,         /**< Packet accepted by application */
    SPT_STATUS_BUFFER_ERROR,        /**< Packet not meant for this application */     
};

/******************************************************************************/
/** i2c_slave_transmit_status 
 * Values used by MCTP module to check for i2c slave transmit protocol
 * @note
 * -----------------------
 * Usage notes:
 * -----------------------
 * The user's i2c driver is expected to use the values in this enumeration
 * to notify the MCTP module if it is expecting a packet as part of slave transmit
 * phase
 *******************************************************************************/
enum i2c_slave_transmit_status
{
    I2C_SLAVE_TRANSMIT_FALSE = false    /**< Slave Transmit phase not required */
    ,I2C_SLAVE_TRANSMIT_TRUE= true      /**< Slave Transmit phase required */
};

/******************************************************************************/
/** struct MCTP_BUFFER_INFO
 * This structure is used to store information of a slave receive buffer XmitCount
 * is used along with slaveXmitDoneFlag for chaining slave transmit data
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * The I2C/SPT driver is expected allocate a structure of this type and use it
 * to pass information to MCTP module about the received packet
 * -----------------------
 * Example:
 * -----------------------
 * Refer MCTP_SLAVE_FUNC_PTR
*******************************************************************************/
typedef struct MCTP_BUFFER_INFO
{
    uint8_t *   buffer_ptr;          /**< Pointer to buffer memory */
    uint16_t    TimeStamp;           /**< Packet received timestamp */
    uint8_t     DataLen;             /**< Data Length of packet received */
    uint8_t     XmitCount;           /**< Number of times slave has transmitted using this buffer for this transaction */
    uint8_t     RxCount;             /**< Number of times slave has received using this buffer for this transaction */
    uint8_t     pecFlag;             /**< PEC valid/invalid flag */
    uint8_t     channel;             /**< Channel on which this packet is received */
    uint8_t     slaveXmitDoneFlag;   /**< Flag indicating if xmit is completed by slave application */
    bool        sdoneFlag;           /**< Flag to indicate if SDONE occured for this buffer */
} MCTP_BUFFER_INFO;


/******************************************************************************/
/** MCTP_SLAVE_FUNC_PTR - Slave application function pointer
 * The first argument is pointer to MCTP_BUFFER_INFO structure which will contain
 * details of the packet received. The second parameter is flag to indicate
 * slave transmit phase. In case of slave transmit phase, the application
 * should provide the data to be transmitted in the same buffer and indicate
 * whether PEC should be enabled for transmit phase
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * The I2C driver is expected allocate a structure of this type and use it
 * to pass information to MCTP module about the received packet
 * -----------------------
 * Example:
 * -----------------------
 * struct i2c_packet_t
 * {
 *     uint8_t i2c_rx_buff[MAX_I2C_PKT_LEN];
 *     uint8_t len;
 * };
 * struct i2c_packet i2c_pkt[MAX_I2C_CHANNELS];
 * MCTP_BUFFER_INFO i2c_rx_packet;
 * 
 * uint8_t i2c_store_rxd_pkt(channel, uint8_t *data, uint8_t len)
 * {
 *     memcpy(&i2c_pkt[channel].i2c_rx_buff[0], data, len);
 *     i2c_pkt[channel].len = len;
 * }
 * 
 * uint8_t i2c_process_mctp_pkt(uint8_t channel)
 * {
 *      i2c_rx_packet.buffer_ptr = i2c_pkt[channel];
 *      i2c_rx_packet.TimeStamp  = get_current_timestamp();
 *      i2c_rx_packet.DataLen    = i2c_pkt[channel].len;
 *      i2c_rx_packet.pecFlag    = 1;
 *      i2c_rx_packet.channel    = channel;
 * 
 *      mctp_rx_cb[channel](&i2c_rx_packet, I2C_SLAVE_TRANSMIT_FALSE);
 * }
 * ############################################################################
 * -----------------------
 * Note:
 * -----------------------
 * 1. Please note that the minimum packet length is 12 bytes
 *    which includes destination slave address and PEC byte 
 * 2. MCTP does not support slave transmit protocol - Refer MCTP over I2C/SMBUS
 *    transport binding specification, section 6.3 for further details
 * ############################################################################
 * ############################################################################
*******************************************************************************/
typedef uint8_t (*MCTP_SLAVE_FUNC_PTR)(MCTP_BUFFER_INFO *buffer_info, 
                                      uint8_t slaveTransmitFlag);

/******************************************************************************/

/** I2C_MASTER_FUNC_PTR - Master transmit status function pointer
 * This function pointer should be saved by the driver and called later
 * to inform the MCTP module about the status of the packet transmission.
 * The first parameters should contain the channel information
 * The second parameter should contian the packet transmission status,
 * refer enum MasterStatus for a list of valid values
 * The third parameter should contain a pointer to the transmit buffer being 
 * containing the packet currently being transmitted
 * The fourth parameter is reserved for furture use and is currently ignored
 * by the MCTP module
 * A function of this type will return a status code to the caller which 
 * will be of type enum ApplicationReturnValues
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * When MCTP module initiates a packet transmission using mctp_i2c_tx interface
 * function, it also provides a pointer to a function of this type to the 
 * I2C driver, which is expected to be saved by the I2C driver. The I2C driver
 * is then expected to call this function after every packet sent on the bus
 * to inform the MCTP module about the status of the transmission. Based on 
 * status code, mctp layer will schedule re-transmission of packet or drop the 
 * packet / mark buffer available by returning one of the values of type 
 * enum ApplicationReturnValues
 * -----------------------
 * Example:
 * -----------------------
 * Refer mctp_i2c_tx interface function
 * ############################################################################
*******************************************************************************/
typedef uint8_t (*I2C_MASTER_FUNC_PTR)(uint8_t channel, 
                                       uint8_t status, 
                                       uint8_t *buffer_ptr, 
                                       I2C_MAPP_CBK_NEW_TX *newTxParams);

/******************************************************************************/
/** mctp_i2c_tx
 * Initiates I2C master operation
 * @param channel          i2c channel
 * @param buffer_ptr       Buffer for the smbus transaction
 * @param smb_protocol     smbus protocol byte
 * @param pecEnable        Flag to enable/disable PEC
 * @param WriteCount       Number of bytes to transmit
 * @param di_master_req    Data Isolation structure
 * @param readChainedFlag  flag to indicate if read needs to be done
 *                         using dma chaining
 * @param writeChainedFlag flag to indicate if write needs to be done
 *                         using dma chaining
 * @return                 MASTER_OK on success, MASTER_ERROR if i2c is 
 *                         not ready for master mode operation
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the MCTP module whenever it wants to
 * initiate a master transaction on the bus. If this function returns
 * MASTER_ERROR, the MCTP module will retry after some time
 * For SMBUS ReadBlock protocol the application should provide a 80 byte
 * buffer. This function should be defined by the user and a call to the
 * driver i2c master mode write function should be made inside the body.
 * If the user i2c driver is busy carrying out some other operation
 * the user should return MASTER_ERROR otherwise the user should return 
 * MASTER_OK while exiting this function.
 * -----------------------
 * Example:
 * -----------------------
 * I2C_MASTER_FUNC_PTR i2c_tx_status i2c_tx_status_cb;
 * uint8_t i2c_tx_status[MAX_I2C_CHANNELS];
 * uint8_t i2c_tx_done[MAX_I2C_CHANNELS];
 * uint8_t *tx_buffer = NULL;
 * uint8_t mctp_i2c_tx(const uint8_t channel, uint8_t *buffer_ptr, const uint8_t smb_protocol, 
 *                     const uint8_t writeCount, I2C_MASTER_FUNC_PTR func_ptr, const uint8_t pecEnable, 
 *                     const uint8_t readChainedFlag, const uint8_t writeChainedFlag);
 * {
 *     uint8_t status = MASTER_OK;
 *     i2c_tx_status[channel] = SUCCESS_TX;
 *     i2c_tx_status_cb = func_ptr;
 *     tx_buffer = buffer_ptr;
 *     status = i2c_write_data(channel, buffer_ptr, writeCount, pecEnable);
 *     if(status < 0)
 *     {
 *          status = MASTER_ERROR;
 *     }
 *     return status;
 * }
 * 
 * void i2c_tx_isr(void)
 * {
 *     uint8_t channel;
 *     for(channel=0; channel<MAX_I2C_CHANNELS; channel++)
 *     {
 *         i2c_tx_done[channel] = i2c_get_tx_done(channel);
 *         i2c_tx_status[channel] = i2c_get_tx_status(channel);
 *     }
  * }
 * 
 * void i2c_send_tx_status(void)
 * {
 *     uint8_t channel = 0;
 *     for(channel=0; channel<MAX_I2C_CHANNELS; channel++)
 *     {
 *         if(i2c_tx_done[channel] == true)
 *         {
 *             switch(i2c_tx_status[channel])
 *             {
 *                 case TX_ERROR_BUS_ERR:
 *                 status = ERROR_BER_NON_TIMEOUT;
 *                 break;
 *                 case TX_ERROR_BUS_ERR_TOUT:
 *                 status = ERROR_BER_TIMEOUT;
 *                 break;
 *                 case TX_ERROR_LOST_ARB:
 *                 status = ERROR_LAB;
 *                 break;
 *                 case TX_ERROR_ADDR_NACK:
 *                 status = ERROR_MADDR_NAKX;
 *                 break;
 *                 case TX_ERROR_DATA_NACK:
 *                 status = ERROR_MDATA_NAKX;
 *                 break;
 *                 case TX_ERROR_PEC:
 *                 status = ERROR_PEC;
 *                 break;
 *                 case TX_ERROR_CHAN_DISABLED:
 *                 status = ERROR_SMB_DISABLED;
 *                 break;
 *                 default:
 *             }
 *             i2c_tx_status_cb(channel, status, tx_buffer, NULL);
 *         }
 *     }
 * }
 * ############################################################################
*******************************************************************************/
extern uint8_t mctp_i2c_tx(const uint8_t channel, 
                    uint8_t *buffer_ptr, 
                    const uint8_t smb_protocol, 
                    const uint8_t writeCount, 
                    const uint8_t pecEnable, 
                    I2C_MASTER_FUNC_PTR func_ptr, 
                    const uint8_t readChainedFlag,
                    const uint8_t writeChainedFlag);

/******************************************************************************/
/** mctp_i2c_rx_register
 * This function registers a I2C slave application
 * @channel the channel on which the slave is registered
 * @param slaveFuncPtr The application function to call on receiving a packet
 * @return             I2C_SLAVE_APP_STATUS_OK on successful registration, 
 *                     else error status
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * Whenever an application expects data from smbus (i.e acts as slave) it
 * needs to register using this function.
 * The application function that is registered should only copy the
 * packet from smbus buffer, it should not the process the data in that
 * function
 * -----------------------
 * Example:
 * -----------------------
 * I2C_SLAVE_FUNC_PTR mctp_rx_cb[MAX_I2C_CHANNELS];
 * uint8_t mctp_i2c_rx_register(const uint8_t channel, I2C_SLAVE_FUNC_PTR slaveFuncPtr)
 * {
 *     uint8_t status = I2C_SLAVE_APP_STATUS_APP_NOT_REGISTERED;
 *      
 *     do
 *     {
 *         if(channel > MAX_I2C_CHANNELS)
 *         {
 *             status = I2C_SLAVE_APP_STATUS_MAX_APP_REGISTERED;
 *             break;
 *         }
 *         
 *         if(mctp_rx_cb[channel] != NULL)
 *         {
 *             status = I2C_SLAVE_APP_STATUS_ALREADY_REGISTERED;
 *             break;
 *         }
 *         
 *         mctp_rx_cb[channel] = slaveFuncPtr;
 *         status = I2C_SLAVE_APP_STATUS_OK;
 *     }
 *     while(0);
 * }
 * ############################################################################
*******************************************************************************/
extern uint8_t mctp_i2c_rx_register(const uint8_t channel, 
                            MCTP_SLAVE_FUNC_PTR slaveFuncPtr);

/** SPT_TX_FUNC_PTR - Master transmit status function pointer
 * This function pointer should be saved by the driver and called later
 * to inform the MCTP module about the status of the packet transmission.
 * The first parameters should contain the channel information
 * The second parameter should contian the packet transmission status,
 * refer enum MasterStatus for a list of valid values
 * The third parameter should contain a pointer to the transmit buffer being 
 * containing the packet currently being transmitted
 * The fourth parameter is reserved for furture use and is currently ignored
 * by the MCTP module
 * A function of this type will return a status code to the caller which 
 * will be of type enum ApplicationReturnValues
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * When MCTP module initiates a packet transmission using mctp_spt_tx interface
 * function, it also provides a pointer to a function of this type to the 
 * I2C driver, which is expected to be saved by the I2C driver. The I2C driver
 * is then expected to call this function after every packet sent on the bus
 * to inform the MCTP module about the status of the transmission. Based on 
 * status code, mctp layer will schedule re-transmission of packet or drop the 
 * packet / mark buffer available by returning one of the values of type 
 * enum ApplicationReturnValues
 * -----------------------
 * Example:
 * -----------------------
 * Refer mctp_spt_tx interface function
 * ############################################################################
*******************************************************************************/
typedef uint8_t (*SPT_TX_FUNC_PTR)(uint8_t channel, 
                                       uint8_t status, 
                                       uint8_t *buffer_ptr, 
                                       SPT_MAPP_CBK_NEW_TX *newTxParams);

/******************************************************************************/
/** mctp_spt_tx
 * Initiates SPI Target operation
 * @param channel          SPI Target channel
 * @param buffer_ptr       Buffer for the SPT(SPI Target) transaction
 * @param pecEnable        Flag to enable/disable PEC
 * @param WriteCount       Number of bytes to transmit
 * @param di_master_req    Data Isolation structure
 * @param readChainedFlag  flag to indicate if read needs to be done
 *                         using dma chaining
 * @param writeChainedFlag flag to indicate if write needs to be done
 *                         using dma chaining
 * @return                 SPT_TX_OK on success, SPT_TX_BUSY if previous
 *                         TX is in progress
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the MCTP module whenever it wants to
 * initiate a transaction on the bus. If this function returns
 * SPT_TX_BUSY, the MCTP module will retry after some time
 * This function should be defined by the user and a call to the
 * driver spt  write function should be made inside the body.
 * If the user SPT driver is busy carrying out some other operation
 * the user should return SPT_BUSY otherwise the user should return 
 * SPT_TX_OK while exiting this function.
 * -----------------------
 * Example:
 * -----------------------
 * SPT_TX_FUNC_PTR spt_tx_status i2c_tx_status_cb;
 * uint8_t spt_tx_status[MAX_SPT_CHANNELS];
 * uint8_t spt_tx_done[MAX_SPT_CHANNELS];
 * uint8_t *tx_buffer = NULL;
 * uint8_t mctp_spt_tx(const uint8_t channel, uint8_t *buffer_ptr,  
 *                     const uint8_t writeCount, SPT_TX_FUNC_PTR func_ptr, const uint8_t pecEnable, 
 *                     const uint8_t readChainedFlag, const uint8_t writeChainedFlag);
 * {
 *     uint8_t status = MASTER_OK;
 *     spt_tx_status[channel] = SUCCESS_TX;
 *     spt_tx_status_cb = func_ptr;
 *     tx_buffer = buffer_ptr;
 *     status = spt_write(channel, buffer_ptr, writeCount, pecEnable);
 *     if(status < 0)
 *     {
 *          status = SPT_TX_BUSY ;
 *     }
 *     return status;
 * }
*******************************************************************************/
extern uint8_t mctp_spt_tx(const uint8_t channel, 
                    uint8_t *buffer_ptr, 
                    const uint8_t writeCount, 
                    const uint8_t pecEnable, 
                    SPT_TX_FUNC_PTR func_ptr, 
                    const uint8_t readChainedFlag,
                    const uint8_t writeChainedFlag);
/******************************************************************************/
/** mctp_spt_rx_register
 * This function registers a SPT(SPI Target) application
 * @channel the channel on which the slave is registered
 * @param slaveFuncPtr The application function to call on receiving a packet
 * @return             SPT_SLAVE_APP_STATUS_OK on successful registration, 
 *                     else error status
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * Whenever an application expects data from SPT, it
 * needs to register using this function.
 * The application function that is registered should only copy the
 * packet from SPT buffer, it should not the process the data in that
 * function
 * -----------------------
 * Example:
 * -----------------------
 * MCTP_SLAVE_FUNC_PTR mctp_rx_cb[MAX_SPT_CHANNELS];
 * uint8_t mctp_spt_rx_register(const uint8_t channel, MCTP_SLAVE_FUNC_PTR slaveFuncPtr)
 * {
 *     uint8_t status = SPT_SLAVE_APP_STATUS_APP_NOT_REGISTERED;
 *      
 *     do
 *     {
 *         if(channel > MAX_SPT_CHANNELS)
 *         {
 *             status = SPT_SLAVE_APP_STATUS_APP_NOT_REGISTERED;
 *             break;
 *         }
 *         
 *         if(mctp_rx_cb[channel] != NULL)
 *         {
 *             status = SPT_SLAVE_APP_STATUS_ALREADY_REGISTERED;
 *             break;
 *         }
 *         mctp_rx_cb[channel] = slaveFuncPtr;
 *         status = SPT_SLAVE_APP_STATUS_OK;
 *     }
 *     while(0);
 * }
 * ############################################################################
*******************************************************************************/
extern uint8_t mctp_spt_rx_register(const uint8_t channel, 
                            MCTP_SLAVE_FUNC_PTR slaveFuncPtr);

/******************************************************************************/
/** mctp_i2c_configure_and_enable
 * This function can be used to start and enable the i2c controller
 * @param channel     channel (I2C controller number)
 * @param own_address 7-bit smb address
 * @param speed       speed
 * @param port        I2C port (I2C port number inside the I2C controller)
 * @param configFlag  |      BIT0      |      BIT1      |      BIT2      |    BIT3 - BIT7 |
 *                    | i2c module     |     Unused     | MCTP fairness  |      Unused    |
 *                    | enable/disable |                |protocol enable |                |
 * @return            None
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the MCTP task with all parameters being populated
 * from the user selection in MCC project configuration. The user is expected
 * to define this function in his application and call their i2c driver 
 * initialization routine making use of parameters as needed by the driver
 * routine. 
 * The configFlag paramter is optional and if the user's driver does
 * not support MCTP fairness protocol it can disacard this parameter.
 * The channel parameter is optional and if the user driver supports only 
 * one controller, it can discard this parameter
 * The port parameter is used in case the user's driver supports 
 * more than one port per I2C channel/controller. Otherwise, the user can 
 * discard this parameter
 * -----------------------
 * Example:
 * -----------------------
 * void mctp_i2c_configure_and_enable(uint8_t channel, 
 *                                    uint16_t own_address, 
 *                                    uint8_t speed, 
 *                                    uint8_t port, 
 *                                    uint8_t configFlag)
 * {
 *      uint8_t fairness_proto_en = (config_Flag & 0x04);
 *      i2c_configure(channel, own_address, speed, fairness_proto_en);
 *      i2c_start(channel);
 * }
 * ############################################################################
*******************************************************************************/
extern void mctp_i2c_configure_and_enable(uint8_t channel, 
                                   uint16_t own_address, 
                                   uint8_t speed, 
                                   uint8_t port, 
                                   uint8_t configFlag);

/******************************************************************************/
/** mctp_i2c_get_chan_busy_status
 * This function checks if master resource is available on a given
 * channel
 * @param channel i2c channel
 * @return I2C_MASTER_AVAILABLE/I2C_MASTER_BUSY
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the MCTP module to check if an I2C channel is
 * busy performing some other operation. The MCTP module will wait for the 
 * I2C channel to be free before calling mctp_i2c_tx interface function. The
 * user is expected to define this function in his application and call their
 * i2c driver function inside the body to check for channel busy status. Refer
 * enum master_busy_status
 * -----------------------
 * Example:
 * -----------------------
 * uint8_t mctp_i2c_get_chan_busy_status(uint8_t channel)
 * {
 *      uint8_t status = I2C_MASTER_BUSY;
 *      if(i2c_is_channel_busy(channel) == False)
 *      {
 *          status = I2C_MASTER_AVAILABLE
 *      }
 *      return status;
 * }
 * ############################################################################
*******************************************************************************/
extern uint8_t mctp_i2c_get_chan_busy_status(uint8_t channel);

/******************************************************************************/
/** mctp_i2c_get_current_timestamp
 * Function to retrieve the current timestamp
 * @param None
 * @return 16-bit timestamp value
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the MCTP module to check if for packet timeouts.
 * The user expected to define this function and call xTaskGetTickCount()
 * API of FreeRTOS depending on the rate at which their i2c driver task is
 * running
 * -----------------------
 * Example:
 * -----------------------
 * Assume user i2c driver task running every 10 RTOS ticks
 * uint16_t mctp_i2c_get_current_timestamp(void)
 * {
 *     return (uint16_t) (xTaskGetTickCount() / 10);
 * }
 * ############################################################################
 ******************************************************************************/
extern uint16_t mctp_i2c_get_current_timestamp(void);

/******************************************************************************/
/** mctp_spt_configure_and_enable
 * This function can be used to start and enable the SPI Target(SPT)
 * @param channel     channel (SPT Channel number)
 * @param io_mode     SINGLE/QUAD
 * @param wait_time   SPT Wait cycles
 * @param tar_time    SPT Turn around time
 * @return            None
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the MCTP task with all parameters being populated
 * from the user selection in MCC project configuration. The user is expected
 * to define this function in his application and call their SPT driver 
 * initialization routine making use of parameters as needed by the driver
 * routine. 
 * -----------------------
 * Example:
 * -----------------------
 * void mctp_spt_configure_and_enable(uint8_t channel, 
 *                                    uint8_t io_mode, 
 *                                    uint8_t wait_time, 
 *                                    uint8_t tar_time)
 * {
 *      spt_config_and_enable(channel, io_mode, wait_time, tar_time);
 * }
 * ############################################################################
*******************************************************************************/
extern void mctp_spt_configure_and_enable(uint8_t channel, 
                                   uint8_t io_mode, 
                                   uint8_t wait_time, 
                                   uint8_t tar_time, 
                                   uint8_t pec_enable);

/******************************************************************************/
/** mctp_spt_get_chan_busy_status
 * This function checks if SPT resource is available on a given
 * channel
 * @param channel SPT channel
 * @return SPT_TX_
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the MCTP module to check if an I2C channel is
 * busy performing some other operation. The MCTP module will wait for the 
 * I2C channel to be free before calling mctp_i2c_tx interface function. The
 * user is expected to define this function in his application and call their
 * i2c driver function inside the body to check for channel busy status. Refer
 * enum master_busy_status
 * -----------------------
 * Example:
 * -----------------------
 * uint8_t mctp_spt_get_chan_busy_status(uint8_t channel)
 * {
 *      uint8_t status = SPT_TX_BUSY;
 *      if(spt_check_channel_busy(channel) == False)
 *      {
 *          status = I2C_MASTER_AVAILABLE
 *      }
 *      return status;
 * }
 * ############################################################################
*******************************************************************************/
extern uint8_t mctp_spt_get_chan_busy_status(uint8_t channel);

/******************************************************************************/
/** mctp_spt_get_current_timestamp
 * Function to retrieve the current timestamp
 * @param None
 * @return 16-bit timestamp value
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the MCTP module to check if for packet timeouts.
 * The user expected to define this function and call xTaskGetTickCount()
 * API of FreeRTOS depending on the rate at which their i2c driver task is
 * running
 * -----------------------
 * Example:
 * -----------------------
 * Assume user SPT driver task running every 10 RTOS ticks
 * uint16_t mctp_spt_get_current_timestamp(void)
 * {
 *     return (uint16_t) (xTaskGetTickCount() / 10);
 * }
 * ############################################################################
 ******************************************************************************/
extern uint16_t mctp_spt_get_current_timestamp(void);

/******************************************************************************/
/** mctp_wait_for_done_spdm(void)
 * MCTP wait for SPDM application to be done
 * @param None
 * @return None
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the MCTP module after filling mctp_pktbuf[MCTP_BUF3]
 * with the SPDM request packet. The SPDM module is expected to inform the MCTP
 * module after it has done processing the SPDM request packet.
 * ############################################################################
*******************************************************************************/
extern void mctp_wait_for_done_spdm(void);
/******************************************************************************/
/** mctp_app_done_inform_i2c(void)
 * Inform I2C driver that MCTP has received SPDM response packet
 * @param None
 * @return None
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function is called by the MCTP module after it has received the SPDM
 * (or) PLDM reponse packet and is ready to be sent on the I2C bus. This 
 * interface function is used in conjunction with mctp_wait_for_done_spdm()
 * interface function. These interface functions are available to the user
 * for cases where the user's I2C driver double / triple buffer capability,
 * and wants to NACK the host if applications are busy processing the 
 * previous request packets.
 * ############################################################################
*******************************************************************************/
extern void mctp_app_done_inform_i2c(void);
/******************************************************************************/
/** mctp_app_task_create(void)
 * Create MCTP FreeRTOS task
 * @param pvParams  This parameter is not used
 * @param pTaskPrivRegValues Privilege values for task
 * @return -1 :Fail, 0: Pass
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function creates and prepares the MCTP task to be run by the FreeRTOS
 * task scheduler. The user is expected to call this function in their main
 * function along with any other application task creation routines and start
 * the FreeRTOS scheduler. Make sure that all necessary peripheral initializations
 * have been completed before calling this function
 * -----------------------
 * Example:
 * -----------------------
 * int main ( void )
 * {
 *    
 *    SYS_Initialize ( NULL );
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
int mctp_app_task_create(void *pvParams, CEC_AHB_PRIV_REGS_VALUES *pTaskPrivRegValues);

/******************************************************************************/
/** SET_MCTP_EVENT_FLAG()
 * Set event flag to trigger MCTP packet processing
 * @param  none
 * @return none
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function should be called by SPDM module when an SPDM response packet
 * has been populated in mctp_pktbuf[MCTP_BUF2] and ready to be processed
 * by MCTP module
 * -----------------------
 * Example:
 * -----------------------
 * MCTP_PKT_BUF *spdm_buf_tx = MCTP_NULL;
 * MCTP_PKT_BUF *spdm_buf_rx = MCTP_NULL;
 * void spdm_packet_process(void)
 * {
 *      spdm_buf_rx = mctp_pktbuf[MCTP_BUF3];
 *      spdm_buf_tx = mctp_pktbuf[MCTP_BUF2];
 *      spdm_populate_reponse(spdm_buf_rx, spdm_buf_tx);
 *      SET_MCTP_EVENT_FLAG();
 * }
 * ############################################################################
*******************************************************************************/
void SET_MCTP_EVENT_FLAG(void);

/******************************************************************************/
/** mctp_update_eid()
 * Update the device endpoint id
 * @param  eid endpoint id value
 * @return none
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * This function should be called by the user to update the endpoint id
 * of the MCTP device. This API is useful in cases where the device endpoint
 * id is store and updated externally, in which case, the user could call this
 * API to keep updating the device's EID as it keeps changing
 * -----------------------
 * Example:
 * -----------------------
 * Assume that the device endpoint id is stored in an extrnal flash
 * at location 0x20000, which is updated as part of a firmware update
 * 
 * #define DEV_EID_QMSPI_ADDR       0x20000U
 * #define DEV_EID_QMSPINUM_BYTES   1U
 * 
 * int8_t flash_device_read_and update_eid(void)
 * {
 *      uint8_t dev_eid = 0;
 *      int8_t status = 0;
 * 
 *      if(qmspi_read_request(&dev_eid, DEV_EID_QMSPINUM_BYTES, DEV_EID_QMSPI_ADDR); < 0)
 *      {
 *          status = -1;
 *      }
 *      else
 *      {
 *          mctp_update_eid(dev_eid);
 *      }
 * 
 *      return status;
 * }
 * ############################################################################
*******************************************************************************/
void mctp_update_eid(uint8_t eid);

/******************************************************************************/
/** SET_SPDM_EVENT_FLAG()
 * Set event flag to trigger SPDM packet processing
 * @param  none
 * @return none
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * When the MCTP module identifies that the received packet is an SPDM packet,
 * it stores the packet into mctp_pktbuf[MCTP_BUF3], and calls this function.
 * This function should be defined by the user's SPDM module and should ideally
 * include an IPC mechanism to trigger the user's SPDM module.
 * -----------------------
 * Example:
 * -----------------------
 * Assume that user SPDM task is using eventgroups for notifications from 
 * other tasks and uses BIT0 to wait and trigger SPDM packet processing event
 * 
 * EventGroupHandle_t spdmEventGroupHandle;
 * StaticEventGroup_t spdmCreatedEventGroup;
 * 
 * int spdm_task_create(void *pvParams)
 * {
 *      spdmEventGroupHandle = xEventGroupCreateStatic(spdmCreatedEventGroup);
 *      // User SPDM task creation
 *      // assume task routine = spdm_task
 * }
 * 
 * static void spdm_task(void* pvParameters)
 * {
 *     EventBits_t uxBits;
 *     uxBits = xEventGroupWaitBits(spdmEventGroupHandle, SPDM_EVENT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
 *     if (SPDM_EVENT_BIT == (uxBits & SPDM_EVENT_BIT))
 *     {
 *          // User SPDM packet processing routine(s)
 *     }
 * }
 * 
 * void SET_SPDM_EVENT_FLAG(void)
 * {
 *      xEventGroupSetBits(spdmEventGroupHandle, SPDM_EVENT_BIT);
 * }
 * ############################################################################
*******************************************************************************/
extern void SET_SPDM_EVENT_FLAG(void);
/******************************************************************************/
/** SET_PLDM_EVENT_FLAG()
 * Set event flag to trigger PLDM packet processing
 * @param  none
 * @return none
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * When the MCTP module identifies the received packet as being an PLDM packet,
 * it stores the packet into mctp_pktbuf[MCTP_BUF3], and calls this function.
 * This function should be defined by the user's PLDM module and should ideally
 * include an IPC mechanism to trigger the user's PLDM module.
 * -----------------------
 * Example:
 * -----------------------
 * Assume that user PLDM task is using eventgroups for notifications from 
 * other tasks and uses BIT0 to wait and trigger PLDM packet processing event
 * 
 * EventGroupHandle_t pldmEventGroupHandle;
 * StaticEventGroup_t pldmCreatedEventGroup;
 * 
 * int pldm_task_create(void *pvParams)
 * {
 *      pldmEventGroupHandle = xEventGroupCreateStatic(pldmCreatedEventGroup);
 *      // User PLDM task creation
 *      // assume task routine = pldm_task
 * }
 * 
 * static void pldm_task(void* pvParameters)
 * {
 *     EventBits_t uxBits;
 *     uxBits = xEventGroupWaitBits(pldmEventGroupHandle, PLDM_EVENT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
 *     if (PLDM_EVENT_BIT == (uxBits & PLDM_EVENT_BIT))
 *     {
 *          // User PLDM packet processing routine(s)
 *     }
 * }
 * 
 * void SET_PLDM_EVENT_FLAG(void)
 * {
 *      xEventGroupSetBits(pldmEventGroupHandle, PLDM_EVENT_BIT);
 * }
 * ############################################################################
*******************************************************************************/
extern void SET_PLDM_EVENT_FLAG(void);

/******************************************************************************/
/** SET_PLDM_RESP_EVENT_FLAG()
 * Set event flag to trigger PLDM packet processing dirung
 * firmware update request which is a multiple packet request
 * @param  none
 * @return none
 * @note
 * ############################################################################
 * -----------------------
 * Usage notes:
 * -----------------------
 * After the PLDM modure sends the send the firmware data request  ,
 * MCTP module sends the request stored in [MCTP_BUF5] and call this
 * function to triggers the PLDM module to create the next Packet request
 * for firmware data. This function should be defined by the user's
 * PLDM module and should ideally include an IPC mechanism to
 * trigger the user's PLDM module.
 * -----------------------
 * Example:
 * -----------------------
 * Assume that user PLDM task is using eventgroups for notifications from 
 * other tasks and uses BIT0 to wait and trigger PLDM packet processing event
 * 
 * EventGroupHandle_t pldmEventGroupHandle;
 * StaticEventGroup_t pldmCreatedEventGroup;
 * 
 * int pldm_task_create(void *pvParams)
 * {
 *      pldmEventGroupHandle = xEventGroupCreateStatic(pldmCreatedEventGroup);
 *      // User PLDM task creation
 *      // assume task routine = pldm_task
 * }
 * 
 * static void pldm_task(void* pvParameters)
 * {
 *     EventBits_t uxBits;
 *     uxBits = xEventGroupWaitBits(pldmEventGroupHandle, PLDM_RESP_EVENT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
 *     if (PLDM_RESP_EVENT_BIT == (uxBits & PLDM_RESP_EVENT_BIT))
 *     {
 *          // User PLDM packet processing routine(s)
 *     }
 * }
 *
 * void SET_PLDM_RESP_EVENT_FLAG(void)
 * {
 *      xEventGroupSetBits(pldmEventGroupHandle, PLDM_RESP_EVENT_BIT);
 * }
 * ############################################################################
*******************************************************************************/
extern void SET_PLDM_RESP_EVENT_FLAG(void);

/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* MCTP_H */

/* *****************************************************************************
 End of File
 */