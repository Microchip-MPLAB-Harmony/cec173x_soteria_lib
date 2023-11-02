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

#ifndef INCLUDE_ROM_API_MPU_H_
#define INCLUDE_ROM_API_MPU_H_

typedef uint32_t (*FP_RU32_V)(void);

#define MCHP_EXTRA_IN_DESCS 0
#define NUM_PK_INST 1 /* MCHP */
#define MCHP_OK 0

/** A cryptomaster DMA descriptor */
struct mchpdesc
{
    char *addr;
    struct mchpdesc *next;
    uint32_t sz;
    uint32_t dmatag;
};

/** Input and output descriptors and related state for cmdma */
struct mchp_dmaslot
{
    uint32_t cfg;
    uint8_t extramem[26];
    struct mchpdesc indescs[6 + MCHP_EXTRA_IN_DESCS];
    struct mchpdesc outdescs[5];
};

typedef enum bk_ecc_curve
{
    /**
     * \brief Curve NIST-P192
     *
     * \details The elliptic-curve cryptosystem specified by the NIST-P192/secp192r1 domain parameters:
     *
     * - BK curve ID: 0x00
     * - size in bytes of private keys: 24
     * - size in bytes of public keys: 49
     * - size in bytes of ECDSA signatures: 48
     * - size in bytes of ECDH shared secrets: 24
     * - size in bytes of ECIES header: 49
     * - maximal security strength for operations over this curve: 96-bit
     */
    BK_ECC_CURVE_NIST_P192 = 0x00,
    /**
     * \brief Curve NIST-P224
     *
     * \details The elliptic-curve cryptosystem specified by the NIST-P224/secp224r1 domain parameters:
     *
     * - BK curve ID: 0x01
     * - size in bytes of private keys: 28
     * - size in bytes of public keys: 57
     * - size in bytes of ECDSA signatures: 56
     * - size in bytes of ECDH shared secrets: 28
     * - size in bytes of ECIES header: 57
     * - maximal security strength for operations over this curve: 112-bit
     */
    BK_ECC_CURVE_NIST_P224 = 0x01,
    /**
     * \brief Curve NIST-P256
     *
     * \details The elliptic-curve cryptosystem specified by the NIST-P256/secp256r1 domain parameters:
     *
     * - BK curve ID: 0x02
     * - size in bytes of private keys: 32
     * - size in bytes of public keys: 65
     * - size in bytes of ECDSA signatures: 64
     * - size in bytes of ECDH shared secrets: 32
     * - size in bytes of ECIES header: 65
     * - maximal security strength for operations over this curve: 128-bit
     */
    BK_ECC_CURVE_NIST_P256 = 0x02,
    /**
     * \brief Curve NIST-P384
     *
     * \details The elliptic-curve cryptosystem specified by the NIST-P384/secp384r1 domain parameters:
     *
     * - BK curve ID: 0x03
     * - size in bytes of private keys: 48
     * - size in bytes of public keys: 97
     * - size in bytes of ECDSA signatures: 96
     * - size in bytes of ECDH shared secrets: 48
     * - size in bytes of ECIES header: 97
     * - maximal security strength for operations over this curve: 192-bit
     */
    BK_ECC_CURVE_NIST_P384 = 0x03,
    /**
     * \brief Curve NIST-P521
     *
     * \details The elliptic-curve cryptosystem specified by the NIST-P521/secp521r1 domain parameters:
     *
     * - BK curve ID: 0x04
     * - size in bytes of private keys: 66
     * - size in bytes of public keys: 133
     * - size in bytes of ECDSA signatures: 132
     * - size in bytes of ECDH shared secrets: 66
     * - size in bytes of ECIES header: 133
     * - maximal security strength for operations over this curve: 260-bit
     */
    BK_ECC_CURVE_NIST_P521 = 0x04,
} bk_ecc_curve_t;

/** Description of the (hardware) accelerator capabilities and features.*/
struct mchp_pk_capabilities
{
    /** Maximum pending requests at any time. */
    int maxpending;
    /** Maximum operand size for prime field operands. 0 when not supported. */
    int max_gfp_opsz;
    /** Maximum operand size for elliptic curve operands. 0 when not supported. */
    int max_ecc_opsz;
    /** Maximum operand size for binary field operands. 0 when not supported. */
    int max_gfb_opsz;
};

/** DMA controller
 *
 * For internal use only. Don't access directly.
 */
struct mchp_dmactl
{
    struct mchp_regs *regs;
    struct mchpdesc *mapped_in;
    struct mchpdesc *mapped_out;
    struct mchp_dmaslot dmamem;
};

struct mchp_regs
{
    char *base;
};

struct mchp_pk_accel
{
    struct mchp_regs regs;
    char *cryptoram;
    int op_size;
    const struct mchp_pk_cmd_def *cmd;
    struct mchp_pk_cnx *cnx;
    const char *outputops[4];
    void *userctxt;
#ifdef MCHP_BAREMETAL_EMUL_UIO
    int fd;
#endif
};


typedef struct mchp_pk_accel mchp_pk_accel;

struct mchphashstate
{
    char *m;
    size_t sz;
};

struct mchphash
{
    struct mchpdesc *d;
    struct mchphashstate *h;
    const struct hashalgo *algo;
    uint32_t cntindescs;
    struct mchp_dmactl dma;
    uint32_t feedsz;
};

/**
*  \name mchp_ecdsa_generate_td
*
*  \param [in] curve structure
*  \param [in] private key,d
*  \param [in] random number, k
*  \param [in] hash digest, h
*  \param [in] r term, r
*  \param [in] s_term, s
*
*  \return PK Engine Return status code
*
*  \details This API is used to generate ecdsa signature.
*/

struct mchp_pk_cnx
{
    struct mchp_pk_accel instances[NUM_PK_INST];
    struct mchp_pk_capabilities caps;
};

/** Elliptic curve configuration and parameters.
 *
 * To be used only via the functions in ec_curves.h The internal members of the
 * structure should not be accessed directly.
 */
struct mchp_pk_ecurve
{
    uint32_t curveflags;
    int sz;
    const char *params;
    int offset;
    struct mchp_pk_cnx *cnx;
};

struct mchp_buf
{
    size_t sz;
    uint8_t *bytes;
};

typedef struct mchp_buf mchp_op;

typedef uint8_t bk_ecc_private_key_code_t[116];
typedef uint8_t bk_ecc_public_key_code_t[180];
typedef uint8_t bk_key_purpose_t;
typedef bk_key_purpose_t bk_ecc_key_purpose_t;


/**
 * \typedef bk_key_source_id_t
 * \brief Key source identifier
 * \enum bk_key_source_id
 * \brief Enumeration of key sources
 *
 * Enumerates the allowed sources from which a generic, symmetric or elliptic curve private key can be generated for use
 * with cryptographic functions..
 */
typedef enum bk_key_source_id
{
    /**
     * \brief Key derived from PUF
     *
     * \details The key is derived in a direct line from the SRAM PUF's device unique start-up data. Keys derived in this
     * way can always be exactly rederived by calling cryptographic functions with the same arguments in the same
     * cryptographic context, and on the same device.
     */
    BK_KEY_SOURCE_PUF_DERIVED = 0x00,
    /**
     * \brief Randomly generated key
     *
     * \details The key is randomly generated using BK's internal cryptographically secure random number generator which is
     *  seeded by entropy coming from the device noise. Keys generated in this way cannot be rederived by cryptographic
     *  functions.
     */
    BK_KEY_SOURCE_RNG_DERIVED = 0x01,
    /**
     * \brief Key provided by user
     *
     * \details The key is an external value which is provided by the calling application to the bk_create_private_key()
     * function.
     */
    BK_KEY_SOURCE_USER_PROVIDED = 0x02,
} bk_key_source_id_t;

/**
 * \typedef bk_ecc_key_source_t
 * \brief Defines the allowed sources from which an elliptic curve private key can be generated for use with bk_create_private_key().
 */
typedef bk_key_source_id_t bk_ecc_key_source_t;

uint32_t MPU_API_rom_ver(void);

// efuse related APIs
/******************************************************************************/
/** MPU_API_efuse_byte_read ();
* This function will read the efuse memory for the index provided. The return value
* will indicate if the performed operation is successful or not.
* If a read is attempted to a read locked memory region – the function will 
* return the current operation as failure.
* 
* @param byte_idx Index\ Offset of the efuse byte to read from the efuse memory layout
* @param out_data Byte pointer for the data read from the efuse memory for the given index
* 
* @return 0 – read operation failed
* @return 1 – read operation is successful
*******************************************************************************/
uint8_t MPU_API_efuse_byte_read(uint16_t byte_idx, uint8_t *out_data);

/******************************************************************************/
/** MPU_API_efuse_byte_write ();
* This function will write the efuse memory for the index provided. The return 
* value will indicate if the performed operation is successful or not. Once written
* the function will read back the memory region to make sure the data is written 
* proper. Only non-set value in the efuse will be programmed and checked against the 
* given value.
* If a write is attempted to a write locked memory region – the function will 
* return the current operation as failure
* 
* @param byte_idx Index \ Offset of the efuse byte to read from the efuse memory layout
* @param in_data Data(byte) to write be written to the efuse memory for the given index
* 
* The return byte value will determine if the write operation is successful or failed
* @return 0 – Write operation is successful
* @return 0xFF – requested data is identical to the efuse content – no write performed
* @return 0xFE – requested offset is out of range
* @return 0xFD – requested offset is in the locked region 
* @return 0xFC – Data read back from the device not matching with the write data
*******************************************************************************/
uint8_t MPU_API_efuse_byte_write(uint16_t byte_idx, uint8_t in_data);

/******************************************************************************/
/** MPU_API_efuse_read_lock ();
* This function is to set OTP read lock, region wise.
* @param region - Region to set read lock
* @return 0 – Read lock operation successful
* @return 1 - Read lock operation failure
*******************************************************************************/
bool MPU_API_efuse_read_lock(uint8_t region);

/******************************************************************************/
/** MPU_API_efuse_write_lock ();
* This function is to set Write Lock for OTP, region wise.
* @param region - Region to set read lock
* @return 0 – Read lock operation successful
* @return 1 - Read lock operation failure
*******************************************************************************/
bool MPU_API_efuse_write_lock(uint8_t region);

/******************************************************************************/
/** MPU_API_check_efuse_read_lock ();
* This function is to check Read Lock for OTP, byte wise.
* @param region - Byte to check for read lock
* @return 0 – Byte not read lock
* @return 1 - Byte is read lock
*******************************************************************************/
bool MPU_API_check_efuse_read_lock(uint16_t byte_index);

// QMSPI related APIs
/******************************************************************************/
/** MPU_API_qmspi_init ();
* This function configures the frequency of SPI, the mode of operation and interface 
* control.
* The permitted frequencies for the SPI are 48 MHz, 24 MHz, 16 MHz, and 12 MHz 
* The SPI supports 4 modes of operation (SPI_MODE_0, SPI_MODE_1, 
* SPI_MODE_2, SPI_MODE_3).
* @param spi_mode An unsigned 8 bit integer indicating the mode. 
*                 The following modes of operation are permitted 
*                  •SPI_MODE_0
*                  •SPI_MODE_1
*                  •SPI_MODE_2
*                  •SPI_MODE_3.
* @param freq_div An unsigned 8 bit integer indicating frequency. 
*                 Refer EXAMPLE SPI FREQUENCIES below for frequency values 
* @param ifc An unsigned 16 bit integer indicating interface control. 
*            Refer the Data sheet of target for bit definitions.
* @param qmspi_port An 8 bit unsigned integer indicating QMSPI port assigned
*                   with the flash device selected
* •If flash device is External (BMC) – then qmspi_port have to be value 0
* •If flash device is External (CPU) – then qmspi_port have to be value 1
* •If flash device is Internal – then qmspi_port can be either 0 or 1
* 
* Listed below Spi Mode macro and its enum value to be used.
* Macro Name    Value
* SPI_MODE_0    (0x00u)
* SPI_MODE_1    (0x06u)
* SPI_MODE_2    (0x01u)
* SPI_MODE_3    (0x07u)
* 
* Listed below QMSPI frequencies and divide table.
* CLOCK_DIVIDE  SPI Clock Frequency
* 0             375 KHz
* 1             96 MHz
* 2             48 MHz
* 3             32 MHz
* 4             24 MHz
* 96            1 MHz
* 128           750 KHz
* 255           376 KHz
* @return None
*******************************************************************************/
void MPU_API_qmspi_init(uint8_t spi_mode, uint8_t freq_div, uint8_t ifc , uint8_t port);

/******************************************************************************/
/** MPU_API_qmspi_port_ctrl ();
* This function controls SPI port control. It facilitates the selection of ports and
* offers enable/disable control. By selection of ports, the GPIO’s and chip selects
* are configured as necessary.
* If any port numbers other that the one’s mentioned below are used, the function will
* not perform any operation.
* @param dev_sel An 8 bit unsigned integer indicating port number. The permitted device
*                select numbers are 
*                 • 0 (External (BMC)
*                 • 1 (External (CPU )
*                 • 2 (Internal).
* @param Pin_mask Specifies the pin(s) of the selected QMSPI port that needs to be modified
*                 b[0]=chip-select, b[1]=clock, b[2]=IO0, b[3]=IO1, b[4]=IO2, b[5]=IO3.
* @param En A boolean input. The permitted values are 
*           •   1 (Enable)
*           •   0 (Disable)
* @return 1 for failure completion of the request
* @return 0 for successful completion of the request 
*******************************************************************************/
uint8_t MPU_API_qmspi_port_ctrl(uint8_t dev_sel, uint8_t Pin_mask, uint8_t en);


/******************************************************************************/
/** MPU_API_qmspi_flash_cmd ();
* This function Transmit up to TX_FIFO length bytes and optionally read up to RX_FIFO
* length bytes SPI transaction is closed (chip select de-asserted when done). 
* Transmit and receive are performed full-duplex. This routine is useful for all the 
* miscellaneous flash command such as read enable, read status, write status, etc. 
* This routine is blocking as it does not use descriptors use the function pointer to
* implement any timeout or Error handling logic. This function not uses DMA.
* @param ntx An unsigned 32 bit integer indicating the number of bytes used for the 
* current transmit operation – if none leave as 0
* @param ptx A unsigned byte pointer to the data buffer to be transmitted – 
*            if none set as NULL
* @param nrx An unsigned 32 bit integer indicating the number of bytes to receive 
*            from the SPI interface for the current transmit operation
*            – if none leave as 0
* @param prx A unsigned byte pointer to the data buffer to be copy the received
*            data – if none set as NULL
* @param ftmout Function pointer for the timeout or error handling 
* @param qmspi_port An 8 bit unsigned integer indicating QMSPI port assigned with
*                   the flash device selected
* •   If flash device is External (BMC) – then qmspi_port have to be value 0
* •   If flash device is External (CPU) – then qmspi_port have to be value 1
* •   If flash device is Internal – then qmspi_port can be either 0 or 1
* @return 0 Success no failure indicated. 
* @return 1 Error if number of bytes is true and invalid pointers for TX (ptx) or RX (prx)
* @return 2 QMSPI is currently busy with previous transmit
* @return 3 Timeout \ error handler return 0 
* 
* Note: Other Function dependencies, the following API’s need to be invoked before
* calling this API for proper QMSPI pin config and initialization
* 1.api_qmspi_port_ctrl
* 2.api_qmspi_port_drv_slew – optional
* 3.api_qmspi_init
*******************************************************************************/
uint8_t MPU_API_qmspi_flash_cmd(uint32_t ntx, uint8_t *ptx, uint32_t nrx, uint8_t *prx,
                                FP_RU32_V ftmout, uint8_t port);

/******************************************************************************/
/** MPU_API_qmspi_is_done_status (), MPU_API_qmspi_is_done_status_ISR ();
* This function gets the status of spi, updates the status into the pointer passed as
* argument, and returns the done status by evaluating the status register value. 
* 
* If the SPI block is busy with the current activity the function will return 0 or False.
* If the SPI block is done with the current activity the function will return 1 or True.
* 
* @param Qmspi_status A pointer to an unsigned 32 bit integer where the status of qmspi is
*                     stored
* @param qmspi_port An 8 bit unsigned integer indicating QMSPI port assigned with the
*                   flash device selected
* •If flash device is External (BMC) – then qmspi_port have to be value 0
* •If flash device is External (CPU) – then qmspi_port have to be value 1
* •If flash device is Internal – then qmspi_port can be either 0 or 1
* Bit Number    Definition
* 0             XFR_COMPLETE
* 1             DMA_COMPLETE
* 2             TX_BUFF_ERR
* 3             RX_BUFF_ERR
* 4             PROG_ERR
* 8             TX_BUFF_FULL
* 9             TX_BUFF_EMPTY
* 10            TX_BUFF_REQ
* 11            TX_BUFF_STALL
* 12            RX_BUFF_FULL
* 13            RX_BUFF_EMPTY
* 15            RX_BUFF_STALL
* 16            XFR_ACTIVE
* 23:17         Reserved
* 27:24         CURRENT_DESCRIPTION_BUFFER
* 31:28         Reserved
* 
* @return  1  Tx Done
* @return  0  QMSPI is still busy.
* Note : ISR variant function is for getting qmspi done status from ISR context
*******************************************************************************/
uint8_t MPU_API_qmspi_is_done_status(uint32_t *Qmspi_status, uint8_t qmspi_port);
uint8_t MPU_API_qmspi_is_done_status_ISR(uint32_t *Qmspi_status, uint8_t qmspi_port);

/******************************************************************************/
/** MPU_API_qmspi_flash_program_dma ();
* This function Supports SPI Flash Page and Block program command protocol 1-1-1.
* Page Program 0x02 or similar command with 1-1-1 protocol. 
* Note:
* • This API is limited to 4 * 0x7FFF bytes at a time. 
* • DMA supports only multiple of 16 bytes data write.
* 
* @param prog_cmd An unsigned 8 bit integer indicating the SPI command to send 
* @param spi_addr An unsigned 32 bit integer indicating the SPI address targeting
*                 for the RX or TX
* @param nbytes An unsigned 32 bit integer indicating the number of bytes used
*               for the current operation
* @param qmspi_port An 8 bit unsigned integer indicating QMSPI port assigned with
*                   the flash device selected
* • If flash device is External (BMC) – then qmspi_port have to be value 0
* • If flash device is External (CPU) – then qmspi_port have to be value 1
* • If flash device is Internal – then qmspi_port can be either 0 or 1
* @return Returns number of bytes processed for the current actions
*******************************************************************************/
uint8_t MPU_API_qmspi_flash_program_dma(uint8_t prog_cmd, uint32_t spi_addr, uint32_t nbytes, uint8_t qmspi_port);

/******************************************************************************/
/** MPU_API_dma_dev_xfr_cfg ();
* This function controls the DMA configuration for the device transfer associated 
* with RX or TX on the QMSPI. 
* 
* @param chan_id An 8 bit unsigned integer which is used to refer to the DMA 
* Channel to be used. Refer qmspi_start_dma section for description regarding dmach_id.
* @param dev_id An 8 bit unsigned integer which is used to refer to the DMA direction 
* for Rx or TX
* • For Transmit (TX) and to Indicates the transmit is from Memory to Device
* If flash device is External (BMC) \ Internal with device qmspi_port as 0 use value 0x15
* If flash device is External (CPU) \ Internal with device qmspi_port as 1 use value 0x19
* 
* • For Receive (RX) and to Indicates if a Receive is Device to Memory
* If flash device is External (BMC) \ Internal with device qmspi_port as 0 use value 0x14 –
* If flash device is External (CPU) \ Internal with device qmspi_port as 1 use value 0x18
* 
* @param maddr An unsigned 32 bit integer indicating Memory address from where to 
*              store the data for the read command or to get the data on a transfer
* @param nbytes Number of bytes used in the current transfer 
* @param flags Flags for the following operation 
* Bit[0] = 1  Start the DMA
* 
* Listed below DMA channel ids and their enum values
* Channel name    Value
* DMA_CH00_ID       0
* DMA_CH01_ID       1
* DMA_CH02_ID       2
* DMA_CH03_ID       3
* DMA_CH04_ID       4
* DMA_CH05_ID       5
* DMA_CH06_ID       6
* DMA_CH07_ID       7
* DMA_CH08_ID       8
* DMA_CH09_ID       9
* 
* @return 0 failure completion of the request 
* @return 1 successful completion of request
*******************************************************************************/
uint8_t MPU_API_dma_dev_xfr_cfg(uint8_t chan_id, uint32_t dev_id, uint32_t maddr, 
  uint32_t nbytes, uint32_t flags);

/******************************************************************************/
/** MPU_API_qmspi_port_drv_slew ();
* This function configures the drive strength and slew rate for GPIO’s based on 
* selected port.
* If any port numbers other that the one’s mentioned below are used, the function 
* will not perform any operation.
* @param port_id  An 8 bit unsigned integer indicating port number. The permitted
*                 device select numbers are
*               • ROM_PORT_QMSPI_SHD
*               • ROM_PORT_QMSPI_PVT
*               • ROM_PORT_QMSPI_INT
* @param width_mask Specifies the pin(s) of the selected QMSPI port that needs 
*                    to be modified
*                   b[0]=chip-select, b[1]=clock, b[2]=IO0, b[3]=IO1, b[4]=IO2, b[5]=IO3.
* @param drv_slew An 8 bit unsigned integer indicating drv slew values. The permitted
*                 values for Drive strength and slew rate are 
*                 Drive strength - 
*                 GPIO_DRV_STR_2MA, GPIO_DRV_STR_4MA, GPIO_DRV_STR_8MA, GPIO_DRV_STR_12MA. 
*                 Slew Rate -  
*                 GPIO_DRV_SLEW_SLOW, GPIO_DRV_SLEW_FAST. 
*                 The parameter drv_slew corresponds to a hardware register. 
*                 Please refer the User Manual of target device for description. 
* @return Returns number of bytes processed for the current actions
*******************************************************************************/
uint8_t MPU_API_qmspi_port_drv_slew(uint8_t port_id, uint8_t width_mask, uint8_t drv_slew);

/******************************************************************************/
/** MPU_API_qmpsi_start ();
* This function starts the SPI operation with the specified interrupt mask for the
* GIRQ18.
* @param Ien_mask An unsigned 16 bit integer specifying the interrupt mask. 
*                 The bit definition of interrupt enable mask corresponds to Status
*                 register bit definitions mentioned in the below table
* @param qmspi_port An 8 bit unsigned integer indicating QMSPI port assigned 
*                   with the flash device selected
*        • If flash device is External (BMC) – then qmspi_port have to be value 0
*        • If flash device is External (CPU) – then qmspi_port have to be value 1
*        • If flash device is Internal – then qmspi_port can be either 0 or 1
* QMSPI Interrupt Mask bit definitions:
* Bit   Interrupt enable bit definitions
* 15    Reserved
* 14    RECEIVE_BUFFER_REQUEST_ENABLE
* 13    RECEIVE_BUFFER_EMPTY_ENABLE
* 12    RECEIVE_BUFFER_FULL_ENABLE
* 11    Reserved
* 10    TRANSMIT_BUFFER_REQUEST_ENABLE
* 9     TRANSMIT_BUFFER_EMPTY_ENABLE
* 8     TRANSMIT_BUFFER_FULL_ENABLE
* 7:5   Reserved R 
* 4     PROGRAMMING_ERROR_ENABLE
* 3     RECEIVE_BUFFER_ERROR_ENABLE
* 2     TRANSMIT_BUFFER_ERROR_ENABLE
* 1     DMA_COMPLETE_ENABLE
* 0     TRANSFER_COMPLETE_ENABLE
* 
* @return None
* Note: Other Function dependencies, before invoking this API all QMSPI initialization
*       and configuration functions are proper.
*******************************************************************************/
void MPU_API_qmpsi_start(uint16_t Ien_mask, uint8_t qmspi_port);

/******************************************************************************/
/** MPU_API_qmspi_flash_read24_dma ();
* This function Issues a SPI Read command 0x03\0B\3B\6B followed by a 24-bit 
* address(A23-A0) and copy the data to the memory address provided.
* @param cmd_id An unsigned 8 bit integer indicating the mode of read to perform
* • 0  Issue a SPI Read command 0x03
* • 1  Issue a SPI Read command 0x0B
* • 2  Issue a SPI Read command 0x3B
* • 3  Issue a SPI Read command 0x6B.
* @param spi_addr An unsigned 32 bit integer indicating Memory address of the SPI interface
*             to access for the current transfer
* @param nbytes An unsigned 32 bit integer indicating the number of bytes of data to read
* @param maddr An unsigned 32 bit integer indicating Memory address from where to
*           store the data for the read command
* @param qmspi_port An 8 bit unsigned integer indicating QMSPI port assigned with the flash
*             device selected
*             • If flash device is External (BMC) – then qmspi_port have to be value 0
*             • If flash device is External (CPU) – then qmspi_port have to be value 1
*             • If flash device is Internal – then qmspi_port can be either 0 or 1
* @return an unsigned 32 bit integer – indicated the number of bytes read from
*         the SPI interface
* 
* Note: Other Function dependencies, api_dma_dev_xfr_cfg need to be used in 
* conjunction with this API for the proper DMA config for the and activate of the DMA block
*******************************************************************************/
uint32_t MPU_API_qmspi_flash_read24_dma(uint8_t cmd_id, uint32_t spi_addr, uint32_t nbytes, 
                                        uint32_t maddr,uint8_t qmspi_port);

/******************************************************************************/
/** MPU_API_qmspi_flash_read_24_32_dma ();
* This function Issues a SPI Read command 0x03\0B\3B\6B followed by a 24-bit and\or
* 0x13\0x0C\0x3C\ 0x6C followed by a 32Bit address and copy the data to the memory 
* address provided.
* @param cmd_id An unsigned 8 bit integer indicating the mode of read to perform
* • 0 → Issue a SPI Read command 0x03 \ 0x13 ( 24 or 32 bit address)
* • 1 → Issue a SPI Read command 0x0B \ 0x0C ( 24 or 32 bit address)
* • 2 → Issue a SPI Read command 0x3B \ 0x3C ( 24 or 32 bit address)
* • 3 → Issue a SPI Read command 0x6B \ 0x6C ( 24 or 32 bit address)
* @param spi_addr An unsigned 32 bit integer indicating Memory address of the SPI 
*                 interface to access for the current transfer
* @param nbytes An unsigned 32 bit integer indicating the number of bytes of data to read
* @param maddr An unsigned 32 bit integer indicating Memory address from where to 
*                 store the data for the read command
* @param qmspi_port An 8 bit unsigned integer indicating QMSPI port assigned with
*                   the flash device selected
* • If flash device is External (BMC) – then qmspi_port have to be value 0
* • If flash device is External (CPU) – then qmspi_port have to be value 1
* • If flash device is Internal – then qmspi_port can be either 0 or 1
* @return An unsigned 32 bit integer – indicated the number of bytes read from the SPI interface
* 
* Note: Other Function dependencies, api_dma_dev_xfr_cfg need to be used in 
* conjunction with this API for the proper DMA config for the and activate of the DMA block.
*******************************************************************************/
uint32_t MPU_API_qmspi_flash_read_24_32_dma(uint8_t cmd_id, uint32_t spi_addr, 
                                            uint32_t nbytes, uint32_t maddr, uint8_t port);

/******************************************************************************/
/** MPU_API_qmspi_flash_read_24_32_ldma ();
* This function Issues a SPI Read command 0x03\0B\3B\6B followed by a 24-bit and\or
* 0x13\0x0C\0x3C\ 0x6C followed by a 32Bit address and copy the data to the memory
* address provided and uses local DMA for the read action. 
* @param cmd_id An unsigned 8 bit integer indicating the mode of read to perform
* • 0 → Issue a SPI Read command 0x03 \ 0x13 ( 24 or 32 bit address)
* • 1 → Issue a SPI Read command 0x0B \ 0x0C ( 24 or 32 bit address)
* • 2 → Issue a SPI Read command 0x3B \ 0x3C ( 24 or 32 bit address)
* • 3 → Issue a SPI Read command 0x6B \ 0x6C ( 24 or 32 bit address)
* @param spi_addr An unsigned 32 bit integer indicating Memory address of the SPI interface
*                 to access for the current transfer
* @param nbytes An unsigned 32 bit integer indicating the number of bytes of data to read
* @param maddr An unsigned 32 bit integer indicating Memory address from where to store the
*         data for the read command
* @param qmspi_port An 8 bit unsigned integer indicating QMSPI port assigned with
*                   the flash device selected
* • If flash device is External (BMC) – then qmspi_port have to be value 0
* • If flash device is External (CPU) – then qmspi_port have to be value 1
* • If flash device is Internal – then qmspi_port can be either 0 or 1
* @return An unsigned 32 bit integer – indicated the number of bytes read from the SPI interface
* Note: Other Function dependencies, api_dma_dev_xfr_cfg need to be used in
* conjunction with this API for the proper DMA config for the and activate of the DMA block
*******************************************************************************/
uint32_t MPU_API_qmspi_flash_read_24_32_ldma(uint8_t cmd_id, uint32_t spi_addr, uint32_t nbytes,
                                             uint32_t maddr, uint8_t qmspi_port);

// Cryptography related APIs
/******************************************************************************/
/** MPU_API_mchp_hash_create_sha1 ();
* Prepare a new SHA1 hash. The user allocated mchphash object 'c' will contain the 
* internal state.
* needed to run the hash. After creation, it shall be passed to mchp_hash_feed() or 
* mchp_hash_digest().
* @param mchphash *c A structure which references to internal state of HASH engine 
* @return status based on status codes as per hardware accelerator instance.
* 
* Bit Number    Definition (all bits are read only)
* -3            Incompatible hardware
* -2            Retry due to hardware unavailability
* 0             Operation is success
*******************************************************************************/
int MPU_API_mchp_hash_create_sha1(struct mchphash *c);

/******************************************************************************/
/** MPU_API_mchp_hash_create_sha224 ();
*Prepare a new SHA224 hash. The user allocated mchphash object 'c' will contain
*the internal state needed to run the hash. After creation, it shall be passed
*to mchp_hash_feed() or mchp_hash_digest().
*@param mchphash *c A structure which references to internal state of HASH engine 
*@return status based on status codes as per hardware accelerator instance
*
*Bit Number     Definition (all bits are read only)
*-3             Incompatible hardware
*-2             Retry due to hardware unavailability
*0              Operation is success
*******************************************************************************/
int MPU_API_mchp_hash_create_sha224(struct mchphash *c);

/******************************************************************************/
/** MPU_API_mchp_hash_create_sha256 ();
* Prepare a new SHA256 hash. The user allocated mchphash object 'c' will contain
* the internal state needed to run the hash. After creation, it shall be passed
* to mchp_hash_feed() or mchp_hash_digest().
* @param mchphash *c A structure which references to internal state of HASH engine 
* @return status based on status codes as per hardware accelerator instance
* 
* Bit Number    Definition (all bits are read only)
* -3            Incompatible hardware
* -2            Retry due to hardware unavailability
* 0             Operation is success
*******************************************************************************/
int MPU_API_mchp_hash_create_sha256(struct mchphash *c);

/******************************************************************************/
/** MPU_API_mchp_hash_create_sha384 ();
* Prepare a new SHA384 hash. The user allocated mchphash object 'c' will contain
* the internal state needed to run the hash. After creation, it shall be passed
* mchp_hash_feed() or mchp_hash_digest().
* @param mchphash *c A structure which references to internal state of HASH engine 
* @return status based on status codes as per hardware accelerator instance
* 
* Bit Number    Definition (all bits are read only)
* -3            Incompatible hardware
* -2            Retry due to hardware unavailability
* 0             Operation is success
*******************************************************************************/
int MPU_API_mchp_hash_create_sha384(struct mchphash *c);

/******************************************************************************/
/** MPU_API_mchp_hash_create_sha512 ();
* Prepare a new SHA512 hash. The user allocated mchphash object 'c' will
* contain the internal state needed to run the hash. After creation, it shall be
* passed to mchp_hash_feed() or mchp_hash_digest().
* @param mchphash *c A structure which references to internal state of HASH engine 
* @return status based on status codes as per hardware accelerator instance
* 
* Bit Number    Definition (all bits are read only)
* -3            Incompatible hardware
* -2            Retry due to hardware unavailability
* 0             Operation is success
*******************************************************************************/
int MPU_API_mchp_hash_create_sha512(struct mchphash *c);

/******************************************************************************/
/** MPU_API_mchp_hash_feed ();
* Assign data to be hashed
* If this function is called with size of message 0, no data will be assigned to be hashed.
* This function can be called multiple times to assemble pieces of the message
* scattered in the memory, if the called in excess, error status -64 (buffer size more)
* will be returned.
* @param mchphash *c A structure which references to internal state of HASH engine 
* @param msg data to be hashed.
* @param size of 'msg' in bytes
* @return status based on status codes as per hardware accelerator instance
* Bit Number    Definition (all bits are read only)
* -33           Failure due to non initialization of HASH internal state
* -69           Too many feeds were inputed
* -64           Input or output buffer size too large
* 0             Success 
*******************************************************************************/
int MPU_API_mchp_hash_feed(struct mchphash *c, const char *msg, size_t sz);

/******************************************************************************/
/** MPU_API_mchp_hash_digest ();
* Starts the hashing operation
* @param mchphash *c A structure which references to internal state of HASH engine 
* @param digest computed digest
* @return status based on status codes as per hardware accelerator instance
* Bit Number        Definition (all bits are read only)
* -33               Failure due to non initialization of HASH internal state
* 0                 Success 
*******************************************************************************/
int MPU_API_mchp_hash_digest(struct mchphash *c, char *digest);

/******************************************************************************/
/** MPU_API_mchp_hash_wait ();
* Wait until the hashing finishes. When done, return the operations status. 
* See mchp_hash_status().
* @param mchphash *c A structure which references to internal state of HASH engine 
* @return status based on status codes as per hardware accelerator instance
* 
* Bit Number   Definition (all bits are read only)
* -33          Failure due to non initialization of HASH internal state
* -1           Waiting on the hardware to process this operation
* 0         Success 
*******************************************************************************/
int MPU_API_mchp_hash_wait(struct mchphash *c);

/******************************************************************************/
/** MPU_API_mchp_pk_get_curve_nistp384 ();
* Get a reference to the predefined NIST P384 elliptic curve
* @param mchp_pk_cnx*cnx A structure which references to internal state of PKE engine 
* @return the reference to the curve parameters for the type of curve instantiated.
*******************************************************************************/
struct mchp_pk_ecurve MPU_API_mchp_pk_get_curve_nistp384(struct mchp_pk_cnx *cnx);

/******************************************************************************/
/** MPU_API_mchp_pk_get_curve_nistp256 ();
* Get a reference to the predefined NIST P256 elliptic curve
* @param mchp_pk_cnx*cnx A structure which references to internal state of PKE engine 
* @return the reference to the curve parameters for the type of curve instantiated.
*******************************************************************************/
struct mchp_pk_ecurve MPU_API_mchp_pk_get_curve_nistp256(struct mchp_pk_cnx *cnx);

/******************************************************************************/
/** MPU_API_mchp_pk_init ();
* An initialization routine that initializes both the PK HW and the PK library.
* The routine, mchp_pk_init, takes a pointer to an aligned buffer of minimum size 64 bytes.
* It clears the PCR crypto blocks SleepEnable bit, Clears PK shared crypto memory(SCM), 
* Initializes struct mchp_pk_cnx with memory provided by caller and returns a pointer to
* a struct mchp_pk_cnx used by some of 
* the PK API's. (eg: instantiation of ec curves, curve param operations etc)
* @param cnxmem Pointer to buffer of size 64 bytes (PK_CNX_MEM_SIZE/4)
* cnxmsz PK_CNX_MEM_SIZE = 64 bytes
* @return status based on status codes as per hardware accelerator instance
* 
* Bit Number Definition (all bits are read only)
* NULL         Due to condition failure: Memory must be aligned >= 4 bytes
*               and have size >= PK_CNX_MEM_SIZE
* 
* On success, returns Pointer to struct mchp_pk_cnx.
*******************************************************************************/
struct mchp_pk_cnx *MPU_API_mchp_pk_init(uint32_t *cnxmem, size_t cnxmsz);

/******************************************************************************/
/** MPU_API_mchp_ecdsa_verify ();
* Verify the ECDSA signature on an elliptic curve 
* Signature is the r and s
* @param mchp_pk_ecurve*curve A structure which references to elliptic curve params
* @param mchp_op *qx Pointer to buffer containing x coordinate of Public key
* @param mchp_op *qy Pointer to buffer containing y coordinate of Public key
* @param mchp_op *r, pointer to buffer containing r term of signature
* @param 
* @param mchp_op *s pointer to buffer containing s term of signature
* @param 
* @param mchp_op *h pointer to buffer containing hash of message
* @return status based on status codes as per hardware accelerator instance
* Bit Number Definition (all bits are read only)
* -47         The signature is not valid
* -53         The input operand is too large
* -41         The function or operation was given an invalid parameter
* -50         The input value is outside the expected range
* 0         Success
* Return if success, means the message is authenticated successfully.
*******************************************************************************/
int MPU_API_mchp_ecdsa_verify(const struct mchp_pk_ecurve *curve, const mchp_op *qx, \
                              const mchp_op *qy, const mchp_op *r, const mchp_op *s, const mchp_op *h);

/******************************************************************************/
/** MPU_API_sb_otp_validate();
* Validates OTP values
* @param otpmap_rev - OTP map revision number
* @param fw_rev - Compatible Firmare revision number
* @return True/False
*******************************************************************************/                         
bool MPU_API_sb_otp_validate(uint8_t otpmap_rev, uint8_t fw_rev);

/******************************************************************************/
/** MPU_API_mchp_async_ecdsa_generate_go ();
* Generate an ECDSA signature on an elliptic curve in asynchronous manner.
* Signature is the r and s
* Ensure that mchp_pk_wait() is called before calling mchp_async_ecdsa_generate_end()
* to obtain the generated signature.
* @param mchp_pk_ecurve*curve A structure which references to elliptic curve params
* @param mchp_op *d Pointer to buffer containing EC based Private key
* @param mchp_op *k Pointer to buffer containing Random number (Byte size based on curve size)
* @param mchp_op *h Pointer Buffer containing hash of the message
* @param mchp_op *r pointer to result buffer containing s term of signature
* @param mchp_op *s pointer to result buffer containing r term of signature
* @return eturn status based on status codes as per hardware accelerator instance
* 
* The api returns structure mchp_pk_dreq whose status member can be used to identify the 
* operation is success or not.
* 
* //PKE Hardware accelerator status (mainly used for asynchronous apis)
* struct mchp_pk_dreq
* {
*    mchp_pk_accel *req;
*    int status;
* };
* 
* Bit Number Definition (all bits are read only)
* -47         The signature is not valid
* -53         The input operand is too large
* -41         The function or operation was given an invalid parameter
* -50         The input value is outside the expected range
* 0         Success
* 
* Return pointer to structure mchp_pk_dreq which points to the hardware accelerator 
* instance. The mchp_pk_dreq instance has the status parameter which can be used to 
* determine the operation is success or not.
*******************************************************************************/
struct mchp_pk_dreq MPU_API_mchp_async_ecdsa_generate_go(const struct mchp_pk_ecurve *curve,
        const mchp_op *d,
        const mchp_op *k,
        const mchp_op *h,
        mchp_op *r,
        mchp_op *s);

/******************************************************************************/
/** MPU_API_bk_ecdsa_sign ();
* This function signs a message or a hash of message using ECDSA with an elliptic 
* curve private key in the internal private key code format. Signing can be done
* with either a random seed or a deterministically derived seed as indicated by 
* the calling application.
* @param private_key_code Pointer to a private key code type buffer which holds 
*                         the elliptic curve private key to be used for signing,
*                         and as was created by bk_create_private_key 
* @param deterministic_signature Value which specifies if deterministic or 
*                         non-deterministic signing will be used. If this value
*                         equals False, message will be signed using the standard
*                          ECDSA non-deterministic algorithm. 
* @param message Pointer to a byte array buffer which holds the message or the 
*                 message hash that will be signed. 
* @param message_length Value which specifies the size in bytes of the message buffer. 
* @param message_is_hash Value which specifies if the provided message buffer
*                         contains an already hashed message, or a raw message
*                         byte array. 
* @param signature Pointer to a byte array buffer which will hold the computed
*                  ECDSA signature. Its exact size in bytes depends on the used
*                  curve, as contained in the private key code 
* @param signature_length As input, this is the length in bytes of the allocated
*                         output buffer pointed to by signature. This value
*                         is used to check for buffer overflow. 
* @return  IID_SUCCESS Indicating the successful execution of the called function 
* @return IID_INVALID_PARAMETERS Indicating that at least one of the parameters 
*                                passed as argument with this function call has
*                                an invalid form and/or content 
* @return IID_NOT_ALLOWED Indicating that the given function call is not allowed
*                           in the current state 
* @return IID_INVALID_PRIVATE_KEY_CODE Indicating that the provided private key
*                                        code input is not a valid private key code
*                                        for the called function on this device. 
* @return IID_ECC_NOT_ALLOWED Indicating that the provided private and/or public
*                             key code inputs do not have the right purpose flags
*                             for being used by the called function. 
*******************************************************************************/
int MPU_API_bk_ecdsa_sign(const bk_ecc_private_key_code_t *const priv_key_code,
                          const bool deterministic,
                          const uint8_t *message,
                          const uint32_t message_length,
                          const bool message_is_hash,
                          uint8_t *const signature,
                          uint16_t *const signature_length);

/******************************************************************************/
/** MPU_API_bk_create_private_key ();
* This function transforms an elliptic curve private key into a protected 
* private key code which is only usable within the same cryptographic context, 
* and on the same unique device, it was created on. 
* Note:
* This function can take private keys from three possible sources: 
* • device-unique private keys derived from the device’s secret fingerprint 
* • randomly generated private keys derived from the device power-up noise 
* •  user-provided private keys 
* @param curve Specifies the named elliptic curve on which the considered 
*              private key is defined 
* @param purpose_flags Flag which specifies the usage purpose of the private key.
*                      It must be a valid flag of the bk_ecc_key_purpose_t 
*                      enumeration which is declared in iidbroadkey.h. 
* @param usage_context Pointer to a byte array buffer which holds an (optional)
*                      usage context. 
* @param usage_context_length Value which specifies the length in bytes of the
*                             usage_context buffer. If this length is set to 0,
*                             no usage context is taken into account. 
* @param key_source Specifies the source of the elliptic curve private key. 
*                     It must be a valid source of the bk_ecc_key_source_t 
*                     enumeration which is declared in iidbroadkey.h. 
* @param private_key Pointer to a byte array buffer which holds the private 
*                     key used when key_source is BK_ECC_KEY_SOURCE_USER_PROVIDED. 
*                     The expected input is binary in network byte order representation 
* @param private_key_code Pointer to a private key code type buffer which will hold 
*                         the created elliptic curve private key. 
* @return IID_SUCCESS - Indicating the successful execution of the called function 
* @return IID_INVALID_PARAMETERS - Indicating that at least one of the parameters 
*                                  passed as argument with this function call has an 
*                                  invalid form and/or content 
* @return IID_NOT_ALLOWED  Indicating that the given function call is not allowed in
*                          the current state 
* @return IID_INVALID_PRIVATE_KEY  Indicating that an externally provided private 
*                              key input value (to bk_create_private_key or 
*                              bk_derive_public_key) is an invalid private key for
*                              the specified elliptic curve.
*******************************************************************************/                        
int MPU_API_bk_create_private_key(const bk_ecc_curve_t curve, 
        const bk_ecc_key_purpose_t purpose_flags, 
        const uint8_t * const usage_context, 
        const uint32_t usage_context_length, 
        const bk_ecc_key_source_t key_source, 
        const uint8_t * const private_key, 
        bk_ecc_private_key_code_t * const private_key_code);

/******************************************************************************/
/** MPU_API_bk_compute_public_from_private_key ();
* This function computes the elliptic curve public key corresponding to a private key
* code created with bk_create_private_key, and outputs the public key in a 
* corresponding public key code format. The curve and purpose flags of the public key
* (code) will be the same as the one of the provided private key (code).
* A call to bk_compute_public_from_private_key is only allowed when BroadKey
* is either in the Enrolled or in the Started state; a call from any other state 
* will return IID_NOT_ALLOWED
* Note:
* Buffer addresses must be at a 32-bit boundary, so the lowest two bits of the
* address must be 0. 
* @param private_key_code Pointer to a private key code type buffer which holds
*                         the elliptic curve private key, and was created by
*                         bk_create_private_key 
* @param public_key_code Pointer to a public key code type buffer which will hold the 
*                        elliptic curve public key computed from the provided private 
*                        key (code) input. 
* @return IID_SUCCESS - Indicating the successful execution of the called function 
* @return IID_INVALID_PARAMETERS - Indicating that at least one of the parameters 
*                                  passed as argument with this function call has an
*                                   invalid form and/or content 
* @return IID_NOT_ALLOWED - Indicating that the given function call is not allowed in
*                           the current state 
* @return IID_INVALID_PRIVATE_KEY_CODE - Indicating that the provided private key code
*                                        input is not a valid private key code for the
*                                        called function on this device. 
*******************************************************************************/ 
int MPU_API_bk_compute_public_from_private_key(const bk_ecc_private_key_code_t * const private_key_code, 
                    bk_ecc_public_key_code_t * const public_key_code);

/******************************************************************************/
/** MPU_API_bk_ecdsa_verify ();
* This function verifies the ECDSA signature of a message or a hash of message with
* an elliptic curve public key in the internal public key code format.
* Note:
* bk_ecdsa_verify has no explicit output parameters, only inputs. 
* Its result is contained in its return code. If IID_SUCCESS is returned,
* the signature on the message is successfully verified. 
* If IID_INVALID_SIGNATURE is returned, the signature on the message is invalid. 
* For other return codes bk_ecdsa_verify failed to complete
* @param public_key_code Pointer to a public key code type buffer which holds
*                         the elliptic curve public key to be used for signature
*                         verification 
* @param message Pointer to a byte array buffer which holds the message or 
*                 the message hash on which a signature will be verified. 
* @param message_length Value which specifies the size in bytes of the message buffer 
*                         If message_is_hash is True (i.e. the provided message is 
*                         actually a message hash), the size must be equal to the size
*                         in bytes of the used private key, as determined by the used curve 
*                         Otherwise (i.e. the provided message is a raw message byte array),
*                         the size in bytes must be equal to the raw message length.
* @param message_is_hash Value which specifies if the provided message buffer contains
*                         an already hashed message, or a raw message byte array. 
*                         If this value equals False, bk_ecdsa_verify will treat the message
*                         buffer as a raw message, and will hash it first using SHA-256 and
*                         the trailing bytes will be truncated to equal the size of the used
*                         elliptic curve private key 
*                         Otherwise, bk_ecdsa_verify will treat the message buffer as an
*                         already hashed message, and it will be verified directly. 
* @param signature Pointer to a byte array buffer which holds the to-be-verified ECDSA signature. 
* @param signature_length This is the length in bytes of the provided signature. 
* @return IID_SUCCESS - Indicating the successful execution of the called function 
* @return IID_INVALID_PARAMETERS - Indicating that at least one of the parameters passed as
*                                  argument with this function call has an invalid form and/or content 
* @return IID_NOT_ALLOWED - Indicating that the given function call is not allowed in the current state 
* @return IID_INVALID_PUBLIC_KEY_CODE  - Indicating that the provided public key code input
*                                        is not a valid public key code for the called
*                                        function on this device 
* @return IID_ECC_NOT_ALLOWED - Indicating that the provided private and/or public key code
*                                inputs do not have the right purpose flags for being used by the
*                                called function. 
* @return ID_INVALID_SIGNATURE - Indicating that the signature input provided to bk_ecdsa_verify
*                                is not a valid signature for the provided message under
*                                the provided public key 
*******************************************************************************/ 
int MPU_API_bk_ecdsa_verify(const bk_ecc_public_key_code_t * const public_key_code, 
        const uint8_t * const message, 
        const uint32_t message_length, 
        const bool message_is_hash, 
        const uint8_t * const signature, 
        const uint16_t signature_length);


/******************************************************************************/
/** MPU_API_mchp_pk_wait ();
* Wait until the current PKE operation finishes. After the operation finishes,
* return the operation status code
* 
* Note: The mchp_pk_wait() is mandatory to be used for asynchronous apis.
* (Non blocking, CM operations)
* This api should be called once the asynchronous api (xx_go, xx_cm_go) 
* is executed before trying to obtain the result from (xx_end, xx_go_end) operations.
* 
* @param mchp_pk_accel* req A structure which describes the hardware accelerator 
* @return status based on status codes as per hardware accelerator instance
* 
* Bit Number Definition (all bits are read only)
* -52         The input point is not on the defined elliptic curve
* -50         The input value is outside the expected range
* -48         The functionality or operation is not supported
* -47         The signature is not valid
* -54         A platform specific error
* -53         The input operand is too large
* -43         The operation is still executing
* -41         The function or operation was given an invalid parameter
* 0         Success
*******************************************************************************/
int MPU_API_mchp_pk_wait(mchp_pk_accel *req);

/******************************************************************************/
/** MPU_API_mchp_ndrng_init ();
* The below macros are to be defined for NDRNG initialization
* @param enable_conditioning Set to true or false if conditioning function enabled 
* @param fifo_wake_thresh FIFO level below which the module leaves the idle state
*                      to refill the FIFO, expressed in number of 128bit blocks.
*                      - NDRNG_FIFO_WAKEUP_LVL
* @param ring_pwr_down_delay Switch off timer value - NDRNG_OFF_TIMER_VAL
* @param nb_cond conditioning number of 128-bit blocks - NDRNG_NB_128BIT_BLOCKS
* @param rng_clk_div Sample clock divider. The frequency at which the outputs of
*                    the rings are sampled is given by 
*                    Fs = Fpclk/( NDRNG_CLKDIV + 1).  
* @param rng_init_wait Initial wait counter value - NDRNG_INIT_WAIT_VAL 
* @return status based on status codes as per hardware accelerator instance
* 
* Bit Number Definition (all bits are read only)
* -4         Generic invalid parameter
* -2         No hardware available for a new operation. Retry later.
* -99         NDRNG hardware errror
* 0         Success
*******************************************************************************/
int8_t MPU_API_mchp_ndrng_init(bool enable_conditioning,
                            uint32_t fifo_wake_thresh,
                            uint32_t ring_pwr_down_delay,
                            uint32_t nb_cond,
                            uint32_t rng_clk_div,
                            uint32_t rng_init_wait);

/******************************************************************************/
/** MPU_API_mchp_ndrng_enable ();
* Enable or disable the NDRNG block.
* Should be enabled after initializing the NDRNG block.
* @param enable Set to true to enable or false to disable T_WAIT_VAL 
* @return None
*******************************************************************************/
void MPU_API_mchp_ndrng_enable(bool enable);

/******************************************************************************/
/** MPU_API_mchp_ndrng_soft_reset ();
* Reset the NDRNG block.
* Note: 
* Soft reset does not clear then NDRNG enable bit
* First call NDRNG initialization to clear enable before calling this function.
* @param None 
* @return None
*******************************************************************************/
void MPU_API_mchp_ndrng_soft_reset(void);

/******************************************************************************/
/** MPU_API_mchp_crypt_sleep_control ();
To set or clear the PCR SleepEnable bit for the crypto blocks.
Carlsbad HW uses one PCR bit for all crypto blocks.
@param sleep_en Enable or disable sleep bit of all crypto blocks
@return None
*******************************************************************************/
void MPU_API_mchp_crypt_sleep_control(bool sleep_en);

/******************************************************************************/
/** MPU_API_mchp_ndrng_intr_en ();
* To set or clear the PCR SleepEnable bit for the crypto blocks. 
* Carlsbad HW uses one PCR bit for all crypto blocks.
* Interrupt events supported are FIFO full, Repetition test fail and Adaptive 
* proportion test fail.This api will set the corresponding bit for the interrupt 
* based on the bit position specified in intr_bitmap param. 
* @param intr_bitmap Bit position in control register specifying the interrupt event
* @param enable True to enable the interrupt or disable if false
* @return None
*******************************************************************************/
void MPU_API_mchp_ndrng_intr_en(uint8_t intr_bitmap, uint8_t enable);

/******************************************************************************/
/** MPU_API_mchp_ndrng_read_bytes ();
* Fill buffer with requested number of bytes of random data from NDRNG.
* Note:
* Loops while NDRNG FIFO is not empty and reads 32-bit words from the FIFO. 
* Word values are broken into bytes and saved to dest. This routine can block
* if FIFO is emptied. This api also checks Health of data after data is read
* from the FIFO.
* @param dest pointer to byte buffer
* @param nbytes number of bytes to read
* @return return status based on status codes as per hardware accelerator instance
* 
* Bit Number Definition (all bits are read only)
* -4         Generic invalid parameter(If buffer pointer is NULL or number
*               of bytes)
* -2         No hardware available for a new operation. Retry later.
* -98         NDRNG not healthy (Error in status bits)
* -99         NDRNG hardware error
* On success, returns the number of bytes read.
*******************************************************************************/
int8_t MPU_API_mchp_ndrng_read_bytes(uint8_t *dest, int nbytes);

/******************************************************************************/
/** MPU_API_mchp_async_ecdsa_generate_end ();
* Finish asynchronous (non-blocking) EC signature generation. This api must be called after 
* mchp_async_ecdsa_generate_go().
* 
* Input Parameter
* @param mchp_pk_accel*req A structure which describes the hardware accelerator 
* @param mchp_op *r pointer to result buffer containing r term of generated signature
* @param mchp_op *s pointer to result buffer containing s term of generated signature
* @return None
* The r and s buffer have the signature result.
*******************************************************************************/
void MPU_API_mchp_async_ecdsa_generate_end(struct mchp_pk_accel *req,
        mchp_op *r,
        mchp_op *s);

/******************************************************************************/
/** MPU_API_mchp_hash_init_state ();
* Initialize the memory for hash context saving 
* The caller passes a buffer of size (64+128+16) bytes to be used for save/resume.
* This routine copies the Hash algorithm initial value into the save/resume buffer.
* State buffer is for two purposes: save/resume state and for padding when
* mchp_hash_digest is called
* NOTE: Allocate state buffer size = 2 * block size  
* // That is  2 * maximum block size (SHA-384/512) 
* @param mchphash *c A structure which references to internal state of HASH engine 
* @param mchphashstate*h The state for a hash operation.
* @param dmamem address of the location to save the state
* @return None
*******************************************************************************/
void MPU_API_mchp_hash_init_state(struct mchphash *c,
                                  struct mchphashstate *h,
                                  char *dmamem);

/******************************************************************************/
/** MPU_API_mchp_hash_resume_state ();
* Builds a DMA descriptor pointing to the data in the save/resume buffer.
* Resume hashing of previously saves hash engine state.
* @param mchphash *c A structure which references to internal state of HASH engine 
* @param mchphashstate*h state of previous hashing
* @return status based on status codes as per hardware accelerator instance
* 
* Bit Number Definition (all bits are read only)
* 0         Success 
*******************************************************************************/                                
int MPU_API_mchp_hash_resume_state(struct mchphash *c,
                                   struct mchphashstate *h);

/******************************************************************************/
/** MPU_API_mchp_hash_save_state ();
* Saves state of current hashing.
* Exports the state of partial hashing to the memory allocated by mchp_hash_init_state().
* 
* Note:
* In order to export the state of partial hashing, the total size of data fed
* in the current resume-save step must be multiple of block size of the algorithm
* used. For SHA1/224/256/SM3 block size is 64 bytes and for SHA384/512 block
* size is 128 bytes.
* @param mchphash *c A structure which references to internal state of HASH engine 
* @return status based on status codes as per hardware accelerator instance
* 
* Bit Number Definition (all bits are read only)
* -33         Failure due to non initialization of HASH internal state
* -70         Input data size granularity is incorrect
* 0             Success
*******************************************************************************/   
int MPU_API_mchp_hash_save_state(struct mchphash *c);

#endif /* INCLUDE_ROM_API_MPU_H_ */
