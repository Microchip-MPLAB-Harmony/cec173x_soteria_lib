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
 
#ifndef QMSPI_API_H
#define QMSPI_API_H

#ifdef __cplusplus
extern "C" {
#endif

#define DEV_ENABLE   1u
#define DEV_DISABLE  0u
#define NONE     0u

/** QMSPI Supported interfaces */
#define SHD_SPI         0u   /*<QMSPI Port 0 = Shared SPI signals */
#define PVT_SPI         1u   /*<QMSPI Port 1 = Private SPI signals */
#define INT_SPI         2u   /*<QMSPI Port 2 = Internal SPI signals */

#define INT_SPI_SLEW    0x00
#define INT_SPI_PORT_ID 0x02
#define INT_SPI_PORT_NO 0x00

/** QMSPI PIN Width Mask*/
#define SPI_IO_FD_DUAL  0x0Eu
#define SPI_IO_QUAD     0x3Eu

#define FLASH_CMD_PAGE_WRITE_4B 0x12

/** QMSPI SPI Mode selection
 * Used when accessing MODE register
 * SPI signalling mode field.
 */
typedef enum
{
    QMSPI_SPI_MODE_0 = 0x00u,
    QMSPI_SPI_MODE_1 = 0x06u,
    QMSPI_SPI_MODE_2 = 0x01u,
    QMSPI_SPI_MODE_3 = 0x07u,
    QMSPI_SPI_MODE_0_FALL_EDGE = 0X04u    /* Mode selection for 48Mhz quad mode */
} QMSPI_SPI_MODE;

/** QMSPI SPI frequency selection  */
typedef enum
{
    QMSPI_FREQ_96M = 1u,
    QMSPI_FREQ_48M = 2u,
    QMSPI_FREQ_24M = 4u,
    QMSPI_FREQ_16M = 6u,
    QMSPI_FREQ_12M = 8u,
    QMSPI_FREQ_8M = 12u,
    QMSPI_FREQ_6M = 16u,
    QMSPI_FREQ_4M = 24u,
    QMSPI_FREQ_3M = 32u,
    QMSPI_FREQ_2M = 48u,
    QMSPI_FREQ_1M = 96u
} QMSPI_FREQ;

/*typedef*/ enum DMA_CHANNEL_E
{
    DMA_CH00_ID = 0,
    DMA_CH01_ID,
    DMA_CH02_ID,
    DMA_CH03_ID,
    DMA_CH04_ID,
    DMA_CH05_ID,
    DMA_CH06_ID,
    DMA_CH07_ID,
    DMA_CH08_ID,
    DMA_CH09_ID,
    DMA_CH10_ID,
    DMA_CH11_ID,
    DMA_CH12_ID,
    DMA_CH13_ID,
    DMA_MAX_ID
} /*DMA_CHANNEL*/;

enum SPI_CHANNEL
{
    SPI_CHANNEL_0,
    SPI_CHANNEL_1,
    SPI_CHANNEL_MAX
};

/** CHIP SELECT */
typedef enum
{
    SELECT,
    DESELECT
} CS_SPI;

// Device ID used with DMA block
#define QMSPI_TX_DMA_REQ_ID     (DMA_CH10_ID)
#define QMSPI_RX_DMA_REQ_ID     (DMA_CH11_ID)

// DMA Device ID in bits[7:1] and DMA direction in bit[0]
// Matches DMA Control bits[15:8]
#define QMSPI_TX_DMA_REQ_DIR0   ((QMSPI_TX_DMA_REQ_ID << 1) + (1ul)) // Transmit is Memory to Device
#define QMSPI_RX_DMA_REQ_DIR0   ((QMSPI_RX_DMA_REQ_ID << 1) + (0ul)) // Receive is Device to Memory

// Device ID used with DMA block
#define QMSPI1_TX_DMA_REQ_ID     (DMA_CH12_ID)
#define QMSPI1_RX_DMA_REQ_ID     (DMA_CH13_ID)

#define DMA_DEVICE_ID   QMSPI_RX_DMA_REQ_DIR0
#define DMA_CHANNEL_ID  DMA_CH07_ID //Config any DMA Channel for the opeartion

// DMA Device ID in bits[7:1] and DMA direction in bit[0]
// Matches DMA Control bits[15:8]
#define QMSPI1_TX_DMA_REQ_DIR0   ((QMSPI1_TX_DMA_REQ_ID << 1) + (1ul)) // Transmit is Memory to Device
#define QMSPI1_RX_DMA_REQ_DIR0   ((QMSPI1_RX_DMA_REQ_ID << 1) + (0ul)) // Receive is Device to Memory

#define CTRL_DMA_RUN (1ul << 0u)
#define CTRL_DMA_BUSY_RO (1ul << 5u)
#define CTRL_DMA_DIR_DEV2MEM (0ul << 8u)
#define CTRL_DMA_DIR_MEM2DEV (1ul << 8u)
#define CTRL_DMA_INCR_MEM_ADDR (1ul << 16u)
#define CTRL_DMA_INCR_DEV_ADDR (1ul << 17u)
#define CTRL_DMA_LOCK_CHAN (1ul << 18u)
#define CTRL_DMA_HW_FLOW (0ul << 19u)
#define CTRL_DMA_SW_FLOW (1ul << 19u)
#define CTRL_DMA_XFR_UNIT_1B (1ul << 20u)
#define CTRL_DMA_XFR_UNIT_2B (2ul << 20u)
#define CTRL_DMA_XFR_UNIT_4B (4ul << 20u)
#define CTRL_DMA_SW_XFR_GO (1ul << 24u)
#define CTRL_DMA_ABORT (1ul << 25u)

#define CTRL_LDMA_INCR_MEM_ADDR (1U << 6U)

/* Marco for glacier */
#define DMA_MEM_NO_INCR     0u
#define DMA_MEM_INCR        1u

#define SPI_GET_PORT(cs) ((cs >> 2) & 0x3)

/**
 * qmspi_spi_flash_dev_reset - Resets the flash memory to default state
 * @param cs chip select
 * @return None
 */
void qmspi_spi_flash_dev_reset(uint8_t cs);

#ifdef __cplusplus
}
#endif

#endif /*QMSPI_API_H*/