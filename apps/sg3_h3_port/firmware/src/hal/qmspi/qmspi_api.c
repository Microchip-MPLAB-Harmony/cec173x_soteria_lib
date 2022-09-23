/*****************************************************************************
* © 2022 Microchip Technology Inc. and its subsidiaries.
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
******************************************************************************

Version Control Information (Perforce)
******************************************************************************
$Revision: #1 $ 
$DateTime: 2022/09/05 23:00:53 $ 
$Author: i64652 $
Last Change:    None
******************************************************************************/
/** @file qmspi_api.c
* \brief QMSPI API Source file
* \author pramans
* 
* This file implements the QMSPI API functions  
******************************************************************************/

/** @defgroup QMSPI
 *  @{
 */

#include "common.h"
#include "qmspi_api.h"
#include "peripheral/qmspi/plib_qmspi_common.h"
#include "peripheral/qmspi/plib_qmspi0.h"
#include "peripheral/qmspi/plib_qmspi1.h"

extern void timer_delay_us(uint32_t num_us);

/**
 * qmspi_flash_program_ldma - Writes data to flash memory using local DMA
 * @param prog_cmd Write command code
 * @param spi_addr destination address in flash
 * @param nbytes number of bytes to write
 * @param maddr source address
 * @param port QMSPI port
 * @return number of bytes written
 */
uint32_t qmspi_flash_program_ldma(uint8_t prog_cmd, uint32_t spi_addr, 
									uint32_t nbytes, uint32_t maddr, 
									uint8_t port)
{
	QMSPI_DESCRIPTOR_XFER_T qmspi_xfer_cfg;
	uint32_t noofbytes = 0;

	memset(&qmspi_xfer_cfg, 0x00, sizeof(qmspi_xfer_cfg));

	if (SPI_4BYTE_ADDR_REQUIRED(spi_addr, nbytes))
	{
		qmspi_xfer_cfg.address_32_bit_en = true;
		prog_cmd = FLASH_CMD_PAGE_WRITE_4B;
	} else {
		// 3 byte/24 bit address mode
	}

	qmspi_xfer_cfg.command = prog_cmd;

	switch(prog_cmd)
	{
		case 0x32:
			qmspi_xfer_cfg.qmspi_ifc_mode = QUAD_OUTPUT;
			// no dummy bytes
			break;
		case 0x34:
			qmspi_xfer_cfg.qmspi_ifc_mode = QUAD_OUTPUT;
			// no dummy bytes
			break;
		case 0x12:
		case 0x02:
		default:
			qmspi_xfer_cfg.qmspi_ifc_mode = SINGLE_BIT_SPI;
			// no dummy bytes
			// no break
	}

	qmspi_xfer_cfg.address = spi_addr;
	qmspi_xfer_cfg.ldma_enable = true;
	qmspi_xfer_cfg.ldma_channel_num = QMSPI_LDMA_CHANNEL_0;

	switch(port)
	{
		case PVT_SPI:
			noofbytes = QMSPI1_DMATransferWrite(&qmspi_xfer_cfg, (void*)maddr, nbytes);
			break;
		default: //SHD_SPI and INT_SPI
			noofbytes = QMSPI0_DMATransferWrite(&qmspi_xfer_cfg, (void*)maddr, nbytes);
			// no break
	}

	return noofbytes;
}

/**
 * qmspi_spi_flash_dev_reset - Resets the flash memory to default state
 * @param cs chip select
 * @return None
 */
void qmspi_spi_flash_dev_reset(uint8_t cs)
{
    uint32_t qmode = 0;
	uint32_t port = 0;
	uint32_t nbytes = 0;
	uint8_t temp_buffer = 0;
	bool status = 0;
	QMSPI_XFER_T qmspi_write_cfg;

    port = SPI_GET_PORT(cs);
	memset(&qmspi_write_cfg, 0x00, sizeof(qmspi_write_cfg));

	qmode = QMSPI_SPI_MODE_0;
	MPU_API_qmspi_init(qmode, QMSPI_FREQ_12M, 0 ,port);

	/* Transmit 24 clocks in Quad Mode */
	/* Quad mode is 2 clocks/byte. 24 clocks = 12 bytes */
	/* ---------------------------------------------------------------------- */
	MPU_API_qmspi_init(qmode, QMSPI_FREQ_12M, 0 ,port);

	qmspi_write_cfg.command = 0x00;
	qmspi_write_cfg.address_enable = false; /* No address for this command */
	/* qmspi_write_cfg.address_32_bit_en = dont care */
	/* qmspi_write_cfg.address = dont care */
	qmspi_write_cfg.num_of_dummy_byte = 12;
	qmspi_write_cfg.qmspi_ifc_mode = QUAD_CMD; /* For Winbond devices */
	nbytes = 1;

	chipSelectSPI(cs, SELECT);
	switch(port)
	{
		case PVT_SPI:
			status = QMSPI1_Read(&qmspi_write_cfg, (void*)&temp_buffer, nbytes);
			break;
		default: //SHD_SPI and INT_SPI
			status = QMSPI0_Read(&qmspi_write_cfg, (void*)&temp_buffer, nbytes);
			// no break
	}
	chipSelectSPI(cs, DESELECT);
	/* ---------------------------------------------------------------------- */

	/* Transmit full-duplex: 0xAB */
	/* ---------------------------------------------------------------------- */
	MPU_API_qmspi_init(qmode, QMSPI_FREQ_12M, 0 ,port);

	qmspi_write_cfg.command = 0xAB;
	qmspi_write_cfg.address_enable = false; /* No address for this command */
	/* qmspi_write_cfg.address_32_bit_en = dont care */
	/* qmspi_write_cfg.address = dont care */
	qmspi_write_cfg.num_of_dummy_byte = 0x03;
	qmspi_write_cfg.qmspi_ifc_mode = SINGLE_BIT_SPI; /* For Winbond devices */
	nbytes = 1;

	chipSelectSPI(cs, SELECT);
	switch(port)
	{
		case PVT_SPI:
			status = QMSPI1_Read(&qmspi_write_cfg, (void*)&temp_buffer, nbytes);
			break;
		default: //SHD_SPI and INT_SPI
			status = QMSPI0_Read(&qmspi_write_cfg, (void*)&temp_buffer, nbytes);
			// no break
	}
	chipSelectSPI(cs, DESELECT);
	/* ---------------------------------------------------------------------- */
	timer_delay_us(200);

	/* Transmit 24 clocks in Quad Mode */
	/* Quad mode is 2 clocks/byte. 24 clocks = 12 bytes */
	/* ---------------------------------------------------------------------- */
	MPU_API_qmspi_init(qmode, QMSPI_FREQ_12M, 0 ,port);

	qmspi_write_cfg.command = 0x00;
	qmspi_write_cfg.address_enable = false; /* No address for this command */
	/* qmspi_write_cfg.address_32_bit_en = dont care */
	/* qmspi_write_cfg.address = dont care */
	qmspi_write_cfg.num_of_dummy_byte = 12;
	qmspi_write_cfg.qmspi_ifc_mode = QUAD_CMD; /* For Winbond devices */
	nbytes = 1;

	chipSelectSPI(cs, SELECT);
	switch(port)
	{
		case PVT_SPI:
			status = QMSPI1_Read(&qmspi_write_cfg, (void*)&temp_buffer, nbytes);
			break;
		default: //SHD_SPI and INT_SPI
			status = QMSPI0_Read(&qmspi_write_cfg, (void*)&temp_buffer, nbytes);
			// no break
	}
	chipSelectSPI(cs, DESELECT);
	/* ---------------------------------------------------------------------- */

	/* Transmit full-duplex: 0xF0,0xD0 */
	/* ---------------------------------------------------------------------- */
	MPU_API_qmspi_init(qmode, QMSPI_FREQ_12M, 0 ,port);
	qmspi_write_cfg.command = 0xF0;
	qmspi_write_cfg.address_enable = false; /* No address for this command */
	/* qmspi_write_cfg.address_32_bit_en = dont care */
	/* qmspi_write_cfg.address = dont care */
	/* qmspi_write_cfg.num_of_dummy_byte = dont care */
	qmspi_write_cfg.qmspi_ifc_mode = SINGLE_BIT_SPI; /* For Winbond devices */
	nbytes = 0;

	chipSelectSPI(cs, SELECT);
	switch(port)
	{
		case PVT_SPI:
			status = QMSPI1_Write(&qmspi_write_cfg, (void*)&temp_buffer, nbytes);
			break;
		default: //SHD_SPI and INT_SPI
			status = QMSPI0_Write(&qmspi_write_cfg, (void*)&temp_buffer, nbytes);
			// no break
	}

	qmspi_write_cfg.command = 0xD0;
	qmspi_write_cfg.address_enable = false; /* No address for this command */
	/* qmspi_write_cfg.address_32_bit_en = dont care */
	/* qmspi_write_cfg.address = dont care */
	/* qmspi_write_cfg.num_of_dummy_byte = dont care */
	qmspi_write_cfg.qmspi_ifc_mode = SINGLE_BIT_SPI; /* For Winbond devices */
	nbytes = 0;

	switch(port)
	{
		case PVT_SPI:
			status = QMSPI1_Write(&qmspi_write_cfg, (void*)&temp_buffer, nbytes);
			break;
		default: //SHD_SPI and INT_SPI
			status = QMSPI0_Write(&qmspi_write_cfg, (void*)&temp_buffer, nbytes);
			// no break
	}
	chipSelectSPI(cs, DESELECT);
	/* ---------------------------------------------------------------------- */
	timer_delay_us(200);

	/* Transmit full-duplex: 0x66*/
	/* ---------------------------------------------------------------------- */
	MPU_API_qmspi_init(qmode, QMSPI_FREQ_12M, 0 ,port);

	qmspi_write_cfg.command = 0x66;
	qmspi_write_cfg.address_enable = false; /* No address for this command */
	/* qmspi_write_cfg.address_32_bit_en = dont care */
	/* qmspi_write_cfg.address = dont care */
	/* qmspi_write_cfg.num_of_dummy_byte = dont care */
	qmspi_write_cfg.qmspi_ifc_mode = SINGLE_BIT_SPI; /* For Winbond devices */
	nbytes = 0;

	chipSelectSPI(cs, SELECT);
	switch(port)
	{
		case PVT_SPI:
			status = QMSPI1_Write(&qmspi_write_cfg, (void*)&temp_buffer, nbytes);
			break;
		default: //SHD_SPI and INT_SPI
			status = QMSPI0_Write(&qmspi_write_cfg, (void*)&temp_buffer, nbytes);
			// no break
	}
	chipSelectSPI(cs, DESELECT);
	/* ---------------------------------------------------------------------- */
	timer_delay_us(200);

	/* Transmit full-duplex: 0x66*/
	/* ---------------------------------------------------------------------- */
	MPU_API_qmspi_init(qmode, QMSPI_FREQ_12M, 0 ,port);

	qmspi_write_cfg.command = 0x66;
	qmspi_write_cfg.address_enable = false; /* No address for this command */
	/* qmspi_write_cfg.address_32_bit_en = dont care */
	/* qmspi_write_cfg.address = dont care */
	/* qmspi_write_cfg.num_of_dummy_byte = dont care */
	qmspi_write_cfg.qmspi_ifc_mode = SINGLE_BIT_SPI; /* For Winbond devices */
	nbytes = 0;

	chipSelectSPI(cs, SELECT);
	switch(port)
	{
		case PVT_SPI:
			status = QMSPI1_Write(&qmspi_write_cfg, (void*)&temp_buffer, nbytes);
			break;
		default: //SHD_SPI and INT_SPI
			status = QMSPI0_Write(&qmspi_write_cfg, (void*)&temp_buffer, nbytes);
			// no break
	}
	chipSelectSPI(cs, DESELECT);
	/* ---------------------------------------------------------------------- */
	timer_delay_us(200);
}

/* end of qmspi_api.c */
/**   @}
 */

