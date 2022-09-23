/*****************************************************************************
* � 2019 Microchip Technology Inc. and its subsidiaries.
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
#include "../common/include/common.h"
#include "platform_serial_flash.h"

/******************************************************************************/
/** platform_serial_flash_read_ldma_nonblocking_no_fill();
* read the content from the SPI interface using DMA interface. 
* Supported SPI Read commands are 
*       0 =  Read data Single Output(0x03)
*       1 =  Fast Read Single Output(0x0B)
*       2 =  Fast Read Dual Output(0x3B)
*       3 =  Fast Read Quad Output(0x6B)
* @param spi_addr - SPI address to start reading from
* @param mem_addr - Memory address to copy data
* @param data_len - Number of bytes to read
* @param mode - Read mode
* @param *qmspi_status - Pointer to store qmspi status value
* @param spi_select - spi select
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t platform_serial_flash_read_ldma_nonblocking_no_fill(uint32_t spi_addr, uint32_t mem_addr, uint32_t data_len, uint8_t mode, uint32_t *qmspi_status,  uint8_t spi_select)
{
	uint32_t noofbytes;
	uint8_t ret_val = SPI_DONE_OK;
	uint8_t port = 0u;

	port = SPI_GET_PORT(spi_select);

    if(INT_SPI == port)
    {
        port = SHD_SPI;
    }

	chipSelectSPI(spi_select, SELECT);

	QMSPI_DESCRIPTOR_XFER_T qmspi_xfer_cfg;
	memset(&qmspi_xfer_cfg, 0x00, sizeof(qmspi_xfer_cfg));

	// 0x03/0x13, 0x0b/0x0c, 0x3b/0x3c, 0x6b/0x6c
	if (SPI_4BYTE_ADDR_REQUIRED(spi_addr, data_len))
	{
		qmspi_xfer_cfg.address_32_bit_en = true;
		switch(mode)
		{
			case 1:
				qmspi_xfer_cfg.command = 0x0C;
				qmspi_xfer_cfg.num_of_dummy_byte = 0x01;
				qmspi_xfer_cfg.qmspi_ifc_mode = SINGLE_BIT_SPI;
				break;
			case 2:
				qmspi_xfer_cfg.command = 0x3C;
				qmspi_xfer_cfg.num_of_dummy_byte = 0x01;
				qmspi_xfer_cfg.qmspi_ifc_mode = DUAL_OUTPUT;
				break;
			case 3:
				qmspi_xfer_cfg.command = 0x6C;
				qmspi_xfer_cfg.num_of_dummy_byte = 0x02;
				qmspi_xfer_cfg.qmspi_ifc_mode = QUAD_OUTPUT;
				break;
			case 0: // fall through
			default:
				qmspi_xfer_cfg.command = 0x13;
				qmspi_xfer_cfg.qmspi_ifc_mode = SINGLE_BIT_SPI;
				// 0 dummy bytes
		}
	}
	else
	{
		switch(mode)
		{
			case 1:
				qmspi_xfer_cfg.command = 0x0B;
				qmspi_xfer_cfg.num_of_dummy_byte = 0x01;
				qmspi_xfer_cfg.qmspi_ifc_mode = SINGLE_BIT_SPI;
				break;
			case 2:
				qmspi_xfer_cfg.command = 0x3B;
				qmspi_xfer_cfg.num_of_dummy_byte = 0x02;
				qmspi_xfer_cfg.qmspi_ifc_mode = DUAL_OUTPUT;
				break;
			case 3:
				qmspi_xfer_cfg.command = 0x6B;
				qmspi_xfer_cfg.num_of_dummy_byte = 0x04;
				qmspi_xfer_cfg.qmspi_ifc_mode = QUAD_OUTPUT;
				break;
			case 0: // fall through
			default:
				qmspi_xfer_cfg.command = 0x03;
				qmspi_xfer_cfg.qmspi_ifc_mode = SINGLE_BIT_SPI;
				// 0 dummy bytes
		}
	}

	qmspi_xfer_cfg.address = spi_addr;
	qmspi_xfer_cfg.ldma_enable = true;
	qmspi_xfer_cfg.ldma_incr_addr_disable = true;
	qmspi_xfer_cfg.ldma_channel_num = QMSPI_LDMA_CHANNEL_0;

	switch(port)
	{
		case PVT_SPI:
			noofbytes = QMSPI1_DMATransferRead(&qmspi_xfer_cfg, (void*)mem_addr, data_len);
			break;
		default: //SHD_SPI and INT_SPI
			noofbytes = QMSPI0_DMATransferRead(&qmspi_xfer_cfg, (void*)mem_addr, data_len);
			break;
	}

	return ret_val;
}

/**   @}                                                                       
 */
