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
*****************************************************************************/

#ifndef INCLUDE_ROM_API_MPU_H_
#define INCLUDE_ROM_API_MPU_H_

typedef uint32_t (*FP_RU32_V)(void);

void MPU_API_qmspi_init(uint8_t spi_mode, uint8_t freq_div, uint8_t ifc , uint8_t port);
uint8_t MPU_API_qmspi_port_ctrl(uint8_t port_id, uint8_t width_mask, uint8_t enable);
uint8_t MPU_API_qmspi_flash_cmd(uint32_t ntx, uint8_t *ptx, uint32_t nrx, uint8_t *prx, FP_RU32_V ftmout, uint8_t port);
uint8_t MPU_API_qmspi_is_done_status(uint32_t *pstatus, uint8_t port);
uint8_t MPU_API_qmspi_flash_program_dma(uint8_t prog_cmd, uint32_t spi_addr, uint32_t nbytes, uint8_t port);
uint8_t MPU_API_dma_dev_xfr_cfg(uint8_t chan_id, uint32_t dev_id, uint32_t maddr, uint32_t nbytes, uint32_t flags);
uint8_t MPU_API_qmspi_port_drv_slew(uint8_t port_id, uint8_t width_mask, uint8_t drv_slew);
void MPU_API_qmpsi_start(uint16_t ien_mask, uint8_t port);
uint32_t MPU_API_qmspi_flash_read24_dma(uint8_t cmd_id, uint32_t spi_addr, uint32_t nbytes, uint32_t maddr,uint8_t port);
uint32_t MPU_API_qmspi_flash_read_24_32_dma(uint8_t cmd_id, uint32_t spi_addr, uint32_t nbytes, uint32_t maddr, uint8_t port);
uint32_t MPU_API_qmspi_flash_read_24_32_ldma(uint8_t cmd_id, uint32_t spi_addr, uint32_t nbytes, uint32_t maddr, uint8_t port);

#endif /* INCLUDE_ROM_API_MPU_H_ */
