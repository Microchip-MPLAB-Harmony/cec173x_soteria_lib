/****************************************************************************
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

#include <stdio.h>
#include <string.h>
#include "spt_drv.h"
#include "definitions.h"
#include "spt_app.h"
#include "crc8.h"
#include "platform.h"

SPT_BSS_ATTR SPT_DRV_CONTEXT* spt_drv_ctxt[2]={NULL,NULL};

static SPT_BSS_ATTR SPT_DRV_CONTEXT ctxt[2];
static SPT_BSS_ATTR uint8_t spt_rx_buf[SPT_RX_BUF_SIZE];
static SPT_BSS_ATTR uint8_t spt_tx_buf[SPT_TX_BUF_SIZE];
        
static void Create_SPT_Driver_instance(uint8_t channel)
{
    if(channel)
    {
        channel = SPT1;
    }
    else
    {
        channel = SPT0;
    }
       
    if(NULL==spt_drv_ctxt[channel])
    {
        ctxt[channel].spt_stat_handler.write_state = SPT_WRITE_IDLE_STATE;
        ctxt[channel].spt_stat_handler.read_state = SPT_READ_IDLE_STATE;
        ctxt[channel].spt_mem_cfg.write_count = 0;
        ctxt[channel].spt_mem_cfg.read_count = 0;
        ctxt[channel].host_mbx.host2ec_mbx_sts = 0;
        ctxt[channel].ec_mbx.ec2host_mbx_sts = 0;
        ctxt[channel].spt_mem_cfg.tx_buf_ptr = spt_tx_buf;
        ctxt[channel].spt_mem_cfg.rx_buf_ptr = spt_rx_buf;
        ctxt[channel].spt_mem_cfg.rx_buf_max_limit = SPT_RX_BUF_SIZE;
        ctxt[channel].spt_mem_cfg.tx_buf_max_limit = SPT_TX_BUF_SIZE;
        ctxt[channel].appl_info.applRxFuncPtr = NULL;
        ctxt[channel].spt_enable = 1;
        if(SPT0 == channel)
        {
            SPT0_ECToHostMBXWrite(ctxt[channel].ec_mbx.ec2host_mbx_sts);
           // SPT0_CallbackRegister(spt_isr, (uintptr_t)SPT0);
            // Host wite only
            SPT0_MEM0Config((uint32_t)ctxt[channel].spt_mem_cfg.rx_buf_ptr, 
                    ctxt[channel].spt_mem_cfg.rx_buf_max_limit, 0);
            //Host read only
            SPT0_MEM1Config((uint32_t)ctxt[channel].spt_mem_cfg.tx_buf_ptr, 
                    0, ctxt[channel].spt_mem_cfg.tx_buf_max_limit);
        }
        else
        {
            SPT1_ECToHostMBXWrite(ctxt[channel].ec_mbx.ec2host_mbx_sts);
            //SPT1_CallbackRegister(spt_isr, (uintptr_t)SPT1);
            // Host wite only
            SPT1_MEM0Config((uint32_t)ctxt[channel].spt_mem_cfg.rx_buf_ptr, 
                    ctxt[channel].spt_mem_cfg.rx_buf_max_limit, 0);
            //Host read only
            SPT1_MEM1Config((uint32_t)ctxt[channel].spt_mem_cfg.tx_buf_ptr, 
                    0, ctxt[channel].spt_mem_cfg.tx_buf_max_limit);
        }
        spt_drv_ctxt[channel] = &ctxt[channel];
    }
    else
    {
        //Already initialized
    }
}

static void spt_drv_enable(uint8_t channel)
{
    Create_SPT_Driver_instance(channel);
}

static SPT_DRV_CONTEXT* get_spt_drv_instance(uint8_t channel)
{
    if(channel)
    {
        channel = SPT1;
    }
    else
    {
        channel = SPT0;
    }
    return spt_drv_ctxt[channel];
}


uint8_t spt_config_and_enable(uint8_t channel, uint8_t io_mode, uint8_t spt_wait_time,
        uint8_t tar_time, uint8_t pec_enable)
{
    uint8_t ret_val = 1;
    SPT_DRV_CONTEXT* drv_ctxt = get_spt_drv_instance(channel);
    if(NULL == drv_ctxt)
    {
        spt_drv_enable(channel);
    }
    
    drv_ctxt = get_spt_drv_instance(channel);
    if(NULL != drv_ctxt)
    {
        drv_ctxt->spt_io_cfg.io_mode   = io_mode;
        drv_ctxt->spt_io_cfg.tar_time  = tar_time;
        drv_ctxt->spt_io_cfg.wait_time = spt_wait_time;
        drv_ctxt->spt_stat_handler.read_state = SPT_READ_IDLE_STATE;
        drv_ctxt->spt_stat_handler.write_state = SPT_WRITE_IDLE_STATE;
        drv_ctxt->appl_info.pec_enable = pec_enable;
        if(SPT0 == channel)
        {

            ret_val = 0;
            SPT0_Disable();
            SPT0_WaitTimeSet(drv_ctxt->spt_io_cfg.wait_time);
            SPT0_TARTimeSet(drv_ctxt->spt_io_cfg.tar_time);
            if(SPT_IO_QUAD == drv_ctxt->spt_io_cfg.io_mode)
            {
                SPT0_QuadModeEnable();
            }
            else
            {
                SPT0_QuadModeDisable();
            }
            
            SPT0_Enable();
            drv_ctxt->spt_enable = 1;
        }
        else if(SPT1 == channel)
        {
            ret_val = 0;
            SPT1_Disable();
            SPT1_WaitTimeSet(drv_ctxt->spt_io_cfg.wait_time);
            SPT1_TARTimeSet(drv_ctxt->spt_io_cfg.tar_time);
            if(SPT_IO_QUAD == drv_ctxt->spt_io_cfg.io_mode)
            {
                SPT1_QuadModeEnable();
            }
            else
            {
                SPT1_QuadModeDisable();
            }
            
            SPT1_Enable();
            drv_ctxt->spt_enable = 1;
        }
    }
    
    return ret_val;
}

void spt_read_event_handler()
{

    for(uint8_t channel = SPT0; channel < SPT_MAX; ++channel)
    {
        SPT_SPT_STATUS_REG spt_sts_reg_msk;
        spt_sts_reg_msk.spt_sts = 0;
        uint8_t rd_buff_sts;
        SPT_DRV_CONTEXT* spt_drv_ctxt = get_spt_drv_instance(channel);
        if(NULL != spt_drv_ctxt)
        {
            if(spt_drv_ctxt->spt_enable)
            {
                switch(spt_drv_ctxt->spt_stat_handler.read_state)
                {
                    case SPT_READ_IDLE_STATE:
                        if(spt_drv_ctxt->host_mbx.
                                HOST2EC_MBX_SPT_STATUS_BITS.host_wr_done)
                        {
                            spt_drv_ctxt->spt_stat_handler.read_state = 
                                SPT_READ_IN_PROGRESS_STATE;
                            spt_drv_ctxt->host_mbx.
                                HOST2EC_MBX_SPT_STATUS_BITS.host_wr_done = 0;
                            spt_event_trigger();
                        } 
                        break;
                    case SPT_READ_IN_PROGRESS_STATE:
                        // Host write done , Check if Data is received
                        if(SPT0== channel)
                        {
                            spt_drv_ctxt->spt_sts_reg.spt_sts = SPT0_ECStatusRegGet();
                        }
                        else
                        {
                            spt_drv_ctxt->spt_sts_reg.spt_sts = SPT1_ECStatusRegGet();
                        }

                        // wait for memory write busy clear
                        if(!spt_drv_ctxt->spt_sts_reg.SPT_STATUS_REG_BITS.mem_wr_bsy)
                        {
                            if(spt_drv_ctxt->spt_sts_reg.SPT_STATUS_REG_BITS.mem_wr_done)
                            {
                                spt_sts_reg_msk.SPT_STATUS_REG_BITS.mem_wr_done = 1;
                            }

                            if(SPT0 == channel)
                            {
                                SPT0_ECStatusRegClear(spt_sts_reg_msk.spt_sts);
                                spt_drv_ctxt->spt_mem_cfg.read_count = SPT0_RXFIFOByteCountGet();
                            }
                            else
                            {
                                SPT1_ECStatusRegClear(spt_sts_reg_msk.spt_sts);
                                spt_drv_ctxt->spt_mem_cfg.read_count = SPT1_RXFIFOByteCountGet();   
                            }

                            spt_drv_ctxt->spt_stat_handler.read_state = SPT_PROCESS_DATA_READ_STATE;
                        }
                        spt_event_trigger();
                        break;
                    case SPT_PROCESS_DATA_READ_STATE:
                        rd_buff_sts = spt_handle_received_data(channel);
                        if(SPT_APP_BUFFER_DONE == rd_buff_sts)
                        {
                            // Rx processing done, move to next state
                            spt_drv_ctxt->spt_stat_handler.read_state = SPT_READ_DONE_STATE;
                        }
                        else if(SPT_APP_BUFFER_ERROR == rd_buff_sts)
                        {
                            spt_drv_ctxt->spt_stat_handler.read_state = SPT_READ_ENTER_ERROR_STATE;
                        }
                        spt_event_trigger();
                        break;
                    case SPT_READ_DONE_STATE:
                        // Set EC Read Done in mailbox register 
                        spt_drv_ctxt->ec_mbx.SPT_STATUS_BITS_EC2HOST_MBX.ec_rd_done = 1;
                        if(spt_drv_ctxt->host_mbx.
                                HOST2EC_MBX_SPT_STATUS_BITS.host_wr_done)
                        {
                            spt_drv_ctxt->host_mbx.
                                HOST2EC_MBX_SPT_STATUS_BITS.host_wr_done = 0;
                        }
                        spt_drv_ctxt->spt_stat_handler.read_state = SPT_WAIT_READ_DONE_ACK_STATE;
                        if(SPT0 == channel)
                        {
                            SPT0_ECToHostMBXWrite(spt_drv_ctxt->ec_mbx.ec2host_mbx_sts);
                        }
                        else
                        {
                            SPT1_ECToHostMBXWrite(spt_drv_ctxt->ec_mbx.ec2host_mbx_sts);
                        }
                        spt_event_trigger();
                        break;
                    case SPT_WAIT_READ_DONE_ACK_STATE:
                        // Wait for Host write done clear
                        if(spt_drv_ctxt->host_mbx.
                                HOST2EC_MBX_SPT_STATUS_BITS.host_wr_done)
                        {
                           // Clean up buffer and flags move to next state
                           spt_drv_ctxt->spt_stat_handler.read_state = SPT_READ_COMPLETE_STATE;
                           spt_drv_ctxt->host_mbx.
                                HOST2EC_MBX_SPT_STATUS_BITS.host_wr_done = 0;
                           spt_event_trigger();
                        }
                        break;
                    case SPT_READ_COMPLETE_STATE:
                        spt_drv_ctxt->ec_mbx.SPT_STATUS_BITS_EC2HOST_MBX.ec_rd_done = 0;
                        if(SPT0 == channel)
                        {
                            SPT0_ECToHostMBXWrite(spt_drv_ctxt->ec_mbx.ec2host_mbx_sts);
                        }
                        else
                        {
                            SPT1_ECToHostMBXWrite(spt_drv_ctxt->ec_mbx.ec2host_mbx_sts);
                        }
                        spt_drv_ctxt->spt_stat_handler.read_state = SPT_READ_IDLE_STATE;
                        spt_event_trigger();
                        break;
                    case SPT_READ_ENTER_ERROR_STATE:
                        spt_drv_ctxt->ec_mbx.SPT_STATUS_BITS_EC2HOST_MBX.ec_err =1;
                        if(SPT0 == channel)
                        {
                            SPT0_ECToHostMBXWrite(spt_drv_ctxt->ec_mbx.ec2host_mbx_sts);
                        }
                        else
                        {
                            SPT1_ECToHostMBXWrite(spt_drv_ctxt->ec_mbx.ec2host_mbx_sts);
                        }
                        spt_drv_ctxt->spt_stat_handler.read_state = SPT_READ_WAIT_ERROR_STATE;
                        spt_event_trigger();
                        break;
                    case SPT_READ_WAIT_ERROR_STATE:
                        if(spt_drv_ctxt->host_mbx.HOST2EC_MBX_SPT_STATUS_BITS.host_err)
                        {

                            if(spt_drv_ctxt->ec_mbx.SPT_STATUS_BITS_EC2HOST_MBX.ec_rd_done)
                            {
                                spt_drv_ctxt->ec_mbx.SPT_STATUS_BITS_EC2HOST_MBX.ec_rd_done = 0;
                            }

                            spt_drv_ctxt->spt_stat_handler.read_state = SPT_READ_EXIT_ERROR_STATE;
                            spt_event_trigger();
                        }
                        break;
                    case SPT_READ_EXIT_ERROR_STATE:
                        spt_drv_ctxt->host_mbx.HOST2EC_MBX_SPT_STATUS_BITS.host_err = 0;
                        spt_drv_ctxt->ec_mbx.SPT_STATUS_BITS_EC2HOST_MBX.ec_err =0;
                        if(SPT0 == channel)
                        {
                            SPT0_ECToHostMBXWrite(spt_drv_ctxt->ec_mbx.ec2host_mbx_sts);
                        }
                        else
                        {
                            SPT1_ECToHostMBXWrite(spt_drv_ctxt->ec_mbx.ec2host_mbx_sts);
                        }
                        spt_drv_ctxt->spt_stat_handler.read_state = SPT_READ_IDLE_STATE;
                        spt_event_trigger();
                        break;
                    default:
                        break;        
                }
            }
        }
            
    }
}

void spt_write_event_handler()
{
    SPT_SPT_STATUS_REG spt_ec_st;
    spt_ec_st.spt_sts = 0;

    for(uint8_t channel = SPT0; channel < SPT_MAX; ++channel)
    {
        SPT_DRV_CONTEXT* spt_drv_ctxt = get_spt_drv_instance(channel);
        if(NULL != spt_drv_ctxt)
        {
            if(spt_drv_ctxt->spt_enable)
            {
                switch(spt_drv_ctxt->spt_stat_handler.write_state)
                {
                    case SPT_WRITE_IDLE_STATE:
                        if(spt_drv_ctxt->ec_mbx.SPT_STATUS_BITS_EC2HOST_MBX.ec_wr_done)
                        {
                            spt_drv_ctxt->spt_stat_handler.write_state = SPT_WRITE_DONE_STATE;
                            if(SPT0 == channel)
                            {
                                SPT0_ECToHostMBXWrite(spt_drv_ctxt->ec_mbx.ec2host_mbx_sts);
                            }
                            else
                            {
                                SPT1_ECToHostMBXWrite(spt_drv_ctxt->ec_mbx.ec2host_mbx_sts);
                            }
                            spt_event_trigger();
                        }
                        break;

                    case SPT_WRITE_DONE_STATE:
                      
                        if(spt_drv_ctxt->host_mbx.HOST2EC_MBX_SPT_STATUS_BITS.host_rd_done)
                        {
                            spt_drv_ctxt->spt_stat_handler.write_state = SPT_WAIT_WRITE_COMPLETE_STATE;
                            spt_drv_ctxt->host_mbx.HOST2EC_MBX_SPT_STATUS_BITS.host_rd_done = 0;
                            spt_drv_ctxt->ec_mbx.SPT_STATUS_BITS_EC2HOST_MBX.ec_wr_done = 0;

                            if(SPT0 == channel)
                            {
                                SPT0_ECToHostMBXWrite(spt_drv_ctxt->ec_mbx.ec2host_mbx_sts);
                            }
                            else
                            {
                                SPT1_ECToHostMBXWrite(spt_drv_ctxt->ec_mbx.ec2host_mbx_sts);
                            }
                            spt_event_trigger();
                        }
                        break;

                    case SPT_WAIT_WRITE_COMPLETE_STATE:
                        if(SPT0 == channel)
                        {
                            spt_ec_st.spt_sts = SPT0_ECStatusRegGet();
                        }
                        else
                        {
                            spt_ec_st.spt_sts = SPT1_ECStatusRegGet();
                        }
                        if(0 == spt_ec_st.SPT_STATUS_REG_BITS.obf)
                        {
                            spt_drv_ctxt->spt_stat_handler.write_state = SPT_WRITE_COMPLETE_STATE;
                        }
                        spt_event_trigger();
                        break;

                    case SPT_WRITE_COMPLETE_STATE:
                        spt_tx_done_callback(channel, SPT_DRV_APP_SUCCESS_TX, 
                                spt_drv_ctxt->appl_info.applTxFuncPtr, spt_drv_ctxt->appl_info.app_tx_buff);
                        spt_drv_ctxt->spt_stat_handler.write_state = SPT_WRITE_IDLE_STATE;
                        spt_event_trigger();
                        break;

                    default:
                        break;
                }
            }
        }
    }
}

void spt_isr(uint32_t status, uintptr_t context)
{
    if(SPT_MAX <= context)
    {
        return;
    }

    SPT_DRV_CONTEXT* spt_drv_ctxt = get_spt_drv_instance((uint8_t)(context&UINT8_MAX));
    if(NULL == spt_drv_ctxt)
    {
        return;
    }

        
    spt_drv_ctxt->spt_sts_reg.spt_sts = status;
    if(spt_drv_ctxt->spt_sts_reg.SPT_STATUS_REG_BITS.ibf)
    {
        uint32_t host_mbx = 0;
        if(SPT0 == ((uint8_t)(context & UINT8_MAX)))
        {
            host_mbx = SPT0_HostToECMBXRead();
            SPT0_HostToECMBXClr();
        }
        else
        {
            host_mbx = SPT1_HostToECMBXRead();
            SPT1_HostToECMBXClr();
        }
        spt_drv_ctxt->host_mbx.host2ec_mbx_sts |= host_mbx;
        
        spt_raise_interrrupt_event();
    }
 
    
}

uint8_t spt_check_tx_status(uint8_t channel)
{
    SPT_DRV_CONTEXT* spt_drv_ctxt = get_spt_drv_instance(channel);
    uint8_t retval = 0;
    if(NULL != spt_drv_ctxt)
    {
        retval = spt_drv_ctxt->spt_stat_handler.write_state;
        if((retval == SPT_WRITE_IDLE_STATE) && 
                !(spt_drv_ctxt->ec_mbx.SPT_STATUS_BITS_EC2HOST_MBX.ec_wr_done))
        {
            // Available
            retval = 1;
        }
        else
        {
            retval = 0;
        }
    }
    else
    {
        retval = 0;
    }
    return retval;
}

uint8_t spt_handle_received_data(uint8_t channel)
{
    SPT_DRV_CONTEXT* spt_drv_ctxt = get_spt_drv_instance(channel);
    uint8_t retval = SPT_APP_BUFFER_DONE;

    do 
    {
        if(NULL == spt_drv_ctxt)
        {
            break;
        }
        
        SPT_BUFFER_INFO app_buff = {0};
        app_buff.DataLen = spt_drv_ctxt->spt_mem_cfg.read_count;
        app_buff.RxCount = spt_drv_ctxt->spt_mem_cfg.read_count;
        if (spt_drv_ctxt->spt_io_cfg.tar_time < PRECISION_GENERIC(1U)) // Coverity INT34-C
        {
            if(SPT_IO_QUAD == spt_drv_ctxt->spt_io_cfg.io_mode) {
                app_buff.DataLen = app_buff.RxCount - (((BIT_n_MASK(spt_drv_ctxt->spt_io_cfg.tar_time) / QUAD_CYCLES_PER_BYTE) + (spt_drv_ctxt->spt_io_cfg.wait_time)) + FINAL_STATUS_NUM_BYTES);
            } else {
                app_buff.DataLen = app_buff.RxCount - (((BIT_n_MASK(spt_drv_ctxt->spt_io_cfg.tar_time) / SINGLE_CYCLES_PER_BYTE) + (spt_drv_ctxt->spt_io_cfg.wait_time)) + FINAL_STATUS_NUM_BYTES);
            }
        }
        app_buff.TimeStamp= spt_get_current_timestamp();
        app_buff.buffer_ptr = spt_drv_ctxt->spt_mem_cfg.rx_buf_ptr;
        app_buff.channel = channel;

        if(spt_drv_ctxt->appl_info.pec_enable)
        {
            if(app_buff.DataLen > 1)
            {
                uint8_t crc = crc8_init();
                uint8_t calculated_crc = crc8_update(crc, app_buff.buffer_ptr, app_buff.DataLen-1);
                if(calculated_crc == app_buff.buffer_ptr[app_buff.DataLen-1])
                {
                    app_buff.pecFlag = 1;
                }
                else
                {
                    app_buff.pecFlag = 0;
                }
            }
        }
        else
        {
            app_buff.pecFlag = 0;
        }
        
        if(NULL != spt_drv_ctxt->appl_info.applRxFuncPtr)
        {
           retval = spt_drv_ctxt->appl_info.applRxFuncPtr(&app_buff, channel);
        }
    }while(0);

   
    return retval;
}

uint8_t spt_register_app_rx_callback(uint8_t channel, SPT_SLAVE_FUNC_PTR slaveFuncPtr)
{
    SPT_DRV_CONTEXT* spt_drv_ctxt = get_spt_drv_instance(channel);
    uint8_t retval = 1;
    if(NULL != spt_drv_ctxt)
    {
        if(NULL == spt_drv_ctxt->appl_info.applRxFuncPtr)
        {
            spt_drv_ctxt->appl_info.applRxFuncPtr = slaveFuncPtr;
            retval = 0;
        }
        else
        {
            // Application already registered
        }
    }
    
    return retval;
}

uint8_t spt_write(uint8_t channel, uint8_t* buff_ptr, uint16_t writecount, uint8_t pecEnable, TX_FUNC_PTR func_ptr)
{
    SPT_DRV_CONTEXT* spt_drv_ctxt = get_spt_drv_instance(channel);
    uint8_t retval = SPT_DRV_APP_SUCCESS_TX;
    do
    {
        if(!spt_check_tx_status(channel))
        {
            retval = SPT_DRV_APP_TX_BUSY_ERROR;
            break;
        }
        
        if(NULL == spt_drv_ctxt)
        {
            retval = SPT_DRV_APP_ENABLE_ERROR;
            break;
        }
        
        if(NULL == buff_ptr)
        {
            retval = SPT_DRV_APP_TX_BUFF_ERROR;
            break;
        }

        if(pecEnable)
        {
            // add 1byte for CRC
            if((writecount+1) > spt_drv_ctxt->spt_mem_cfg.tx_buf_max_limit)
            {
                retval = SPT_DRV_APP_TX_BUFF_ERROR;
                break;
            }
        }
        else
        {
            if((writecount) > spt_drv_ctxt->spt_mem_cfg.tx_buf_max_limit)
            {
                retval = SPT_DRV_APP_TX_BUFF_ERROR;
                break;
            }
        }
        
        memcpy(spt_drv_ctxt->spt_mem_cfg.tx_buf_ptr, buff_ptr, writecount);
        if(pecEnable)
        {
            //calculate crc 
            uint8_t crc = crc8_init();
            uint8_t calculated_crc = crc8_update(crc, buff_ptr, writecount);
            spt_drv_ctxt->spt_mem_cfg.tx_buf_ptr[writecount] = calculated_crc;
            ++writecount;
        }
        spt_drv_ctxt->ec_mbx.SPT_STATUS_BITS_EC2HOST_MBX.ec_wr_done = 1;
        spt_drv_ctxt->appl_info.applTxFuncPtr = func_ptr;
        spt_drv_ctxt->appl_info.app_tx_buff = buff_ptr;
        
    }while(0);
    
    if(retval != SPT_DRV_APP_SUCCESS_TX)
    {
        if(NULL != spt_drv_ctxt)
        {
            spt_tx_done_callback(channel, retval, 
                    func_ptr,
                    buff_ptr);
        }
        else
        {
            spt_event_trigger();
        }
    }
    else
    {
        spt_event_trigger();
    }
    return retval;
}

void spt_disable(uint8_t channel)
{
    SPT_DRV_CONTEXT* spt_drv_ctxt = get_spt_drv_instance(channel);
    if(SPT0 == channel)
    {
        SPT0_HostToECMBXClr();
        SPT0_ECToHostMBXWrite(0);
        SPT0_Disable();
        
    }
    else
    {
        SPT1_Disable();
        SPT1_ECToHostMBXWrite(0);
        SPT1_HostToECMBXClr();
    }
    
    if(NULL != spt_drv_ctxt)
    {
        spt_drv_ctxt->spt_stat_handler.read_state = SPT_READ_IDLE_STATE;
        spt_drv_ctxt->spt_stat_handler.read_state = SPT_WRITE_IDLE_STATE;
        spt_drv_ctxt->ec_mbx.ec2host_mbx_sts = 0;
        spt_drv_ctxt->host_mbx.host2ec_mbx_sts = 0;
        spt_drv_ctxt->spt_enable = 0;
        spt_drv_ctxt->spt_sts_reg.spt_sts = 0;
        
    }
}