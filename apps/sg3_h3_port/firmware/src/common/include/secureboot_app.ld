/* Linker script to configure memory regions. */
/*
 * CEC173x-M linker script
 * Total SRAM is 352K: 288K Code @ 0xD0000 64K Data @ 0x118000.
 * CEC173x allows execution from data SRAM. To help with MPU power of 2
 * alignment restrictions we will lower the boundary from 0x118000 to
 * 0x110000 reducing code space from 288KB to 256KB.
 */

 /* SPIMON1 SRAM section size : 64K from 0x138000
  * In generated map file, this region will not be seen under
  * "Memory Usage Report", but will be seen under "Memory Configuration" and
  * "Linker script and memory map" sections.
  * Current usage is 32K,  Free space is 32K.
  */
MEMORY
{
    rom (rx)  : ORIGIN = 0x000C8000, LENGTH = 224K
    ram (rwx) : ORIGIN = 0x00100000, LENGTH = 152K
    extd_ram (rwx) : ORIGIN = 0x00138000, LENGTH = 64K
    LOG_MBX (rw): ORIGIN = 0x126600, LENGTH =  6000
}

/* Library configurations - Note required with XC32 v4.10 */
/*GROUP(libgcc.a libc.a libm.a libnosys.a)*/

/* Variables used by FreeRTOS-MPU.
 * v10.4.1 has increased kernel data size. We must increase protection
 * region to next power of 2 size, 1024.
 */
__privileged_functions_region_size__    = 8K;

__shared_data_size__                    = 64;
smb_bss0_size_                          = 2K;
smb_bss1_size_                          = 1K;
__privileged_data3_region_size__        = 2K;
led_bss_size_                           = 1K;
shared_bss_size_                        = 1K;
flash_copy_bss1_size_                   = 1K;
__privileged_data0_region_size__        = 8K;
__privileged_data1_region_size__        = 8K;
__privileged_data2_region_size__        = 16K;
spdm_bss0_size_                         = 16K;
spdm_bss1_size_                         = 8K;
spdm_bss2_size_                         = 16K;
crypto_bss0_size_                       = 8K;
crypto_bss1_size_                       = 2K;
spimon_bss0_size_                       = 8K;
i2c_bss0_size_                          = 4K;
monitor_bss0_size_                      = 1K;
monitor_bss1_size_                      = 1K;
__privileged_data5_region_size__        = 1K;
qmspi_bss_size_                         = 1K;
__privileged_data4_region_size__        = 2K;
spt_bss_size_                           = 2K;
smb_bss2_size_                          = 2K;
mctp_bss0_size_                         = 2K;
sb_core_bss0_size_                      = 16K;
sb_core_bss1_size_                      = 16K;
sb_core_bss2_size_                      = 4K;
flash_copy_bss0_size_                   = 4K;
i2c_bss1_size_                          = 4K;
mctp_bss1_size_                         = 1K;
rpmc_header_data_size_                  = 2K;
BR_scratch_reg_0                        = 0x4000FD80;
build_info_start_addr                   = 0x000FF000;
rpmc_header_start_addr                  = 0x000FF800;


__FLASH_segment_start__ = ORIGIN( rom );
__FLASH_segment_end__ = __FLASH_segment_start__ + LENGTH( rom );

/* data not code */
__SRAM_segment_start__ = ORIGIN( ram );
__SRAM_segment_end__ = __SRAM_segment_start__ + LENGTH(ram);

__privileged_functions_start__ = __FLASH_segment_start__;
__privileged_functions_end__   = __FLASH_segment_start__ + __privileged_functions_region_size__;

/* Linker script to place sections and symbol values. Should be used together
 * with other linker script that defines memory regions FLASH and RAM.
 * It references following symbols, which must be defined in code:
 *   Reset_Handler : Entry of reset handler
 *
 * It defines following symbols, which code can use without definition:
 *   __exidx_start
 *   __exidx_end
 *   __copy_table_start__
 *   __copy_table_end__
 *   __zero_table_start__
 *   __zero_table_end__
 *   __etext
 *   __data_start__
 *   __preinit_array_start
 *   __preinit_array_end
 *   __init_array_start
 *   __init_array_end
 *   __fini_array_start
 *   __fini_array_end
 *   __data_end__
 *   __bss_start__
 *   __bss_end__
 *   __end__
 *   end
 *   __HeapBase
 *   __HeapLimit
 *   __StackLimit
 *   __StackTop
 *   __stack
 */
ENTRY(Reset_Handler)

SECTIONS
{
    /* The startup code and FreeRTOS kernel code are placed into privileged ROM */
    .privileged_functions :
    {
        . = ALIGN(4);
        KEEP(*(.vectors))
        KEEP(*(.vectors*))
        KEEP(*(.isr_vector))
        __isr_vector_size__ = .;
        . = ALIGN(4);
        KEEP(*(.startup))
        . = ALIGN(4);
        *(privileged_functions)
        *(privileged_functions*)
        . = ALIGN(4);
        FILL(0xDEAD);
        /*
         * Ensure that non-privileged code is placed after the region reserved for
         * privileged kernel code.
         */
        /*
         * Note that dot (.) actually refers to the byte offset from the start of
         * the current section (.privileged_functions in this case). As a result,
         * setting dot (.) to a value sets the size of the section.
         */
        . = __privileged_functions_region_size__;
    } >rom

    /* The unprivileged program code and initialization data into ROM */
    .text :
    {
        /* Locate FreeRTOS system calls at beginning of unprivileged region */
        . = ALIGN(4);
        __syscalls_flash_start__ = .;
        *(freertos_system_calls)
        *(freertos_system_calls*)
        __rom_api_syscalls_start__ = .;
        *(rom_api_sys_calls)
        *(rom_api_sys_calls*)
        __rom_api_syscalls_end__ = .;
        __ahb_api_syscalls_start__ = .;
        *(ahb_api_sys_calls)
        *(ahb_api_sys_calls*)
        __ahb_api_syscalls_end__ = .;
        __sram_mbox_api_syscalls_start__ = .;
        *(sram_mbox_api_sys_calls)
        *(sram_mbox_api_sys_calls*)
        __sram_mbox_api_syscalls_end__ = .;
        __sram_rlog_api_syscalls_start__ = .;
        *(sram_rlog_api_sys_calls)
        *(sram_rlog_api_sys_calls*)
        __sram_rlog_api_syscalls_end__ = .;
        __sram_rpmc_api_syscalls_start__ = .;
        *(rpmc_api_sys_calls)
        *(rpmc_api_sys_calls*)
        __sram_rpmc_api_syscalls_end__ = .;
        __syscalls_flash_end__ = .;
        . = ALIGN(4);

        *(.text)
        *(.text*)
        *(.glue_7) /* glue arm to thumb code */
        *(.glue_7t) /* glue thumb to arm code */

        KEEP(*(.init))
        KEEP(*(.fini))

        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(4);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN (__fini_array_end = .);

        /* .ctors */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)

        /* .dtors */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)

        *(.rodata*)

        *(vtable)
        
        *(.gnu.linkonce.t.*)

/*        KEEP(*(.eh_frame*)) */
    } >rom

    .sim_efuse_table :
    {
        . = ALIGN(4);
        __sim_efuse_table_start = .;
        *(sim_efuse_data*)
        __sim_efuse_table_end = .;
    } >rom

    .eh_frame :
    {
        KEEP(*(.eh_frame*))

    } >rom

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } >rom

    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } >rom
    __exidx_end = .;

    
    /* To copy multiple ROM to RAM sections,
     * uncomment .copy.table section and,
     * define __STARTUP_COPY_MULTIPLE in startup_ARMCMx.S */
    /*
    .copy.table :
    {
        . = ALIGN(4);
        __copy_table_start__ = .;
        LONG (__etext)
        LONG (__data_start__)
        LONG (__data_end__ - __data_start__)
        LONG (__etext2)
        LONG (__data2_start__)
        LONG (__data2_end__ - __data2_start__)
        __copy_table_end__ = .;
    } >rom
    */

    /* To clear multiple BSS sections,
     * uncomment .zero.table section and,
     * define __STARTUP_CLEAR_BSS_MULTIPLE in startup_ARMCMx.S */
    /*
    .zero.table :
    {
        . = ALIGN(4);
        __zero_table_start__ = .;
        LONG (__bss_start__)
        LONG (__bss_end__ - __bss_start__)
        LONG (__bss2_start__)
        LONG (__bss2_end__ - __bss2_start__)
        __zero_table_end__ = .;
    } >rom
    */

    /* Location counter can end up 2byte aligned with narrow Thumb code but
     *  __etext is assumed by startup code to be the LMA of a section in RAM
     *  which must be 4byte aligned
     */
    __etext = ALIGN (4);

    /*************** BSS sections start **************/        

    .smb_bss0 (0x100000) (NOLOAD):
    {    
        . = ALIGN(4);
        __bss_start__ = .;
        smb_bss0_start = .;
        *(smb_bss0*)
        . = smb_bss0_start + smb_bss0_size_;
        smb_bss0_end = .;
    } >ram

    .smb_bss1 (smb_bss0_end) (NOLOAD):  
    {
        . = ALIGN(4);
        smb_bss1_start = .;
        *(smb_bss1*)
        . = smb_bss1_start + smb_bss1_size_;
        smb_bss1_end = .;
    } >ram

    .privileged_bss3 (smb_bss1_end) (NOLOAD) :
    {
        . = ALIGN(4);
        __privileged_bss3_start__ = .;
        *(privileged_bss3*)
        . = __privileged_bss3_start__ + __privileged_data3_region_size__;
        __privileged_bss3_end__ = .;
    }>ram

    .led_bss (__privileged_bss3_end__) (NOLOAD) :
    {
        . = ALIGN(4);
        led_bss_start = .;
        *(led_bss*)
        . = led_bss_start + led_bss_size_;
        led_bss_end = .;
    } >ram

    .shared_bss (led_bss_end) (NOLOAD) :
    {
        . = ALIGN(4);
        shared_bss_start = .;
        *(shared_bss*)
        . = shared_bss_start + shared_bss_size_;
        shared_bss_end = .;    
    } >ram

    .flash_copy_bss1 (shared_bss_end) (NOLOAD) :
    {
        . = ALIGN(4);
        flash_copy_bss1_start = .;
        *(flash_copy_bss1*)
        . = flash_copy_bss1_start + flash_copy_bss1_size_;
        flash_copy_bss1_end = .;
    } >ram


    .spdm_bss1 (flash_copy_bss1_end) (NOLOAD) :
    {
        . = ALIGN(4);
        spdm_bss1_start = .;
        *(spdm_bss1_8aligned*)
        *(pldm_bss1_8aligned*)
        *(spdm_bss1_4aligned*)
        *(spdm_bss1*)
        *(pldm_bss1*)
        . = spdm_bss1_start + spdm_bss1_size_;
        spdm_bss1_end = .;
    } >ram

    .spdm_bss0 (spdm_bss1_end) (NOLOAD) :
    {
        . = ALIGN(4);
        spdm_bss0_start = .;
        *(spdm_bss0_1024aligned*)
        *(spdm_bss0_8aligned*)
        *(pldm_bss0_8aligned*)
        *(spdm_bss0*)
        *(pldm_bss0*)
        . = spdm_bss0_start + spdm_bss0_size_;
        spdm_bss0_end = .;
    } >ram


    .crypto_bss0 (spdm_bss0_end) (NOLOAD) :
    {
        . = ALIGN(4);
        crypto_bss0_start = .;
        *(crypto_bss0_256aligned*)
        *(crypto_bss0_8aligned*)
        *(crypto_bss0_4aligned*)
        *(crypto_bss0_aligned*)
        *(crypto_bss0*)
        . = crypto_bss0_start + crypto_bss0_size_;
        crypto_bss0_end = .;
    } >ram



    .privileged_bss0 (crypto_bss0_end) (NOLOAD) :
    {
         . = ALIGN(4);
        __privileged_data_start__ = .;
        __privileged_bss0_start__ = __privileged_data_start__;
        *(privileged_bss0*)
        . = __privileged_bss0_start__ + __privileged_data0_region_size__;
        __privileged_bss0_end__ = .;
    }>ram

    .privileged_bss1 (__privileged_bss0_end__) (NOLOAD) :
    {
        . = ALIGN(4);
        __privileged_bss1_start__ = .;
        *(privileged_bss1*)
        . = __privileged_bss1_start__ + __privileged_data1_region_size__;
        __privileged_bss1_end__ = .;
    }>ram

    .privileged_bss2 (__privileged_bss1_end__) (NOLOAD) :
    {
        . = ALIGN(4);
        __privileged_bss2_start__ = .;
        *(privileged_data*)
        *(privileged_bss2*)
        . = __privileged_bss2_start__ + __privileged_data2_region_size__;
        __privileged_bss2_end__ = .;
        __privileged_data_end__ = __privileged_bss2_end__;
    }>ram

    .monitor_bss0 (__privileged_bss2_end__) (NOLOAD) :
    {
        . = ALIGN(4);
        monitor_bss0_start = .;
        *(monitor_bss_256aligned*)
        *(monitor_bss0)
        . = monitor_bss0_start + monitor_bss0_size_;
        monitor_bss0_end = .;
    } >ram

    .monitor_bss1 (monitor_bss0_end) (NOLOAD) :
    {
        . = ALIGN(4);
        monitor_bss1_start = .;
        *(monitor_bss1*)
        . = monitor_bss1_start + monitor_bss1_size_;
        monitor_bss1_end = .;
    } >ram


    .privileged_bss5 (monitor_bss1_end) (NOLOAD) :
    {
        . = ALIGN(4);
        __privileged_bss5_start__ = .;
        *(privileged_bss5*)
        . = __privileged_bss5_start__ + __privileged_data5_region_size__; 
        __privileged_bss5_end__ = .;
    }>ram

    .qmspi_bss (__privileged_bss5_end__) (NOLOAD) :
    {
        . = ALIGN(4);
        qmspi_bss_start = .;
        *(qmspi_bss_256alinged*)
        *(qmspi_bss*)
        . = qmspi_bss_start + qmspi_bss_size_;
        qmspi_bss_end = .;
    } >ram


    .flash_copy_bss0 (qmspi_bss_end) (NOLOAD) :
    {
        . = ALIGN(4);
        flash_copy_bss0_start = .;
        *(flash_copy_bss_2048align*)
        *(flash_copy_buf)
        *(flash_copy_bss0)
        . = flash_copy_bss0_start + flash_copy_bss0_size_;
        flash_copy_bss0_end = .;
    } >ram

    .spdm_bss2 (flash_copy_bss0_end) (NOLOAD) :
    {
        . = ALIGN(4);
        spdm_bss2_start = .;
        *(spdm_bss2_8aligned*)
        *(pldm_bss2_8aligned*)
        *(pldm_bss2*)
        *(spdm_bss2*)
        . = spdm_bss2_start + spdm_bss2_size_;
        spdm_bss2_end = .;
    } > ram



    .spimon_bss0 (spdm_bss2_end) (NOLOAD) :
    {
        . = ALIGN(4);
        spimon_bss0_start = .;
        *(spimon_bss0_1024aligned*)
        *(spimon_bss0_512aligned*)
        *(spimon_bss0_8aligned*)
        *(spimon_bss0*)
        . = spimon_bss0_start + spimon_bss0_size_;
        spimon_bss0_end = .;
    } >ram



    .privileged_bss4 (spimon_bss0_end) (NOLOAD) :
    {
        . = ALIGN(4);
        __privileged_bss4_start__ = .;
        *(privileged_bss4*)
        . = __privileged_bss4_start__ + __privileged_data4_region_size__;
        __privileged_bss4_end__ = .;
    }>ram


    .spt_bss (__privileged_bss4_end__) (NOLOAD) :
    {
        . = ALIGN(4);
        spt_bss_start = .;
        *(spt_bss_attr_256align*)
        *(spt_bss*)
        . = spt_bss_start + spt_bss_size_; 
        spt_bss_end = .;
    } >ram

    .i2c_bss0 (spt_bss_end) (NOLOAD) :
    {
        . = ALIGN(4);
        i2c_bss0_start = .;
        *(i2c_bss0_2048align*)
        *(i2c_bss0_8align*)
        *(i2c_bss0*)
        . = i2c_bss0_start + i2c_bss0_size_;
        i2c_bss0_end = .;
    } >ram


    .smb_bss2 (i2c_bss0_end) (NOLOAD) :
    {
        . = ALIGN(4);
        smb_bss2_start = .;
        *(smb_bss2_256align)
        *(smb_bss2)
        *(smbBufBlock*)
        . = smb_bss2_start + smb_bss2_size_;
        smb_bss2_end = .;    
    } >ram

    .mctp_bss0 (smb_bss2_end) (NOLOAD) :
    {
        . = ALIGN(4);
        mctp_bss0_start = .;
        *(mctp_bss_512aligned*)
        *(mctp_bss_128aligned*)
        *(mctp_bss_8aligned*)
        *(mctp_bss0)
        *(mctp_bss)
        . = mctp_bss0_start + mctp_bss0_size_;
        mctp_bss0_end = .;
    } >ram


    .i2c_bss1 (mctp_bss0_end) (NOLOAD) :
    {
        . = ALIGN(4);
        i2c_bss1_start = .;
        *(i2c_bss1*)
        . = i2c_bss1_start + i2c_bss1_size_;
        i2c_bss1_end = .;
    }>ram

    .sb_core_bss2 (i2c_bss1_end) (NOLOAD) :
    {
        . = ALIGN(4);
        sb_core_bss2_start = .;
        *(sb_core_bss2_buf)
        *(sb_core_bss2*)
        . = sb_core_bss2_start + sb_core_bss2_size_;
        sb_core_bss2_end = .;
    } >ram


    .crypto_bss1 (sb_core_bss2_end) (NOLOAD) :
    {
        . = ALIGN(4);
        crypto_bss1_start = .;
        *(crypto_bss1*)
        . = crypto_bss1_start + crypto_bss1_size_;
        crypto_bss1_end = .;
    } >ram


    .mctp_bss1 (crypto_bss1_end) (NOLOAD) :
    {
        . = ALIGN(4);
        mctp_bss1_start = .;
        *(mctp_bss1*)
        . = mctp_bss1_start + mctp_bss1_size_;
        mctp_bss1_end = .;
    } >ram

    .bss (mctp_bss1_end) (NOLOAD) :
    {
        . = ALIGN(4);
        *(core_stack)
        *(crypto_stack)
        *(spt_stack)
        *(spdm_stack)
        *(led_stack)
        *(mctp_stack)
        *(spimon_stack)
        *(smb_stack)
        *(monitor_stack)
        *(qmspi_stack)
        *(i2c_stack)
        *(fhcpy_stack)
        *(oem_stack)
        *(.bss*)
        *(COMMON)
        . = ALIGN(64);
        __bss_end__ = .;
    } >ram

    .sb_core_bss0 (0x138000) (NOLOAD) :
    {
        . = ALIGN(4);
        __sram_bss_start__ = .;
        sb_core_bss0_start = .;
        *(sb_core_bss0*)
        . = sb_core_bss0_start + sb_core_bss0_size_;
        sb_core_bss0_end = .;
    } >extd_ram

    .sb_core_bss1 (sb_core_bss0_end) (NOLOAD) :
    {
        . = ALIGN(4);
        sb_core_bss1_start = .;
        *(sb_core_bss1*)
        . = sb_core_bss1_start + sb_core_bss1_size_;
        sb_core_bss1_end = .;
        __sram_bss_end__ = .;
    } >extd_ram
    /*************** BSS sections end **************/

    /*************** Data sections start **************/
    .shared_data (__bss_end__) : 
    {

        . = ALIGN(64);
        __data_start__ = .;

        . = ALIGN(4);
        shared_data_start = .;
        *(shared_data*)    
        . = ALIGN(4);
        KEEP(*(.jcr*))
        . = ALIGN(4);

        *(.data*)
        *(.dinit*)
        
        . = shared_data_start + __shared_data_size__;
        shared_data_end = .;
    
        /* All data end */
        __data_end__ = .;
        
    } >ram AT>rom

    /*************** Data sections end **************/

    .ssp_sym (BR_scratch_reg_0):
    {
        . = ALIGN(4);
        __stack_chk_guard = .;
    }
    
    .heap (COPY) :
    {
        __HeapBase = .;
        __end__ = .;
        PROVIDE(end = .);
        *(.heap*)
        __HeapLimit = .;
    } >ram

    /* The xc32 linker requires this section to be named .stack */
    .stack (COPY):
    {
        PROVIDE(__stack = __StackTop);
        *(.stack*)
    } >ram

    /* Set stack top to end of RAM, and stack limit move down by
     * size of stack_dummy section */
    __StackTop = ORIGIN(ram) + LENGTH(ram);
    __StackLimit = __StackTop - SIZEOF(.stack);

    .rpmc_header (rpmc_header_start_addr):
    {
      rpmc_header_data_start = .;
      KEEP(*(rpmc_header_data*))
      . = rpmc_header_data_start + rpmc_header_data_size_;
      rpmc_header_data_end = .;
    } >rom

    .build_info (build_info_start_addr) :
    {
      KEEP(*(build_info*))
    } >rom
    
}
