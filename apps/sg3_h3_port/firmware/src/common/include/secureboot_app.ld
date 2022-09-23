/* Linker script to configure memory regions. */
/*
 * CEC173x-M linker script
 * Total SRAM is 352K: 288K Code @ 0xD0000 64K Data @ 0x118000.
 * CEC173x allows execution from data SRAM. To help with MPU power of 2
 * alignment restrictions we will lower the boundary from 0x118000 to
 * 0x110000 reducing code space from 288KB to 256KB.
 */
MEMORY
{
    rom (rx)  : ORIGIN = 0x000C8000, LENGTH = 224K              
	ram (rwx) : ORIGIN = 0x00100000, LENGTH = 152K
    LOG_MBX (r): ORIGIN = 0x126600, LENGTH =  6000
}

/* Library configurations */
GROUP(secureboot_app_lib.a secureboot_LIB.a)

/* Variables used by FreeRTOS-MPU.
 * v10.4.1 has increased kernel data size. We must increase protection
 * region to next power of 2 size, 1024.
 */
__privileged_functions_region_size__ 	= 16K;

__shared_data_size__ 					= 2K;
__smb_data0_size 						= 2K;
__smb_data1_size 						= 1K;

__privileged_data0_region_size__		= 4K;
__privileged_data1_region_size__		= 8K;
__privileged_data2_region_size__		= 8K;
__flash_copy_bss_size	 				= 8K;
__spdm_bss0_size 						= 8K;
__spdm_bss1_size 						= 8K;
__sb_core_bss0_size 					= 16K;
__sb_core_bss1_size 					= 16K;
__crypto_bss0_size						= 8K;
__crypto_bss1_size						= 2K;
__spimon_bss0_size 						= 8K;
__i2c_bss_size							= 8K;
__sb_core_bss2_size 					= 4K;
__mctp_bss_size 						= 4K;
__monitor_bss_size						= 4K;
__qmspi_bss_size 						= 4K;
__smb_bss_size 							= 4K;
__led_bss_size							= 2K;
__shared_bss_size__ 					= 2K;
__spimon_bss1_size 						= 1K;
__spdm_bss2_size                        = 2K;
__spimon_bss2_size                      = 1K;
__BR_scratch_reg_0__					= 0x4000FD80;

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
        __isr_vector_size__ = .;
        . = ALIGN(4);
        KEEP(*(.startup))
        . = ALIGN(4);
        *(privileged_functions)
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
        __rom_api_syscalls_start__ = .;
		*(rom_api_sys_calls*)
        __rom_api_syscalls_end__ = .;
        __ahb_api_syscalls_start__ = .;
		*(ahb_api_sys_calls*)
        __ahb_api_syscalls_end__ = .;
        __sram_mbox_api_syscalls_start__ = .;
		*(sram_mbox_api_sys_calls*)
        __sram_mbox_api_syscalls_end__ = .;
        __sram_rlog_api_syscalls_start__ = .;
		*(sram_rlog_api_sys_calls*)
        __sram_rlog_api_syscalls_end__ = .;
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

	/*************** Data sections start **************/		
	
	.shared_data 0x100000 :
    {
        . = ALIGN(4);
        _sdata = .;

        . = ALIGN(4);
        __data_start__ = .;

        . = ALIGN(4);
        __shared_data_start__ = .;
        *(shared_data*)    

		. = ALIGN(4);
        KEEP(*(.jcr*))
        . = ALIGN(4);

        *(.data*)
		*(.dinit*)

        . = __shared_data_start__ + __shared_data_size__;
        __shared_data_end__ = .;
		
    } >ram AT>rom

    .smb_data_0 (__shared_data_end__) :  AT (__etext + (__shared_data_end__ - _sdata))
    {
        . = ALIGN(4);
        __smb_data0_start = .;
        *(smb_data0*)
        . = __smb_data0_start + __smb_data0_size;
        __smb_data0_end = .;

	} >ram

    .smb_data_1 (__smb_data0_end) :  AT (__etext + (__smb_data0_end - _sdata))
    {
        . = ALIGN(4);
        __smb_data1_start = .;
        *(smb_data1*)
        . = __smb_data1_start + __smb_data1_size;
        __smb_data1_end = .;

		/* All data end */
		__data_end__ = .;
	} >ram

	/*************** Data sections end **************/	

	/*************** BSS sections start **************/

	.spimon_bss1 (__data_end__) (NOLOAD) :
	{
        . = ALIGN(4);
		__bss_start__ = .;
        __spimon_bss1_start = .;
        *(spimon_bss1*)
        . = __spimon_bss1_start + __spimon_bss1_size;
        __spimon_bss1_end = .;
	} >ram

	.privileged_bss0 (__spimon_bss1_end) (NOLOAD) :
	{
 		. = ALIGN(1024);
		__privileged_data_start__ = .;
		__privileged_bss0_start__ = __privileged_data_start__;
		*(privileged_bss0*)
		. = __privileged_bss0_start__ + __privileged_data0_region_size__;
		__privileged_bss0_end__ = .;
	}>ram

	.privileged_bss1 (__privileged_bss0_end__) (NOLOAD) :
	{
		__privileged_bss1_start__ = .;
		*(privileged_bss1*)
		. = __privileged_bss1_start__ + __privileged_data1_region_size__;
		__privileged_bss1_end__ = .;
	}>ram

	.privileged_bss2 (__privileged_bss1_end__) (NOLOAD) :
	{
		__privileged_bss2_start__ = .;
		*(privileged_data*)
		*(privileged_bss2*)
		. = __privileged_bss2_start__ + __privileged_data2_region_size__;
		__privileged_bss2_end__ = .;
		__privileged_data_end__ = __privileged_bss2_end__;
	}>ram

	.led_bss (__privileged_data_end__) (NOLOAD) :
	{
		. = ALIGN(4);
		__led_bss_start = .;
        *(led_bss*)
        . = __led_bss_start + __led_bss_size;
        __led_bss_end = .;
	} >ram
	
	.monitor_bss (__led_bss_end) (NOLOAD) :
	{
		. = ALIGN(4);
		__monitor_bss_start = .;
		*(monitor_bss*)
		. = __monitor_bss_start + __monitor_bss_size;
		__monitor_bss_end = .;
	} >ram

    .flash_copy_bss (__monitor_bss_end) (NOLOAD) :
    {
		__flash_copy_bss_start = .;
        *(flash_copy_bss*)
        . = __flash_copy_bss_start + __flash_copy_bss_size;
		__flash_copy_bss_end = .;
    } >ram

	.spdm_bss0 (__flash_copy_bss_end) (NOLOAD) :
	{
		. = ALIGN(4);
		__spdm_bss0_start = .;
        *(spdm_bss0*)
        . = __spdm_bss0_start + __spdm_bss0_size;
        __spdm_bss0_end = .;
	} >ram

	.spdm_bss1 (__spdm_bss0_end) (NOLOAD) :
	{
		. = ALIGN(4);
		__spdm_bss1_start = .;
        *(spdm_bss1*)
        . = __spdm_bss1_start + __spdm_bss1_size;
        __spdm_bss1_end = .;
	} >ram

	.crypto_bss0 (__spdm_bss1_end) (NOLOAD) :
	{
		. = ALIGN(4);
		__crypto_bss0_start = .;
        *(crypto_bss0*)
        . = __crypto_bss0_start + __crypto_bss0_size;
        __crypto_bss0_end = .;

	} >ram

    .sb_core_bss0 (__crypto_bss0_end) (NOLOAD) :
	{
        . = ALIGN(4);
        __sb_core_bss0_start = .;
		*(sb_core_bss0*)
        . = __sb_core_bss0_start + __sb_core_bss0_size;
        __sb_core_bss0_end = .;
	} >ram

	.sb_core_bss2 (__sb_core_bss0_end) (NOLOAD) :
	{
        . = ALIGN(4);
        __sb_core_bss2_start = .;
		*(sb_core_bss2*)
        . = __sb_core_bss2_start + __sb_core_bss2_size;
        __sb_core_bss2_end = .;
	} >ram

	.qmspi_bss (__sb_core_bss2_end) (NOLOAD) :
	{
		. = ALIGN(4);
		__qmspi_bss_start = .;
        *(qmspi_bss*)
        . = __qmspi_bss_start + __qmspi_bss_size;
        __qmspi_bss_end = .;
	} >ram

	.i2c_bss (__qmspi_bss_end) (NOLOAD) :
	{
		. = ALIGN(4);
		__i2c_bss_start = .;
        *(i2c_bss*)
        . = __i2c_bss_start + __i2c_bss_size;
        __i2c_bss_end = .;
	} >ram

	.sb_core_bss1 (__i2c_bss_end) (NOLOAD) :
	{
        . = ALIGN(4);
        __sb_core_bss1_start = .;
		*(sb_core_bss1*)
        . = __sb_core_bss1_start + __sb_core_bss1_size;
        __sb_core_bss1_end = .;
	} >ram

	.spimon_bss0 (__sb_core_bss1_end) (NOLOAD) :
	{
        . = ALIGN(4);
        __spimon_bss0_start = .;
        *(spimon_bss0*)
        . = __spimon_bss0_start + __spimon_bss0_size;
        __spimon_bss0_end = .;
	} >ram

    .mctp_bss (__spimon_bss0_end) (NOLOAD) :
    {
        . = ALIGN(4);
		__mctp_bss_start = .;
        *(mctp_bss*)
        . = __mctp_bss_start + __mctp_bss_size;
		__mctp_bss_end = .;
    } >ram

    .smb_bss (__mctp_bss_end) (NOLOAD) :
	{	
        . = ALIGN(4);
		__smb_bss_start = .;
        *(smb_bss)
        *(smbBufBlock*)
        . = __smb_bss_start + __smb_bss_size;
		__smb_bss_end = .;	
    } >ram

    .spdm_bss2 (__smb_bss_end) (NOLOAD) :
    {
        . = ALIGN(4);
		__spdm_bss2_start = .;
        *(spdm_bss2*)
        . = __spdm_bss2_start + __spdm_bss2_size;
        __spdm_bss2_end = .;	
    }

    .crypto_bss1 (__spdm_bss2_end) (NOLOAD) :
	{
		. = ALIGN(4);
		__crypto_bss1_start = .;
        *(crypto_bss1*)
        . = __crypto_bss1_start + __crypto_bss1_size;
        __crypto_bss1_end = .;

	} >ram

    .shared_bss (__crypto_bss1_end) (NOLOAD) :
    {
        . = ALIGN(4);
		__shared_bss_start__ = .;
        *(shared_bss*)
        . = __shared_bss_start__ + __shared_bss_size__;
        __shared_bss_end__ = .;	
    } >ram

    .spimon_bss2 (__shared_bss_end__) (NOLOAD) :
    {
        . = ALIGN(4);
        __spimon_bss2_start = .;
        *(spimon_bss2*)
        . = __spimon_bss2_start + __spimon_bss2_size;
        __spimon_bss2_end = .;       
    } >ram

    .bss (__spimon_bss2_end) (NOLOAD) :
    {
        . = ALIGN(4);
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } >ram

	/*************** BSS sections end **************/

	.ssp_sym (__BR_scratch_reg_0__):
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

    /* The xc32 linker requires this symbol to be PROVIDEd when the
       stack is allocated by the linker script. */
    PROVIDE(_stack = ABSOLUTE(__StackTop)); 

    .build_info 0x000FFC00 :
    {
      *(build_info*)
    } >rom
    
}
