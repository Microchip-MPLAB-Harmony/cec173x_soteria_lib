
#include <stddef.h>
#include <stdint.h>

/*
 * Fault handlers are here for convenience as they use compiler specific syntax
 * and this file is specific to the GCC compiler.
 */
#define ARM_CM4_NUM_STACKED_REGS 8U
#define STACKED_R0_IDX 0U
#define STACKED_R1_IDX 1U
#define STACKED_R2_IDX 2U
#define STACKED_R3_IDX 3U
#define STACKED_R12_IDX 4U
#define STACKED_LR_IDX 5U
#define STACKED_PC_IDX 6U
#define STACKED_PSR_IDX 7U

volatile uint32_t stacked_regs[ARM_CM4_NUM_STACKED_REGS];

void hard_fault_handler(uint32_t *fault_args)
{
    stacked_regs[STACKED_R0_IDX] = ((uint32_t)fault_args[0]);
    stacked_regs[STACKED_R1_IDX] = ((uint32_t)fault_args[1]);
    stacked_regs[STACKED_R2_IDX] = ((uint32_t)fault_args[2]);
    stacked_regs[STACKED_R3_IDX] = ((uint32_t)fault_args[3]);
    stacked_regs[STACKED_R12_IDX] = ((uint32_t)fault_args[4]);
    stacked_regs[STACKED_LR_IDX] = ((uint32_t)fault_args[5]);
    stacked_regs[STACKED_PC_IDX] = ((uint32_t)fault_args[6]);
    stacked_regs[STACKED_PSR_IDX] = ((uint32_t)fault_args[7]);

    /* Inspect stacked_pc to locate the offending instruction. */
    for (;;)
        ;
}
/*-----------------------------------------------------------*/

void HardFault_Handler(void) __attribute__((naked));
void HardFault_Handler(void)
{
    __asm volatile(" tst lr, #4"
                   "\n"
                   " ite eq"
                   "\n"
                   " mrseq r0, msp"
                   "\n"
                   " mrsne r0, psp"
                   "\n"
                   " ldr r1, [r0, #24]"
                   "\n"
                   " ldr r2, handler_address_const"
                   "\n"
                   " bx r2"
                   "\n"
                   " handler_address_const: .word hard_fault_handler\n");
}
/*-----------------------------------------------------------*/

void MemManage_Handler(void) __attribute__((naked));
void MemManage_Handler(void)
{
    __asm volatile(" tst lr, #4"
                   "\n"
                   " ite eq"
                   "\n"
                   " mrseq r0, msp"
                   "\n"
                   " mrsne r0, psp"
                   "\n"
                   " ldr r1, [r0, #24]"
                   "\n"
                   " ldr r2, handler2_address_const"
                   "\n"
                   " bx r2"
                   "\n"
                   " handler2_address_const: .word hard_fault_handler\n");
} /*-----------------------------------------------------------*/
