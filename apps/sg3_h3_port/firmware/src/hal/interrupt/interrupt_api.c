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

#include "common.h"
#include "peripheral/ecia/plib_ecia.h"
#include "peripheral/nvic/plib_nvic.h"

static void enable_nvic_bitmap(uint32_t bitmap, uint8_t direct);
static inline void nvic_enable_extirq(uint32_t nvic_input_num);
static void nvic_priorities_set(uint8_t new_pri);
static void nvic_enpend_clr(void);

/* ------------------------------------------------------------------------------- */
/*                  NVIC,ECIA Routing Policy for Direct Mode                       */
/* ------------------------------------------------------------------------------- */
/* In Direct Mode, some interrupts could be configured to be used as aggregated.
 * Configuration:
 *      1. Always set ECS Interrupt Direct enable bit.
 *      2. If GIRQn aggregated set Block Enable bit.
 *      3. If GIRQn direct then clear Block Enable bit and enable individual NVIC inputs.
 *  Switching issues:
 *  Aggregate enable/disable requires set/clear single GIRQn bit in GIRQ Block En/Clr registers.
 *  Also requires set/clear of individual NVIC Enables.
 *
 * Note: interrupt_is_girq_direct() internal function uses this policy to detect
 * if any interrupt is configured as direct or aggregated
*/

/** Enables interrupt for a device
 * @param dev_iroute - source IROUTING information
 * @param is_aggregated - 1 [Aggregated]; 0 - [Disaggregated]
 * @note This function disables and enables global interrupt
 */
void interrupt_device_enable(uint32_t dev_iroute, uint8_t is_aggregated)
{
    uint32_t nvic_isave; // Coverity DCL37-C Identifier should not start with is or to
    IRQn_Type nvic_num;

    if (is_aggregated == 1) {
        nvic_num = (IRQn_Type)((dev_iroute >> (ECIA_IA_NVIC_ID_BITPOS)) & UINT8_MAX);
    } else {
        nvic_num = (IRQn_Type)((dev_iroute >> (ECIA_NVIC_ID_BITPOS)) & UINT8_MAX);
    }

    nvic_isave = __get_PRIMASK(); // Coverity DCL37-C Identifier should not start with is or to
    NVIC_INT_Disable();
    NVIC_EnableIRQ(nvic_num);
    ECIA_GIRQSourceEnable((ECIA_INT_SOURCE)dev_iroute);
    __DSB();

    if (!nvic_isave) { // Coverity DCL37-C Identifier should not start with is or to
        NVIC_INT_Enable();
    }
}


/** Disables interrupt for a device
 * @param dev_iroute - source IROUTING information
 * @param is_aggregated - 1 [Aggregated]; 0 - [Disaggregated]
 * @note This function disables and enables global interrupt
 */
void interrupt_device_disable(uint32_t dev_iroute, uint8_t is_aggregated)
{
    uint32_t nvic_isave; // Coverity DCL37-C Identifier should not start with is or to
    IRQn_Type nvic_num;

    nvic_isave = __get_PRIMASK(); // Coverity DCL37-C Identifier should not start with is or to
    NVIC_INT_Disable();

    if (is_aggregated == 1){
        nvic_num = (IRQn_Type)((dev_iroute >> (ECIA_IA_NVIC_ID_BITPOS)) & UINT8_MAX);
    } else {
        nvic_num = (IRQn_Type)((dev_iroute >> (ECIA_NVIC_ID_BITPOS)) & UINT8_MAX);
    }
    NVIC_DisableIRQ(nvic_num);
    ECIA_GIRQSourceDisable((ECIA_INT_SOURCE)dev_iroute);
    __DSB();

    if (!nvic_isave) { // Coverity DCL37-C Identifier should not start with is or to
        NVIC_INT_Enable();
    }
}

/* ------------------------------------------------------------------------------- */
/*                  ECIA APIs using device IROUTE() as input                       */
/* ------------------------------------------------------------------------------- */

/** Clear Source in the ECIA for the device
 * @param devi - device IROUTING value
 */
void interrupt_device_ecia_source_clear(const uint32_t dev_iroute)
{
    ECIA_GIRQSourceClear((ECIA_INT_SOURCE)dev_iroute);
    __DSB();
}

/** Get the Result bit in the ECIA for the device
 * @param devi - device IROUTING value
 * @return 0 if result bit not set; else non-zero value
 */
uint32_t interrupt_device_ecia_result_get(const uint32_t dev_iroute)
{
    uint8_t girq_num;
    uint8_t ia_bit_pos;
    uint32_t retVal;

    girq_num = (uint8_t)(dev_iroute >> (ECIA_GIRQ_ID_BITPOS)) & 0x1Fu;
    ia_bit_pos = (uint8_t)(dev_iroute >> (ECIA_GIRQ_BIT_BITPOS)) & 0x1Fu;

    retVal = ECIA_GIRQResultGet((ECIA_INT_SOURCE)dev_iroute);

    return retVal;
}

/** Disable the specified interrupt in the ECIA for the device
 * @param devi - device IROUTING value
 * @return 0 if result bit not set; else non-zero value
 */
void interrupt_device_ecia_enable_clear(const uint32_t dev_iroute)
{
    uint8_t girq_num;
    uint8_t ia_bit_pos;

    girq_num = (uint8_t)(dev_iroute >> (ECIA_GIRQ_ID_BITPOS)) & 0x1Fu;
    ia_bit_pos = (uint8_t)(dev_iroute >> (ECIA_GIRQ_BIT_BITPOS)) & 0x1Fu;

    ECIA_GIRQSourceDisable((ECIA_INT_SOURCE)dev_iroute);
}

/** Enable the specified interrupt in the ECIA for the device
 * @param devi - device IROUTING value
 * @return 0 if result bit not set; else non-zero value
 */
void interrupt_device_ecia_enable_set(const uint32_t dev_iroute)
{
    uint8_t girq_num;
    uint8_t ia_bit_pos;

    girq_num = (uint8_t)(dev_iroute >> (ECIA_GIRQ_ID_BITPOS)) & 0x1Fu;
    ia_bit_pos = (uint8_t)(dev_iroute >> (ECIA_GIRQ_BIT_BITPOS)) & 0x1Fu;

    ECIA_GIRQSourceEnable((ECIA_INT_SOURCE)dev_iroute);
}

/**
 * mchp_ecia_init - Initial EC Interrupt Aggregator and NVIC
 * external interrupt registers.
 *
 * @param direct_bitmap Bit map of GIRQ's caller wants to use direct NVIC
 * connection.
 *
 * @param dflt_priority - Priority 0=highest to 7=lowest to set all
 * external peripheral NVIC priority to.
 *
 * @note - Touches NVIC register. Caller must be Privileged.
 * Restrictions on GIRQ's:
 * Aggregated only: 8-12 and 24-26
 * Direct or Aggregated: 13-21 and 23
 * No connection to NVIC: GIRQ22 used to wake AHB fabric only.
 * The function parameters will be masked with allowed bitmaps.
 *
 * Sets all NVIC enables for direct and aggregated interrupt
 * sources.
 */
void mchp_privileged_ecia_init(uint32_t direct_bitmap, uint8_t dflt_priority)
{
    uint32_t aggr_bitmap, i;

    NVIC_INT_Disable();

    /*
     * Disconnect all direct capable GIRQ sources from the NVIC
     * allowing us to clear direct NVIC pending bits
     */
    EC_REG_BANK_REGS->EC_REG_BANK_INTR_CTRL &= (~BIT_0_MASK);

    /* disconnect all GIRQ aggregated block outputs from NVIC */
    ECIA_GIRQBlockDisableAll();

    /* clear all ECIA GIRQ individual enables and status(source) bits */
    ECIA_GIRQSourceDisableAll();

    ECIA_GIRQSourceClearAll();

    /* clear all NVIC enables and pending status */
    nvic_enpend_clr();

    /* Set priority*/

    nvic_priorities_set(dflt_priority);

    /* mask out GIRQ's that cannot do direct */
    direct_bitmap &= MCHP_ECIA_DIRECT_BITMAP;

    aggr_bitmap = MCHP_ECIA_ALL_MASK & ~(direct_bitmap);

    /* Route all aggregated GIRQn outputs to NVIC */
    for(i=ECIA_GIRQ_BLOCK_NUM8; i<ECIA_GIRQ_BLOCK_NUM_MAX; i++)
    {
        if(aggr_bitmap & (0x01 << i)) {
            ECIA_GIRQBlockEnable((ECIA_GIRQ_BLOCK_NUM)i);
        }
    }

    enable_nvic_bitmap(aggr_bitmap, 0U);

    /* enable any direct connections? */
    if (direct_bitmap) {
        /* Disconnect aggregated GIRQ output for direct mapped */
    for(i=ECIA_GIRQ_BLOCK_NUM8; i<ECIA_GIRQ_BLOCK_NUM_MAX; i++)
    {
        if(direct_bitmap & (0x01 << i)) {
            ECIA_GIRQBlockDisable((ECIA_GIRQ_BLOCK_NUM)i);
        }
    }
        EC_REG_BANK_REGS->EC_REG_BANK_INTR_CTRL |= BIT_0_MASK;
        enable_nvic_bitmap(direct_bitmap, 1U);
    }
}

static void enable_nvic_bitmap(uint32_t bitmap, uint8_t direct)
{
    uint32_t dbm = bitmap & MCHP_ECIA_ALL_MASK;
    uint32_t bpos = 0U;

    while (dbm) {
        bpos = 31U - __builtin_clz(dbm);

        if ((bpos >= MCHP_FIRST_GIRQ) && (bpos <= MCHP_LAST_GIRQ)) {
            const struct girq_route *pgr =
                &girq_routing_tbl[bpos - MCHP_FIRST_GIRQ];

            if (direct && (pgr->ndirect) && (pgr->nmap)) {
                const struct nvic_map *pm = pgr->nmap;
                for (uint32_t n = 0U; n < pgr->ndirect; n++) {
                    nvic_enable_extirq(pm->nvic_direct);
                    pm++;
                }
            } else {
                nvic_enable_extirq(pgr->nvic_aggr);
            }
        }

        dbm &= ~(1UL << (bpos));
    }
}

/**
 * Clear all aggregator GIRQn status registers
 * @param None
 * @return None
 */
void interrupt_device_girqs_source_reset(void)
{
    ECIA_GIRQSourceClearAll();
}

/*
 * Enable NVIC external input. Input is zero based.
 * !!! Touches NVIC registers. Caller must be Privileged !!!
 */
static inline void nvic_enable_extirq(uint32_t nvic_input_num)
{
    uint32_t reg_ofs = ((nvic_input_num >> 5U) & 0x1FU) << 2U;
    uint32_t bit_pos = nvic_input_num % 32U;

    *(uint32_t *)((&NVIC->ISER[0]) + reg_ofs) |= bit_pos;
}

/** Set NVIC external priorities to specified priority (0 - 7)
 * @param zero-based 3-bit priority value: 0=highest, 7=lowest.
 * @note NVIC highest priority is the value 0, lowest is all 1's.
 * Each external interrupt has an 8-bit register and the priority
 * is left justified in the registers. MECxxx implements 8 priority
 * levels or bits [7:5] in the register. Lowest priority = 0xE0
 */
static void nvic_priorities_set(uint8_t new_pri)
{
    uint16_t i;

    for ( i = 0UL; i < MAX_IRQn; i++ ) {
        NVIC_SetPriority((IRQn_Type)i, new_pri);
    }
}

/** Clear all NVIC external enables and pending bits */
static void nvic_enpend_clr(void)
{
    uint32_t i, m;

    // Clear NVIC enables & pending status
    m = (uint32_t)(MAX_IRQn) >> 5;
    if ( (uint32_t)(MAX_IRQn) & 0x1FUL ) { m++; }

    for ( i = 0UL; i < m ; i++ )
    {
        NVIC->ICER[i] = UINT32_MAX;
        NVIC->ICPR[i] = UINT32_MAX;
    }
}

/* ------------------------------------------------------------------------------- */