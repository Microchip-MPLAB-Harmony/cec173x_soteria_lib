/*****************************************************************************
* © 2015 Microchip Technology Inc. and its subsidiaries.
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
$Revision: #2 $ 
$DateTime: 2023/01/02 04:42:23 $ 
$Author: i64652 $
Last Change: Renamed ecia_init to interrupt_init
******************************************************************************/
/** @file interrupt_api.h
* \brief Interrupt Header File
* \author jvasanth
* 
* This file implements the Interrupt Module Header file  
******************************************************************************/

/** @defgroup Interrupt
 *  @{
 */

#ifndef INTERRUPT_H
#define INTERRUPT_H

#define INTS_Type INTS_INST_Type
#define EC_REG_BANK_Type EcRegBank

// GIRQ IDs for EC Interrupt Aggregator
enum MEC_GIRQ_IDS
{
    MEC_GIRQ08_ID = 0,
    MEC_GIRQ09_ID,
    MEC_GIRQ10_ID,
    MEC_GIRQ11_ID,
    MEC_GIRQ12_ID,
    MEC_GIRQ13_ID,
    MEC_GIRQ14_ID,
    MEC_GIRQ15_ID,
    MEC_GIRQ16_ID,
    MEC_GIRQ17_ID,
    MEC_GIRQ18_ID,
    MEC_GIRQ19_ID,
    MEC_GIRQ20_ID,
    MEC_GIRQ21_ID,
    MEC_GIRQ22_ID,
    MEC_GIRQ23_ID,
    MEC_GIRQ24_ID,
    MEC_GIRQ25_ID,
    MEC_GIRQ26_ID,
    MEC_GIRQ_ID_MAX
};

//Bitmask of GIRQ in ECIA Block Registers
#define MEC_GIRQ08_BITMASK          (1UL << (MEC_GIRQ08_ID + 8))
#define MEC_GIRQ09_BITMASK          (1UL << (MEC_GIRQ09_ID + 8))  
#define MEC_GIRQ10_BITMASK          (1UL << (MEC_GIRQ10_ID + 8))  
#define MEC_GIRQ11_BITMASK          (1UL << (MEC_GIRQ11_ID + 8)) 
#define MEC_GIRQ12_BITMASK          (1UL << (MEC_GIRQ12_ID + 8)) 
#define MEC_GIRQ13_BITMASK          (1UL << (MEC_GIRQ13_ID + 8)) 
#define MEC_GIRQ14_BITMASK          (1UL << (MEC_GIRQ14_ID + 8)) 
#define MEC_GIRQ15_BITMASK          (1UL << (MEC_GIRQ15_ID + 8)) 
#define MEC_GIRQ16_BITMASK          (1UL << (MEC_GIRQ16_ID + 8)) 
#define MEC_GIRQ17_BITMASK          (1UL << (MEC_GIRQ17_ID + 8)) 
#define MEC_GIRQ18_BITMASK          (1UL << (MEC_GIRQ18_ID + 8)) 
#define MEC_GIRQ19_BITMASK          (1UL << (MEC_GIRQ19_ID + 8)) 
#define MEC_GIRQ20_BITMASK          (1UL << (MEC_GIRQ20_ID + 8)) 
#define MEC_GIRQ21_BITMASK          (1UL << (MEC_GIRQ21_ID + 8)) 
#define MEC_GIRQ22_BITMASK          (1UL << (MEC_GIRQ22_ID + 8)) 
#define MEC_GIRQ23_BITMASK          (1UL << (MEC_GIRQ23_ID + 8)) 
#define MEC_GIRQ24_BITMASK          (1UL << (MEC_GIRQ24_ID + 8)) 
#define MEC_GIRQ25_BITMASK          (1UL << (MEC_GIRQ25_ID + 8)) 
#define MEC_GIRQ26_BITMASK          (1UL << (MEC_GIRQ26_ID + 8)) 

#define INTERRUPT_MODE_ALL_AGGREGATED        (0u)
#define INTERRUPT_MODE_DIRECT                (1u)

// Bit map of GIRQs whose sources can be directly connected to the NVIC
// GIRQs 13 - 19, 21, 23, 24-26
#define ECIA_GIRQ_DIRECT_BITMAP     (0x07AFE000ul)

/* Added based on CEC173x */
#define MCHP_ECIA_ALL_MASK 0x05F7FF00ul

#define MCHP_ECIA_AGGR_BITMAP                                                  \
    ((1ul << 8) | (1ul << 9) | (1ul << 10) | (1ul << 11) | (1ul << 12) |       \
     (1ul << 22) | (1ul << 26))

#define MCHP_ECIA_DIRECT_BITMAP                                                \
    ((1ul << 13) | (1ul << 14) | (1ul << 15) | (1ul << 16) | (1ul << 17) |     \
     (1ul << 18) | (1ul << 20) | (1ul << 21) | (1ul << 23) | (1ul << 24))

#define ECIA_NVIC_ID_BITPOS             (0u)
#define ECIA_IA_NVIC_ID_BITPOS          (8u)
#define ECIA_GIRQ_ID_BITPOS             (16u)
#define ECIA_GIRQ_BIT_BITPOS            (24u)

/*
 * EC Interrupt Aggregator and NVIC configuration.
 * Implementation:
 * All NVIC enables and priorities will be set at initialization based
 * upon the supplied direct bitmap and NVIC priority.
 * Peripheral level code will use the ECIA GIRQ and any peripheral interrupt
 * enable/status registers.
 * Reason:
 * NVIC interrupt registers may only be accessed when the Cortex-M4 is in
 * privileged mode.
 */
struct nvic_map {
    uint16_t girq_bitpos;
    uint16_t nvic_direct;
};

struct girq_route {
    uint16_t nvic_aggr;
    uint8_t ndirect;
    uint8_t rsvd1;
    const struct nvic_map *nmap;
};

/*
 * NOTE: Based upon DE GIRQ_mapping.xlsx
 */
static const struct nvic_map girq13_nvic_map[] = {
    {0U, 20U}, {1U, 21U}, {2U, 22U}, {3U, 23U}, {4U, 158U}};
#define GIRQ13_NUM_DIRECT (sizeof(girq13_nvic_map) / sizeof(struct nvic_map))

static const struct nvic_map girq14_nvic_map[] = {
    {0U, 24U}, {1U, 25U}, {2U, 26U}, {3U, 27U}, {4U, 28U},
    {5U, 29U}, {6U, 30U}, {7U, 31U}, {8U, 32U}, {9U, 33U}};
#define GIRQ14_NUM_DIRECT (sizeof(girq14_nvic_map) / sizeof(struct nvic_map))

static const struct nvic_map girq15_nvic_map[] = {{0U, 40U}};
#define GIRQ15_NUM_DIRECT (sizeof(girq15_nvic_map) / sizeof(struct nvic_map))

static const struct nvic_map girq16_nvic_map[] = {
    {0U, 65U}, {2U, 67U}, {3U, 68U}};
#define GIRQ16_NUM_DIRECT (sizeof(girq16_nvic_map) / sizeof(struct nvic_map))

static const struct nvic_map girq17_nvic_map[] = {{13U, 83U}, {14U, 84U}};
#define GIRQ17_NUM_DIRECT (sizeof(girq17_nvic_map) / sizeof(struct nvic_map))

static const struct nvic_map girq18_nvic_map[] = {
    {0U, 90U},   {1U, 91U},   {2U, 92U},   {20U, 146U},
    {21U, 147U}, {22U, 148U}, {23U, 149U}, {24U, 150U},
    {25U, 151U}, {26U, 152U}, {27U, 153U}, {28U, 154U}};
#define GIRQ18_NUM_DIRECT (sizeof(girq18_nvic_map) / sizeof(struct nvic_map))

static const struct nvic_map girq20_nvic_map[] = {
    {0U, 186U}, {1U, 186U}, {2U, 186U}, {3U, 173U}, {8U, 185U}, {9U, 174U}};
#define GIRQ20_NUM_DIRECT (sizeof(girq20_nvic_map) / sizeof(struct nvic_map))

static const struct nvic_map girq21_nvic_map[] = {{2U, 171U}, {24U, 134U}};
#define GIRQ21_NUM_DIRECT (sizeof(girq21_nvic_map) / sizeof(struct nvic_map))

static const struct nvic_map girq23_nvic_map[] = {
    {4U, 140U},  {5U, 141U},  {10U, 111U}, {11U, 181U}, {12U, 182U},
    {13U, 183U}, {14U, 184U}, {16U, 112U}, {17U, 113U}};
#define GIRQ23_NUM_DIRECT (sizeof(girq23_nvic_map) / sizeof(struct nvic_map))

static const struct nvic_map girq24_nvic_map[] = {
    {0U, 188U}, {1U, 189U}, {2U, 190U}, {4U, 191U}, {5U, 192U}, {6U, 193U}};
#define GIRQ24_NUM_DIRECT (sizeof(girq24_nvic_map) / sizeof(struct nvic_map))


static const struct girq_route girq_routing_tbl[MEC_GIRQ_ID_MAX] = {
    {0U, 0U, 0U, NULL}, /* GIRQ08 - GIRQ12 aggregated only */
    {1U, 0U, 0U, NULL},
    {2U, 0U, 0U, NULL},
    {3U, 0U, 0U, NULL},
    {4U, 0U, 0U, NULL},
    {5U, GIRQ13_NUM_DIRECT, 0U, girq13_nvic_map},
    {6U, GIRQ14_NUM_DIRECT, 0U, girq14_nvic_map},
    {7U, GIRQ15_NUM_DIRECT, 0U, girq15_nvic_map},
    {8U, GIRQ16_NUM_DIRECT, 0U, girq16_nvic_map},
    {9U, GIRQ17_NUM_DIRECT, 0U, girq17_nvic_map},
    {10U, GIRQ18_NUM_DIRECT, 0U, girq18_nvic_map},
    {0xffffU, 0U, 0U, NULL}, /* GIRQ19 N/A */
    {12U, GIRQ20_NUM_DIRECT, 0U, girq20_nvic_map},
    {13U, GIRQ21_NUM_DIRECT, 0U, girq21_nvic_map},
    {0xffffU, 0U, 0U, NULL}, /* GIRQ22 clock wake only */
    {14U, GIRQ23_NUM_DIRECT, 0U, girq23_nvic_map},
    {15U, GIRQ24_NUM_DIRECT, 0U, girq24_nvic_map},
    {0xffffU, 0U, 0U, NULL}, /* GIRQ25 N/A */
    {17U, 0U, 0U, NULL}      /* GIRQ26 aggregated only */
};


#define MCHP_FIRST_GIRQ 8u
#define MCHP_LAST_GIRQ 26u

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
void interrupt_device_enable(uint32_t dev_iroute, uint8_t is_aggregated);

/** Disables interrupt for a device
 * @param dev_iroute - source IROUTING information 
 * @param is_aggregated - 1 [Aggregated]; 0 - [Disaggregated] 
 * @note This function disables and enables global interrupt 
 */
void interrupt_device_disable(uint32_t dev_iroute, uint8_t is_aggregated);

/* ------------------------------------------------------------------------------- */
/*                  ECIA APIs using device IROUTE() as input                       */ 
/* ------------------------------------------------------------------------------- */

/** Clear Source in the ECIA for the device  
 * @param devi - device IROUTING value  
 */
void interrupt_device_ecia_source_clear(const uint32_t dev_iroute);

/** Get the Result bit in the ECIA for the device  
 * @param devi - device IROUTING value  
 * @return 0 if result bit not set; else non-zero value
 */
uint32_t interrupt_device_ecia_result_get(const uint32_t dev_iroute);

/** Disable the specified interrupt in the ECIA for the device
 * @param devi - device IROUTING value
 * @return 0 if result bit not set; else non-zero value
 */
void interrupt_device_ecia_enable_clear(const uint32_t dev_iroute);

/** Enable the specified interrupt in the ECIA for the device
 * @param devi - device IROUTING value
 * @return 0 if result bit not set; else non-zero value
 */
void interrupt_device_ecia_enable_set(const uint32_t dev_iroute);

/**
 * Clear all aggregator GIRQn status registers
 * @param None
 * @return None
 */
void interrupt_device_girqs_source_reset(void);

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
void mchp_privileged_ecia_init(uint32_t direct_bitmap, uint8_t dflt_priority);

#endif // #ifndef INTERRUPT_H
/* end interrupt_api.h */
/**   @}
 */



