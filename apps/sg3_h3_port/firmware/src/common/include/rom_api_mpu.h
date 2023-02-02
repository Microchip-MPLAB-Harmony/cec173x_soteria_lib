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

#ifndef INCLUDE_ROM_API_MPU_H_
#define INCLUDE_ROM_API_MPU_H_

typedef uint32_t (*FP_RU32_V)(void);

#define MCHP_EXTRA_IN_DESCS 0
#define NUM_PK_INST 1 /* MCHP */

/** A cryptomaster DMA descriptor */
struct mchpdesc
{
    char *addr;
    struct mchpdesc *next;
    uint32_t sz;
    uint32_t dmatag;
};

/** Input and output descriptors and related state for cmdma */
struct mchp_dmaslot
{
    uint32_t cfg;
    uint8_t extramem[26];
    struct mchpdesc indescs[6 + MCHP_EXTRA_IN_DESCS];
    struct mchpdesc outdescs[5];
};

typedef enum bk_ecc_curve
{
    /**
     * \brief Curve NIST-P192
     *
     * \details The elliptic-curve cryptosystem specified by the NIST-P192/secp192r1 domain parameters:
     *
     * - BK curve ID: 0x00
     * - size in bytes of private keys: 24
     * - size in bytes of public keys: 49
     * - size in bytes of ECDSA signatures: 48
     * - size in bytes of ECDH shared secrets: 24
     * - size in bytes of ECIES header: 49
     * - maximal security strength for operations over this curve: 96-bit
     */
    BK_ECC_CURVE_NIST_P192 = 0x00,
    /**
     * \brief Curve NIST-P224
     *
     * \details The elliptic-curve cryptosystem specified by the NIST-P224/secp224r1 domain parameters:
     *
     * - BK curve ID: 0x01
     * - size in bytes of private keys: 28
     * - size in bytes of public keys: 57
     * - size in bytes of ECDSA signatures: 56
     * - size in bytes of ECDH shared secrets: 28
     * - size in bytes of ECIES header: 57
     * - maximal security strength for operations over this curve: 112-bit
     */
    BK_ECC_CURVE_NIST_P224 = 0x01,
    /**
     * \brief Curve NIST-P256
     *
     * \details The elliptic-curve cryptosystem specified by the NIST-P256/secp256r1 domain parameters:
     *
     * - BK curve ID: 0x02
     * - size in bytes of private keys: 32
     * - size in bytes of public keys: 65
     * - size in bytes of ECDSA signatures: 64
     * - size in bytes of ECDH shared secrets: 32
     * - size in bytes of ECIES header: 65
     * - maximal security strength for operations over this curve: 128-bit
     */
    BK_ECC_CURVE_NIST_P256 = 0x02,
    /**
     * \brief Curve NIST-P384
     *
     * \details The elliptic-curve cryptosystem specified by the NIST-P384/secp384r1 domain parameters:
     *
     * - BK curve ID: 0x03
     * - size in bytes of private keys: 48
     * - size in bytes of public keys: 97
     * - size in bytes of ECDSA signatures: 96
     * - size in bytes of ECDH shared secrets: 48
     * - size in bytes of ECIES header: 97
     * - maximal security strength for operations over this curve: 192-bit
     */
    BK_ECC_CURVE_NIST_P384 = 0x03,
    /**
     * \brief Curve NIST-P521
     *
     * \details The elliptic-curve cryptosystem specified by the NIST-P521/secp521r1 domain parameters:
     *
     * - BK curve ID: 0x04
     * - size in bytes of private keys: 66
     * - size in bytes of public keys: 133
     * - size in bytes of ECDSA signatures: 132
     * - size in bytes of ECDH shared secrets: 66
     * - size in bytes of ECIES header: 133
     * - maximal security strength for operations over this curve: 260-bit
     */
    BK_ECC_CURVE_NIST_P521 = 0x04,
} bk_ecc_curve_t;

/** Description of the (hardware) accelerator capabilities and features.*/
struct mchp_pk_capabilities
{
    /** Maximum pending requests at any time. */
    int maxpending;
    /** Maximum operand size for prime field operands. 0 when not supported. */
    int max_gfp_opsz;
    /** Maximum operand size for elliptic curve operands. 0 when not supported. */
    int max_ecc_opsz;
    /** Maximum operand size for binary field operands. 0 when not supported. */
    int max_gfb_opsz;
};

/** DMA controller
 *
 * For internal use only. Don't access directly.
 */
struct mchp_dmactl
{
    struct mchp_regs *regs;
    struct mchpdesc *mapped_in;
    struct mchpdesc *mapped_out;
    struct mchp_dmaslot dmamem;
};

struct mchp_regs
{
    char *base;
};

struct mchp_pk_accel
{
    struct mchp_regs regs;
    char *cryptoram;
    int op_size;
    const struct mchp_pk_cmd_def *cmd;
    struct mchp_pk_cnx *cnx;
    const char *outputops[4];
    void *userctxt;
#ifdef MCHP_BAREMETAL_EMUL_UIO
    int fd;
#endif
};


typedef struct mchp_pk_accel mchp_pk_accel;

struct mchphashstate
{
    char *m;
    size_t sz;
};

struct mchphash
{
    struct mchpdesc *d;
    struct mchphashstate *h;
    const struct hashalgo *algo;
    uint32_t cntindescs;
    struct mchp_dmactl dma;
    uint32_t feedsz;
};

/**
*  \name mchp_ecdsa_generate_td
*
*  \param [in] curve structure
*  \param [in] private key,d
*  \param [in] random number, k
*  \param [in] hash digest, h
*  \param [in] r term, r
*  \param [in] s_term, s
*
*  \return PK Engine Return status code
*
*  \details This API is used to generate ecdsa signature.
*/

struct mchp_pk_cnx
{
    struct mchp_pk_accel instances[NUM_PK_INST];
    struct mchp_pk_capabilities caps;
};

/** Elliptic curve configuration and parameters.
 *
 * To be used only via the functions in ec_curves.h The internal members of the
 * structure should not be accessed directly.
 */
struct mchp_pk_ecurve
{
    uint32_t curveflags;
    int sz;
    const char *params;
    int offset;
    struct mchp_pk_cnx *cnx;
};

struct mchp_buf
{
    size_t sz;
    uint8_t *bytes;
};

typedef struct mchp_buf mchp_op;

typedef uint8_t bk_ecc_private_key_code_t[116];
typedef uint8_t bk_ecc_public_key_code_t[180];
typedef uint8_t bk_key_purpose_t;
typedef bk_key_purpose_t bk_ecc_key_purpose_t;


/**
 * \typedef bk_key_source_id_t
 * \brief Key source identifier
 * \enum bk_key_source_id
 * \brief Enumeration of key sources
 *
 * Enumerates the allowed sources from which a generic, symmetric or elliptic curve private key can be generated for use
 * with cryptographic functions..
 */
typedef enum bk_key_source_id
{
    /**
     * \brief Key derived from PUF
     *
     * \details The key is derived in a direct line from the SRAM PUF's device unique start-up data. Keys derived in this
     * way can always be exactly rederived by calling cryptographic functions with the same arguments in the same
     * cryptographic context, and on the same device.
     */
    BK_KEY_SOURCE_PUF_DERIVED = 0x00,
    /**
     * \brief Randomly generated key
     *
     * \details The key is randomly generated using BK's internal cryptographically secure random number generator which is
     *  seeded by entropy coming from the device noise. Keys generated in this way cannot be rederived by cryptographic
     *  functions.
     */
    BK_KEY_SOURCE_RNG_DERIVED = 0x01,
    /**
     * \brief Key provided by user
     *
     * \details The key is an external value which is provided by the calling application to the bk_create_private_key()
     * function.
     */
    BK_KEY_SOURCE_USER_PROVIDED = 0x02,
} bk_key_source_id_t;

/**
 * \typedef bk_ecc_key_source_t
 * \brief Defines the allowed sources from which an elliptic curve private key can be generated for use with bk_create_private_key().
 */
typedef bk_key_source_id_t bk_ecc_key_source_t;

uint32_t MPU_API_rom_ver(void);

// efuse related APIs
uint8_t MPU_API_efuse_byte_read(uint16_t byte_idx, uint8_t *out_data);
uint8_t MPU_API_efuse_byte_write(uint16_t byte_idx, uint8_t in_data);
bool MPU_API_efuse_read_lock(uint8_t region);
bool MPU_API_efuse_write_lock(uint8_t region);
bool MPU_API_check_efuse_read_lock(uint16_t byte_index);

// QMSPI related APIs
void MPU_API_qmspi_init(uint8_t spi_mode, uint8_t freq_div, uint8_t ifc , uint8_t port);
uint8_t MPU_API_qmspi_port_ctrl(uint8_t port_id, uint8_t width_mask, uint8_t enable);
uint8_t MPU_API_qmspi_flash_cmd(uint32_t ntx, uint8_t *ptx, uint32_t nrx, uint8_t *prx, FP_RU32_V ftmout, uint8_t port);
uint8_t MPU_API_qmspi_is_done_status(uint32_t *pstatus, uint8_t port);
uint8_t MPU_API_qmspi_is_done_status_ISR(uint32_t *pstatus, uint8_t port);
uint8_t MPU_API_qmspi_flash_program_dma(uint8_t prog_cmd, uint32_t spi_addr, uint32_t nbytes, uint8_t port);
uint8_t MPU_API_dma_dev_xfr_cfg(uint8_t chan_id, uint32_t dev_id, uint32_t maddr, uint32_t nbytes, uint32_t flags);
uint8_t MPU_API_qmspi_port_drv_slew(uint8_t port_id, uint8_t width_mask, uint8_t drv_slew);
void MPU_API_qmpsi_start(uint16_t ien_mask, uint8_t port);
uint32_t MPU_API_qmspi_flash_read24_dma(uint8_t cmd_id, uint32_t spi_addr, uint32_t nbytes, uint32_t maddr,uint8_t port);
uint32_t MPU_API_qmspi_flash_read_24_32_dma(uint8_t cmd_id, uint32_t spi_addr, uint32_t nbytes, uint32_t maddr, uint8_t port);
uint32_t MPU_API_qmspi_flash_read_24_32_ldma(uint8_t cmd_id, uint32_t spi_addr, uint32_t nbytes, uint32_t maddr, uint8_t port);

// Cryptography related APIs
int MPU_API_mchp_hash_create_sha1(struct mchphash *c);
int MPU_API_mchp_hash_create_sha224(struct mchphash *c);
int MPU_API_mchp_hash_create_sha256(struct mchphash *c);
int MPU_API_mchp_hash_create_sha384(struct mchphash *c);
int MPU_API_mchp_hash_create_sha512(struct mchphash *c);
int MPU_API_mchp_hash_feed(struct mchphash *c, const char *msg, size_t sz);
int MPU_API_mchp_hash_digest(struct mchphash *c, char *digest);
int MPU_API_mchp_hash_wait(struct mchphash *c);
struct mchp_pk_ecurve MPU_API_mchp_pk_get_curve_nistp384(struct mchp_pk_cnx *cnx);
struct mchp_pk_ecurve MPU_API_mchp_pk_get_curve_nistp256(struct mchp_pk_cnx *cnx);
struct mchp_pk_cnx *MPU_API_mchp_pk_init(uint32_t *cnxmem, size_t cnxmsz);
int MPU_API_mchp_ecdsa_verify(const struct mchp_pk_ecurve *curve, const mchp_op *qx, \
                              const mchp_op *qy, const mchp_op *r, const mchp_op *s, const mchp_op *h);
bool MPU_API_sb_otp_validate(uint8_t otpmap_rev, uint8_t fw_rev);
struct mchp_pk_dreq MPU_API_mchp_async_ecdsa_generate_go(const struct mchp_pk_ecurve *curve,
        const mchp_op *d,
        const mchp_op *k,
        const mchp_op *h,
        mchp_op *r,
        mchp_op *s);
int MPU_API_bk_ecdsa_sign(const bk_ecc_private_key_code_t *const priv_key_code,
                          const bool deterministic,
                          const uint8_t *message,
                          const uint32_t message_length,
                          const bool message_is_hash,
                          uint8_t *const signature,
                          uint16_t *const signature_length);
int MPU_API_bk_create_private_key(const bk_ecc_curve_t curve, 
        const bk_ecc_key_purpose_t purpose_flags, 
        const uint8_t * const usage_context, 
        const uint32_t usage_context_length, 
        const bk_ecc_key_source_t key_source, 
        const uint8_t * const private_key, 
        bk_ecc_private_key_code_t * const private_key_code);
int MPU_API_bk_compute_public_from_private_key(const bk_ecc_private_key_code_t * const private_key_code, 
                    bk_ecc_public_key_code_t * const public_key_code);
int MPU_API_bk_ecdsa_sign_data(const bk_ecc_private_key_code_t * const priv_key_code,
        const bool deterministic,
        const uint8_t *message,
        const uint32_t message_length,
        const bool message_is_hash,
        uint8_t *const signature,
        uint16_t *const signature_length);
int MPU_API_bk_ecdsa_verify(const bk_ecc_public_key_code_t * const public_key_code, 
        const uint8_t * const message, 
        const uint32_t message_length, 
        const bool message_is_hash, 
        const uint8_t * const signature, 
        const uint16_t signature_length);
int MPU_API_mchp_pk_wait(mchp_pk_accel *req);
int8_t MPU_API_mchp_ndrng_init(bool enable_conditioning,
                            uint32_t fifo_wake_thresh,
                            uint32_t ring_pwr_down_delay,
                            uint32_t nb_cond,
                            uint32_t rng_clk_div,
                            uint32_t rng_init_wait);
void MPU_API_mchp_ndrng_enable(bool enable);
void MPU_API_mchp_ndrng_soft_reset(void);
void MPU_API_mchp_crypt_sleep_control(bool sleep_en);
void MPU_API_mchp_ndrng_intr_en(uint8_t intr_bitmap, uint8_t enable);
int8_t MPU_API_mchp_ndrng_read_bytes(uint8_t *dest, int nbytes);
void MPU_API_mchp_async_ecdsa_generate_end(struct mchp_pk_accel *req,
        mchp_op *r,
        mchp_op *s);
void MPU_API_mchp_hash_init_state(struct mchphash *c,
                                  struct mchphashstate *h,
                                  char *dmamem);
int MPU_API_mchp_hash_resume_state(struct mchphash *c,
                                   struct mchphashstate *h);

int MPU_API_mchp_hash_save_state(struct mchphash *c);

#endif /* INCLUDE_ROM_API_MPU_H_ */
