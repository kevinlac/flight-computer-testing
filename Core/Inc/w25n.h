#ifndef W25N_H
#define W25N_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/*
 * opcodes
 * see section 8.1.2/8.1.3
 * */
#define CMD_WRITE_ENABLE            0x06
#define CMD_WRITE_DISABLE           0x04
#define CMD_READ_STATUS_REG         0x0F
#define CMD_WRITE_STATUS_REG        0x1F
#define CMD_PAGE_DATA_READ          0x13
#define CMD_READ_DATA               0x03
#define CMD_FAST_READ               0x0B
#define CMD_LOAD_PROGRAM_DATA       0x02
#define CMD_RANDOM_LOAD_PROGRAM     0x84
#define CMD_PROGRAM_EXECUTE         0x10
#define CMD_BLOCK_ERASE_128KB       0xD8
#define CMD_DEVICE_RESET            0xFF
#define CMD_READ_JEDEC_ID           0x9F

/*
 * registers
 * Notes section on page 29
 * */
#define REG_STATUS_1_PROTECTION     0xA0
#define REG_STATUS_2_CONFIGURATION  0xB0
#define REG_STATUS_3_STATUS         0xC0

/*
 * protection register/status register 1
 * section 7.1
 * */
#define SR1_SRP1                    0x01
#define SR1_WP_E                    0x02
#define SR1_TB                      0x04
#define SR1_BP3                     0x08
#define SR1_BP2                     0x10
#define SR1_BP1                     0x20
#define SR1_BP0                     0x40
#define SR1_SRP0                    0x80

/*
 * configuration register/status register 2
 * section 7.2
 */
#define SR2_OTP_L                   0x80
#define SR2_OTP_E                   0x40
#define SR2_SR1_L                   0x20
#define SR2_ECC_E                   0x10
#define SR2_BUF                     0x08
#define SR2_ODS1                    0x04
#define SR2_ODS0                    0x02
#define SR2_H_DIS                   0x01

/*
 * status register 3
 * section 7.3
 */
#define SR3_ECC1                    0x20
#define SR3_ECC0                    0x10
#define SR3_P_FAIL                  0x08
#define SR3_E_FAIL                  0x04
#define SR3_WEL                     0x02
#define SR3_BUSY                    0x01

/* Page geometry (W25N02KV: 2048 data + 128 spare bytes per page, 17-bit page address) */
#define W25N_PAGE_SIZE_BYTES        2048
#define W25N_PAGE_SPARE_BYTES       128
#define W25N_PAGE_TOTAL_BYTES       (W25N_PAGE_SIZE_BYTES + W25N_PAGE_SPARE_BYTES)
#define W25N_PAGES_PER_BLOCK        64
#define W25N_BLOCK_COUNT            2048
#define W25N_MAX_PAGE_ADDR          131071u  /* 17-bit page address, PA[16:0] */

#define W25N02KV_MFG_ID              0xEF
#define W25N02KV_DEVICE_ID           0xAA22

typedef enum {
    W25N_OK       = 0x00,
    W25N_ERROR    = 0x01,
    W25N_BUSY     = 0x02,
    W25N_TIMEOUT  = 0x03,
    W25N_ECC_FAIL = 0x04,  /* Hardware ECC could not correct the read data */
    W25N_ECC_WARN = 0x05,  /* ECC corrected the data, but it's getting weak - consider re-writing the block */
    W25N_PROGRAM_FAIL = 0x06,
    W25N_ERASE_FAIL   = 0x07
} W25N_Status_t;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    uint32_t           timeout;

    /* Sequential logger state (see W25N_Log_Write/Read/Dump below).
     * Zero-initialised automatically for a statically-declared handle;
     * reset to 0 explicitly if you want to start a fresh log after
     * erasing the chip. */
    uint32_t           log_next_page;
} W25N_Handle_t;

/* --- lifecycle / identification --- */
W25N_Status_t W25N_Reset(W25N_Handle_t *dev);
W25N_Status_t W25N_Read_JEDEC_ID(W25N_Handle_t *dev, uint8_t *mfg_id, uint16_t *device_id);

/* Convenience: reads JEDEC ID and confirms it matches W25N02KV
 * (mfg EFh, device AA22h). Returns W25N_OK on match, W25N_ERROR otherwise. */
W25N_Status_t W25N_Verify_ID(W25N_Handle_t *dev);

/* --- status/config registers (generic, any of the 3 register addresses) --- */
W25N_Status_t W25N_Read_Status_Register(W25N_Handle_t *dev, uint8_t reg_addr, uint8_t *value);
W25N_Status_t W25N_Write_Status_Register(W25N_Handle_t *dev, uint8_t reg_addr, uint8_t value);

/* --- write protection --- */
W25N_Status_t W25N_Write_Enable(W25N_Handle_t *dev);
W25N_Status_t W25N_Write_Disable(W25N_Handle_t *dev);

/* These chips ship from the factory with the entire array
 * write-protected via the block-protect bits in the protection register
 * (status register 1) - separate from, and in addition to, the WEL bit
 * that W25N_Write_Enable sets. Call this once after
 * W25N_Reset/W25N_Verify_ID, before any erase/program call, or every
 * erase will fail with W25N_ERASE_FAIL (SR3 E_FAIL bit set). */
W25N_Status_t W25N_Unlock_All_Blocks(W25N_Handle_t *dev);

/* --- busy polling --- */
W25N_Status_t W25N_Wait_Busy(W25N_Handle_t *dev, uint32_t poll_timeout_ms, uint8_t *out_sr3);

/* --- low level array <-> buffer primitives --- */
W25N_Status_t W25N_Page_Data_Read(W25N_Handle_t *dev, uint32_t page_addr);
W25N_Status_t W25N_Read_Data(W25N_Handle_t *dev, uint16_t column_addr, uint8_t *buffer, uint32_t length);
W25N_Status_t W25N_Load_Program_Data(W25N_Handle_t *dev, uint16_t column_addr, uint8_t *data, uint32_t length);
W25N_Status_t W25N_Random_Load_Program_Data(W25N_Handle_t *dev, uint16_t column_addr, uint8_t *data, uint32_t length);
W25N_Status_t W25N_Program_Execute(W25N_Handle_t *dev, uint32_t page_addr);
W25N_Status_t W25N_Block_Erase(W25N_Handle_t *dev, uint32_t page_addr);

/* --- high level helpers (do the full array<->buffer dance + wait + fail checks) --- */
W25N_Status_t W25N_Page_Read(W25N_Handle_t *dev, uint32_t page_addr, uint16_t column_addr, uint8_t *buffer, uint32_t length);
W25N_Status_t W25N_Page_Program(W25N_Handle_t *dev, uint32_t page_addr, uint16_t column_addr, uint8_t *data, uint32_t length);
W25N_Status_t W25N_Erase_Block(W25N_Handle_t *dev, uint32_t page_addr);

/* --- simple sequential logger ---
 * Writes one fixed-size record per page, starting at page 0 and
 * advancing automatically (tracked in dev->log_next_page). Deliberately
 * generic - it just writes/reads whatever buffer you give it, so it
 * works for any record type without this driver needing to know
 * anything about what's actually being logged.
 *
 * NAND cells must be erased (all bits -> 1) before they can be
 * programmed, and erase only works a whole 64-page block at a time, so
 * W25N_Log_Write erases lazily: only the block about to be written to,
 * and only the first time a page within it is used - rather than
 * erasing the entire chip up front (2048 blocks, would take a while).
 */
W25N_Status_t W25N_Log_Write(W25N_Handle_t *dev, const void *record, uint32_t length);
W25N_Status_t W25N_Log_Read(W25N_Handle_t *dev, uint32_t page_addr, void *record, uint32_t length);

/* Callback used by W25N_Log_Dump for each page: page_addr is which page
 * it came from, data/length is the raw record read back. Cast data to
 * your actual record type and format/print it however you like - this
 * keeps the driver itself free of any knowledge of your record layout. */
typedef void (*W25N_Log_Dump_Cb)(uint32_t page_addr, const uint8_t *data, uint32_t length);

/* Reads back pages [0, page_count) and calls print_fn for each one.
 * record_buf must be a caller-supplied buffer at least `length` bytes -
 * this driver never allocates a page-sized buffer itself (2048 bytes
 * would overflow a small embedded stack), so reuse a same-sized
 * variable you already have (e.g. your record struct) as scratch space. */
void W25N_Log_Dump(W25N_Handle_t *dev, uint32_t page_count, void *record_buf, uint32_t length, W25N_Log_Dump_Cb print_fn);

#endif
