#include "w25n.h"

/* ------------------------------------------------------------------------
 * Framing verified against the W25N02KVxxIE datasheet (Rev M, section
 * 8.1.2/8.2), Buffer Read mode (BUF=1, the power-on default):
 *  - Page Data Read / Program Execute / Block Erase: opcode + 24-bit
 *    page address (PA[23:17] ignored - this part only uses PA[16:0]).
 *  - Read Data (03h): opcode + 16-bit column address + 8 dummy bits.
 *  - Load/Random Load Program Data: opcode + 16-bit column address (only
 *    CA[11:0] effective) + data.
 *  - JEDEC ID (9Fh): opcode + 8 dummy bits + mfg byte (EFh) + 2 device
 *    ID bytes (AA22h for the W25N02KV).
 * ------------------------------------------------------------------------ */

static inline void CS_Low(W25N_Handle_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void CS_High(W25N_Handle_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

/* Transmit `header` (opcode + address/dummy bytes), then optionally
 * transmit `tx_data` or receive into `rx_data`, all within one CS pulse.
 * Only one of tx_data/rx_data should be non-NULL. */
static W25N_Status_t W25N_Transfer(W25N_Handle_t *dev,
                                    uint8_t *header, uint16_t header_len,
                                    uint8_t *tx_data, uint8_t *rx_data, uint32_t data_len)
{
    HAL_StatusTypeDef st = HAL_OK;

    CS_Low(dev);

    if (header_len > 0) {
        st = HAL_SPI_Transmit(dev->hspi, header, header_len, dev->timeout);
    }

    if (st == HAL_OK && data_len > 0) {
        if (tx_data != NULL) {
            st = HAL_SPI_Transmit(dev->hspi, tx_data, data_len, dev->timeout);
        } else if (rx_data != NULL) {
            st = HAL_SPI_Receive(dev->hspi, rx_data, data_len, dev->timeout);
        }
    }

    CS_High(dev);

    return (st == HAL_OK) ? W25N_OK : W25N_ERROR;
}

/* ---------------- lifecycle / identification ---------------- */

W25N_Status_t W25N_Reset(W25N_Handle_t *dev)
{
    uint8_t cmd = CMD_DEVICE_RESET;
    W25N_Status_t status = W25N_Transfer(dev, &cmd, 1, NULL, NULL, 0);
    if (status != W25N_OK) return status;

    HAL_Delay(2); /* tRST, generous - datasheet allows up to ~500us */
    return W25N_Wait_Busy(dev, dev->timeout, NULL);
}

W25N_Status_t W25N_Read_JEDEC_ID(W25N_Handle_t *dev, uint8_t *mfg_id, uint16_t *device_id)
{
    uint8_t header[2] = { CMD_READ_JEDEC_ID, 0x00 }; /* opcode + dummy byte */
    uint8_t rx[3] = { 0 };

    W25N_Status_t status = W25N_Transfer(dev, header, 2, NULL, rx, 3);
    if (status != W25N_OK) return status;

    if (mfg_id)    *mfg_id = rx[0];
    if (device_id) *device_id = ((uint16_t)rx[1] << 8) | rx[2];

    return W25N_OK;
}

W25N_Status_t W25N_Verify_ID(W25N_Handle_t *dev)
{
    uint8_t mfg = 0;
    uint16_t device = 0;

    W25N_Status_t status = W25N_Read_JEDEC_ID(dev, &mfg, &device);
    if (status != W25N_OK) return status;

    if (mfg != W25N02KV_MFG_ID || device != W25N02KV_DEVICE_ID) {
        return W25N_ERROR;
    }
    return W25N_OK;
}

/* ---------------- status/config registers ---------------- */

W25N_Status_t W25N_Read_Status_Register(W25N_Handle_t *dev, uint8_t reg_addr, uint8_t *value)
{
    uint8_t header[2] = { CMD_READ_STATUS_REG, reg_addr };
    uint8_t rx = 0;

    W25N_Status_t status = W25N_Transfer(dev, header, 2, NULL, &rx, 1);
    if (status == W25N_OK && value) *value = rx;
    return status;
}

W25N_Status_t W25N_Write_Status_Register(W25N_Handle_t *dev, uint8_t reg_addr, uint8_t value)
{
    uint8_t packet[3] = { CMD_WRITE_STATUS_REG, reg_addr, value };
    return W25N_Transfer(dev, packet, 3, NULL, NULL, 0);
}

/* ---------------- write protection ---------------- */

W25N_Status_t W25N_Write_Enable(W25N_Handle_t *dev)
{
    uint8_t cmd = CMD_WRITE_ENABLE;
    return W25N_Transfer(dev, &cmd, 1, NULL, NULL, 0);
}

W25N_Status_t W25N_Write_Disable(W25N_Handle_t *dev)
{
    uint8_t cmd = CMD_WRITE_DISABLE;
    return W25N_Transfer(dev, &cmd, 1, NULL, NULL, 0);
}

W25N_Status_t W25N_Unlock_All_Blocks(W25N_Handle_t *dev)
{
    /* Writing the protection register itself needs WEL set first, same
     * as any other write - separate from the block-protect bits it's
     * about to clear. */
    W25N_Status_t status = W25N_Write_Enable(dev);
    if (status != W25N_OK) return status;

    /* Clears BP0-3/TB/WP-E/SRP0-1 - i.e. removes all block protection
     * so the array can actually be erased and programmed. */
    return W25N_Write_Status_Register(dev, REG_STATUS_1_PROTECTION, 0x00);
}

/* ---------------- busy polling ----------------
 * Polls SR3 (the "status" register, which contains BUSY/WEL/P_FAIL/
 * E_FAIL/ECC bits) until BUSY clears or poll_timeout_ms elapses.
 * Optionally hands back the final SR3 value so the caller can check
 * P_FAIL/E_FAIL/ECC bits without a second transaction.
 */
W25N_Status_t W25N_Wait_Busy(W25N_Handle_t *dev, uint32_t poll_timeout_ms, uint8_t *out_sr3)
{
    uint32_t start = HAL_GetTick();
    uint8_t sr3 = 0;

    do {
        W25N_Status_t status = W25N_Read_Status_Register(dev, REG_STATUS_3_STATUS, &sr3);
        if (status != W25N_OK) return status;

        if ((sr3 & SR3_BUSY) == 0) {
            if (out_sr3) *out_sr3 = sr3;
            return W25N_OK;
        }
    } while ((HAL_GetTick() - start) < poll_timeout_ms);

    return W25N_TIMEOUT;
}

/* ---------------- low level array <-> buffer primitives ---------------- */

static W25N_Status_t W25N_Check_Page_Addr(uint32_t page_addr)
{
    return (page_addr > W25N_MAX_PAGE_ADDR) ? W25N_ERROR : W25N_OK;
}

W25N_Status_t W25N_Page_Data_Read(W25N_Handle_t *dev, uint32_t page_addr)
{
    if (W25N_Check_Page_Addr(page_addr) != W25N_OK) return W25N_ERROR;

    /* opcode + 24-bit page address (PA[23:17] ignored - only PA[16:0] is
     * valid on this 2Gb part, so byte 2 mostly carries just bit 16). */
    uint8_t header[4] = {
        CMD_PAGE_DATA_READ,
        (uint8_t)((page_addr >> 16) & 0xFF),
        (uint8_t)((page_addr >> 8) & 0xFF),
        (uint8_t)(page_addr & 0xFF)
    };
    return W25N_Transfer(dev, header, 4, NULL, NULL, 0);
}

W25N_Status_t W25N_Read_Data(W25N_Handle_t *dev, uint16_t column_addr, uint8_t *buffer, uint32_t length)
{
    /* opcode + 16-bit column address + 8-bit dummy, then data out */
    uint8_t header[4] = {
        CMD_READ_DATA,
        (uint8_t)(column_addr >> 8),
        (uint8_t)(column_addr & 0xFF),
        0x00
    };
    return W25N_Transfer(dev, header, 4, NULL, buffer, length);
}

W25N_Status_t W25N_Load_Program_Data(W25N_Handle_t *dev, uint16_t column_addr, uint8_t *data, uint32_t length)
{
    /* opcode + 16-bit column address, then data in.
     * Resets the rest of the internal buffer to 0xFF - use this for a
     * fresh page write, use the "random" variant to patch bytes into
     * an already-loaded buffer without clobbering the rest. */
    uint8_t header[3] = {
        CMD_LOAD_PROGRAM_DATA,
        (uint8_t)(column_addr >> 8),
        (uint8_t)(column_addr & 0xFF)
    };
    return W25N_Transfer(dev, header, 3, data, NULL, length);
}

W25N_Status_t W25N_Random_Load_Program_Data(W25N_Handle_t *dev, uint16_t column_addr, uint8_t *data, uint32_t length)
{
    uint8_t header[3] = {
        CMD_RANDOM_LOAD_PROGRAM,
        (uint8_t)(column_addr >> 8),
        (uint8_t)(column_addr & 0xFF)
    };
    return W25N_Transfer(dev, header, 3, data, NULL, length);
}

W25N_Status_t W25N_Program_Execute(W25N_Handle_t *dev, uint32_t page_addr)
{
    if (W25N_Check_Page_Addr(page_addr) != W25N_OK) return W25N_ERROR;

    /* opcode + 24-bit page address - commits the internal buffer to the
     * NAND array at this page. */
    uint8_t header[4] = {
        CMD_PROGRAM_EXECUTE,
        (uint8_t)((page_addr >> 16) & 0xFF),
        (uint8_t)((page_addr >> 8) & 0xFF),
        (uint8_t)(page_addr & 0xFF)
    };
    return W25N_Transfer(dev, header, 4, NULL, NULL, 0);
}

W25N_Status_t W25N_Block_Erase(W25N_Handle_t *dev, uint32_t page_addr)
{
    if (W25N_Check_Page_Addr(page_addr) != W25N_OK) return W25N_ERROR;

    /* Any page address within the target 128KB block works - the chip
     * masks down to the block boundary internally. */
    uint8_t header[4] = {
        CMD_BLOCK_ERASE_128KB,
        (uint8_t)((page_addr >> 16) & 0xFF),
        (uint8_t)((page_addr >> 8) & 0xFF),
        (uint8_t)(page_addr & 0xFF)
    };
    return W25N_Transfer(dev, header, 4, NULL, NULL, 0);
}

/* ---------------- high level helpers ---------------- */

W25N_Status_t W25N_Page_Read(W25N_Handle_t *dev, uint32_t page_addr, uint16_t column_addr,
                              uint8_t *buffer, uint32_t length)
{
    W25N_Status_t status;
    uint8_t sr3 = 0;

    /* 1. array -> internal buffer */
    status = W25N_Page_Data_Read(dev, page_addr);
    if (status != W25N_OK) return status;

    /* 2. wait for the array->buffer transfer to finish, and pick up ECC bits */
    status = W25N_Wait_Busy(dev, dev->timeout, &sr3);
    if (status != W25N_OK) return status;

    if ((sr3 & SR3_ECC1) && !(sr3 & SR3_ECC0)) {
        /* ECC1:ECC0 = 10 -> uncorrectable error, data is not trustworthy */
        return W25N_ECC_FAIL;
    }

    /* 3. internal buffer -> MCU */
    status = W25N_Read_Data(dev, column_addr, buffer, length);
    if (status != W25N_OK) return status;

    if ((sr3 & SR3_ECC1) && (sr3 & SR3_ECC0)) {
        /* ECC1:ECC0 = 11 -> corrected, but bit errors are piling up */
        return W25N_ECC_WARN;
    }

    return W25N_OK;
}

W25N_Status_t W25N_Page_Program(W25N_Handle_t *dev, uint32_t page_addr, uint16_t column_addr,
                                 uint8_t *data, uint32_t length)
{
    W25N_Status_t status;
    uint8_t sr3 = 0;

    status = W25N_Write_Enable(dev);
    if (status != W25N_OK) return status;

    /* 1. MCU -> internal buffer */
    status = W25N_Load_Program_Data(dev, column_addr, data, length);
    if (status != W25N_OK) return status;

    /* 2. internal buffer -> NAND array */
    status = W25N_Program_Execute(dev, page_addr);
    if (status != W25N_OK) return status;

    status = W25N_Wait_Busy(dev, dev->timeout, &sr3);
    if (status != W25N_OK) return status;

    if (sr3 & SR3_P_FAIL) return W25N_PROGRAM_FAIL;
    return W25N_OK;
}

W25N_Status_t W25N_Erase_Block(W25N_Handle_t *dev, uint32_t page_addr)
{
    W25N_Status_t status;
    uint8_t sr3 = 0;

    status = W25N_Write_Enable(dev);
    if (status != W25N_OK) return status;

    status = W25N_Block_Erase(dev, page_addr);
    if (status != W25N_OK) return status;

    status = W25N_Wait_Busy(dev, dev->timeout, &sr3);
    if (status != W25N_OK) return status;

    if (sr3 & SR3_E_FAIL) return W25N_ERASE_FAIL;
    return W25N_OK;
}

/* ---------------- simple sequential logger ---------------- */

W25N_Status_t W25N_Log_Write(W25N_Handle_t *dev, const void *record, uint32_t length)
{
    W25N_Status_t status;

    if (dev->log_next_page > W25N_MAX_PAGE_ADDR) {
        /* Log is full. Not auto-wrapping to page 0, since that would
         * silently overwrite old data - decide deliberately if/how you
         * want to handle this (stop logging, wrap, signal a fault, etc). */
        return W25N_ERROR;
    }

    if ((dev->log_next_page % W25N_PAGES_PER_BLOCK) == 0) {
        status = W25N_Erase_Block(dev, dev->log_next_page);
        if (status != W25N_OK) return status;
    }

    status = W25N_Page_Program(dev, dev->log_next_page, 0, (uint8_t *)record, length);
    if (status == W25N_OK) {
        dev->log_next_page++;
    }
    return status;
}

W25N_Status_t W25N_Log_Read(W25N_Handle_t *dev, uint32_t page_addr, void *record, uint32_t length)
{
    return W25N_Page_Read(dev, page_addr, 0, (uint8_t *)record, length);
}

void W25N_Log_Dump(W25N_Handle_t *dev, uint32_t page_count, void *record_buf, uint32_t length, W25N_Log_Dump_Cb print_fn)
{
    for (uint32_t p = 0; p < page_count; p++) {
        if (W25N_Log_Read(dev, p, record_buf, length) != W25N_OK) {
            continue; /* extend with a status-aware callback variant if per-page error reporting is needed */
        }
        print_fn(p, (const uint8_t *)record_buf, length);
    }
}
