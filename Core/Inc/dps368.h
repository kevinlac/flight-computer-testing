#ifndef DPS368_H
#define DPS368_H

#include "stm32f4xx_hal.h"   /* change to your family's HAL header if not F4 */
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* Register addresses (Datasheet Table 15 - Register Map)                     */
/* -------------------------------------------------------------------------- */
#define DPS368_PSR_B2       0x00
#define DPS368_TMP_B2       0x03
#define DPS368_PRS_CFG      0x06
#define DPS368_TMP_CFG      0x07
#define DPS368_MEAS_CFG     0x08
#define DPS368_CFG_REG      0x09
#define DPS368_INT_STS      0x0A
#define DPS368_FIFO_STS     0x0B
#define DPS368_RESET        0x0C
#define DPS368_PROD_ID      0x0D
#define DPS368_COEF         0x10   /* 0x10 - 0x21, 18 bytes */
#define DPS368_COEF_SRCE    0x28

#define DPS368_WHOAMI_VAL   0x10   /* reset value of the ID register (REV_ID=1, PROD_ID=0) */

/* I2C 7-bit address (SDO/address pin tied to GND -> 0x76). HAL wants it
 * pre-shifted left by 1 for HAL_I2C_Mem_xxx calls. */
#define DPS368_I2C_ADDR_7BIT   0x76
#define DPS368_I2C_ADDR        (DPS368_I2C_ADDR_7BIT << 1)   /* 0xEC */

/* -------------------------------------------------------------------------- */
/* CFG_REG (0x09) interrupt/shift/FIFO bit positions                          */
/* -------------------------------------------------------------------------- */
#define DPS368_INT_HL           (1 << 7)  /* interrupt active-high */
#define DPS368_INT_FIFO_EN      (1 << 6)
#define DPS368_INT_TMP_EN       (1 << 5)
#define DPS368_INT_PRS_EN       (1 << 4)
#define DPS368_TMP_SHIFT_EN     (1 << 3)
#define DPS368_PRS_SHIFT_EN     (1 << 2)
#define DPS368_FIFO_EN          (1 << 1)
#define DPS368_SPI_3WIRE_EN     (1 << 0)

/* -------------------------------------------------------------------------- */
/* Types                                                                      */
/* -------------------------------------------------------------------------- */
typedef struct {
    I2C_HandleTypeDef *i2c_handle;
    uint16_t            device_addr;   /* pass DPS368_I2C_ADDR here */
} Baro_t;

/* MEAS_CFG[2:0] operating modes */
enum dps368_opmode {
    DPS368_OPMODE_IDLE                 = 0x00,
    DPS368_OPMODE_CMD_PRS              = 0x01,
    DPS368_OPMODE_CMD_TMP              = 0x02,
    DPS368_OPMODE_BACKGROUND_PRS       = 0x05,
    DPS368_OPMODE_BACKGROUND_TMP       = 0x06,
    DPS368_OPMODE_BACKGROUND_PRS_TMP   = 0x07,
};

/* PRS_CFG/TMP_CFG measurement RATE field, bits [6:4] - pre-shifted */
enum dps368_meas_rate {
    DPS368_MEAS_RATE_1    = 0x00,
    DPS368_MEAS_RATE_2    = 0x10,
    DPS368_MEAS_RATE_4    = 0x20,
    DPS368_MEAS_RATE_8    = 0x30,
    DPS368_MEAS_RATE_16   = 0x40,
    DPS368_MEAS_RATE_32   = 0x50,
    DPS368_MEAS_RATE_64   = 0x60,
    DPS368_MEAS_RATE_128  = 0x70,
};

/* PRS_CFG/TMP_CFG oversampling (PRC) field, bits [3:0] - also used
 * directly as the index into the scale_factor[] table in the .c file. */
enum dps368_samp_rate {
    DPS368_SAMP_RATE_1    = 0,
    DPS368_SAMP_RATE_2    = 1,
    DPS368_SAMP_RATE_4    = 2,
    DPS368_SAMP_RATE_8    = 3,
    DPS368_SAMP_RATE_16   = 4,
    DPS368_SAMP_RATE_32   = 5,
    DPS368_SAMP_RATE_64   = 6,
    DPS368_SAMP_RATE_128  = 7,
};

/* -------------------------------------------------------------------------- */
/* API                                                                        */
/* -------------------------------------------------------------------------- */
uint8_t dps368_init(Baro_t *dev);
void    dps368_set_opmode(Baro_t *dev, enum dps368_opmode mode);
void    dps368_config_tmp(Baro_t *dev, enum dps368_meas_rate mr, enum dps368_samp_rate sr);
void    dps368_config_prs(Baro_t *dev, enum dps368_meas_rate mr, enum dps368_samp_rate sr);
void    dps368_config_int(Baro_t *dev, uint8_t int_source);
void    dps368_clear_intflgs(Baro_t *dev);
void    dps368_get_result(Baro_t *dev, int32_t *tmp, int32_t *prs);
float   dps368_get_altitude(int32_t prs);

#endif /* DPS368_H */
