#include "dps368.h"
#include <math.h>

#define COEF_SIZE   18
#define RES_SIZE    3

static uint8_t  source;
static int16_t  c0, c1, c01, c11, c20, c21, c30;
static int32_t  c00, c10;
static enum dps368_samp_rate sr_tmp, sr_prs;

static const float scale_factor[] = {
    524288, 1572864, 3670016, 7864320,
    253952, 516096,  1040384, 2088960
};


/* -------------------------------------------------------------------------- */

static HAL_StatusTypeDef dps368_write(Baro_t *dev, uint8_t addr, uint8_t data)
{
    return HAL_I2C_Mem_Write(dev->i2c_handle, dev->device_addr,
            addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

static HAL_StatusTypeDef dps368_read(Baro_t *dev, uint8_t addr, uint8_t *out)
{
    return HAL_I2C_Mem_Read(dev->i2c_handle, dev->device_addr,
            addr, I2C_MEMADD_SIZE_8BIT, out, 1, 100);
}

static HAL_StatusTypeDef dps368_read_bytes(Baro_t *dev, uint8_t addr, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(dev->i2c_handle, dev->device_addr,
            addr, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}


/* -------------------------------------------------------------------------- */

static int32_t sign_extend24(uint32_t val)
{
    return ((int32_t)(val << 8)) >> 8;
}

static int32_t sign_extend20(uint32_t val)
{
    if (val & (1 << 19)) val |= 0xFFF00000;
    return (int32_t)val;
}

static int16_t sign_extend12(uint16_t val)
{
    if (val & (1 << 11)) val |= 0xF000;
    return (int16_t)val;
}

static void read_coefs(Baro_t *dev)
{
    uint8_t b[COEF_SIZE];
    dps368_read_bytes(dev, DPS368_COEF, b, COEF_SIZE); /* status ignored here; init() already verified comms via WHOAMI */

    c0  = sign_extend12(((uint16_t)b[0] << 4)           | (b[1] >> 4));
    c1  = sign_extend12(((uint16_t)(b[1] & 0x0F) << 8)  | b[2]);

    c00 = sign_extend20(((uint32_t)b[3] << 12) | ((uint32_t)b[4] << 4) | (b[5] >> 4));
    c10 = sign_extend20(((uint32_t)(b[5] & 0x0F) << 16) | ((uint32_t)b[6] << 8) | b[7]);

    c01 = (int16_t)(((uint16_t)b[8]  << 8) | b[9]);
    c11 = (int16_t)(((uint16_t)b[10] << 8) | b[11]);
    c20 = (int16_t)(((uint16_t)b[12] << 8) | b[13]);
    c21 = (int16_t)(((uint16_t)b[14] << 8) | b[15]);
    c30 = (int16_t)(((uint16_t)b[16] << 8) | b[17]);
}


/* -------------------------------------------------------------------------- */

uint8_t dps368_init(Baro_t *dev)
{
    uint8_t whoami = 0;
    HAL_StatusTypeDef hal_status = dps368_read(dev, DPS368_PROD_ID, &whoami);

    if (hal_status != HAL_OK) {
        return 0xFF;   /* sentinel: I2C comms failure, not a real WHOAMI mismatch */
    }
    if (whoami != DPS368_WHOAMI_VAL) {
        return whoami;
    }

    dps368_write(dev, DPS368_RESET, 0x09);
    HAL_Delay(50);

    uint8_t meas_cfg;
    do {
        dps368_read(dev, DPS368_MEAS_CFG, &meas_cfg);
    } while ((meas_cfg & 0xC0) != 0xC0);

    uint8_t coef_srce;
    dps368_read(dev, DPS368_COEF_SRCE, &coef_srce);
    source = coef_srce & 0x80;
    read_coefs(dev);

    dps368_set_opmode(dev, DPS368_OPMODE_IDLE);

    return whoami;
}

void dps368_set_opmode(Baro_t *dev, enum dps368_opmode mode)
{
    dps368_write(dev, DPS368_MEAS_CFG, mode);
}

void dps368_config_tmp(Baro_t *dev, enum dps368_meas_rate mr, enum dps368_samp_rate sr)
{
    dps368_write(dev, DPS368_TMP_CFG, source | mr | sr);
    sr_tmp = sr;
}

void dps368_config_prs(Baro_t *dev, enum dps368_meas_rate mr, enum dps368_samp_rate sr)
{
    dps368_write(dev, DPS368_PRS_CFG, mr | sr);
    sr_prs = sr;
}

void dps368_config_int(Baro_t *dev, uint8_t int_source)
{
    uint8_t reg = 0;
    if (sr_tmp >= DPS368_SAMP_RATE_16) reg |= (1 << 3);
    if (sr_prs >= DPS368_SAMP_RATE_16) reg |= (1 << 2);
    reg |= int_source;
    dps368_write(dev, DPS368_CFG_REG, reg);
}

void dps368_clear_intflgs(Baro_t *dev)
{
    uint8_t discard;
    dps368_read(dev, DPS368_INT_STS, &discard);
}

void dps368_get_result(Baro_t *dev, int32_t *tmp, int32_t *prs)
{
    uint8_t res[RES_SIZE * 2];
    dps368_read_bytes(dev, 0x00, res, RES_SIZE * 2);

    int32_t p_raw = sign_extend24(((uint32_t)res[0] << 16) | ((uint32_t)res[1] << 8) | res[2]);
    int32_t t_raw = sign_extend24(((uint32_t)res[3] << 16) | ((uint32_t)res[4] << 8) | res[5]);

    float t_sc = t_raw / scale_factor[sr_tmp];
    float p_sc = p_raw / scale_factor[sr_prs];

    *tmp = (int32_t)((c0 * 0.5f + c1 * t_sc) * 100);

    *prs = (int32_t)(c00
        + p_sc * (c10 + p_sc * (c20 + p_sc * c30))
        + t_sc *  c01
        + t_sc * p_sc * (c11 + p_sc * c21));
}

float dps368_get_altitude(int32_t prs)
{
    return 44330.0f * (1.0f - powf((float)prs / 101325.0f, 0.1903f));
}
