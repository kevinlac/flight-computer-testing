/*
 * IMU.c
 *
 *  Created on: 11 May 2026
 *      Author: Prashan Anjanpalage
 */

#include "IMU.h"

/* LSM6DSOX power/config registers */
#define IMU_CTRL1_XL   0x10   /* accelerometer power/config */
#define IMU_CTRL2_G    0x11   /* gyroscope power/config */
#define IMU_CTRL3_C    0x12   /* common control 3: BDU, IF_INC, etc. */

#define CTRL3_C_BDU    (1 << 6)  /* Block Data Update: L/H output bytes latch together */
#define CTRL3_C_IF_INC (1 << 2)  /* register address auto-increment (already default-on, set explicitly) */

static void SPI_WriteRegister(IMU *dev, uint8_t address, uint8_t data)
{
    uint8_t transmit[2] = { (uint8_t)(address & 0x7F), data }; /* bit 7 = 0 for write */

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(dev->spi_handle, transmit, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

uint8_t IMU_Initialise(IMU *dev, SPI_HandleTypeDef *spi_handle, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    dev->spi_handle = spi_handle;
    dev->cs_port = cs_port;
    dev->cs_pin  = cs_pin;
    dev->acceleration[0] = 0.0f;
    dev->acceleration[1] = 0.0f;
    dev->acceleration[2] = 0.0f;
    dev->gyro[0] = 0.0f;
    dev->gyro[1] = 0.0f;
    dev->gyro[2] = 0.0f;

    /* CS must idle high (deselected) before the first transaction, and the
     * device needs its full turn-on time (Ton = 35 ms max, datasheet Table 4)
     * after VDD is applied before it will respond correctly. If IMU_Initialise
     * runs immediately after HAL_Init()/SystemClock_Config(), VDD may only just
     * have stabilised, so this delay is what was causing WHO_AM_I reads to fail
     * intermittently on cold boot. */
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    HAL_Delay(40);

    uint8_t whoami = SPI_ReadRegister(dev, IMU_WHO_AM_I_REG);
    if (whoami != IMU_WHOAMI_VAL) {
        return whoami;
    }

    /* BDU=1: OUT_x_L/OUT_x_H are frozen together until both bytes of a given
     * axis have been read, so a read landing across the sensor's own internal
     * ODR update boundary can't return a "torn" sample (high byte from one
     * conversion, low byte from the next). This matters here because
     * ReadAcceleration/ReadGyroscope issue the Hi and Lo byte reads as two
     * separate SPI transactions rather than one burst read.
     * IF_INC=1 is the power-on default, set explicitly for clarity. */
    SPI_WriteRegister(dev, IMU_CTRL3_C, CTRL3_C_BDU | CTRL3_C_IF_INC);

    uint8_t accelPower = 0b01011000; /* ODR 208 Hz, FS +/-4g, first-stage digital filtering output */
    uint8_t gyroPower   = 0b01010000; /* ODR 208 Hz, FS 250 dps */

    SPI_WriteRegister(dev, IMU_CTRL1_XL, accelPower);
    SPI_WriteRegister(dev, IMU_CTRL2_G, gyroPower);

    return whoami;
}

/* Sentinel returned when the SPI transfer itself fails (HAL_ERROR/HAL_TIMEOUT/HAL_BUSY),
 * as opposed to a real byte read back from the device (which could legitimately be 0xFF).
 */
#define IMU_HAL_FAIL 0xFE

uint8_t SPI_ReadRegister(IMU *dev, uint8_t address)
{
    uint8_t transmit[2] = { (uint8_t)(READ | address), 0x00 };
    uint8_t receive[2]  = { 0 };

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(dev->spi_handle, transmit, receive, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    if (status != HAL_OK) {
        return IMU_HAL_FAIL;
    }

    return receive[1];
}

void ReadGyroscope(IMU *dev, uint8_t addressHi, uint8_t addressLo, uint8_t index)
{
    uint8_t dataHi = SPI_ReadRegister(dev, addressHi);
    uint8_t dataLo = SPI_ReadRegister(dev, addressLo);

    int16_t gyrodata = (int16_t)((dataHi << 8) | dataLo);

    dev->gyro[index] = 8.75f * gyrodata; /* 250dps sensitivity, output in mdps */
}

void ReadAcceleration(IMU *dev, uint8_t addressHi, uint8_t addressLo, uint8_t index)
{
    uint8_t dataHi = SPI_ReadRegister(dev, addressHi);
    uint8_t dataLo = SPI_ReadRegister(dev, addressLo);

    int16_t acceldata = (int16_t)((dataHi << 8) | dataLo);

    dev->acceleration[index] = 0.122f * acceldata; /* +/-4g sensitivity, output in mg */
}

/* Convenience wrapper: reads all 6 axes in one call so main doesn't have to
 * remember all 12 register addresses / (Hi, Lo, index) triples every loop. */
void ReadAllIMU(IMU *dev)
{
    ReadAcceleration(dev, accelXRegHi, accelXRegLow, 0);
    ReadAcceleration(dev, accelYRegHi, accelYRegLow, 1);
    ReadAcceleration(dev, accelZRegHi, accelZRegLow, 2);

    ReadGyroscope(dev, gyroXRegHi, gyroXRegLow, 0);
    ReadGyroscope(dev, gyroYRegHi, gyroYRegLow, 1);
    ReadGyroscope(dev, gyroZRegHi, gyroZRegLow, 2);
}
