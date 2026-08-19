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

    uint8_t whoami = SPI_ReadRegister(dev, IMU_WHO_AM_I_REG);
    if (whoami != IMU_WHOAMI_VAL) {
        return whoami;
    }

    uint8_t accelPower = 0b01011000; /* powers up accelerometer, ODR 208Hz, first-stage digital filtering output */
    uint8_t gyroPower   = 0b01010000; /* powers up gyroscope, ODR 208Hz, full-scale 250dps */

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

    dev->gyro[index] = 8.75f * gyrodata; /* 250dps sensitivity */
}

void ReadAcceleration(IMU *dev, uint8_t addressHi, uint8_t addressLo, uint8_t index)
{
    uint8_t dataHi = SPI_ReadRegister(dev, addressHi);
    uint8_t dataLo = SPI_ReadRegister(dev, addressLo);

    int16_t acceldata = (int16_t)((dataHi << 8) | dataLo);

    dev->acceleration[index] = 0.122f * acceldata;
}
