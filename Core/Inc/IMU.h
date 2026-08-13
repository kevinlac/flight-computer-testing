/*
 * IMU.h
 *
 *  Created on: 11 May 2026
 *      Author: Prashan Anjanpalage
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define READ 0x80

#define IMU_WHO_AM_I_REG   0x0F
#define IMU_WHOAMI_VAL     0x6C   /* LSM6DSOX WHO_AM_I expected value */

/* Output register map */
#define accelXRegLow 0x28
#define accelXRegHi  0x29
#define accelYRegLow 0x2A
#define accelYRegHi  0x2B
#define accelZRegLow 0x2C
#define accelZRegHi  0x2D
#define gyroXRegLow  0x22
#define gyroXRegHi   0x23
#define gyroYRegLow  0x24
#define gyroYRegHi   0x25
#define gyroZRegLow  0x26
#define gyroZRegHi   0x27

typedef struct {
    SPI_HandleTypeDef *spi_handle;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;

    /* (X, Y, Z) */
    float acceleration[3];
    float gyro[3];
} IMU;

uint8_t IMU_Initialise(IMU *dev, SPI_HandleTypeDef *spi_handle, GPIO_TypeDef *cs_port, uint16_t cs_pin);
uint8_t SPI_ReadRegister(IMU *dev, uint8_t address);
void    ReadGyroscope(IMU *dev, uint8_t addressHi, uint8_t addressLo, uint8_t index);
void    ReadAcceleration(IMU *dev, uint8_t addressHi, uint8_t addressLo, uint8_t index);

#endif /* INC_IMU_H_ */
