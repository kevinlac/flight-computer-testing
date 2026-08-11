#ifndef H3LIS_H
#define H3LIS_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* Register map */
#define H3LIS_WHO_AM_I        0x0F
#define H3LIS_CTRL_REG1       0x20
#define H3LIS_CTRL_REG2       0x21
#define H3LIS_CTRL_REG3       0x22
#define H3LIS_CTRL_REG4       0x23
#define H3LIS_CTRL_REG5       0x24
#define H3LIS_HP_FILTER_RESET 0x25
#define H3LIS_REFERENCE       0x26
#define H3LIS_STATUS_REG      0x27
#define H3LIS_OUT_X           0x29
#define H3LIS_OUT_Y           0x2B
#define H3LIS_OUT_Z           0x2D
#define H3LIS_INT1_CFG        0x30
#define H3LIS_INT1_SRC        0x31
#define H3LIS_INT1_THS        0x32
#define H3LIS_INT1_DURATION   0x33
#define H3LIS_INT2_CFG        0x34
#define H3LIS_INT2_SRC        0x35
#define H3LIS_INT2_THS        0x36
#define H3LIS_INT2_DURATION   0x37

/* CTRL_REG1 bit fields */
#define H3LIS_PM_POWERDOWN    (0x00 << 5)
#define H3LIS_PM_NORMAL       (0x01 << 5)
#define H3LIS_PM_LP_0HZ5      (0x02 << 5)
#define H3LIS_PM_LP_1HZ       (0x03 << 5)
#define H3LIS_PM_LP_2HZ       (0x04 << 5)
#define H3LIS_PM_LP_5HZ       (0x05 << 5)
#define H3LIS_PM_LP_10HZ      (0x06 << 5)

#define H3LIS_DR_50HZ         (0x00 << 3)
#define H3LIS_DR_100HZ        (0x01 << 3)
#define H3LIS_DR_400HZ        (0x02 << 3)

#define H3LIS_ZEN             (1 << 2)
#define H3LIS_YEN             (1 << 1)
#define H3LIS_XEN             (1 << 0)

/* WHO_AM_I expected value */
#define H3LIS_WHOAMI_VAL      0x32

typedef struct {
    I2C_HandleTypeDef *i2c_handle;
    uint16_t device_addr;
} Accel_t;

typedef struct {
    int8_t accel_x;
    int8_t accel_y;
    int8_t accel_z;
} AccelData_t;

int  h3lis_init(Accel_t *dev);
void h3lis_read(Accel_t *dev, AccelData_t *data);


#endif
