#include "h3lis.h"

int h3lis_init(Accel_t* dev) {
	uint8_t whoami = 0;

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->i2c_handle,
			dev->device_addr,
			H3LIS_WHO_AM_I,
			I2C_MEMADD_SIZE_8BIT,
			&whoami, 1, 100);

	if (status != HAL_OK) {
		return 0xFF;   // distinct sentinel so you know it's a comms failure, not a bad whoami value
	}

	if (whoami != H3LIS_WHOAMI_VAL) {
		return whoami;
	}

	// write config to CTRL_REG_1
	uint8_t data_to_send = H3LIS_PM_NORMAL | H3LIS_DR_400HZ | H3LIS_ZEN | H3LIS_YEN | H3LIS_XEN;
	HAL_I2C_Mem_Write(dev->i2c_handle,
			dev->device_addr,
			H3LIS_CTRL_REG1,
			I2C_MEMADD_SIZE_8BIT,
			&data_to_send, 1, 100);

	return whoami;
}


void h3lis_read(Accel_t* dev, AccelData_t* accelData) {
	int8_t accel_x, accel_y, accel_z;
	HAL_I2C_Mem_Read(dev->i2c_handle, dev->device_addr, H3LIS_OUT_X, I2C_MEMADD_SIZE_8BIT, &accel_x, 1, 100);
	HAL_I2C_Mem_Read(dev->i2c_handle, dev->device_addr, H3LIS_OUT_Y, I2C_MEMADD_SIZE_8BIT, &accel_y, 1, 100);
	HAL_I2C_Mem_Read(dev->i2c_handle, dev->device_addr, H3LIS_OUT_Z, I2C_MEMADD_SIZE_8BIT, &accel_z, 1, 100);
	accelData->accel_x = accel_x;
	accelData->accel_y = accel_y;
	accelData->accel_z = accel_z;
}
