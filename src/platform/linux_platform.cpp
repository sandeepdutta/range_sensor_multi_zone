/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <range_sensor_multi_zone/platform.h>

#define VL53L1_MAX_I2C_XFER_SIZE (8*1024-2) // 8KB - 2 bytes for register address

static uint8_t buffer[VL53L1_MAX_I2C_XFER_SIZE + 2];/* GLOBAL I2C comm buff */

uint8_t VL53L5CX_RdByte(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{
    return  VL53L5CX_RdMulti(p_platform, RegisterAdress, p_value, 1);
}

uint8_t VL53L5CX_WrByte(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value)
{
    return  VL53L5CX_WrMulti(p_platform, RegisterAdress, &value, 1);
}

uint8_t VL53L5CX_WrMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
    uint32_t bytes_to_write = size;
    uint32_t offset = 0;
    while (bytes_to_write > 0) {
        uint32_t bytes_to_write_this_time = bytes_to_write;
        if (bytes_to_write_this_time > VL53L1_MAX_I2C_XFER_SIZE) {
            bytes_to_write_this_time = VL53L1_MAX_I2C_XFER_SIZE;
        }
        buffer[0] = RegisterAdress >> 8;
        buffer[1] = RegisterAdress & 0xFF;
        memcpy(&buffer[2], &p_values[offset], bytes_to_write_this_time);
        uint32_t ret = write(p_platform->i2c_hdl, buffer, bytes_to_write_this_time + 2);
        if (ret != (bytes_to_write_this_time + 2)) {
            return -1;
        }
        offset += bytes_to_write_this_time;
        bytes_to_write -= bytes_to_write_this_time;
        RegisterAdress += bytes_to_write_this_time;
    }
	return 0;
}

uint8_t VL53L5CX_RdMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	uint8_t status = 255;

	if ((size + 1) > VL53L1_MAX_I2C_XFER_SIZE)
		return status;
	buffer[0] = RegisterAdress >> 8;
	buffer[1] = RegisterAdress & 0xFF;
    // write register address
    uint32_t ret = write(p_platform->i2c_hdl, buffer, 2);
	if (ret != 2) {
		return status;
	}
	// read data from register
    p_values[0] = RegisterAdress;
	ret = read(p_platform->i2c_hdl, p_values, size);
	if (ret != size) {
		return -1;
	}
	return 0;
}

uint8_t VL53L5CX_Reset_Sensor(
		VL53L5CX_Platform *p_platform)
{
	uint8_t status = 0;

	/* (Optional) Need to be implemented by customer. This function returns 0 if OK */

	/* Set pin LPN to LOW */
	/* Set pin AVDD to LOW */
	/* Set pin VDDIO  to LOW */
	VL53L5CX_WaitMs(p_platform, 100);

	/* Set pin LPN of to HIGH */
	/* Set pin AVDD of to HIGH */
	/* Set pin VDDIO of  to HIGH */
	VL53L5CX_WaitMs(p_platform, 100);

	return status;
}

void VL53L5CX_SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size)
{
	uint32_t i, tmp;

	/* Example of possible implementation using <string.h> */
	for(i = 0; i < size; i = i + 4)
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);

		memcpy(&(buffer[i]), &tmp, 4);
	}
}

uint8_t VL53L5CX_WaitMs(
		VL53L5CX_Platform *p_platform,
		uint32_t TimeMs)
{
    (void)p_platform;
    usleep(TimeMs * 1000);

	return 0;
}

int8_t VL53L5CX_Sensor_Select(VL53L5CX_Platform *p_platform, uint8_t dev, uint8_t i2c_Addr) {

	if (ioctl(p_platform->i2c_hdl, I2C_SLAVE, 0x70) < 0) {
		/* ERROR HANDLING; you can check errno to see what went wrong */
		return -1;
	}
	uint8_t buff = 1<<dev;
	int	ret = write(p_platform->i2c_hdl, &buff, 1);
	if (ret != 1) {
		return -2;
	}
	if (ioctl(p_platform->i2c_hdl, I2C_SLAVE, i2c_Addr) < 0) {
		return -3;
	}
	return 0;
}
