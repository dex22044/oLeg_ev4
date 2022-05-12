#pragma once

#include <cstdio>
#include <unistd.h>                     //Needed for I2C port
#include <fcntl.h>                      //Needed for I2C port
#include <sys/ioctl.h>                  //Needed for I2C port
#include <linux/i2c-dev.h>              //Needed for I2C port
#include <cstdint>
#include <cstring>

#define I2C_SMBUS       0x0720  /* SMBus-level access */

#define I2C_SMBUS_READ  1
#define I2C_SMBUS_WRITE 0

// SMBus transaction types

#define I2C_SMBUS_QUICK             0
#define I2C_SMBUS_BYTE              1
#define I2C_SMBUS_BYTE_DATA         2
#define I2C_SMBUS_WORD_DATA         3
#define I2C_SMBUS_PROC_CALL         4
#define I2C_SMBUS_BLOCK_DATA        5
#define I2C_SMBUS_I2C_BLOCK_BROKEN  6
#define I2C_SMBUS_BLOCK_PROC_CALL   7           /* SMBus 2.0 */
#define I2C_SMBUS_I2C_BLOCK_DATA    8

struct MPU6500_InData {
	int16_t ax, ay, az;
	int16_t temperature;
	int16_t gx, gy, gz;
};

class MPU6500 {
private:
	int addr, i2c_fd;
public:
	MPU6500_InData inData;
	int Init(int i2c_fd, int addr);
	int ReadData();
};
