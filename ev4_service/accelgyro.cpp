#include "accelgyro.h"

union i2c_smbus_data
{
    uint8_t  byte;
    uint16_t word;
    uint8_t  block[32 + 2];
};

static inline int i2c_smbus_access (int fd, char rw, uint8_t command, int size, union i2c_smbus_data *data)
{
    struct i2c_smbus_ioctl_data args;

    args.read_write = rw;
    args.command    = command;
    args.size       = size;
    args.data       = data;
    return ioctl (fd, I2C_SMBUS, &args);
}

int16_t bswap16(int16_t val) {
	return (val >> 8) | ((val & 0xFF) << 8);
}

int MPU6500::Init(int i2c_fd, int addr) {
	this->addr = addr;
	this->i2c_fd = i2c_fd;
	return 0;
}

int MPU6500::ReadData() {
	if(ioctl(i2c_fd, I2C_SLAVE, addr) < 0)
    {
        printf("Failed to connect to the bus (slaveAddr 0x%2x)\r\n", addr);
        return -1;
    }

    i2c_smbus_data data;
	for(int i = 0; i < 7; i++) {
        // read registers from slave
        if(i2c_smbus_access(i2c_fd, I2C_SMBUS_READ, 58 + i * 2, I2C_SMBUS_WORD_DATA, &data) != 0)
        {
            printf("Failed to read data from slave (slaveAddr 0x%2x)\r\n", addr);
            return -1;
		}
		((int16_t*)&inData)[i] = data.word;
	}

	return 0;
}
