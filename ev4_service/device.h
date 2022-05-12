#pragma once

#include <fcntl.h>				//Needed for SPI port
#include <sys/ioctl.h>			//Needed for SPI port
#include <linux/spi/spidev.h>	//Needed for SPI port
#include <unistd.h>			//Needed for SPI port
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <unistd.h>
#include <cstring>
#include <wiringPi.h>

#define ADDR0_PIN 0
#define ADDR1_PIN 5
#define ADDR2_PIN 6
#define ADDR3_PIN 13

class SPI_SlaveDevice {
private:
	int spiline_fd;
	int port;

public:
	char rxdata[32];
	char txdata[32];

public:
	SPI_SlaveDevice(int fd, int port);
	int MakeTransaction();
};
