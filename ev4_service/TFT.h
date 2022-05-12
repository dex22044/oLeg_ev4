#pragma once

#include <fcntl.h>                              //Needed for SPI port
#include <sys/ioctl.h>                  //Needed for SPI port
#include <linux/spi/spidev.h>   //Needed for SPI port
#include <unistd.h>                     //Needed for SPI port
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <unistd.h>
#include <cstring>

class TFT {
public:
	void Reset();
	void Write(unsigned char data);
	void Write16(unsigned short data);
	void WriteCommand(unsigned char command);
	void WriteData(unsigned char data);
};
