#include "device.h"

SPI_SlaveDevice::SPI_SlaveDevice(int fd, int port) {
	this->spiline_fd = fd;
	this->port = port;
}

int SPI_SlaveDevice::MakeTransaction() {
	digitalWrite(ADDR0_PIN, port & 0b0001);
	digitalWrite(ADDR1_PIN, port & 0b0010);
	digitalWrite(ADDR2_PIN, port & 0b0100);
	digitalWrite(ADDR3_PIN, port & 0b1000);
	usleep(100);
	spi_ioc_transfer spi = {0};
	int i = 0;
	int retVal = -1;

	memset(rxdata, 0, 32);

	spi.tx_buf = (unsigned long)txdata;		//transmit from "data"
	spi.rx_buf = (unsigned long)rxdata;		//receive into "data"
	spi.len = 32;
	spi.delay_usecs = 0;
	spi.speed_hz = 100000;
	spi.bits_per_word = 8;
	spi.cs_change = 0;						//0=Set CS high after a transfer, 1=leave CS set low

	retVal = ioctl(spiline_fd, SPI_IOC_MESSAGE(1), &spi);

	if(retVal < 0)
	{
		perror("Error - Problem transmitting spi data..ioctl");
		return -1;
	}

	return retVal;
}
