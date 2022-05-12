#include <cstdio>
#include <unistd.h>            //Needed for I2C port
#include <fcntl.h>            //Needed for I2C port
#include <sys/ioctl.h>            //Needed for I2C port
#include <linux/i2c-dev.h>        //Needed for I2C port
#include <cstring>
#include <cstdint>
#include <cmath>
#include <time.h>
#include <wiringPi.h>
#include <sys/mman.h>
#include <sys/stat.h>        /* For mode constants */
#include <fcntl.h>           /* For O_* constants */

#include "accelgyro.h"
#include "device.h"
#include "orientation.h"
#include "kalmanFilter.h"

int i2c_fd = -1;
int shm_fd = -1;
char* shm_data;

MPU6500 accelgyro;
int axbias = 0, aybias = 0, azbias = 0;
int gxbias = 0, gybias = 0, gzbias = 0;
KalmanFilter filtGx, filtGy, filtGz;
OrientationSolver orientSolver;

float clamp(float x, float l, float g) {
    if(x < l) return l;
    if(x > g) return g;
    return x;
}

timespec prevAgTime = {0}, currAgTime = {0};

void processAccelGyro() {
    clock_gettime(CLOCK_REALTIME, &currAgTime);
    float deltaSecs = (currAgTime.tv_sec - prevAgTime.tv_sec) + (currAgTime.tv_nsec - prevAgTime.tv_nsec) / 1000000000.0f;
    prevAgTime.tv_sec = currAgTime.tv_sec;
    prevAgTime.tv_nsec = currAgTime.tv_nsec;
    //printf("%f\r\n", deltaSecs);
    accelgyro.ReadData();
    float ax = (accelgyro.inData.ax - axbias) / 32768.0f * 2;
    float ay = (accelgyro.inData.ay - aybias) / 32768.0f * 2;
    float az = (accelgyro.inData.az - azbias) / 32768.0f * 2;
    float gx = (accelgyro.inData.gx - gxbias) / 32768.0f * 250;
    float gy = (accelgyro.inData.gy - gybias) / 32768.0f * 250;
    float gz = (accelgyro.inData.gz - gzbias) / 32768.0f * 250;

    gx = filtGx.Filter(gx);
    gy = filtGy.Filter(gy);
    gz = filtGz.Filter(gz);

    if(abs(gx) < 0.3) gx = 0;
    if(abs(gy) < 0.3) gy = 0;
    if(abs(gz) < 0.3) gz = 0;

    //printf("%f\t%f\t%f\t%f\t%f\t%f\r\n", ax, ay, az, gx, gy, gz);
    //return;
    orientSolver.Solve(ax, ay, az, gx, gy, gz, deltaSecs);
}

int main() {
    shm_fd = shm_open("/ev4_shared", O_RDWR | O_CREAT, S_IRWXU | S_IRWXG | S_IRWXO);
    if(shm_fd < 0) {
        printf("shm error\r\n");
        return -1;
    }
    truncate("/dev/shm/ev4_shared", 1024);
    shm_data = (char*)mmap(NULL, 1024, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if(shm_data <= 0) {
        printf("shm mmap error\r\n");
        return -1;
    }
    shm_data[16] = 0xEF;

    wiringPiSetupGpio();

    pinMode(ADDR0_PIN, OUTPUT);
    pinMode(ADDR1_PIN, OUTPUT);
    pinMode(ADDR2_PIN, OUTPUT);
    pinMode(ADDR3_PIN, OUTPUT);

    i2c_fd = open("/dev/i2c-1", O_RDWR);
    if(i2c_fd < 0) {
        printf("I2C bus opening error\r\n");
        return -1;
    }

    accelgyro.Init(i2c_fd, 0x68);

    for(int i = 0; i < 100; i++) {
        accelgyro.ReadData();
        gxbias += accelgyro.inData.gx;
        gybias += accelgyro.inData.gy;
        gzbias += accelgyro.inData.gz;
    }
    gxbias /= 100; gybias /= 100; gzbias /= 100;

    clock_gettime(CLOCK_REALTIME, &prevAgTime);

    int spi_mode = SPI_MODE_0;
    int spi_bitsPerWord = 8;
    int spi_speed = 100000;
    int spifd = open("/dev/spidev1.1", O_RDWR);

    if (spifd < 0)
    {
        perror("Error - Could not open SPI device");
        exit(1);
    }

#pragma region SPI shit
    int status_value = ioctl(spifd, SPI_IOC_WR_MODE, &spi_mode);
    if(status_value < 0)
    {
        perror("Could not set SPIMode (WR)...ioctl fail");
        exit(1);
    }

    status_value = ioctl(spifd, SPI_IOC_RD_MODE, &spi_mode);
    if(status_value < 0)
    {
        perror("Could not set SPIMode (RD)...ioctl fail");
        exit(1);
    }

    status_value = ioctl(spifd, SPI_IOC_WR_BITS_PER_WORD, &spi_bitsPerWord);
    if(status_value < 0)
    {
        perror("Could not set SPI bitsPerWord (WR)...ioctl fail");
        exit(1);
    }

    status_value = ioctl(spifd, SPI_IOC_RD_BITS_PER_WORD, &spi_bitsPerWord);
    if(status_value < 0)
    {
        perror("Could not set SPI bitsPerWord(RD)...ioctl fail");
        exit(1);
    }

    status_value = ioctl(spifd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    if(status_value < 0)
    {
        perror("Could not set SPI speed (WR)...ioctl fail");
        exit(1);
    }

    status_value = ioctl(spifd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
    if(status_value < 0)
    {
        perror("Could not set SPI speed (RD)...ioctl fail");
        exit(1);
    }
#pragma endregion

    SPI_SlaveDevice* devices[16];

    for(int i = 0; i < 16; i++) devices[i] = new SPI_SlaveDevice(spifd, i);

    uint8_t spd = 0;
    while(true) {
        usleep(50000);
        for(int i = 0; i < 2; i++) {
            devices[i]->MakeTransaction();
            //printf("%d: 0x%02x\r\n", i + 1, devices[i]->rxdata[0]);
            int dtype = devices[i]->rxdata[0];
            printf("%d: 0x%02x; %d\r\n", i + 1, dtype, (int)spd);
            if(dtype == 0x13) printf("DIST: %d\r\n", (int)(((uint16_t*)(devices[i]->rxdata + 1))[0]));
            if(dtype == 0x05) {
                devices[i]->txdata[0] = 0;
                devices[i]->txdata[1] = spd;
            }
        }
        spd += 15;
    }
}
