#include "spi.h"
#include "gpio.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <stdint.h>


// Initialize SPI interface.
spi_t SPI_Init(uint32_t speed_hz)
{
	spi_t spi = {
		.speed_hz = speed_hz,
	};
	
	GPIO_Write(spi.nss_fd, GPIO_VAL_HIGH);

	if ((spi.fd = open(SPI_DEVICE_PATH, O_RDWR)) < 0) {
        perror("Failed to open SPI device");
        close(spi.fd);
        return spi;
    }

	uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;

    if (ioctl(spi.fd, SPI_IOC_WR_MODE, &mode) < 0) {
        perror("Failed to set SPI mode");
        close(spi.fd);
        return spi;
    }
    if (ioctl(spi.fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        perror("Failed to set SPI bits per word");
        close(spi.fd);
        return spi;
    }

	return spi;
}


// Sends data without reading.
spi_status_t SPI_Send(spi_t* spi, uint8_t* tx_buffer, uint32_t length, uint32_t timeout)
{
    uint8_t rx_buffer[length];

    struct spi_ioc_transfer transfer = {
        .tx_buf = (unsigned long)tx_buffer,
        .rx_buf = (unsigned long)&rx_buffer[0],
        .len = length,
        .delay_usecs = 0,
        .speed_hz = spi->speed_hz,
        .bits_per_word = 8,
    };
	
	GPIO_Write(spi->nss_fd, GPIO_VAL_LOW);
	
	int res = ioctl(spi->fd, SPI_IOC_MESSAGE(1), &transfer);
	
	GPIO_Write(spi->nss_fd, GPIO_VAL_HIGH);

	if (res < 0) {
        perror("Failed to perform SPI transfer");
        return SPI_ER;
    }

	return SPI_OK;
}


// Sends and recieves data (full-duplex).
spi_status_t SPI_SendAndReceive(spi_t* spi, uint8_t* tx_buffer, uint8_t* rx_buffer, uint32_t length, uint32_t timeout)
{
	memset(rx_buffer, 0, *rx_length);
    struct spi_ioc_transfer transfer = {
        .tx_buf = (unsigned long)tx_buffer,
        .rx_buf = (unsigned long)rx_buffer,
        .len = length,
        .delay_usecs = 0,
        .speed_hz = spi->speed_hz,
        .bits_per_word = 8,
    };
	
	GPIO_Write(spi->nss_fd, GPIO_VAL_LOW);
	
	int res = ioctl(spi->fd, SPI_IOC_MESSAGE(1), &transfer);
	
	GPIO_Write(spi->nss_fd, GPIO_VAL_HIGH);

	if (res < 0) {
        perror("Failed to perform SPI transfer");
        return SPI_ER;
    }

	return SPI_OK;
}