#ifndef SPI_H
#define SPI_H

#include "gpio.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPI_DEVICE_PATH "/dev/spidev0.0"


typedef int8_t spi_status_t;

const spi_status_t SPI_OK = 1;
const spi_status_t SPI_ER = 0;

typedef struct spi_pinout {
    int sck;
    int mosi;
    int miso;
    int nss;
} spi_pinout_t;


typedef struct spi_ctx {
    gpio_fd_t fd;
    uint32_t speed_hz;
    spi_pinout_t pinout;
	gpio_fd_t nss_fd;
} spi_t;


spi_t SPI_Init(uint32_t speed_hz, spi_pinout_t pinout);
spi_status_t SPI_Send(spi_t* spi, uint8_t* tx_buffer, uint32_t length);
spi_status_t SPI_SendAndReceive(spi_t* spi, uint8_t* tx_buffer, uint8_t* rx_buffer, uint32_t length);


#ifdef __cplusplus
}
#endif

#endif /* SPI_H */
