#ifndef SX128X_SPI_H_
#define SX128X_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stddef.h"
#include <stdint.h>
#include <stdbool.h>

// Include HAL library for SPI communication.
#include "hal-spi.h"

extern spi_t* SpiHandle;

void SX1281SetSPI(spi_t *hspi);

void SX1281SpiInit( void );

void SX1281SpiInOut( uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size );

void SX1281SpiIn( uint8_t *txBuffer, uint16_t size );


#ifdef __cplusplus
}
#endif

#endif /* SX128X_SPI_H_ */
