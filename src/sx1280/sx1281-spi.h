/*
 * sx1281-hal-spi.h
 *
 *  Created on: Jul 25, 2025
 *      Author: Roman
 */

#ifndef SX1281_INC_SX1281_SPI_H_
#define SX1281_INC_SX1281_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stddef.h"
#include <stdint.h>
#include <stdbool.h>
#include "hal-spi.h"

extern SPI_HandleTypeDef* SpiHandle;

void SX1281SetSPI(SPI_HandleTypeDef *hspi, uint8_t useDMA);

void SX1281SpiInit( void );

void SX1281SpiInOut( uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size );

void SX1281SpiIn( uint8_t *txBuffer, uint16_t size );


#ifdef __cplusplus
}
#endif

#endif /* SX1281_INC_SX1281_SPI_H_ */
