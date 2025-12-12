#include "sx1281-spi.h"
#include "hal-spi.h"

volatile bool blockingDmaFlag;

#define WAIT_FOR_BLOCKING_FLAG         while( blockingDmaFlag ) { }

spi_t* SpiHandle;

void SX1281SetSPI(spi_t *hspi) {
	SpiHandle = hspi;
}


/*!
 * @brief Sends txBuffer and receives rxBuffer
 *
 * @param [IN] txBuffer Byte to be sent
 * @param [OUT] rxBuffer Byte to be sent
 * @param [IN] size Byte to be sent
 */
void SX1281SpiInOut( uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size )
{
    SPI_SendAndReceive( SpiHandle, txBuffer, rxBuffer, size, 100 );
}

void SX1281SpiIn( uint8_t *txBuffer, uint16_t size )
{
    SPI_Send( SpiHandle, txBuffer, size, HAL_MAX_DELAY );
}

void HAL_SPI_TxRxCpltCallback(spi_t *hspi)
{
    blockingDmaFlag = false;
}

void HAL_SPI_TxCpltCallback(spi_t *hspi)
{
    blockingDmaFlag = false;
}








