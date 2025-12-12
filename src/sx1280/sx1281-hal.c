/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Handling of the node configuration protocol

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Matthieu Verdy
*/
#include "sx1281-hal.h"
#include "hal-gpio.h"


int NSS_PIN = 48;
int BUSY_PIN = 4;
int DIO1_PIN = 145;
int NRESET_PIN = 54;




/*!
 * \brief Define the size of tx and rx hal buffers
 *
 * The Tx and Rx hal buffers are used for SPI communication to
 * store data to be sent/receive to/from the chip.
 *
 * \warning The application must ensure the maximal useful size to be much lower
 *          than the MAX_HAL_BUFFER_SIZE
 */
#define MAX_HAL_BUFFER_SIZE   0xFFF

#define IRQ_HIGH_PRIORITY  0

/*!
 * Radio driver structure initialization
 */
const struct SX1281_s SX1281 =
{
    SX1281Init,
    SX1281HalReset,
    SX1281GetStatus,
    SX1281HalWriteCommand,
    SX1281HalReadCommand,
    SX1281HalWriteRegisters,
    SX1281HalWriteRegister,
    SX1281HalReadRegisters,
    SX1281HalReadRegister,
    SX1281HalWriteBuffer,
    SX1281HalReadBuffer,
    SX1281HalGetDioStatus,
    SX1281GetFirmwareVersion,
    SX1281SetRegulatorMode,
    SX1281SetStandby,
    SX1281SetPacketType,
    SX1281SetModulationParams,
    SX1281SetPacketParams,
    SX1281SetRfFrequency,
    SX1281SetBufferBaseAddresses,
    SX1281SetTxParams,
    SX1281SetDioIrqParams,
    SX1281SetSyncWord,
    SX1281SetRx,
    SX1281GetPayload,
    SX1281SendPayload,
    SX1281SetPollingMode,
    SX1281SetInterruptMode,
    SX1281SetRegistersDefault,
    SX1281GetOpMode,
    SX1281SetSleep,
    SX1281SetFs,
    SX1281SetTx,
    SX1281SetRxDutyCycle,
    SX1281SetCad,
    SX1281SetTxContinuousWave,
    SX1281SetTxContinuousPreamble,
    SX1281GetPacketType,
    SX1281SetCadParams,
    SX1281GetRxBufferStatus,
    SX1281GetPacketStatus,
    SX1281GetRssiInst,
    SX1281GetIrqStatus,
    SX1281ClearIrqStatus,
    SX1281Calibrate,
    SX1281SetSaveContext,
    SX1281SetAutoTx,
    SX1281SetAutoFS,
    SX1281SetLongPreamble,
    SX1281SetPayload,
    SX1281SetSyncWordErrorTolerance,
    SX1281SetCrcSeed,
    SX1281SetBleAccessAddress,
    SX1281SetBleAdvertizerAccessAddress,
    SX1281SetCrcPolynomial,
    SX1281SetWhiteningSeed,
    SX1281GetFrequencyError,
};

static uint8_t halTxBuffer[MAX_HAL_BUFFER_SIZE] = {0x00};
static uint8_t halRxBuffer[MAX_HAL_BUFFER_SIZE] = {0x00};

/*!
 * \brief Used to block execution waiting for low state on radio busy pin.
 *        Essentially used in SPI communications
 */
void SX1281HalWaitOnBusy( void )
{
    while( HAL_GPIO_ReadPin( BUSY_PIN ) == 1 );
}

void SX1281HalInit( DioIrqHandler **irqHandlers )
{
    SX1281HalReset( );
    SX1281HalIoIrqInit( irqHandlers );
}

void SX1281HalIoIrqInit( DioIrqHandler **irqHandlers )
{
    SX1281GpioSetIrq( DIO1_PIN, IRQ_HIGH_PRIORITY, irqHandlers[0] );
}

void SX1281HalReset( void )
{
    HAL_Delay( 20 );
    SX1281GpioWrite( NRESET_PIN, 0 );
    HAL_Delay( 50 );
    SX1281GpioWrite( NRESET_PIN, 1 );
    HAL_Delay( 20 );
}

void SX1281HalClearInstructionRam( void )
{
    // Clearing the instruction RAM is writing 0x00s on every bytes of the
    // instruction RAM
    uint16_t halSize = 3 + IRAM_SIZE;
    halTxBuffer[0] = RADIO_WRITE_REGISTER;
    halTxBuffer[1] = ( IRAM_START_ADDRESS >> 8 ) & 0x00FF;
    halTxBuffer[2] = IRAM_START_ADDRESS & 0x00FF;
    for( uint16_t index = 0; index < IRAM_SIZE; index++ )
    {
        halTxBuffer[3+index] = 0x00;
    }

    SX1281HalWaitOnBusy( );

    SX1281GpioWrite( NSS_PIN, 0 );

    SX1281SpiIn( halTxBuffer, halSize );

    SX1281GpioWrite( NSS_PIN, 1 );

    SX1281HalWaitOnBusy( );
}

void SX120HalWakeup( void )
{
    __disable_irq( );

    SX1281GpioWrite( NSS_PIN, 0 );

    uint16_t halSize = 2;
    halTxBuffer[0] = RADIO_GET_STATUS;
    halTxBuffer[1] = 0x00;
    SX1281SpiIn( halTxBuffer, halSize );

    SX1281GpioWrite( NSS_PIN, 1 );

    // Wait for chip to be ready.
    SX1281HalWaitOnBusy( );

    __enable_irq( );
}

void SX1281HalWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    uint16_t halSize  = size + 1;
    SX1281HalWaitOnBusy( );

    SX1281GpioWrite( NSS_PIN, 0 );

    halTxBuffer[0] = command;
    memcpy( halTxBuffer + 1, ( uint8_t * )buffer, size * sizeof( uint8_t ) );

    SX1281SpiIn( halTxBuffer, halSize );

    SX1281GpioWrite( NSS_PIN, 1 );

    if( command != RADIO_SET_SLEEP )
    {
        SX1281HalWaitOnBusy( );
    }
}

void SX1281HalReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    uint16_t halSize = 2 + size;
    halTxBuffer[0] = command;
    halTxBuffer[1] = 0x00;
    for( uint16_t index = 0; index < size; index++ )
    {
        halTxBuffer[2+index] = 0x00;
    }

    SX1281HalWaitOnBusy( );

    SX1281GpioWrite( NSS_PIN, 0 );

    SX1281SpiInOut( halTxBuffer, halRxBuffer, halSize );

    memcpy( buffer, halRxBuffer + 2, size );

    SX1281GpioWrite( NSS_PIN, 1 );

    SX1281HalWaitOnBusy( );
}

void SX1281HalWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    uint16_t halSize = size + 3;
    halTxBuffer[0] = RADIO_WRITE_REGISTER;
    halTxBuffer[1] = ( address & 0xFF00 ) >> 8;
    halTxBuffer[2] = address & 0x00FF;
    memcpy( halTxBuffer + 3, buffer, size );

    SX1281HalWaitOnBusy( );

    SX1281GpioWrite( NSS_PIN, 0 );

    SX1281SpiIn( halTxBuffer, halSize );

    SX1281GpioWrite( NSS_PIN, 1 );

    SX1281HalWaitOnBusy( );
}

void SX1281HalWriteRegister( uint16_t address, uint8_t value )
{
    SX1281HalWriteRegisters( address, &value, 1 );
}

void SX1281HalReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    uint16_t halSize = 4 + size;
    halTxBuffer[0] = RADIO_READ_REGISTER;
    halTxBuffer[1] = ( address & 0xFF00 ) >> 8;
    halTxBuffer[2] = address & 0x00FF;
    halTxBuffer[3] = 0x00;
    for( uint16_t index = 0; index < size; index++ )
    {
        halTxBuffer[4+index] = 0x00;
    }

    SX1281HalWaitOnBusy( );

    SX1281GpioWrite( NSS_PIN, 0 );

    SX1281SpiInOut( halTxBuffer, halRxBuffer, halSize );

    memcpy( buffer, halRxBuffer + 4, size );

    SX1281GpioWrite( NSS_PIN, 1 );

    SX1281HalWaitOnBusy( );
}

uint8_t SX1281HalReadRegister( uint16_t address )
{
    uint8_t data;

    SX1281HalReadRegisters( address, &data, 1 );

    return data;
}

void SX1281HalWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    uint16_t halSize = size + 2;
    halTxBuffer[0] = RADIO_WRITE_BUFFER;
    halTxBuffer[1] = ( offset ) >> 8;
    memcpy( halTxBuffer + 2, buffer, size );

    SX1281HalWaitOnBusy( );

    SX1281GpioWrite( NSS_PIN, 0 );

    SX1281SpiIn( halTxBuffer, halSize );

    SX1281GpioWrite( NSS_PIN, 1 );

    SX1281HalWaitOnBusy( );
}

void SX1281HalReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    uint16_t halSize = size + 3;
    halTxBuffer[0] = RADIO_READ_BUFFER;
    halTxBuffer[1] = offset;
    halTxBuffer[2] = 0x00;
    for( uint16_t index = 0; index < size; index++ )
    {
        halTxBuffer[3+index] = 0x00;
    }

    SX1281HalWaitOnBusy( );

    SX1281GpioWrite( NSS_PIN, 0 );

    SX1281SpiInOut( halTxBuffer, halRxBuffer, halSize );

    memcpy( buffer, halRxBuffer + 3, size );

    SX1281GpioWrite( NSS_PIN, 1 );

    SX1281HalWaitOnBusy( );
}

uint8_t SX1281HalGetDioStatus( void )
{
    return ( SX1281GpioRead( DIO1_PIN ) << 1 )
        | ( SX1281GpioRead( BUSY_PIN ) << 0 );
}
