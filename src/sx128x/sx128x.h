/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Driver for SX1281 devices

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Matthieu Verdy
*/
#ifndef __SX128X_H__
#define __SX128X_H__

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "sx128x-hal.h"
#include "sx128x-radio.h"


/*!
 * \brief Compute the two's complement for a register of size lower than
 *        32bits
 *
 * \param [in]  num            The register to be two's complemented
 * \param [in]  bitCnt         The position of the sign bit
 */
static int32_t SX1281complement2( const uint32_t num, const uint8_t bitCnt );

/*!
 * \brief Returns the value of LoRa bandwidth from driver's value
 *
 * The value is returned in Hz so that it can be represented as an integer
 * type. Most computation should be done as integer to reduce floating
 * point related errors.
 *
 * \retval loRaBw              The value of the current bandwidth in Hz
 */
int32_t SX1281GetLoRaBandwidth( void );

/*!
 * \brief DIOs interrupt callback
 *
 * \remark Called to handle all 3 DIOs pins
 */
void SX1281OnDioIrq( void );

/*!
 * \brief Initializes the radio driver
 */
void SX1281Init( RadioCallbacks_t *callbacks );

/*!
 * \brief Set the driver in polling mode.
 *
 * In polling mode the application is responsible to call ProcessIrqs( ) to
 * execute callbacks functions.
 * The default mode is Interrupt Mode.
 * @code
 * // Initializations and callbacks declaration/definition
 * radio = SX1281( mosi, miso, sclk, nss, busy, int1, int2, int3, rst, &callbacks );
 * radio.Init( );
 * radio.SetPollingMode( );
 *
 * while( true )
 * {
 *                            //     IRQ processing is automatically done
 *     radio.ProcessIrqs( );  // <-- here, as well as callback functions
 *                            //     calls
 *     // Do some applicative work
 * }
 * @endcode
 *
 * \see SX1281SetInterruptMode
 */
void SX1281SetPollingMode( void );

/*!
 * \brief Set the driver in interrupt mode.
 *
 * In interrupt mode, the driver communicate with the radio during the
 * interruption by direct calls to ProcessIrqs( ). The main advantage is
 * the possibility to have low power application architecture.
 * This is the default mode.
 * @code
 * // Initializations and callbacks declaration/definition
 * radio = SX1281( mosi, miso, sclk, nss, busy, int1, int2, int3, rst, &callbacks );
 * radio.Init( );
 * radio.SetInterruptMode( );   // Optionnal. Driver default behavior
 *
 * while( true )
 * {
 *     // Do some applicative work
 * }
 * @endcode
 *
 * \see SX1281SetPollingMode
 */
void SX1281SetInterruptMode( void );

/*!
 * \brief Initializes the radio registers to the recommended default values
 */
void SX1281SetRegistersDefault( void );

/*!
 * \brief Returns the current device firmware version
 *
 * \retval      version       Firmware version
 */
uint16_t SX1281GetFirmwareVersion( void );

/*!
 * \brief Gets the current Operation Mode of the Radio
 *
 * \retval      RadioOperatingModes_t last operating mode
 */
RadioOperatingModes_t SX1281GetOpMode( void );

/*!
 * \brief Gets the current radio status
 *
 * \retval      status        Radio status
 */
RadioStatus_t SX1281GetStatus( void );

/*!
 * \brief Sets the radio in sleep mode
 *
 * \param [in]  sleepConfig   The sleep configuration describing data
 *                            retention and RTC wake-up
 */
void SX1281SetSleep( SleepParams_t sleepConfig );

/*!
 * \brief Sets the radio in configuration mode
 *
 * \param [in]  mode          The standby mode to put the radio into
 */
void SX1281SetStandby( RadioStandbyModes_t mode );

/*!
 * \brief Sets the radio in FS mode
 */
void SX1281SetFs( void );

/*!
 * \brief Sets the radio in transmission mode
 *
 * \param [in]  timeout       Structure describing the transmission timeout value
 */
void SX1281SetTx( TickTime_t timeout );

/*!
 * \brief Sets the radio in reception mode
 *
 * \param [in]  timeout       Structure describing the reception timeout value
 */
void SX1281SetRx( TickTime_t timeout );

/*!
 * \brief Sets the Rx duty cycle management parameters
 *
 * \param [in]  rxTime        Structure describing reception timeout value
 * \param [in]  sleepTime     Structure describing sleep timeout value
 */
void SX1281SetRxDutyCycle( RadioTickSizes_t Step, uint16_t NbStepRx, uint16_t RxNbStepSleep );

/*!
 * \brief Sets the radio in CAD mode
 *
 * \see SX1281::SetCadParams
 */
void SX1281SetCad( void );

/*!
 * \brief Sets the radio in continuous wave transmission mode
 */
void SX1281SetTxContinuousWave( void );

/*!
 * \brief Sets the radio in continuous preamble transmission mode
 */
void SX1281SetTxContinuousPreamble( void );

/*!
 * \brief Sets the radio for the given protocol
 *
 * \param [in]  packetType    [PACKET_TYPE_GFSK, PACKET_TYPE_LORA,
 *                             PACKET_TYPE_FLRC, PACKET_TYPE_BLE]
 *
 * \remark This method has to be called before SetRfFrequency,
 *         SetModulationParams and SetPacketParams
 */
void SX1281SetPacketType( RadioPacketTypes_t packetType );

/*!
 * \brief Gets the current radio protocol
 *
 * \retval      packetType    [PACKET_TYPE_GFSK, PACKET_TYPE_LORA,
 *                             PACKET_TYPE_FLRC, PACKET_TYPE_BLE, PACKET_TYPE_NONE]
 */
RadioPacketTypes_t SX1281GetPacketType( void );

/*!
 * \brief Sets the RF frequency
 *
 * \param [in]  frequency     RF frequency [Hz]
 */
void SX1281SetRfFrequency( uint32_t frequency );

/*!
 * \brief Sets the transmission parameters
 *
 * \param [in]  power         RF output power [-18..13] dBm
 * \param [in]  rampTime      Transmission ramp up time
 */
void SX1281SetTxParams( int8_t power, RadioRampTimes_t rampTime );

/*!
 * \brief Sets the number of symbols to be used for Channel Activity
 *        Detection operation
 *
 * \param [in]  cadSymbolNum  The number of symbol to use for Channel Activity
 *                            Detection operations [LORA_CAD_01_SYMBOL, LORA_CAD_02_SYMBOL,
 *                            LORA_CAD_04_SYMBOL, LORA_CAD_08_SYMBOL, LORA_CAD_16_SYMBOL]
 */
void SX1281SetCadParams( RadioLoRaCadSymbols_t cadSymbolNum );

/*!
 * \brief Sets the data buffer base address for transmission and reception
 *
 * \param [in]  txBaseAddress Transmission base address
 * \param [in]  rxBaseAddress Reception base address
 */
void SX1281SetBufferBaseAddresses( uint8_t txBaseAddress, uint8_t rxBaseAddress );

/*!
 * \brief Set the modulation parameters
 *
 * \param [in]  modParams     A structure describing the modulation parameters
 */
void SX1281SetModulationParams( ModulationParams_t *modParams );

/*!
 * \brief Sets the packet parameters
 *
 * \param [in]  packetParams  A structure describing the packet parameters
 */
void SX1281SetPacketParams( PacketParams_t *packetParams );

/*!
 * \brief Gets the last received packet buffer status
 *
 * \param [out] payloadLength Last received packet payload length
 * \param [out] rxStartBuffer Last received packet buffer address pointer
 */
void SX1281GetRxBufferStatus( uint8_t *payloadLength, uint8_t *rxStartBuffer );

/*!
 * \brief Gets the last received packet payload length
 *
 * \param [out] pktStatus     A structure of packet status
 */
void SX1281GetPacketStatus( PacketStatus_t *pktStatus );

/*!
 * \brief Returns the instantaneous RSSI value for the last packet received
 *
 * \retval      rssiInst      Instantaneous RSSI
 */
int8_t SX1281GetRssiInst( void );

/*!
 * \brief   Sets the IRQ mask and DIO masks
 *
 * \param [in]  irqMask       General IRQ mask
 * \param [in]  dio1Mask      DIO1 mask
 * \param [in]  dio2Mask      DIO2 mask
 * \param [in]  dio3Mask      DIO3 mask
 */
void SX1281SetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask );

/*!
 * \brief Returns the current IRQ status
 *
 * \retval      irqStatus     IRQ status
 */
uint16_t SX1281GetIrqStatus( void );

/*!
 * \brief Clears the IRQs
 *
 * \param [in]  irq           IRQ(s) to be cleared
 */
void SX1281ClearIrqStatus( uint16_t irq );

/*!
 * \brief Calibrates the given radio block
 *
 * \param [in]  calibParam    The description of blocks to be calibrated
 */
void SX1281Calibrate( CalibrationParams_t calibParam );

/*!
 * \brief Sets the power regulators operating mode
 *
 * \param [in]  mode          [0: LDO, 1:DC_DC]
 */
void SX1281SetRegulatorMode( RadioRegulatorModes_t mode );

/*!
 * \brief Saves the current selected modem configuration into data RAM
 */
void SX1281SetSaveContext( void );

/*!
 * \brief Sets the chip to automatically send a packet after the end of a packet reception
 *
 * \remark The offset is automatically compensated inside the function
 *
 * \param [in]  time          The delay in us after which a Tx is done
 */
void SX1281SetAutoTx( uint16_t time );

/*!
 * \brief Sets the chip to automatically receive a packet after the end of a packet transmission
 *
 * \remark The offset is automatically compensated inside the function
 *
 * \param [in]  time          The delay in us after which a Rx is done
 */
void SX1281SetAutoFS( uint8_t enable );

/*!
 * \brief Enables or disables long preamble detection mode
 *
 * \param [in]  enable        [0: Disable, 1: Enable]
 */
void SX1281SetLongPreamble( uint8_t enable );

/*!
 * \brief Saves the payload to be send in the radio buffer
 *
 * \param [in]  payload       A pointer to the payload
 * \param [in]  size          The size of the payload
 */
void SX1281SetPayload( uint8_t *payload, uint8_t size );

/*!
 * \brief Reads the payload received. If the received payload is longer
 * than maxSize, then the method returns 1 and do not set size and payload.
 *
 * \param [out] payload       A pointer to a buffer into which the payload will be copied
 * \param [out] size          A pointer to the size of the payload received
 * \param [in]  maxSize       The maximal size allowed to copy into the buffer
 */
uint8_t SX1281GetPayload( uint8_t *payload, uint8_t *size, uint8_t maxSize );

/*!
 * \brief Sends a payload
 *
 * \param [in]  payload       A pointer to the payload to send
 * \param [in]  size          The size of the payload to send
 * \param [in]  timeout       The timeout for Tx operation
 */
void SX1281SendPayload( uint8_t *payload, uint8_t size, TickTime_t timeout );

/*!
 * \brief Sets the Sync Word given by index used in GFSK, FLRC and BLE protocols
 *
 * \remark 5th byte isn't used in FLRC and BLE protocols
 *
 * \param [in]  syncWordIdx   Index of SyncWord to be set [1..3]
 * \param [in]  syncWord      SyncWord bytes ( 5 bytes )
 *
 * \retval      status        [0: OK, 1: NOK]
 */
uint8_t SX1281SetSyncWord( uint8_t syncWordIdx, uint8_t *syncWord );

/*!
 * \brief Defines how many error bits are tolerated in sync word detection
 *
 * \param [in]  errorBits     Number of error bits supported to validate the Sync word detection
 *                            ( default is 4 bit, minimum is 1 bit )
 */
void SX1281SetSyncWordErrorTolerance( uint8_t errorBits );

/*!
 * \brief Sets the Initial value for the LFSR used for the CRC calculation
 *
 * \param [in]  seed          Initial LFSR value ( 4 bytes )
 *
 */
void SX1281SetCrcSeed( uint16_t seed );

/*!
 * \brief Set the Access Address field of BLE packet
 *
 * \param [in]  accessAddress The access address to be used for next BLE packet sent
 */
void SX1281SetBleAccessAddress( uint32_t accessAddress );

/*!
 * \brief Set the Access Address for Advertizer BLE packets
 *
 * All advertizer BLE packets must use a particular value for Access
 * Address field. This method sets it.
 *
 * \see SX1281::SetBleAccessAddress
 */
void SX1281SetBleAdvertizerAccessAddress( void );

/*!
 * \brief Sets the seed used for the CRC calculation
 *
 * \param [in]  seed          The seed value
 *
 */
void SX1281SetCrcPolynomial( uint16_t seed );

/*!
 * \brief Sets the Initial value of the LFSR used for the whitening in GFSK, FLRC and BLE protocols
 *
 * \param [in]  seed          Initial LFSR value
 */
void SX1281SetWhiteningSeed( uint8_t seed );

/*!
 * \brief Return the Estimated Frequency Error in LORA operations
 *
 * \retval efe                The estimated frequency error [Hz]
 */
double SX1281GetFrequencyError( void );

/*!
 * \brief Process the analysis of radio IRQs and calls callback functions
 *        depending on radio state
 */
void SX1281ProcessIrqs( void );

/*!
 * \brief Clears the instruction RAM
 */
void SX1281ClearInstructionRam( void );

/*!
 * \brief Parses 1 HEX file line and writes the content to the instruction memory
 *
 * \param [in]  line          HEX file line string
 *
 * \retval      status        [0: ERROR, 1:OK]
 */
int8_t SX1281ParseHexFileLine( char* line );

/*!
 * \brief Gets individual fields for the given HEX file line
 *
 * \param [in]  line          HEX file line string
 * \param [out] bytes         Bytes array to be written to the instruction memory
 * \param [out] addr          Instruction memory address where to write the bytes array
 * \param [out] num           Number of bytes in Bytes array
 * \param [out] code          HEX file line type [0: instruction, 1: end of file, 2: begin of file]
 *
 * \retval      status        [0: ERROR, 1:OK]
 */
int8_t SX1281GetHexFileLineFields( char* line, uint8_t *bytes, uint16_t *addr, uint16_t *num, uint8_t *code );

#endif // __SX128X_H__
