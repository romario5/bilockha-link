#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "hal-gpio.h"
#include "hal-spi.h"
#include "sx128x-hal.h"
#include "sx128x.h"

uint8_t USE_FLRC = 0;

uint32_t FREQUENCY = 2460000000;
uint8_t TX_OUTPUT_POWER = 13;

char testMsg[5];
uint8_t ready = 1;


void OnTxDone( void )
{
	ready = 1;
}

void OnRxDone( void )
{
	ready = 1;
}

void OnTxTimeout( void )
{
	ready = 1;
}

void OnRxTimeout( void )
{
	ready = 1;
}

void OnRxError( IrqErrorCode_t errorCode )
{
	
}

void OnCadDone( bool channelActivityDetected )
{
	
}


RadioCallbacks_t Callbacks =
{
    &OnTxDone,        // txDone
    &OnRxDone,        // rxDone
    NULL,             // syncWordDone
    NULL,             // headerDone
    &OnTxTimeout,     // txTimeout
    &OnRxTimeout,     // rxTimeout
    &OnRxError,       // rxError
    NULL,             // cadDone
};


int main(void)
{
    // Initialize pins.
    NSS_PIN = 48;
    BUSY_PIN = 4;
    NRESET_PIN = 54;
    DIO1_PIN = 145;
    
    HAL_GPIO_Open( NSS_PIN,    GPIO_DIR_OUT );
    HAL_GPIO_Open( BUSY_PIN,   GPIO_DIR_IN  );
    HAL_GPIO_Open( NRESET_PIN, GPIO_DIR_OUT );
    HAL_GPIO_Open( DIO1_PIN,   GPIO_DIR_IN  );


    // Init SPI interface.
    spi_t spi = SPI_Init(2000000);


    // Set SX128X parameters.
    SX1281SetSPI(&spi);
    SX1281.Init( &Callbacks );
    SX1281.SetRegulatorMode( USE_DCDC );

    ModulationParams_t modulationParams;
    PacketParams_t packetParams;


    if (USE_FLRC) {
        modulationParams.PacketType = PACKET_TYPE_FLRC;
        modulationParams.Params.Flrc.BitrateBandwidth = FLRC_BR_2_080_BW_2_4;
        modulationParams.Params.Flrc.CodingRate = FLRC_CR_1_2;
        modulationParams.Params.Flrc.ModulationShaping = RADIO_MOD_SHAPING_BT_1_0;

        packetParams.PacketType = PACKET_TYPE_FLRC;
        packetParams.Params.Flrc.PreambleLength = PREAMBLE_LENGTH_32_BITS;
        packetParams.Params.Flrc.SyncWordLength = FLRC_SYNCWORD_LENGTH_4_BYTE;
        packetParams.Params.Flrc.SyncWordMatch = RADIO_RX_MATCH_SYNCWORD_1;
        packetParams.Params.Flrc.HeaderType = RADIO_PACKET_FIXED_LENGTH;
        packetParams.Params.Flrc.PayloadLength = 127;
        packetParams.Params.Flrc.CrcLength = RADIO_CRC_1_BYTES;
        packetParams.Params.Flrc.Whitening = RADIO_WHITENING_OFF;
    } else {
        modulationParams.PacketType = PACKET_TYPE_LORA;
        modulationParams.Params.LoRa.SpreadingFactor = LORA_SF5;
        modulationParams.Params.LoRa.Bandwidth = LORA_BW_1600;
        modulationParams.Params.LoRa.CodingRate = LORA_CR_LI_4_5;

        packetParams.PacketType = PACKET_TYPE_LORA;
        packetParams.Params.LoRa.PreambleLength = 12;
        packetParams.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
        packetParams.Params.LoRa.PayloadLength = 127;
        packetParams.Params.LoRa.CrcMode = LORA_CRC_OFF;
        packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
    }

    SX1281.SetStandby( STDBY_XOSC );

    SX1281.SetPacketType( modulationParams.PacketType );
    SX1281.SetModulationParams( &modulationParams );
    SX1281.SetPacketParams( &packetParams );
    SX1281.SetRfFrequency( FREQUENCY );
    SX1281.SetBufferBaseAddresses( 0x00, 0x00 );
    SX1281.SetTxParams(TX_OUTPUT_POWER, RADIO_RAMP_20_US);


    sprintf(testMsg, "12345", 5);

    SX1281.SetDioIrqParams(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_RADIO_NONE, IRQ_RADIO_NONE);

    SX1281SetPollingMode();

    while (1)
    {
        SX1281ProcessIrqs();
        SX1281.SendPayload(testMsg, 5, RX_TX_SINGLE);
    }

    return 0;
}
