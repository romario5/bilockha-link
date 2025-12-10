#ifndef SX1280_H
#define SX1280_H

#include <stdint.h>
#include <stddef.h>
#include "spi.h"

#ifdef __cplusplus
extern "C" {
#endif


#define RES_OK     0
#define RES_ER -1





/*!
 * \brief Represents the states of the radio
 */
typedef enum
{
    RF_IDLE                                 = 0x00,         //!< The radio is idle
    RF_RX_RUNNING,                                          //!< The radio is in reception state
    RF_TX_RUNNING,                                          //!< The radio is in transmission state
    RF_CAD,                                                 //!< The radio is doing channel activity detection
} SX1280_States_t;


/*!
 * \brief Represents the operating mode the radio is actually running
 */
typedef enum
{
    MODE_SLEEP                              = 0x00,         //! The radio is in sleep mode
    MODE_STDBY_RC,                                          //! The radio is in standby mode with RC oscillator
    MODE_STDBY_XOSC,                                        //! The radio is in standby mode with XOSC oscillator
    MODE_FS,                                                //! The radio is in frequency synthesis mode
    MODE_TX,                                                //! The radio is in transmit mode
    MODE_RX,                                                //! The radio is in receive mode
    MODE_CAD                                                //! The radio is in channel activity detection mode
} RSX1280_OperatingModes_t;


/*!
 * \brief Declares the oscillator in use while in standby mode
 *
 * Using the STDBY_RC standby mode allow to reduce the energy consumption
 * STDBY_XOSC should be used for time critical applications
 */
typedef enum
{
    STDBY_RC                                = 0x00,
    STDBY_XOSC                              = 0x01,
} SX1280_StandbyModes_t;


/*!
 * \brief Declares the power regulation used to power the device
 *
 * This command allows the user to specify if DC-DC or LDO is used for power regulation.
 * Using only LDO implies that the Rx or Tx current is doubled
 */
typedef enum
{
    USE_LDO  = 0x00, 
    USE_DCDC = 0x01,
} SX1280_RadioRegulatorModes_t;


/*!
 * \brief Represents the possible packet type (i.e. modem) used
 */
typedef enum
{
    PACKET_TYPE_GFSK                        = 0x00,
    PACKET_TYPE_LORA,
    PACKET_TYPE_RANGING,
    PACKET_TYPE_FLRC,
    PACKET_TYPE_BLE,
    PACKET_TYPE_NONE                        = 0x0F,
} SX1280_PacketTypes_t;


/*!
 * \brief Represents the ramping time for power amplifier
 */
typedef enum
{
    RADIO_RAMP_02_US                        = 0x00,
    RADIO_RAMP_04_US                        = 0x20,
    RADIO_RAMP_06_US                        = 0x40,
    RADIO_RAMP_08_US                        = 0x60,
    RADIO_RAMP_10_US                        = 0x80,
    RADIO_RAMP_12_US                        = 0xA0,
    RADIO_RAMP_16_US                        = 0xC0,
    RADIO_RAMP_20_US                        = 0xE0,
} SX1280_RampTimes_t;


/*!
 * \brief Represents the number of symbols to be used for channel activity detection operation
 */
typedef enum
{
    LORA_CAD_01_SYMBOL                      = 0x00,
    LORA_CAD_02_SYMBOL                      = 0x20,
    LORA_CAD_04_SYMBOL                      = 0x40,
    LORA_CAD_08_SYMBOL                      = 0x60,
    LORA_CAD_16_SYMBOL                      = 0x80,
} SX1280_LoRaCadSymbols_t;


/*!
 * \brief Represents the possible combinations of bitrate and bandwidth for
 *        GFSK and BLE packet types
 *
 * The bitrate is expressed in Mb/s and the bandwidth in MHz
 */
typedef enum
{
    GFS_BLE_BR_2_000_BW_2_4                 = 0x04,
    GFS_BLE_BR_1_600_BW_2_4                 = 0x28,
    GFS_BLE_BR_1_000_BW_2_4                 = 0x4C,
    GFS_BLE_BR_1_000_BW_1_2                 = 0x45,
    GFS_BLE_BR_0_800_BW_2_4                 = 0x70,
    GFS_BLE_BR_0_800_BW_1_2                 = 0x69,
    GFS_BLE_BR_0_500_BW_1_2                 = 0x8D,
    GFS_BLE_BR_0_500_BW_0_6                 = 0x86,
    GFS_BLE_BR_0_400_BW_1_2                 = 0xB1,
    GFS_BLE_BR_0_400_BW_0_6                 = 0xAA,
    GFS_BLE_BR_0_250_BW_0_6                 = 0xCE,
    GFS_BLE_BR_0_250_BW_0_3                 = 0xC7,
    GFS_BLE_BR_0_125_BW_0_3                 = 0xEF,
} SX1280_GfskBleBitrates_t;


/*!
 * \brief Represents the modulation index used in GFSK and BLE packet
 *        types
 */
typedef enum
{
    GFS_BLE_MOD_IND_0_35                    =  0,
    GFS_BLE_MOD_IND_0_50                    =  1,
    GFS_BLE_MOD_IND_0_75                    =  2,
    GFS_BLE_MOD_IND_1_00                    =  3,
    GFS_BLE_MOD_IND_1_25                    =  4,
    GFS_BLE_MOD_IND_1_50                    =  5,
    GFS_BLE_MOD_IND_1_75                    =  6,
    GFS_BLE_MOD_IND_2_00                    =  7,
    GFS_BLE_MOD_IND_2_25                    =  8,
    GFS_BLE_MOD_IND_2_50                    =  9,
    GFS_BLE_MOD_IND_2_75                    = 10,
    GFS_BLE_MOD_IND_3_00                    = 11,
    GFS_BLE_MOD_IND_3_25                    = 12,
    GFS_BLE_MOD_IND_3_50                    = 13,
    GFS_BLE_MOD_IND_3_75                    = 14,
    GFS_BLE_MOD_IND_4_00                    = 15,
} SX1280_GfskBleModIndexes_t;


/*!
 * \brief Represents the possible combination of bitrate and bandwidth for FLRC
 *        packet type
 *
 * The bitrate is in Mb/s and the bitrate in MHz
 */
typedef enum
{
    FLRC_BR_2_600_BW_2_4                    = 0x04,
    FLRC_BR_2_080_BW_2_4                    = 0x28,
    FLRC_BR_1_300_BW_1_2                    = 0x45,
    FLRC_BR_1_040_BW_1_2                    = 0x69,
    FLRC_BR_0_650_BW_0_6                    = 0x86,
    FLRC_BR_0_520_BW_0_6                    = 0xAA,
    FLRC_BR_0_325_BW_0_3                    = 0xC7,
    FLRC_BR_0_260_BW_0_3                    = 0xEB,
} SX1280_FlrcBitrates_t;


/*!
 * \brief Represents the possible values for coding rate parameter in FLRC
 *        packet type
 */
typedef enum
{
    FLRC_CR_1_2                             = 0x00,
    FLRC_CR_3_4                             = 0x02,
    FLRC_CR_1_0                             = 0x04,
} SX1280_FlrcCodingRates_t;


/*!
 * \brief Represents the modulation shaping parameter for GFSK, FLRC and BLE
 *        packet types
 */
typedef enum
{
    RADIO_MOD_SHAPING_BT_OFF                = 0x00,         //! No filtering
    RADIO_MOD_SHAPING_BT_1_0                = 0x10,
    RADIO_MOD_SHAPING_BT_0_5                = 0x20,
} SX1280_ModShapings_t;


/*!
 * \brief Represents the possible spreading factor values in LORA packet types
 */
typedef enum
{
    LORA_SF5                                = 0x50,
    LORA_SF6                                = 0x60,
    LORA_SF7                                = 0x70,
    LORA_SF8                                = 0x80,
    LORA_SF9                                = 0x90,
    LORA_SF10                               = 0xA0,
    LORA_SF11                               = 0xB0,
    LORA_SF12                               = 0xC0,
} SX1280_LoRaSpreadingFactors_t;


/*!
 * \brief Represents the bandwidth values for LORA packet type
 */
typedef enum
{
    LORA_BW_0200                            = 0x34,
    LORA_BW_0400                            = 0x26,
    LORA_BW_0800                            = 0x18,
    LORA_BW_1600                            = 0x0A,
} SX1280_LoRaBandwidths_t;


/*!
 * \brief Represents the coding rate values for LORA packet type
 */
typedef enum
{
    LORA_CR_4_5                             = 0x01,
    LORA_CR_4_6                             = 0x02,
    LORA_CR_4_7                             = 0x03,
    LORA_CR_4_8                             = 0x04,
    LORA_CR_LI_4_5                          = 0x05,
    LORA_CR_LI_4_6                          = 0x06,
    LORA_CR_LI_4_7                          = 0x07,
} SX1280_LoRaCodingRates_t;


/*!
 * \brief Represents the preamble length values for GFSK and FLRC packet
 *        types
 */
typedef enum
{
    PREAMBLE_LENGTH_04_BITS                 = 0x00,         //!< Preamble length: 04 bits
    PREAMBLE_LENGTH_08_BITS                 = 0x10,         //!< Preamble length: 08 bits
    PREAMBLE_LENGTH_12_BITS                 = 0x20,         //!< Preamble length: 12 bits
    PREAMBLE_LENGTH_16_BITS                 = 0x30,         //!< Preamble length: 16 bits
    PREAMBLE_LENGTH_20_BITS                 = 0x40,         //!< Preamble length: 20 bits
    PREAMBLE_LENGTH_24_BITS                 = 0x50,         //!< Preamble length: 24 bits
    PREAMBLE_LENGTH_28_BITS                 = 0x60,         //!< Preamble length: 28 bits
    PREAMBLE_LENGTH_32_BITS                 = 0x70,         //!< Preamble length: 32 bits
} SX1280_PreambleLengths_t;


/*!
 * \brief Represents the SyncWord length for FLRC packet type
 */
typedef enum
{
    FLRC_NO_SYNCWORD                       = 0x00,
    FLRC_SYNCWORD_LENGTH_4_BYTE            = 0x04,
} SX1280_FlrcSyncWordLengths_t;


/*!
 * \brief The length of sync words for GFSK packet type
 */
typedef enum
{
    GFS_SYNCWORD_LENGTH_1_BYTE              = 0x00,         //!< Sync word length: 1 byte
    GFS_SYNCWORD_LENGTH_2_BYTE              = 0x02,         //!< Sync word length: 2 bytes
    GFS_SYNCWORD_LENGTH_3_BYTE              = 0x04,         //!< Sync word length: 3 bytes
    GFS_SYNCWORD_LENGTH_4_BYTE              = 0x06,         //!< Sync word length: 4 bytes
    GFS_SYNCWORD_LENGTH_5_BYTE              = 0x08,         //!< Sync word length: 5 bytes
} SX1280_SyncWordLengths_t;

/*!
 * \brief Represents the possible combinations of SyncWord correlators
 *        activated for GFSK and FLRC packet types
 */
typedef enum
{
    RADIO_RX_MATCH_SYNCWORD_OFF             = 0x00,         //!< No correlator turned on, i.e. do not search for SyncWord
    RADIO_RX_MATCH_SYNCWORD_1               = 0x10,
    RADIO_RX_MATCH_SYNCWORD_2               = 0x20,
    RADIO_RX_MATCH_SYNCWORD_1_2             = 0x30,
    RADIO_RX_MATCH_SYNCWORD_3               = 0x40,
    RADIO_RX_MATCH_SYNCWORD_1_3             = 0x50,
    RADIO_RX_MATCH_SYNCWORD_2_3             = 0x60,
    RADIO_RX_MATCH_SYNCWORD_1_2_3           = 0x70,
} SX1280_SyncWordRxMatchs_t;


/*!
 *  \brief Radio packet length mode for GFSK and FLRC packet types
 */
typedef enum
{
    RADIO_PACKET_FIXED_LENGTH               = 0x00,         //!< The packet is known on both sides, no header included in the packet
    RADIO_PACKET_VARIABLE_LENGTH            = 0x20,         //!< The packet is on variable size, header included
} SX1280_PacketLengthModes_t;


/*!
 * \brief Represents the CRC length for GFSK and FLRC packet types
 *
 * \warning Not all configurations are available for both GFSK and FLRC
 *          packet type. Refer to the datasheet for possible configuration.
 */
typedef enum
{
    RADIO_CRC_OFF                           = 0x00,         //!< No CRC in use
    RADIO_CRC_1_BYTES                       = 0x10,
    RADIO_CRC_2_BYTES                       = 0x20,
    RADIO_CRC_3_BYTES                       = 0x30,
} SX1280_CrcTypes_t;


/*!
 * \brief Radio whitening mode activated or deactivated for GFSK, FLRC and
 *        BLE packet types
 */
typedef enum
{
    RADIO_WHITENING_ON                      = 0x00,
    RADIO_WHITENING_OFF                     = 0x08,
} SX1280_WhiteningModes_t;


/*!
 * \brief Holds the packet length mode of a LORA packet type
 */
typedef enum
{
    LORA_PACKET_VARIABLE_LENGTH             = 0x00,         //!< The packet is on variable size, header included
    LORA_PACKET_FIXED_LENGTH                = 0x80,         //!< The packet is known on both sides, no header included in the packet
    LORA_PACKET_EXPLICIT                    = LORA_PACKET_VARIABLE_LENGTH,
    LORA_PACKET_IMPLICIT                    = LORA_PACKET_FIXED_LENGTH,
} SX1280_LoRaPacketLengthsModes_t;


/*!
 * \brief Represents the CRC mode for LORA packet type
 */
typedef enum
{
    LORA_CRC_ON                             = 0x20,         //!< CRC activated
    LORA_CRC_OFF                            = 0x00,         //!< CRC not used
} SX1280_LoRaCrcModes_t;

/*!
 * \brief Represents the IQ mode for LORA packet type
 */
typedef enum
{
    LORA_IQ_NORMAL                          = 0x40,
    LORA_IQ_INVERTED                        = 0x00,
} SX1280_LoRaIQModes_t;


/*!
 * \brief Represents the connection state for BLE packet type
 */
typedef enum
{
    BLE_MASTER_SLAVE                        = 0x00,
    BLE_ADVERTISER                          = 0x20,
    BLE_TX_TEST_MODE                        = 0x40,
    BLE_RX_TEST_MODE                        = 0x60,
    BLE_RXTX_TEST_MODE                      = 0x80,
} SX1280_BleConnectionStates_t;


/*!
 * \brief Represents the CRC field length for BLE packet type
 */
typedef enum
{
    BLE_CRC_OFF                             = 0x00,
    BLE_CRC_3B                              = 0x10,
} SX1280_BleCrcFields_t;


/*!
 * \brief Represents the specific packets to use in BLE packet type
 */
typedef enum
{
    BLE_PRBS_9                              = 0x00,         //!< Pseudo Random Binary Sequence based on 9th degree polynomial
    BLE_PRBS_15                             = 0x0C,         //!< Pseudo Random Binary Sequence based on 15th degree polynomial
    BLE_EYELONG_1_0                         = 0x04,         //!< Repeated '11110000' sequence
    BLE_EYELONG_0_1                         = 0x18,         //!< Repeated '00001111' sequence
    BLE_EYESHORT_1_0                        = 0x08,         //!< Repeated '10101010' sequence
    BLE_EYESHORT_0_1                        = 0x1C,         //!< Repeated '01010101' sequence
    BLE_ALL_1                               = 0x10,         //!< Repeated '11111111' sequence
    BLE_ALL_0                               = 0x14,         //!< Repeated '00000000' sequence
} SX1280_BlePacketTypes_t;


/*!
 * \brief Represents the interruption masks available for the radio
 *
 * \remark Note that not all these interruptions are available for all packet types
 */
typedef enum
{
    IRQ_RADIO_NONE                          = 0x0000,
    IRQ_TX_DONE                             = 0x0001,
    IRQ_RX_DONE                             = 0x0002,
    IRQ_SYNCWORD_VALID                      = 0x0004,
    IRQ_SYNCWORD_ERROR                      = 0x0008,
    IRQ_HEADER_VALID                        = 0x0010,
    IRQ_HEADER_ERROR                        = 0x0020,
    IRQ_CRC_ERROR                           = 0x0040,
    IRQ_CAD_DONE                            = 0x1000,
    IRQ_CAD_ACTIVITY_DETECTED               = 0x2000,
    IRQ_RX_TX_TIMEOUT                       = 0x4000,
    IRQ_PREAMBLE_DETECTED                   = 0x8000,
    IRQ_RADIO_ALL                           = 0xFFFF,
} SX1280_IrqMasks_t;


/*!
 * \brief Represents the digital input/output of the radio
 */
typedef enum
{
    RADIO_DIO1                              = 0x02,
    RADIO_DIO2                              = 0x04,
    RADIO_DIO3                              = 0x08,
} SX1280_Dios_t;


/*!
 * \brief Represents the tick size available for Rx/Tx timeout operations
 */
typedef enum
{
    RADIO_TICK_SIZE_0015_US                 = 0x00,
    RADIO_TICK_SIZE_0062_US                 = 0x01,
    RADIO_TICK_SIZE_1000_US                 = 0x02,
    RADIO_TICK_SIZE_4000_US                 = 0x03,
} SX1280_TickSizes_t;


/*!
 * \brief Represents all possible opcode understood by the radio
 */
typedef enum RadioCommands_u
{
    RADIO_GET_STATUS                        = 0xC0,
    RADIO_WRITE_REGISTER                    = 0x18,
    RADIO_READ_REGISTER                     = 0x19,
    RADIO_WRITE_BUFFER                      = 0x1A,
    RADIO_READ_BUFFER                       = 0x1B,
    RADIO_SET_SLEEP                         = 0x84,
    RADIO_SET_STANDBY                       = 0x80,
    RADIO_SET_FS                            = 0xC1,
    RADIO_SET_TX                            = 0x83,
    RADIO_SET_RX                            = 0x82,
    RADIO_SET_RXDUTYCYCLE                   = 0x94,
    RADIO_SET_CAD                           = 0xC5,
    RADIO_SET_TXCONTINUOUSWAVE              = 0xD1,
    RADIO_SET_TXCONTINUOUSPREAMBLE          = 0xD2,
    RADIO_SET_PACKETTYPE                    = 0x8A,
    RADIO_GET_PACKETTYPE                    = 0x03,
    RADIO_SET_RFFREQUENCY                   = 0x86,
    RADIO_SET_TXPARAMS                      = 0x8E,
    RADIO_SET_CADPARAMS                     = 0x88,
    RADIO_SET_BUFFERBASEADDRESS             = 0x8F,
    RADIO_SET_MODULATIONPARAMS              = 0x8B,
    RADIO_SET_PACKETPARAMS                  = 0x8C,
    RADIO_GET_RXBUFFERSTATUS                = 0x17,
    RADIO_GET_PACKETSTATUS                  = 0x1D,
    RADIO_GET_RSSIINST                      = 0x1F,
    RADIO_SET_DIOIRQPARAMS                  = 0x8D,
    RADIO_GET_IRQSTATUS                     = 0x15,
    RADIO_CLR_IRQSTATUS                     = 0x97,
    RADIO_CALIBRATE                         = 0x89,
    RADIO_SET_REGULATORMODE                 = 0x96,
    RADIO_SET_SAVECONTEXT                   = 0xD5,
    RADIO_SET_AUTOTX                        = 0x98,
    RADIO_SET_AUTOFS                        = 0x9E,
    RADIO_SET_LONGPREAMBLE                  = 0x9B,
    RADIO_SET_UARTSPEED                     = 0x9D,
} SX1280_Commands_t;


/*!
 * \brief Represents an amount of time measurable by the radio clock
 *
 * @code
 * Time = Step * NbSteps
 * Example:
 * Step = RADIO_TICK_SIZE_4000_US( 4 ms )
 * NbSteps = 1000
 * Time = 4e-3 * 1000 = 4 seconds
 * @endcode
 */
typedef struct TickTime_s
{
    SX1280_TickSizes_t Step;                                  //!< The step of ticktime
    /*!
     * \brief The number of steps for ticktime
     * Special values are:
     *     - 0x0000 for single mode
     *     - 0xFFFF for continuous mode
     */
    uint16_t NbSteps;
} SX1280_TickTime_t;


/*!
* \brief RX_TX_CONTINUOUS and RX_TX_SINGLE are two particular values for TickTime.
* The former keep the radio in Rx or Tx mode, even after successfull reception
* or transmission. It should never generate Timeout interrupt.
* The later let the radio enought time to make one reception or transmission.
* No Timeout interrupt is generated, and the radio fall in StandBy mode after
* reception or transmission.
*/
#define RX_TX_CONTINUOUS ( SX1280_TickTime_t ){ RADIO_TICK_SIZE_0015_US, 0xFFFF }
#define RX_TX_SINGLE     ( SX1280_TickTime_t ){ RADIO_TICK_SIZE_0015_US, 0 }





typedef struct {
	int   nss
    int   reset;
    int   busy;
    int   dio1;
} sx1280_pinout_t;


typedef struct {
    sx1280_modulation_t modulation;
    uint32_t frequency_hz;       /* e.g. 2403000000 */
    uint32_t bandwidth_hz;       /* numeric bandwidth (driver maps to indexes) */
    uint8_t  lora_spread_factor; /* for LoRa: e.g. 7..12 (use 7 default) */
    uint8_t  coding_rate;        /* for LoRa: 1..4 -> 4/5..4/8 (1==4/5) */
    uint8_t  crc_on;             /* 0/1 */
    uint16_t preamble_len;       /* symbols */
    uint16_t packet_len;         /* fixed packet length or max */
} sx1280_radio_t;


typedef struct {
    int nss;
    int reset;
    int busy;
    int dio1;
} sx1280_gpio_t;


typedef struct {
    sx1280_radio_t  radio;
    sx1280_gpio_t   gpio;
    spi_t*          spi;
} sx1280_t;


/* Initialize device context and program initial params.
 * Returns pointer to ctx or NULL on error.
 */
sx1280_t *sx1280_init(const sx1280_pinout_t *pins, const sx1280_radio_t *cfg, spi_t *spi);


/* Deinitialize and free */
void sx1280_deinit(sx1280_t *ctx);


/* Send a single packet synchronously.
 * payload points to data (len bytes).
 * timeout_ms: TX timeout in milliseconds (radio-specific).
 * returns 0 on success, negative on error.
 */
uint8_t sx1280_send(sx1280_t *ctx, const uint8_t *payload, size_t len, uint32_t timeout_ms);


/* Receive a single packet (synchronous) — simplified interface.
 * out_buf: buffer to receive into, max_len its size.
 * rx_timeout_ms: timeout in ms (0 = continuous).
 * on success returns 0 and sets *out_len to received bytes.
 * returns -2 on timeout, negative on error.
 */
uint8_t sx1280_receive(sx1280_t *ctx, uint8_t *out_buf, size_t max_len, uint32_t rx_timeout_ms, size_t *out_len);


/* Reconfigure radio parameters (frequency, modulation, bandwidth, crc, preamble, packet_len).
 * Returns 0 on success.
 */
uint8_t sx1280_reconfigure(sx1280_t *ctx, const sx1280_radio_t *cfg);


#ifdef __cplusplus
}
#endif

#endif /* SX1280_H */
