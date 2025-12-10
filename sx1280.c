#include "sx1280.h"
#include "spi.h"
#include "gpio.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>





/*
 * Notes on SX1280 commands used:
 * - SET_PACKET_TYPE (0x8A)
 * - SET_RF_FREQUENCY (0x86)
 * - SET_MODULATION_PARAMS (0x8B)
 * - SET_PACKET_PARAMS (0x8C)
 * - WRITE_BUFFER (0x1A)
 * - READ_BUFFER (0x1B)
 * - SET_TX (0x83)
 * - SET_RX (0x82)
 * - GET_IRQ_STATUS (0x17)
 * - CLEAR_IRQ_STATUS (0x02)
 *
 * This driver implements common encodings for LoRa and FLRC for SET_MODULATION_PARAMS
 * and SET_PACKET_PARAMS. Please verify values with your SX1280 datasheet/module docs
 * if you need exact/edge-case parameter sets.
 */

/* Commands */
#define OPCODE_SET_PACKET_TYPE      0x8A
#define OPCODE_SET_RF_FREQUENCY     0x86
#define OPCODE_SET_MODULATION_PARAMS 0x8B
#define OPCODE_SET_PACKET_PARAMS    0x8C
#define OPCODE_WRITE_BUFFER         0x1A
#define OPCODE_READ_BUFFER          0x1B
#define OPCODE_SET_TX               0x83
#define OPCODE_SET_RX               0x82
#define OPCODE_CLEAR_IRQ_STATUS     0x02
#define OPCODE_GET_IRQ_STATUS       0x17

/* IRQ masks (common) */
#define IRQ_TX_DONE_MASK            (1 << 2)
#define IRQ_RX_DONE_MASK            (1 << 1)
#define IRQ_RX_ERROR_MASK           (1 << 5)
#define IRQ_ALL                     0xFFFF

/* Helper macros */
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))


/* Sleep helpers */
static inline void msleep(unsigned ms) { usleep(ms * 1000); }

/* NSS helpers */
static inline void set_nss_low(sx1280_t *d)  {
	gpio_write(d->gpio.nss, 0);
}

static inline void set_nss_high(sx1280_t *d) {
	gpio_write(d->gpio.nss, 1);
}

/* Reset sequence */
static void sx1280_reset(sx1280_t *ctx) {
    if (!ctx->gpio.reset) return;
    gpio_write(ctx->gpio.reset, 0);
    msleep(10);
    gpio_write(ctx->gpio.reset, 1);
    msleep(10);
}

/* Busy wait (if busy line exists), timeout in ms */
static uint8_t wait_busy(sx1280_t *ctx, unsigned timeout_ms) {
    if (!ctx->gpio.busy) return 0;
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    while (1) {
        int v = gpio_read(ctx->gpio.busy);
        if (v < 0) return RES_ER;
        if (v = 0) return RES_OK;
        clock_gettime(CLOCK_MONOTONIC, &now);
        unsigned elapsed = (now.tv_sec - start.tv_sec) * 1000 + (now.tv_nsec - start.tv_nsec) / 1000000;
        if (elapsed > timeout_ms) return RES_ER;
        msleep(1);
    }
}


static uint8_t spi_write(sx1280_t *ctx, uint8_t *tx_buf, uint32_t tx_len) {
    if (!ctx) return RES_ER;
    if (wait_busy(ctx, 2000) != 0) return RES_ER;
	SPI_Send(ctx->spi, tx_buf, tx_len);
    return (w == (ssize_t)tx_len) ? RES_OK : RES_ER;
}


static uint8_t spi_transfer(sx1280_t *ctx, uint8_t *tx_buf, uint8_t *rx_buf, uint32_t length) {
    if (!ctx) return RES_ER;
    if (wait_busy(ctx, 2000) != 0) return RES_ER;
    spi_res = SPI_SendAndReceive(ctx, tx_buf, rx_buf, length);
    return (res == SPI_OK) ? RES_ER : RES_OK;
}


static int cmd_write(sx1280_t *ctx, uint8_t opcode, uint8_t *payload, uint32_t payload_len) {
    uint32_t tx_len = 1 + payload_len;
    uint8_t *tx_buf = malloc(tx_len);
    if (!tx_buf) return RES_ER;
    tx_buf[0] = opcode;
    if (payload_len) memcpy(tx_buf + 1, payload, payload_len);
    spi_status_t res = spi_write(ctx, tx_buf, tx_len);
    free(tx_buf);
    return res;
}


static int cmd_read(sx1280_t *ctx, uint8_t opcode, uint8_t* payload, uint32_t payload_len, uint8_t* out, uint32_t out_len) {
	uint32_t buf_len = 1 + payload_len;
    uint8_t *tx_buf = malloc(buf_len);
    uint8_t *rx_buf = malloc(buf_len);
    tx_buf[0] = opcode;
    memcpy(tx_buf + 1, payload, payload_len);
    int ret = spi_transfer(ctx, tx_buf, rx_buf, buf_len);
    if (ret == RES_OK && out && out_len) memcpy(out, rx_buf, out_len);
    free(tx_buf); free(rx_buf);
    return ret;
}


static int sx1280_clear_irqs(sx1280_t *ctx) {
    uint8_t payload[2] = { 0xFF, 0xFF };
    return cmd_write(ctx, OPCODE_CLEAR_IRQ_STATUS, payload, 2);
}


void SX1281SetStandby(sx1280_t *ctx, SX1280_StandbyModes_t standbyConfig)
{
    cmd_write(ctx, RADIO_SET_STANDBY, (uint8_t*)&standbyConfig, 1 );
	
    if( standbyConfig == STDBY_RC )
    {
        OperatingMode = MODE_STDBY_RC;
    }
    else
    {
        OperatingMode = MODE_STDBY_XOSC;
    }
}







/* Map bandwidth_hz -> SX1280 bandwidth index for LoRa
 * Common SX1280 LoRa bandwidth options: 203125, 406250, 812500, 1625000 (approx).
 * We'll map to these common values; adjust if needed.
 */
static uint8_t map_lora_bw(uint32_t bw_hz) {
    if (bw_hz <= 203125) return 0;    /* 203.125 kHz */
    if (bw_hz <= 406250) return 1;    /* 406.25 kHz */
    if (bw_hz <= 812500) return 2;    /* 812.5 kHz */
    return 3;                          /* 1625 kHz */
}

/* Map FLRC bandwidth to index (example mapping) */
static uint8_t map_flrc_bw(uint32_t bw_hz) {
    /* FLRC typical: 162500, 325000, 650000, 1300000 */
    if (bw_hz <= 162500) return 0;
    if (bw_hz <= 325000) return 1;
    if (bw_hz <= 650000) return 2;
    return 3;
}

/* Program modulation & packet params according to ctx->cfg */
static int sx_program_params(sx1280_t *ctx) {
    int r;
    /* Set packet type (LoRa = 0x01, FLRC = 0x02) */
    uint8_t pkt_type = (ctx->cfg.modulation == SX1280_MOD_LORA) ? 0x01 : 0x02;
    r = sx_cmd_write(ctx, OPCODE_SET_PACKET_TYPE, &pkt_type, 1);
    if (r) return r;

    /* Frequency: convert freq_hz to FRF as FRF = freq / FSTEP; FSTEP = 52e6 / 2^25 */
    double fstep = 52000000.0 / (double)(1UL << 25);
    uint32_t frf = (uint32_t)((double)ctx->cfg.frequency_hz / fstep + 0.5);
    uint8_t freq_payload[4] = {
        (uint8_t)((frf >> 24) & 0xFF),
        (uint8_t)((frf >> 16) & 0xFF),
        (uint8_t)((frf >> 8) & 0xFF),
        (uint8_t)(frf & 0xFF)
    };
    r = sx_cmd_write(ctx, OPCODE_SET_RF_FREQUENCY, freq_payload, 4);
    if (r) return r;

    /* SET_MODULATION_PARAMS (opcode 0x8B) and SET_PACKET_PARAMS (0x8C)
     * Encodings below follow typical SX1280 conventions:
     *
     * For LoRa:
     *   SET_MODULATION_PARAMS payload (4 bytes):
     *     [ SpreadingFactor (e.g. 7..12), BandwidthIndex (0..3), CodeRate (1..4), LowDatarateOptimize(0/1) ]
     *
     *   SET_PACKET_PARAMS payload (6 bytes):
     *     [ PreambleLength MSB, PreambleLength LSB, HeaderType (0=Implicit,1=Explicit),
     *       PayloadLength, CRC (0=OFF,1=ON), InvertIQ (0/1))
     *
     * For FLRC:
     *   SET_MODULATION_PARAMS payload (4 bytes):
     *     [ BitRateIndex, BandwidthIndex, ModParam3, ModParam4 ] (we place placeholders)
     *
     * NOTE: Please cross-check these byte layouts with your SX1280 datasheet / firmware doc.
     */

    if (ctx->cfg.modulation == SX1280_MOD_LORA) {
        uint8_t sf = ctx->cfg.lora_spread_factor; /* 7..12 */
        if (sf < 7) sf = 7;
        if (sf > 12) sf = 12;
        uint8_t bw_idx = map_lora_bw(ctx->cfg.bandwidth_hz);
        uint8_t cr = ctx->cfg.coding_rate; /* 1..4 maps to 1..4 (4/5..4/8) */
        if (cr < 1 || cr > 4) cr = 1;
        uint8_t lowdr = 0; /* set 0 by default, could enable for SF>10 */

        uint8_t mod_payload[4] = { sf, bw_idx, cr - 1 /* often CR encoded as 0..3 */, lowdr };
        r = sx_cmd_write(ctx, OPCODE_SET_MODULATION_PARAMS, mod_payload, sizeof(mod_payload));
        if (r) return r;

        /* Packet params */
        uint16_t pre = ctx->cfg.preamble_len;
        uint8_t pre_msb = (pre >> 8) & 0xFF;
        uint8_t pre_lsb = pre & 0xFF;
        uint8_t header_type = 1; /* explicit header */
        uint8_t payload_len = (uint8_t)(ctx->cfg.packet_len & 0xFF);
        uint8_t crc = ctx->cfg.crc_on ? 1 : 0;
        uint8_t invert_iq = 0;

        uint8_t pkt_payload[6] = { pre_msb, pre_lsb, header_type, payload_len, crc, invert_iq };
        r = sx_cmd_write(ctx, OPCODE_SET_PACKET_PARAMS, pkt_payload, sizeof(pkt_payload));
        if (r) return r;
    } else { /* FLRC */
        /* Example FLRC modulation mapping:
         * Bitrate index mapping: choose closest for desired bandwidth/preset datarate
         * This is a simplified mapping. For accurate behavior consult datasheet.
         */
        uint8_t br_idx = 0; /* placeholder choose a typical index */
        uint8_t bw_idx = map_flrc_bw(ctx->cfg.bandwidth_hz);
        uint8_t mod3 = 0;
        uint8_t mod4 = 0;
        uint8_t mod_payload[4] = { br_idx, bw_idx, mod3, mod4 };
        r = sx_cmd_write(ctx, OPCODE_SET_MODULATION_PARAMS, mod_payload, sizeof(mod_payload));
        if (r) return r;

        /* Packet params for FLRC (example layout) */
        uint16_t pre = ctx->cfg.preamble_len;
        uint8_t pre_msb = (pre >> 8) & 0xFF;
        uint8_t pre_lsb = pre & 0xFF;
        uint8_t packet_type_flags = 0x00; /* depends on header type, CRC etc. We'll set a general case */
        uint8_t payload_len = (uint8_t)(ctx->cfg.packet_len & 0xFF);
        uint8_t crc = ctx->cfg.crc_on ? 1 : 0;
        uint8_t pkt_payload[6] = { pre_msb, pre_lsb, packet_type_flags, payload_len, crc, 0x00 };
        r = sx_cmd_write(ctx, OPCODE_SET_PACKET_PARAMS, pkt_payload, sizeof(pkt_payload));
        if (r) return r;
    }

    return 0;
}

/* Create context, open SPI and configure GPIO lines using libgpiod */
sx1280_t *sx1280_init(const sx1280_pinout_t *pins, const sx1280_config_t *cfg) {
    if (!pins || !cfg) {
        errno = EINVAL;
        return NULL;
    }


    sx1280_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) return NULL;

	sx1280_fd_t gpio;

    /* copy pin & cfg */
    ctx->gpio = &gpio;
    ctx->cfg = *cfg;

    ctx->spi_speed_hz = pins->spi_speed_hz ? pins->spi_speed_hz : 2000000;
    ctx->spi_dev = strdup(pins->spi_dev ? pins->spi_dev : "/dev/spidev1.0");
    if (!ctx->spi_dev) { free(ctx); return NULL; }

    /* Open SPI device */
    ctx->spi_fd = open(ctx->spi_dev, O_RDWR);
    if (ctx->spi_fd < 0) { perror("open spi"); goto fail; }
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    if (ioctl(ctx->spi_fd, SPI_IOC_WR_MODE, &mode) < 0) perror("SPI mode set");
    if (ioctl(ctx->spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) perror("SPI bits set");
    if (ioctl(ctx->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &ctx->spi_speed_hz) < 0) perror("SPI speed set");


    ctx->gpio.nss = gpiod_chip_open_lookup(NULL); /* open first usable gpiochip */
    if (!ctx->chip) {
        ctx->chip = gpiod_chip_open("/dev/gpiochip0");
        if (!ctx->chip) {
            perror("gpiod_chip_open");
            goto fail;
        }
    }

    /* Request lines by offset (the user supplies Linux GPIO numbers).
     * We use gpiod_chip_get_line_by_number (gpiod_chip_get_line) depending on libgpiod version.
     */
    ctx->pins.nss = gpiod_chip_get_line(ctx->chip, ctx->pins.pin_nss);
    ctx->pins.reset = gpiod_chip_get_line(ctx->chip, ctx->pins.pin_reset);
    ctx->pins.busy = (ctx->pins.pin_busy >= 0) ? gpiod_chip_get_line(ctx->chip, ctx->pins.pin_busy) : NULL;
    ctx->pins.dio1 = (ctx->pins.pin_dio1 >= 0) ? gpiod_chip_get_line(ctx->chip, ctx->pins.pin_dio1) : NULL;

    if (!ctx->pins.nss || !ctx->pins.reset) {
        fprintf(stderr, "required gpio lines not found (nss/reset)\n");
        goto fail;
    }

    /* Request outputs for NSS and RESET */
    if (gpiod_line_request_output(ctx->pins.nss, "sx1280", 1) < 0) { perror("gpiod request nss"); goto fail; } /* idle high */
    if (gpiod_line_request_output(ctx->pins.reset, "sx1280", 1) < 0) { perror("gpiod request reset"); goto fail; }

    /* Request inputs for busy/dio1 if present */
    if (ctx->pins.busy) {
        if (gpiod_line_request_input(ctx->pins.busy, "sx1280") < 0) {
            perror("gpiod request busy");
            /* not fatal */
            gpiod_line_release(ctx->pins.busy);
            ctx->pins.busy = NULL;
        }
    }
    if (ctx->pins.dio1) {
        if (gpiod_line_request_both_edges_events(ctx->pins.dio1, "sx1280") < 0) {
            perror("gpiod request dio1");
            gpiod_line_release(ctx->pins.dio1);
            ctx->pins.dio1 = NULL;
        }
    }

    /* Reset radio */
    radio_reset(ctx);
    /* Wait busy clear if available */
    wait_busy(ctx, 2000);

    /* Standby: use SET_PACKET_TYPE later in program_params */
    sx_clear_irqs(ctx);

    /* Program modulation and packet params */
    if (sx_program_params(ctx) != 0) {
        fprintf(stderr, "sx_program_params failed\n");
        goto fail;
    }

    sx_clear_irqs(ctx);

    return ctx;

fail:
    if (ctx) {
        if (ctx->spi_fd >= 0) close(ctx->spi_fd);
        if (ctx->spi_dev) free(ctx->spi_dev);
        if (ctx->pins.nss) gpiod_line_release(ctx->pins.nss);
        if (ctx->pins.reset) gpiod_line_release(ctx->pins.reset);
        if (ctx->pins.busy) gpiod_line_release(ctx->pins.busy);
        if (ctx->pins.dio1) gpiod_line_release(ctx->pins.dio1);
        if (ctx->chip) gpiod_chip_close(ctx->chip);
        free(ctx);
    }
    return NULL;
}

/* Free resources */
void sx1280_deinit(sx1280_t *ctx) {
    if (!ctx) return;
    sx_clear_irqs(ctx);
    if (ctx->pins.nss) { gpiod_line_set_value(ctx->pins.nss, 1); gpiod_line_release(ctx->pins.nss); }
    if (ctx->pins.reset) { gpiod_line_set_value(ctx->pins.reset, 1); gpiod_line_release(ctx->pins.reset); }
    if (ctx->pins.busy) gpiod_line_release(ctx->pins.busy);
    if (ctx->pins.dio1) gpiod_line_release(ctx->pins.dio1);
    if (ctx->chip) gpiod_chip_close(ctx->chip);
    if (ctx->spi_fd >= 0) close(ctx->spi_fd);
    if (ctx->spi_dev) free(ctx->spi_dev);
    free(ctx);
}

/* Send payload (writes to buffer, starts TX and waits for DIO1 or polls IRQ status).
 * Note: We assume payload length <= 255. Adjust as needed.
 */
int sx1280_send(sx1280_t *ctx, const uint8_t *payload, size_t len, uint32_t timeout_ms) {
    if (!ctx || !payload || len == 0 || len > 255) return -1;
    /* write buffer: opcode + offset + data */
    size_t tx_len = 2 + len;
    uint8_t *tx = malloc(tx_len);
    if (!tx) return -1;
    tx[0] = OPCODE_WRITE_BUFFER;
    tx[1] = 0x00; /* offset */
    memcpy(tx + 2, payload, len);
    int r = sx_spi_write_raw(ctx, tx, tx_len);
    free(tx);
    if (r) return -1;

    /* set TX: payload timeouts in ms */
    uint8_t tx_cmd[3] = { OPCODE_SET_TX, (uint8_t)((timeout_ms >> 8) & 0xFF), (uint8_t)(timeout_ms & 0xFF) };
    r = sx_spi_write_raw(ctx, tx_cmd, 3);
    if (r) return -1;

	/* fallback: poll IRQ status via GET_IRQ_STATUS (not fully implemented), simple sleep */
	//msleep(timeout_ms);
	sx_clear_irqs(ctx);
	return 0;
}

/* Very simplified receive: set RX, wait for DIO1 event, then READ_BUFFER full */
int sx1280_receive(sx1280_t *ctx, uint8_t *out_buf, size_t max_len, uint32_t rx_timeout_ms, size_t *out_len) {
    if (!ctx || !out_buf || !out_len) return -1;
    *out_len = 0;

    /* Clear IRQs */
    sx_clear_irqs(ctx);

    /* SET_RX: opcode + (2-byte timeout) - simplified */
    uint8_t rx_cmd[3] = { OPCODE_SET_RX, (uint8_t)((rx_timeout_ms >> 8) & 0xFF), (uint8_t)(rx_timeout_ms & 0xFF) };
    if (sx_spi_write_raw(ctx, rx_cmd, 3) != 0) return -1;

    /* Wait DIO1 */
    if (!ctx->pins.dio1) {
        msleep(rx_timeout_ms);
        return -2; /* timeout fallback */
    }

    //struct gpiod_line_event event;
    //struct timespec ts;
    //ts.tv_sec = rx_timeout_ms / 1000;
    //ts.tv_nsec = (rx_timeout_ms % 1000) * 1000000;
    //int ev = gpiod_line_event_wait(ctx->pins.dio1, &ts);
    //if (ev != 1) return -2; /* timeout or error */

    //if (gpiod_line_event_read(ctx->pins.dio1, &event) != 0) return -1;

    /* After RxDone, READ_BUFFER:
     * READ_BUFFER sequence: send opcode, offset, dummy, read payload up to max_len.
     * We'll request max_len bytes; user can then inspect *out_len (we return max_len here).
     * A production driver should read actual RX length from GET_PACKET_STATUS or similar.
     */
    size_t tx_len = 3 + max_len;
    uint8_t *tx = calloc(1, tx_len);
    uint8_t *rx = calloc(1, tx_len);
    if (!tx || !rx) { free(tx); free(rx); return -1; }
    tx[0] = OPCODE_READ_BUFFER;
    tx[1] = 0x00; /* offset */
    tx[2] = 0x00; /* dummy */
    if (sx_spi_transfer(ctx, tx, rx, tx_len) != 0) { free(tx); free(rx); return -1; }
    /* copy payload to out_buf */
    size_t copy_len = max_len;
    memcpy(out_buf, rx + 3, copy_len);
    *out_len = copy_len;
    free(tx); free(rx);

    sx_clear_irqs(ctx);
    return 0;
}

int sx1280_reconfigure(sx1280_t *ctx, const sx1280_config_t *cfg) {
    if (!ctx || !cfg) return -1;
    ctx->cfg = *cfg;
    return sx_program_params(ctx);
}

const sx1280_config_t *sx1280_get_config(const sx1280_t *ctx) {
    if (!ctx) return NULL;
    return &ctx->cfg;
}
