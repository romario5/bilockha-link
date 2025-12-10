#include <stdio.h>
#include <string.h>
#include "sx1280.h"

int main(void) {


    sx1280_pinout_t pins = {
        .spi_dev = "/dev/spidev1.0",
        .spi_speed_hz = 4000000,
        .pin_sck = 18,
        .pin_mosi = 20,
        .pin_miso = 19,
        .pin_nss = 25,
        .pin_reset = 24,
        .pin_busy = 23,
        .pin_dio1 = 22
    };

    sx1280_config_t cfg = {
        .modulation = SX1280_MOD_LORA,
        .frequency_hz = 2403000000U,
        .bandwidth_hz = 406250, /* example 406.25 kHz */
        .lora_spread_factor = 7,
        .coding_rate = 1, /* 1 -> 4/5 */
        .crc_on = 1,
        .preamble_len = 12,
        .packet_len = 32
    };

    sx1280_t *dev = sx1280_init(&pins, &cfg);
    if (!dev) {
        fprintf(stderr, "sx1280_init failed\n");
        return 1;
    }

    const char *msg = "Hello from SX1280";
    if (sx1280_send(dev, (const uint8_t *)msg, strlen(msg), 2000) != 0) {
        fprintf(stderr, "sx1280_send failed\n");
    } else {
        printf("Packet sent\n");
    }

    uint8_t buf[256];
    size_t rlen = 0;
    int r = sx1280_receive(dev, buf, sizeof(buf), 5000, &rlen);
    if (r == -2) {
        printf("rx timeout\n");
    } else if (r != 0) {
        printf("rx error\n");
    } else {
        printf("rx %zu bytes: ", rlen);
        fwrite(buf, 1, rlen, stdout);
        putchar('\n');
    }

    sx1280_deinit(dev);
    return 0;
}
