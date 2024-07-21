#include <stddef.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "rfm69.h"

#define SPI_BUS 0
#define GPIO_RFM_RESET  33 /** GPIO33 */
#define GPIO_RFM_DIO0   37
#define GPIO_RFM_DIO3   39
#define GPIO_RFM_NSS     8 /** Chip Select */
#define GPIO_RFM_MISO    9
#define GPIO_RFM_MOSI   10
#define GPIO_RFM_SCK    11


void main_on_rfm69_rx(struct rfm69_device* dev, uint8_t* data, uint16_t size, int16_t rssi) {
    printf("%02x%02x%02x%02x,%d\n", data[0], data[1], data[2], data[3], rssi);
}

int main() {
    printf("rfm69.c -> main()\n");

    struct rfm69_pins pins = {
        .reset = GPIO_RFM_RESET,
        .dio0 = GPIO_RFM_DIO0,
        .dio3 = GPIO_RFM_DIO3,
        .spi_cs = GPIO_RFM_NSS
    };

    struct rfm69_device* dev = rfm69_init(&pins, SPI_BUS);
    if(dev == NULL){
        printf("rfm69_init fail\n");
    }

    rfm69_receive(dev, &(struct rfm69_config) {
        .center_freq_hz = 434000000,
        .br = RFM69_BITRATE_25_KBPS,
        .dcc = RFM69_DC_CANCEL_PRXBW_4,
        .modulation = RFM69_MODULATION_FSK,
        .mode = RFM69_DATA_MODE_PACKET,
        .shaping = RFM69_MODULATION_SHAPING_FSK_NONE,
        .sync_words = { 0xd3, 0x91 },
        .sync_count = 2,
        .preamble_length = 2,
        .payload_length = 4,
        .fsk = {
            .freq_deviation_hz = 25000,
            .bw = RFM69_BW_FSK_50_0_khz
        }        
    }, main_on_rfm69_rx);

    return 0;
}
