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

    return 0;
}
