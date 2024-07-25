#include "rfm69.h"
#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** Socket Headers */
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>


#define SPI_BUS 0
#define GPIO_RFM_RESET 33 /** GPIO33 */
#define GPIO_RFM_DIO0 37
#define GPIO_RFM_DIO3 39
#define GPIO_RFM_NSS 8 /** Chip Select */
#define GPIO_RFM_MISO 9
#define GPIO_RFM_MOSI 10
#define GPIO_RFM_SCK 11

#define IP_ADDRESS_UDP "127.0.0.1"

int g_socket = -1;
int g_port = -1;

struct rfm69_config conf = {
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
        .bw = RFM69_BW_FSK_50_0_khz }
};

static int main_udp_broadcast_init(int port) {

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        printf("socket error");
        return -1;
    }

    int yes = 1;
    int ret = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (char*)&yes, sizeof(yes));
    if (ret == -1) {
        perror("setsockopt error");
        return -1;
    }

    return sock;
}

static int main_udp_broadcast_message(const char* message, int socket, int port) {
    if(message == NULL) { return -EINVAL; }
    if(socket  <0) { return -EINVAL;}
    if(port < 0) { return -EINVAL; }

    struct sockaddr_in broadcast_addr;
    memset((void*)&broadcast_addr, 0, sizeof(struct sockaddr_in));

    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(port);
    inet_pton(AF_INET,"127.0.0.1", &broadcast_addr.sin_addr);

    return sendto(socket, message, strlen(message), 0, (struct sockaddr*)&broadcast_addr, sizeof(struct sockaddr_in));
}


void main_on_rfm69_rx(struct rfm69_device* dev, uint8_t* data, uint16_t size, int16_t rssi) {
    char buffer[64] = {0};
    snprintf(buffer, 64,"%02x%02x%02x%02x,%d\n", data[0], data[1], data[2], data[3], rssi);
    if(main_udp_broadcast_message(buffer, g_socket, g_port) < 0){
        printf("payload:%s error:%s", buffer, strerror(errno));
    }
    rfm69_receive(dev, &conf, main_on_rfm69_rx);
}

int main() {

    g_port = 12345;

    printf("rfm69_pi_c: udp_relay %s:%d\n", IP_ADDRESS_UDP, g_port);

    g_socket = main_udp_broadcast_init(g_port);

    if(g_socket < 0) {
        printf("Error establishing socket %d\n", g_socket);
    }

    struct rfm69_pins pins = {
        .reset = GPIO_RFM_RESET,
        .dio0 = GPIO_RFM_DIO0,
        .dio3 = GPIO_RFM_DIO3,
        .spi_cs = GPIO_RFM_NSS
    };

    struct rfm69_device* dev = rfm69_init(&pins, SPI_BUS);
    if (dev == NULL) {
        printf("rfm69_init fail\n");
    }

    rfm69_receive(dev, &conf, main_on_rfm69_rx);

    // rfm69_print_registers(dev);

    while (1)
        ;

    return 0;
}
