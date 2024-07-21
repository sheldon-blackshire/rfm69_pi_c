#ifndef RFM69_PI_C_RFM69_H_
#define RFM69_PI_C_RFM69_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RFM69_MAX_FIFO_BYTES 66
#define RFM69_SPI_BUS_CLOCK_HZ 10000000

/**  */
struct rfm69_device;


/** @enum rfm69_bit_rate
 *  @brief Preset bit rates from Table 9 of the rfm69 datasheet.
 *  @note In packet/cont mode with Gaussian filtering enabled, the bit
 *  rate can be set to a value not in the following table by:
 *  BR = F_xosc / BitRate
 */
enum rfm69_bit_rate {
    /** Classical modem baud rates, multiples of 1.2 kbps */
    RFM69_BITRATE_1_2_KBPS = 0,
    RFM69_BITRATE_2_4_KBPS,
    RFM69_BITRATE_4_8_KBPS,
    RFM69_BITRATE_9_6_KBPS,
    RFM69_BITRATE_19_2_KBPS,
    RFM69_BITRATE_38_4_KBPS,
    RFM69_BITRATE_76_8_KBPS,
    RFM69_BITRATE_153_6_KBPS,
    /** Classical modem baud rates, multiples of 0.9 kbps */
    RFM69_BITRATE_57_6_KBPS,
    RFM69_BITRATE_115_2_KBPS,
    /** Round bit rates, multiples of 12.5, 25, and 50 kbps */
    RFM69_BITRATE_12_5_KBPS,
    RFM69_BITRATE_25_KBPS,
    RFM69_BITRATE_50_KBPS,
    RFM69_BITRATE_100_KBPS,
    RFM69_BITRATE_150_KBPS,
    RFM69_BITRATE_200_KBPS,
    RFM69_BITRATE_250_KBPS,
    RFM69_BITRATE_300_KBPS
};

enum rfm69_bandwidth_fsk {
    RFM69_BW_FSK_2_6_khz = 0,
    RFM69_BW_FSK_3_1_khz,
    RFM69_BW_FSK_3_9_khz,
    RFM69_BW_FSK_5_2_khz,
    RFM69_BW_FSK_6_3_khz,
    RFM69_BW_FSK_7_8_khz,
    RFM69_BW_FSK_10_4_khz,
    RFM69_BW_FSK_12_5_khz,
    RFM69_BW_FSK_15_6_khz,
    RFM69_BW_FSK_20_8_khz,
    RFM69_BW_FSK_25_0_khz,
    RFM69_BW_FSK_31_3_khz,
    RFM69_BW_FSK_41_7_khz,
    RFM69_BW_FSK_50_0_khz,
    RFM69_BW_FSK_62_5_khz,
    RFM69_BW_FSK_83_3_khz,
    RFM69_BW_FSK_100_0_khz,
    RFM69_BW_FSK_125_0_khz,
    RFM69_BW_FSK_166_7_khz,
    RFM69_BW_FSK_200_0_khz,
    RFM69_BW_FSK_250_0_khz,
    RFM69_BW_FSK_333_3_khz,
    RFM69_BW_FSK_400_0_khz,
    RFM69_BW_FSK_500_0_khz
};

enum rfm69_bandwidth_ook {
    RFM69_BW_OOK_1_3_khz = ((0b10 << 3) | 7),
    RFM69_BW_OOK_1_6_khz = ((0b01 << 3) | 7),
    RFM69_BW_OOK_2_0_khz = ((0b00 << 3) | 7),
    RFM69_BW_OOK_2_6_khz = ((0b10 << 3) | 6),
    RFM69_BW_OOK_3_1_khz = ((0b01 << 3) | 6),
    RFM69_BW_OOK_3_9_khz = ((0b00 << 3) | 6),
    RFM69_BW_OOK_5_2_khz = ((0b10 << 3) | 5),
    RFM69_BW_OOK_6_3_khz = ((0b01 << 3) | 5),
    RFM69_BW_OOK_7_8_khz = ((0b00 << 3) | 5),
    RFM69_BW_OOK_10_4_khz = ((0b10 << 3) | 4),
    RFM69_BW_OOK_12_5_khz = ((0b01 << 3) | 4),
    RFM69_BW_OOK_15_6_khz = ((0b00 << 3) | 4),
    RFM69_BW_OOK_20_8_khz = ((0b10 << 3) | 3),
    RFM69_BW_OOK_25_0_khz = ((0b01 << 3) | 3),
    RFM69_BW_OOK_31_3_khz = ((0b00 << 3) | 3),
    RFM69_BW_OOK_41_7_khz = ((0b10 << 3) | 2),
    RFM69_BW_OOK_50_0_khz = ((0b01 << 3) | 2),
    RFM69_BW_OOK_62_5_khz = ((0b00 << 3) | 2),
    RFM69_BW_OOK_83_3_khz = ((0b10 << 3) | 1),
    RFM69_BW_OOK_100_0_khz = ((0b01 << 3) | 1),
    RFM69_BW_OOK_125_0_khz = ((0b00 << 3) | 1),
    RFM69_BW_OOK_166_7_khz = ((0b10 << 3) | 0),
    RFM69_BW_OOK_200_0_khz = ((0b01 << 3) | 0),
    RFM69_BW_OOK_250_0_khz = ((0b00 << 3) | 0),
};

/**
 * @enum rfm69_dc_cancel_pbw
 * @brief DC cancellation is required in zero-IF
 * architecture transceivers to remove any DC offset
 * generated through self-reception.
 *
 * @note See Table 15: Available DCC Cutoff Frequencies
 * @note The default value of DccFreq cutoff frequency is
 * typically 4% of the RxBw (channel filter BW). The cutoff
 * frequency of the DCC can however be increased to slightly
 * improve the sensitivity, under wider modulation conditions.
 * It is advised to adjust the DCC setting while monitoring the
 * receiver sensitivity.
 */
enum rfm69_dc_cancel_prxbw {
    RFM69_DC_CANCEL_PRXBW_16, // 16.000 %
    RFM69_DC_CANCEL_PRXBW_8, //  8.000 %
    RFM69_DC_CANCEL_PRXBW_4, //  4.000 % (Default)
    RFM69_DC_CANCEL_PRXBW_2, //  2.000 %
    RFM69_DC_CANCEL_PRXBW_1, //  1.000 %
    RFM69_DC_CANCEL_PRXBW_0_5, //  0.500 %
    RFM69_DC_CANCEL_PRXBW_0_25, //  0.250 %
    RFM69_DC_CANCEL_PRXBW_0_125 //  0.125 %
};

enum rfm69_modulation {
    RFM69_MODULATION_FSK = 0,
    RFM69_MODULATION_OOK
};

enum rfm69_data_mode {
    RFM69_DATA_MODE_PACKET = 0,
    RFM69_DATA_MODE_CONT_BIT_SYNC,
    RFM69_DATA_MODE_CONT_NO_BIT_SYNC
};

enum rfm69_modulatuion_shaping {
    RFM69_MODULATION_SHAPING_FSK_NONE = 0,
    RFM69_MODULATION_SHAPING_FSK_GAUSS_BT_1_0,
    RFM69_MODULATION_SHAPING_FSK_GAUSS_BT_0_5,
    RFM69_MODULATION_SHAPING_FSK_GAUSS_BT_0_3,
    RFM69_MODULATION_SHAPING_OOK_NONE,
    RFM69_MODULATION_SHAPING_OOK_FCUT_BR,
    RFM69_MODULATION_SHAPING_OOK_FCUT_2BR,
};

struct rfm69_fsk_config {

    /**
     * @brief The absolute difference (Hz) between the center frequency of
     * signal and the max or min modulated frequency.
     * @note This parameter is only valid during FSK transmission
     * @note Modulation in this context means to encoding information
     * on one carrier wave by changing its frequency.
     * @warning To ensure proper modulation, Fdev + BR/2 <= 500 KHz
     */
    uint32_t freq_deviation_hz;

    enum rfm69_bandwidth_fsk bw;
};

struct rfm69_ook_config {
    enum rfm69_bandwidth_ook bw;
};

struct rfm69_config {
    /**
     * @brief Center frequency (Hz) to use for rx&tx
     * @note The RFM69 transceiver module supports the 315 MHz,
     * 433 MHz, 868 MHz, and 915 MHz bands.
     * @warning The RFM69 isn't tuned for all of the supported bands. HopeRF
     * makes different variants for each of the bands. Please ensure you are
     * using the module that matches your desired center frequency.
     */
    uint32_t center_freq_hz;

    /**
     * @brief Bitrate: Rate at which bits are shifted out of the radio using
     * a specified modulation technique.
     * @warning Bit rate matching between the transmitter and receiver must
     * be better than 6.5%. If the bit rates aren't the same, there are limits
     * to the number of consecutive 1s and 0s the receiver can correctly
     * receive. Consult Section 3.4.13 of the datasheet for more information.
     */
    enum rfm69_bit_rate br;
    enum rfm69_dc_cancel_prxbw dcc;
    enum rfm69_modulation modulation;
    enum rfm69_data_mode mode;
    enum rfm69_modulatuion_shaping shaping;

    uint8_t sync_words[6];
    uint8_t sync_count;
    uint8_t preamble_length;
    uint8_t payload_length;
    uint8_t power_level;

    int8_t cca_rssi;
    uint16_t cca_timeout_ms;


    struct rfm69_fsk_config fsk;
    struct rfm69_ook_config ook;
};

struct rfm69_pins {
    int spi_cs;
    int reset;
    int dio0;
    int dio1;
    int dio2;
    int dio3;
    int dio4;
    int dio5;
};

/**
 * @typedef rfm69_tx_cb()
 * @brief Callback API for transmitting data asynchronously
 * @param device
 */
typedef void (*rfm69_tx_cb)(struct rfm69_device* dev);

/**
 * @typedef rfm69_recv_cb()
 * @brief Callback API for receiving data asynchronously
 * @param device
 * @param data
 * @param size
 * @param rssi
 */
typedef void (*rfm69_rx_cb)(struct rfm69_device* dev, uint8_t* data, uint16_t size, int16_t rssi);

/**
 * @brief Initializes the transceiver and places it in low power mode.
 * @param pins
 * @param spi SPI bus number (0 or 1).
 * @returns Pointer to the rfm69 device structure on success, otherwise NULL
 */
struct rfm69_device* rfm69_init(const struct rfm69_pins* pins, int spi);

/**
 * @brief Callback API for sending data
 * @param device
 * @param data
 * @param size
 * @param config
 * @param callback
 */
int rfm69_transmit(struct rfm69_device* device, 
                   uint8_t* data, uint32_t size,
                   const struct rfm69_config* config, 
                   rfm69_tx_cb callback);

/**
 * @brief Callback API for receiving data
 * @param device
 * @param config
 * @param callback
 */
int rfm69_receive(struct rfm69_device* device,
                  const struct rfm69_config* config, 
                  rfm69_rx_cb callback);

/**
 * @brief Places the transceiver in idle mode, cancelling any receptions
 * or transmissions in progress.
 * @param device
 * @note This places the transceiver in its lowest power state.
 */
int rfm69_idle(struct rfm69_device* device);

/**
 * @brief Prints the rfm69's register table to stdout
 * @param device
 */
int rfm69_print_registers(struct rfm69_device* device);

#ifdef __cplusplus
}
#endif

#endif /* RFM69_PI_C_RFM69_H_ */
