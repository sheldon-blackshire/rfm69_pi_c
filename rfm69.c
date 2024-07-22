/* SPDX-License-Identifier: GPL-3.0 */ 
/*
 * Raspberry PI Linux Driver for HopeRf RFM69
 *
 * Author: Sheldon Blackshire <sheldon.blackshire@gmail.com>
 */

#include <errno.h>
#include "rfm69.h"
#include "rfm69_priv.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>


#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

/**
 * @brief Obtain the maximum of two values.
 *
 * @note Arguments are evaluated twice. Use Z_MAX for a GCC-only, single
 * evaluation version
 *
 * @param a First value.
 * @param b Second value.
 *
 * @returns Maximum value of @p a and @p b.
 */
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

/**
 * @brief Obtain the minimum of two values.
 *
 * @note Arguments are evaluated twice. Use Z_MIN for a GCC-only, single
 * evaluation version
 *
 * @param a First value.
 * @param b Second value.
 *
 * @returns Minimum value of @p a and @p b.
 */
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

/**
 * @brief Clamp a value to a given range.
 *
 * @note Arguments are evaluated multiple times. Use Z_CLAMP for a GCC-only,
 * single evaluation version.
 *
 * @param val Value to be clamped.
 * @param low Lowest allowed value (inclusive).
 * @param high Highest allowed value (inclusive).
 *
 * @returns Clamped value.
 */
#define CLAMP(val, low, high) (((val) <= (low)) ? (low) : MIN(val, high))

/**
 * @brief Checks if a value is within range.
 *
 * @note @p val is evaluated twice.
 *
 * @param val Value to be checked.
 * @param min Lower bound (inclusive).
 * @param max Upper bound (inclusive).
 *
 * @retval true If value is within range
 * @retval false If the value is not within range
 */
#define IN_RANGE(val, min, max) ((val) >= (min) && (val) <= (max))

enum rfm69_super_state {
    RFM69_STATE_SLEEP = 0,
    RFM69_STATE_STANDBY,
    RFM69_STATE_FS,
    RFM69_STATE_RX,
    RFM69_STATE_TX
};

struct rfm69_device_dio {
    enum rfm69_pm_dio0_rx pm_dio0_rx;
    enum rfm69_pm_dio0_tx pm_dio0_tx;
    enum rfm69_pm_dio3_rx pm_dio3_rx;
    enum rfm69_pm_dio3_rx pm_dio3_tx;
};

/** @todo Currently a mess. Need to add a lot more structure/organization
 * once a rough prototype is working.
 */
struct rfm69_device {
    int pa_boost;
    int tx_power_max_dbm;
    uint8_t payload[RFM69_MAX_FIFO_BYTES + 1];
    int payload_length;
    int8_t payload_rssi;

    rfm69_rx_cb rx_cb;
    rfm69_rx_cb tx_cb;

    enum rfm69_super_state state;
    struct rfm69_device_dio dio;
    struct rfm69_pins pins;
    int spi;
};

/** todo Work-in-progress. Rpi has two SPI ports, and supports 
 * 2 select lines per port (More if SPI is be bitbanged). Need to
 * transition to a device array instead of a singleton.
*/
static struct rfm69_device self = {
    .pa_boost = 1,
    .tx_power_max_dbm = 20
};

/**
 *  SPI transaction requires the first byte to be the register address (Table 23).
 *  When the most significant bit of the register address is set, the device will
 *  write to the register. Otherwise a read occurs
 */
#define RFM69_WRITE_MASK 0x80
#define RFM69_READ_ADDRESS(x) ((x) & ~RFM69_WRITE_MASK)
#define RFM69_WRITE_ADDRESS(x) ((x) | RFM69_WRITE_MASK)

/**
 * @brief Read an 8-bit register value over SPI
 * @param device 
 * @param address Register address (See Table 23 - Registers Summary for a complete list)
 * @param value Buffer to store the result of the read
 * @retval 0 On Success
 * @retval -EINVAL if `value` is `NULL`
 * @retval -EBADF on Invalid SPI parameter
 * @retval < 0 for an internal i/o error 
 */
static int rfm69_read_reg(const struct rfm69_device* device, uint8_t address, uint8_t* value) {
    if(device == NULL) { return -EINVAL; }
    if(value == NULL) { return -EINVAL; }

    uint8_t buffer[2] = {
        RFM69_READ_ADDRESS(address),
        0 /** Dummy byte, will be replaced with byte from device */
    };

    int ret = wiringPiSPIDataRW(device->spi, buffer, sizeof(buffer) / sizeof(buffer[0]));
    if(ret < 0) {
        return ret;
    }

    *value = buffer[1];

    return 0;
}

/**
 * @brief Write an 8-bit register value over SPI
 * @param device 
 * @param address Register address (See Table 23 - Registers Summary for a complete list)
 * @param value Contains the value to be written to the register
 * @retval 0 On Success
 * @retval -EBADF on Invalid SPI parameter
 * @retval < 0 for an internal i/o error 
 */
static int rfm69_write_reg(const struct rfm69_device* device, uint8_t address, uint8_t value) {
    if(device == NULL) { return -EINVAL; }

    uint8_t buffer[2] = {
        RFM69_WRITE_ADDRESS(address),
        value
    };

    int ret = wiringPiSPIDataRW(device->spi, buffer, sizeof(buffer) / sizeof(buffer[0]));
    if(ret < 0) {
        return ret;
    }

    return 0;
}

/**
 * @brief Reads/Writes the rx/tx fifo
 * @param device 
 * @param buffer Buffer for fifo contents. The first byte should contain 
 * the RFM69_REG_00_FIFO byte with a r/w modifier `RFM69_READ_ADDRESS` or `RFM69_WRITE_ADDRESS`
 * @param length Size of the buffer. This must be the number of bytes to
 * read from the fifo plus the register byte.
 * @retval Success: Number of fifo bytes read/written (Excluding the register byte)
 * @retval -EINVAL if `buffer` is `NULL`
 * @retval -EPROTO if the register byte `buffer[0]` is invalid.
 * @retval -EMSGSIZE if `buffer` is too small
 * @retval -EBADF on Invalid SPI parameter
 * @retval < 0 for an internal I/O error 
 * @example Reads the Fifo
 * ```
 * uint8_t buffer[RFM69_MAX_FIFO_BYTES + 1] = {
 *     RFM69_READ_ADDRESS(RFM69_REG_00_FIFO)
 * };
 * int ret = rfm69_fifo_transaction(buffer, sizeof(buffer)/sizeof(buffer[0]));
 * if(ret < 0){ 
 *     // Handle Error
 * }
 * ```
 */
static int rfm69_fifo_transaction(const struct rfm69_device* device, uint8_t* buffer, uint8_t size) {
    if(device == NULL) { return -EINVAL; }
    if(buffer == NULL) { return -EINVAL;}
    if(size < 2) { return -EINVAL;}

    switch(buffer[0]) {
        case RFM69_READ_ADDRESS(RFM69_REG_00_FIFO):
        case RFM69_WRITE_ADDRESS(RFM69_REG_00_FIFO):
            break;
        default:
            return -EPROTO;
    }

    /** Max transaction size is fifo size + address */
    if(size > RFM69_MAX_FIFO_BYTES + 1) {
        size = RFM69_MAX_FIFO_BYTES + 1; 
    }

    int ret = wiringPiSPIDataRW(device->spi, buffer, size);
    if(ret < 0) {
        return ret;
    }

    return size - 1;
}

/**
 * @brief Update the mode bits of RFM69_REG_01_OPMODE. 
 * @param bus
 * @param mode The transceivers operating mode. See RegOpMode(0x01) from
 * Table 24 (Common Configuration Registers) for more details.
 * @retval 0 on Success
 * @retval -EPROTONOSUPPORT for an invalid `mode` argument
 * @retval -ETIMEDOUT if the register update doesn't complete in time.
 * @retval -EIO if a reg r/w operation fails
 */
static int rfm69_set_mode(const struct rfm69_device* device, uint8_t mode) {
    if(device == NULL) { return -EINVAL; } 

    switch(mode) {
        case RFM69_OPMODE_MODE_RX:
        case RFM69_OPMODE_MODE_TX:
        case RFM69_OPMODE_MODE_FS:
        case RFM69_OPMODE_MODE_STDBY:
        case RFM69_OPMODE_MODE_SLEEP:
            break;
        default:
            return -EPROTONOSUPPORT;
    }

    int ret = 0;
    uint8_t reg_op_mode = 0;
    /** Get the previous value of op_mode to do read-update-write operation. */
    ret = rfm69_read_reg(device, RFM69_REG_01_OPMODE, &reg_op_mode);
    if(ret < 0) {
        return -EIO;
    }

    /** Only bits 2-4 of the RegOpMode contain the mode. Replace those bits
     * with the input argument `mode`. */
    reg_op_mode &= ~RFM69_OPMODE_MODE;
    reg_op_mode |= (mode & RFM69_OPMODE_MODE);

    ret = rfm69_write_reg(device, RFM69_REG_01_OPMODE, reg_op_mode);
    if(ret < 0){
        return -EIO;
    }

    /** Wait for the mode update to complete. Certain functionality is unavailable
     * until the transition has completed. According to Figure 25, when switching to Sleep mode, 
     * the FIFO can only be used once the ModeReady flag from RFM69_REG_27_IRQFLAGS1 is set
     * (quasi immediate from all modes except from Tx). 
     */
    int start = millis();
    while ((millis() - start) < 1000) {
        uint8_t reg_irq_flags1;
        ret = rfm69_read_reg(device, RFM69_REG_27_IRQFLAGS1, &reg_irq_flags1);
        if(ret < 0){
            return -EIO;
        }
        if(reg_irq_flags1 & RFM69_IRQFLAGS1_MODEREADY){
            return 0;
        }
    }

    /** One second is an eternity for the mode flag to be set. If this timeout
     * occurs we have serious problems. */
    return -ETIMEDOUT;
}

static int rfm69_set_rx(struct rfm69_device* device) {
    if(device == NULL) { return -EINVAL; } 

    int ret = rfm69_set_mode(device, RFM69_OPMODE_MODE_RX);
    if(ret == 0){
        device->state = RFM69_STATE_RX;        
    }else{
        printf("rfm69_set_rx %d\n", ret);
    }
    return ret;
}

static int rfm69_set_tx(struct rfm69_device* device) {
    if(device == NULL) { return -EINVAL; } 

    int ret = rfm69_set_mode(device, RFM69_OPMODE_MODE_TX);
    if(ret == 0) {
        device->state = RFM69_STATE_TX;        
    }else{
        printf("rfm69_set_tx %d\n", ret);
    }
    return ret;
}

static int rfm69_sleep(struct rfm69_device* device) {
    if(device == NULL) { return -EINVAL; } 

    int ret = rfm69_set_mode(device, RFM69_OPMODE_MODE_SLEEP);
    if(ret == 0) {
        device->state = RFM69_STATE_SLEEP;        
    }else{
        printf("rfm69_sleep %d\n", ret);
    }
    return ret;
}

static int rfm69_standby(struct rfm69_device* device) {
    if(device == NULL) { return -EINVAL; } 

    int ret = rfm69_set_mode(device, RFM69_OPMODE_MODE_STDBY);
    if(ret == 0) {
        device->state = RFM69_STATE_STANDBY;        
    }else{
        printf("rfm69_sleep %d\n", ret);
    }
    return ret;
}

/**
 * @brief Perform a manual reset on the hardware
 * @param pin GPIOx pin number of the RESET line
 * @retval 0 on Success
 * @retval -EINVAL for an invalid `pin` input
 * @note According to Section 7.2.2 of the datasheet, a manual reset of the RFM69HCW is possible
 * even for applications in which VDD cannot be physically disconnected. Pin RESET should be 
 * pulled high for a hundred microseconds, and then released. The user should then wait for 5 ms 
 * before using the module. 
 */
static int rfm69_reset(int pin) {
    if(pin < 0) { return -EINVAL; }

    /** Force the RESET pin to a known disabled state */
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delay(10);

    /** 1ms for the RESET line active has been experimentally verified */
    digitalWrite(pin, HIGH);
    delay(1);

    /** Don't attempt to access the chip for at least 5ms, 10 to be safe. */
    digitalWrite(pin, LOW);
    delay(10);

    return 0;
}

static int rfm69_set_fsk_frequency_deviation(const struct rfm69_device* device, uint16_t hz) {
    if(device == NULL) { return -EINVAL; }

    int ret;
    const uint16_t f_dev_hz = (uint16_t)((double)hz / (double)RFM69_FSTEP);

    uint8_t msb = (uint8_t)((f_dev_hz >> 8) & 0xff);
    ret = rfm69_write_reg(device, RFM69_REG_05_FDEVMSB, msb);
    if(ret < 0) {
        return ret;
    }

    uint8_t lsb = (uint8_t)(f_dev_hz & 0xff);
    ret = rfm69_write_reg(device, RFM69_REG_06_FDEVLSB, lsb);
    if(ret < 0) {
        return ret;
    }

    return 0;
}

static int rfm69_set_frequency(const struct rfm69_device *device, uint32_t hz) {
    if(device == NULL) { return -EINVAL; }

    int ret;
    const uint32_t freq = (uint32_t)((double)hz / (double)RFM69_FSTEP);

    const uint8_t msb = (uint8_t)((freq >> 16) & 0xff);
    ret = rfm69_write_reg(device, RFM69_REG_07_FRFMSB, msb);
    if(ret < 0) {
        return ret;
    }

    const uint8_t mid = (uint8_t)((freq >> 8) & 0xff);
    ret = rfm69_write_reg(device, RFM69_REG_08_FRFMID, mid);
    if(ret < 0) {
        return ret;
    }

    const uint8_t lsb = (uint8_t)(freq & 0xff);
    ret = rfm69_write_reg(device, RFM69_REG_09_FRFLSB, lsb);
    if(ret < 0) {
        return ret;
    }

    return 0;
}

/**
 * @brief Sets the receivers modulation via the RegDataModul (0x02) register
 * @param device
 * @param data_mode Data processing mode (Packet, Continuous w/wo Bit Sync)
 * @param modulation Modulation Scheme (FSK or OOK)
 * @param shaping Data Shaping (values dependent on `modulation`)
 * @retval 0 on Success
 * @retval -EINVAL on invalid device
 * @retval -EIO on invalid SPI transfer
 */
static int rfm69_set_modulation(const struct rfm69_device *device, 
                                enum rfm69_data_mode mode, 
                                enum rfm69_modulation modulation, 
                                enum rfm69_modulatuion_shaping shaping) {

    if(device == NULL) { return -EINVAL; }

    uint8_t reg_data_modul_02 = 0;

    switch(mode) {
        case RFM69_DATA_MODE_PACKET:
            reg_data_modul_02 |= RFM69_MODULATION_DATA_MODE_PACKET << 5;
            break;
        case RFM69_DATA_MODE_CONT_BIT_SYNC:
            reg_data_modul_02 |= RFM69_MODULATION_DATA_MODE_CONT_SYNTH << 5;
            break;
        case RFM69_DATA_MODE_CONT_NO_BIT_SYNC:
            reg_data_modul_02 |= RFM69_MODULATION_DATA_MODE_CONT_NO_SYNTH << 5;
            break;
        default: return -EPROTO;
    }

    switch(modulation) {
        case RFM69_MODULATION_FSK:
            reg_data_modul_02 |= RFM69_MODULATION_TYPE_FSK << 3;
            break;
        case RFM69_MODULATION_OOK:
            reg_data_modul_02 |= RFM69_MODULATION_TYPE_OOK << 3;
            break;
        default: return -EPROTO;
    }

    switch(shaping) {
        case RFM69_MODULATION_SHAPING_FSK_NONE:
            reg_data_modul_02 |= RFM69_MODULATION_SHAPE_FSK_NONE;
            break;
        case RFM69_MODULATION_SHAPING_FSK_GAUSS_BT_1_0:
            reg_data_modul_02 |= RFM69_MODULATION_SHAPE_FSK_GAUSS_BT_1_0;
            break;
        case RFM69_MODULATION_SHAPING_FSK_GAUSS_BT_0_5:
            reg_data_modul_02 |= RFM69_MODULATION_SHAPE_FSK_GAUSS_BT_0_5;
            break;
        case RFM69_MODULATION_SHAPING_FSK_GAUSS_BT_0_3:
            reg_data_modul_02 |= RFM69_MODULATION_SHAPE_FSK_GAUSS_BT_0_3;
            break;
        case RFM69_MODULATION_SHAPING_OOK_NONE:
            reg_data_modul_02 |= RFM69_MODULATION_SHAPE_OOK_NONE;
            break;
        case RFM69_MODULATION_SHAPING_OOK_FCUT_BR:
            reg_data_modul_02 |= RFM69_MODULATION_SHAPE_OOK_BR;
            break;
        case RFM69_MODULATION_SHAPING_OOK_FCUT_2BR:
            reg_data_modul_02 |= RFM69_MODULATION_SHAPE_OOK_BR2;
            break;
        default: return -EPROTO;
    }

    if(rfm69_write_reg(device, RFM69_REG_02_DATAMODUL, reg_data_modul_02) < 0) {
        return -EIO;
    }

    return 0;
}

static int rfm69_set_bitrate(const struct rfm69_device* device, enum rfm69_bit_rate bit_rate) {

    if(device == NULL) { return -EINVAL; }

    uint8_t reg_03 = 0;
    uint8_t reg_04 = 0;

    switch (bit_rate) {
        /** Classical modem baud rates, multiples of 1.2 kbps */
        case RFM69_BITRATE_1_2_KBPS: reg_03 = 0x68; reg_04 = 0x2B; break;
        case RFM69_BITRATE_2_4_KBPS: reg_03 = 0x34; reg_04 = 0x15; break;
        case RFM69_BITRATE_4_8_KBPS: reg_03 = 0x1A; reg_04 = 0x0B; break;
        case RFM69_BITRATE_9_6_KBPS: reg_03 = 0x0D; reg_04 = 0x05; break;
        case RFM69_BITRATE_19_2_KBPS: reg_03 = 0x06; reg_04 = 0x83; break;
        case RFM69_BITRATE_38_4_KBPS: reg_03 = 0x03; reg_04 = 0x41; break;
        case RFM69_BITRATE_76_8_KBPS: reg_03 = 0x01; reg_04 = 0xA1; break;
        case RFM69_BITRATE_153_6_KBPS: reg_03 = 0x00; reg_04 = 0xD0; break;
        /** Classical modem baud rates, multiples of 0.9 kbps */
        case RFM69_BITRATE_57_6_KBPS:  reg_03 = 0x02; reg_04 = 0x2C; break;
        case RFM69_BITRATE_115_2_KBPS: reg_03 = 0x01; reg_04 = 0x16; break;
        /** Round bit rates, multiples of 12.5, 25, and 50 kbps */
        case RFM69_BITRATE_12_5_KBPS: reg_03 = 0x0a; reg_04 = 0x00; break;
        case RFM69_BITRATE_25_KBPS:   reg_03 = 0x05; reg_04 = 0x00; break;
        case RFM69_BITRATE_50_KBPS:   reg_03 = 0x02; reg_04 = 0x80; break;
        case RFM69_BITRATE_100_KBPS:  reg_03 = 0x01; reg_04 = 0x40; break;
        case RFM69_BITRATE_150_KBPS:  reg_03 = 0x00; reg_04 = 0xd5; break;
        case RFM69_BITRATE_200_KBPS:  reg_03 = 0x00; reg_04 = 0xa0; break;
        case RFM69_BITRATE_250_KBPS:  reg_03 = 0x00; reg_04 = 0x80; break;
        case RFM69_BITRATE_300_KBPS:  reg_03 = 0x00; reg_04 = 0x6b; break;
    default:
        return -EINVAL;
    }

    int ret = 0;
    ret = rfm69_write_reg(device, RFM69_REG_03_BITRATEMSB, reg_03);
    if(ret < 0) { 
        return -EIO; 
    }
    ret = rfm69_write_reg(device, RFM69_REG_04_BITRATELSB, reg_04);
    if(ret < 0) { 
        return -EIO; 
    }
    return 0;
}

static int rfm69_set_fsk_bandwidth(const struct rfm69_device* device, enum rfm69_bandwidth_fsk khz, enum rfm69_dc_cancel_prxbw prxbw) {
    if(device == NULL) { return -EINVAL; }
    
    uint8_t bw = 0;
    switch(khz) {
        case RFM69_BW_FSK_2_6_khz: bw = ((0b10 << 3) | 7); break; // 2.6 khz
        case RFM69_BW_FSK_3_1_khz: bw = ((0b01 << 3) | 7); break; // 3.1 khz
        case RFM69_BW_FSK_3_9_khz: bw = ((0b00 << 3) | 7); break; // 3.9 khz
        case RFM69_BW_FSK_5_2_khz: bw = ((0b10 << 3) | 6); break; // 5.2 khz
        case RFM69_BW_FSK_6_3_khz: bw = ((0b01 << 3) | 6); break; // 6.3 khz
        case RFM69_BW_FSK_7_8_khz: bw = ((0b00 << 3) | 6); break; // 7.8 khz
        case RFM69_BW_FSK_10_4_khz: bw = ((0b10 << 3) | 5); break; // 10.4 khz
        case RFM69_BW_FSK_12_5_khz: bw = ((0b01 << 3) | 5); break; // 12.5 khz
        case RFM69_BW_FSK_15_6_khz: bw = ((0b00 << 3) | 5); break; // 15.6 khz
        case RFM69_BW_FSK_20_8_khz: bw = ((0b10 << 3) | 4); break; // 20.8 khz
        case RFM69_BW_FSK_25_0_khz: bw = ((0b01 << 3) | 4); break; // 25.0 khz
        case RFM69_BW_FSK_31_3_khz: bw = ((0b00 << 3) | 4); break; // 31.3 khz
        case RFM69_BW_FSK_41_7_khz: bw = ((0b10 << 3) | 3); break; // 41.7 khz
        case RFM69_BW_FSK_50_0_khz: bw = ((0b01 << 3) | 3); break; // 50.0 khz
        case RFM69_BW_FSK_62_5_khz: bw = ((0b00 << 3) | 3); break; // 62.5 khz
        case RFM69_BW_FSK_83_3_khz: bw = ((0b10 << 3) | 2); break; // 83.3 khz
        case RFM69_BW_FSK_100_0_khz: bw = ((0b01 << 3) | 2); break; // 100.0 khz
        case RFM69_BW_FSK_125_0_khz: bw = ((0b00 << 3) | 2); break; // 125.0 khz
        case RFM69_BW_FSK_166_7_khz: bw = ((0b10 << 3) | 1); break; // 166.7 khz
        case RFM69_BW_FSK_200_0_khz: bw = ((0b01 << 3) | 1); break; // 200.0 khz
        case RFM69_BW_FSK_250_0_khz: bw = ((0b00 << 3) | 1); break; // 250.0 khz
        case RFM69_BW_FSK_333_3_khz: bw = ((0b10 << 3) | 0); break; // 333.3 khz
        case RFM69_BW_FSK_400_0_khz: bw = ((0b01 << 3) | 0); break; // 400.0 khz
        case RFM69_BW_FSK_500_0_khz: bw = ((0b00 << 3) | 0); break; // 500.0 khz 
        default:
            return -EINVAL;
    }

    uint8_t percent = 0;
    switch(prxbw) {
        case RFM69_DC_CANCEL_PRXBW_16: percent = (0b000 << 5); break;
        case RFM69_DC_CANCEL_PRXBW_8: percent = (0b001 << 5); break;
        case RFM69_DC_CANCEL_PRXBW_4: percent = (0b010 << 5); break;
        case RFM69_DC_CANCEL_PRXBW_2: percent = (0b011 << 5); break;
        case RFM69_DC_CANCEL_PRXBW_1: percent = (0b100 << 5); break;
        case RFM69_DC_CANCEL_PRXBW_0_5: percent = (0b101 << 5); break;
        case RFM69_DC_CANCEL_PRXBW_0_25: percent = (0b110 << 5); break;
        case RFM69_DC_CANCEL_PRXBW_0_125: percent = (0b111 << 5); break;
        default:
            return -EINVAL;
    }

    int ret = 0;
    uint8_t reg_19 = (uint8_t)khz | (uint8_t)percent;
    ret = rfm69_write_reg(device, RFM69_REG_19_RXBW, reg_19);
    if(ret < 0) { 
        return -EIO; 
    }
    ret = rfm69_write_reg(device, RFM69_REG_1A_AFCBW, reg_19);
    if(ret < 0) { 
        return -EIO; 
    }
    return 0;
}

static int rfm69_set_digital_auto_gain_control(const struct rfm69_device* device, uint8_t dagc) {

    if(device == NULL) { return -EINVAL; }

    return rfm69_write_reg(device, RFM69_REG_6F_TESTDAGC, dagc);
}

static int rfm69_set_packet_config1(const struct rfm69_device* device, 
                                    uint8_t format,
                                    uint8_t encoding,
                                    uint8_t crcOn,
                                    uint8_t filterOnCrc,
                                    uint8_t addressFilter) {
    
    if(device == NULL) { return -EINVAL; }

    uint8_t reg_packet_config_1 = (format & 0x01) << 7;
    reg_packet_config_1 |= (encoding & 0x03) << 5;
    reg_packet_config_1 |= (crcOn & 0x01) << 4;
    reg_packet_config_1 |= (filterOnCrc & 0x01) << 3;
    reg_packet_config_1 |= (addressFilter & 0x03) << 1;

    return rfm69_write_reg(device, RFM69_REG_37_PACKETCONFIG1, reg_packet_config_1);
}

static int rfm69_set_packet_config2(const struct rfm69_device* device, 
                                    uint8_t rx_delay, 
                                    uint8_t restart, 
                                    uint8_t auto_restart_on, 
                                    uint8_t aes_on) {

    if(device == NULL) { return -EINVAL; }

    uint8_t reg_packet_config_2 = 0;
    reg_packet_config_2 |= aes_on;
    reg_packet_config_2 |= (auto_restart_on & 0x01) << 1;
    reg_packet_config_2 |= (restart & 0x01) << 2;
    reg_packet_config_2 |= rx_delay << 4;

    return rfm69_write_reg(device, RFM69_REG_3D_PACKETCONFIG2, reg_packet_config_2);
}

static int rfm69_set_payload_length(const struct rfm69_device* device, uint8_t size) {
    if(device == NULL) { return -EINVAL; }

    uint8_t length = 0;
    length = MIN(size, RFM69_MAX_FIFO_BYTES);
    int ret = rfm69_write_reg(device, RFM69_REG_38_PAYLOADLENGTH, length);
    if(ret < 0){
        return ret;
    }

    return length;
}


/*
To ensure correct operation at the highest power levels, please make sure to adjust the Over Current Protection
Limit accordingly in RegOcp, except above +18dBm where it must be disabled
    // Imax = 45 + 5 x OcpTrim  [mA]
*/
static int rfm69_set_over_current_protection(const struct rfm69_device* device, bool enable, int i_max) {
    if(device == NULL) { return -EINVAL; }

    if(IN_RANGE(i_max, 45, 120) == false) {
        return -ERANGE;
    }

    uint8_t reg_13_val = ((i_max - 45) / 5) | (enable ? 0x10 : 0x00);
    return rfm69_write_reg(device, RFM69_REG_13_OCP, reg_13_val);
}

/*

The Duty Cycle of transmission at +20dBm is limited to 1%, with a maximum VSWR of 3:1 at antenna port, over the
standard operating range [-40;+85°C].
*/
static int rfm69_set_high_power(const struct rfm69_device* device, bool enable) {
    if(device == NULL) { return -EINVAL; }

    uint8_t value = 0;

    value = enable ? RFM69_TESTPA1_BOOST : RFM69_TESTPA1_NORMAL;
    rfm69_write_reg(device, RFM69_REG_5A_TESTPA1, value);

    value = enable ? RFM69_TESTPA2_BOOST : RFM69_TESTPA2_NORMAL;
    rfm69_write_reg(device, RFM69_REG_5C_TESTPA2, value);

    return 0;
}

/**
 * @brief Sets the transmit power level
 * @param device
 * @param dbm
 * @retval 0 on Success
 * @retval -EINVAL on invalid device
 * @retval -EIO on invalid SPI transfer
 * 
 * Andre Hessling had a (now-deprecated) blog related to setting the power for the rfm69. Much of the inspiration
 * for setting the tx-power came from him, the datasheet, and suffering. Here we go:
 * 
 * Three power amplifier blocks are embedded in the RFM69 (Semtech SX1231). The first one, herein referred to as PA0, 
 * can generate up to +13 dBm into a 50 Ohm load. PA0 shares a common front-end pin RFIO (pin 21) with the receiver LNA.
 *
 * PA1 and PA2 are both connected to pin PA_BOOST (pin 23), allowing for two distinct power ranges:
 *     - A low power mode, where -18 dBm < Pout < 13 dBm, with PA1 enabled
 *     - A higher power mode, when PA1 and PA2 are combined, providing up to +17 dBm to a matched load.
 *
 * Note: When PA1 and PA2 are combined to deliver +17 dBm to the antenna, a specific impedance matching / harmonic filtering
 *       design is required to ensure impedance transformation and regulatory compliance.
 *
 * Pa0On Pa1On Pa2On Mode                                   Power Range          Pout Formula      
 *   1     0     0   PA0 output on pin RFIO                -18 to +13 dBm     -18 dBm + OutputPower
 *   0     1     0   PA1 enabled on pin PA_BOOST            -2 to +13 dBm     -18 dBm + OutputPower
 *   0     1     1   PA1 and PA2 combined on pin PA_BOOST   +2 to +17 dBm     -14 dBm + OutputPower
 *   0     1     1   PA1+PA2 on PA_BOOST +20dBm settings    +5 to +20 dBm     -11 dBm + OutputPower
 *
 * Supply current in Transmit mode with appropriate matching, stable across VDD range
 *   RFOP = +20 dBm, on PA_BOOST 130 mA (RFM69HCW)
 *   RFOP = +17 dBm, on PA_BOOST  95 mA
 *   RFOP = +13 dBm, on RFIO pin  45 mA
 *   RFOP = +10 dBm, on RFIO pin  33 mA
 *   RFOP =   0 dBm, on RFIO pin  20 mA
 *   RFOP =  -1 dBm, on RFIO pin  16 mA
 *
**/
static int rfm69_set_power_level(const struct rfm69_device* device, int8_t dbm) {
    if(device == NULL) { return -EINVAL; }

    if (device->pa_boost) {

        /* Device tree specifies tx_power_max based on PA_BOOST pin wiring. */
        dbm = CLAMP(dbm, -2, MIN(device->tx_power_max_dbm, 20));

        if (dbm < 13) {
            int paLevel = RFM69_PALEVEL_PA1ON | ((dbm + 18) & RFM69_PALEVEL_OUTPUTPOWER);
            rfm69_set_high_power(device, false);
            rfm69_set_over_current_protection(device, true, 95);
            rfm69_write_reg(device, RFM69_REG_11_PALEVEL, paLevel);
        } else if (dbm <= 17) {
            int paLevel = (RFM69_PALEVEL_PA1ON | RFM69_PALEVEL_PA2ON) | ((dbm + 14) & RFM69_PALEVEL_OUTPUTPOWER);
            rfm69_write_reg(device, RFM69_REG_11_PALEVEL, paLevel);
            rfm69_set_high_power(device, false);
            rfm69_set_over_current_protection(device, true, 100);
        } else if (dbm <= 20) {
            int paLevel = (RFM69_PALEVEL_PA1ON | RFM69_PALEVEL_PA2ON) | ((dbm + 11) & RFM69_PALEVEL_OUTPUTPOWER);
            rfm69_write_reg(device, RFM69_REG_11_PALEVEL, paLevel);
            rfm69_set_high_power(device, true);
            rfm69_set_over_current_protection(device, false, 120);
        }

    } else {

        dbm = CLAMP(dbm, -18, MIN(device->tx_power_max_dbm, 13));
        int paLevel = RFM69_PALEVEL_PA0ON | ((dbm + 18) & RFM69_PALEVEL_OUTPUTPOWER);
        rfm69_write_reg(device, RFM69_REG_11_PALEVEL, paLevel);
        rfm69_set_high_power(device, false);
        rfm69_set_over_current_protection(device, true, 95);
    }

    return 0;
}

static int rfm69_set_preamble_length(const struct rfm69_device* device, uint16_t count) {
    if(device == NULL) { return -EINVAL; }

    int ret = 0;
    uint8_t msb = count >> 8;
    ret = rfm69_write_reg(device, RFM69_REG_2C_PREAMBLEMSB, msb);
    if(ret < 0){
        return -EIO;
    }

    uint8_t lsb = count & 0xff;
    ret = rfm69_write_reg(device, RFM69_REG_2D_PREAMBLELSB, lsb);
    if(ret < 0){
        return -EIO;
    }

    return 0;
}

static int rfm69_set_sync_words(const struct rfm69_device* device, const uint8_t *sync_words, uint8_t length) {
    if(device == NULL) { 
        return -EINVAL; 
    }
    if(length > 8) { 
        return -ERANGE;
    }

    int ret = 0;
    if((sync_words == NULL) || length == 0) {
        ret = rfm69_write_reg(device, RFM69_REG_2E_SYNCCONFIG, 0); 
        return ret < 0 ? -EIO : 0;
    }

    uint8_t sync_config = 0;
    ret = rfm69_read_reg(device, RFM69_REG_2E_SYNCCONFIG, &sync_config);
    if(ret < 0) {
        return -EIO;
    }

    sync_config |= RFM69_SYNCCONFIG_SYNCON;
    sync_config &= ~RFM69_SYNCCONFIG_SYNCSIZE;
    sync_config |= (length - 1) << 3;
    ret = rfm69_write_reg(device, RFM69_REG_2E_SYNCCONFIG, sync_config);
    if(ret < 0) {
        return -EIO;
    }

    uint8_t reg_i = RFM69_REG_2F_SYNCVALUE1;
    for(int i = 0; i < length; i++) {
        ret = rfm69_write_reg(device, reg_i, sync_words[i]);
        if(ret < 0) {
            return -EIO;
        }
    }

    return 0;
}

// static void rfm69_set_rssi_threshold(struct rfm69_device* device, int16_t rssi) {  

//     if(rssi < 0){
//         rssi *= -1;
//     }
//     rssi /= 2;

//     // Default is -114 (0xE4)
//     rfm69_write_reg(device, SX1231_REG_29_RSSITHRESH, rssi);
// }


static int rfm69_calibraterc_osc(struct rfm69_device* device) {

    /* RC calibration must be triggered in Standby mode. */
    int ret = rfm69_standby(device);
    if(ret < 0){
        return ret;
    }
    
    ret = rfm69_write_reg(device, RFM69_REG_0A_OSC1, RFM69_RC_CAL_START);
    if(ret < 0){
        return -EIO;
    }

    uint8_t value;
    int start = millis();
    while (1) {
        ret = rfm69_read_reg(device, RFM69_REG_0A_OSC1, &value);
        if(ret < 0) {
            return -EIO;
        }
        if (value & RFM69_RC_CAL_DONE) {
            return 0;
        }
        if ((millis() - start) > 10) {
            return -ETIMEDOUT;
        }
    }
}

static void rfm69_dio0_callback(struct rfm69_device* device) {
    if(device == NULL) { return; }
    
    int ret;

    switch(device->state) {
        case RFM69_STATE_SLEEP: break;
        case RFM69_STATE_STANDBY: break;
        case RFM69_STATE_FS: break;
        case RFM69_STATE_RX:

            switch(device->dio.pm_dio0_rx) {
                case RFM69_PM_DIO0_RX_CRC_OK: break;
                case RFM69_PM_DIO0_RX_PAYLOAD_READY:
                    device->payload[0] = RFM69_READ_ADDRESS(RFM69_REG_00_FIFO);
                    ret = rfm69_fifo_transaction(device, device->payload, device->payload_length);
                    if(ret < 0){
                        printf("error reading fifo %d\n", ret);
                    }
                    if(device->rx_cb){
                        device->rx_cb(device, device->payload, device->payload_length, device->payload_rssi);
                    }
                    break;
                case RFM69_PM_DIO0_RX_SYNC_ADDRESS: break;
                case RFM69_PM_DIO0_RX_RSSI: break;
                default: return;                    
            }

            break;
        case RFM69_STATE_TX:
            break;
        default: return;
    }
}

static int8_t rfm69_get_rssi(struct rfm69_device* device) {
    if(device == NULL) {return EINVAL;}

    uint8_t raw;
    int ret = rfm69_read_reg(device, RFM69_REG_24_RSSIVALUE, &raw);
    if(ret < 0){
        return EIO;
    }

    return -(raw / 2);
}

static void rfm69_dio3_callback(struct rfm69_device* device) {
    if(device == NULL) { return; }
    
    switch(device->state) {
        case RFM69_STATE_SLEEP: break;
        case RFM69_STATE_STANDBY: break;
        case RFM69_STATE_FS: break;
        case RFM69_STATE_RX:

            switch(device->dio.pm_dio3_rx) {
                case RFM69_PM_DIO3_RX_FIFO_FULL: break;
                case RFM69_PM_DIO3_RX_RSSI: break;                    
                case RFM69_PM_DIO3_RX_SYNC_ADDRESS: 
                    device->payload_rssi = rfm69_get_rssi(device);
                    break;
                case RFM69_PM_DIO3_RX_PLL_LOCK: break;
                default: return;                    
            }

            break;
        case RFM69_STATE_TX:
            break;
        default: return;
    }
}

/**  */
static void rfm69_dio0_callback_0_0(void) {
    printf("rfm69_dio0_callback_0_0\n");
    rfm69_dio0_callback(&self);
}

static void rfm69_dio3_callback_0_0(void) {
    printf("rfm69_dio3_callback_0_0\n");
    rfm69_dio3_callback(&self);
}

struct rfm69_device* rfm69_init(const struct rfm69_pins *pins, int spi) {
    if(pins == NULL) { return NULL; }
    if(pins->reset < 0) { return NULL; }
    if(spi < 0 || spi > 1) {return NULL;}

    self.pins = *pins;
    self.spi = spi;

    int ret = 0;
    wiringPiSetup();
    wiringPiSPISetup(self.spi, RFM69_SPI_BUS_CLOCK_HZ);
    
    ret = wiringPiSetupGpio();
    if(ret < 0){
        printf("wiringPiSetupGpio %d\n",ret);
    }
   
    ret = rfm69_reset(self.pins.reset);
    if(ret < 0){
        printf("rfm69_reset %d\n", ret);
    }

    ret = rfm69_standby(&self);
        if(ret < 0){
        printf("rfm69_standby %d\n", ret);
    }

    uint8_t silicon = 0;
    ret = rfm69_read_reg(&self, RFM69_REG_10_VERSION, &silicon);
    if(ret < 0){
        printf("can't read id %d", ret);
    }

    /** wiringPi doesn't allow passing a user context to the ISR. This is 
     * problematic because we would ideally pass an rfm69_device pointer
     * to the ISR and take advantage of CONTAINER_OF. To support more than
     * one device, we will need to mount a specific dio_isr depending on the
     * spi bus and cs line used (or something to that effect). For now, we 
     * just have one device corresponding to spi bus 0 and chip select 0.
     * 
     * Second point: The first pass of this driver will focus on capturing a
     * semi-accurate RSSI value when receiving an FSK packet. This will be accomplished
     * by using two dio isr's (dio0 and dio3). dio3 will be configured to fire for a
     * sync-word match, at which point we will take an rssi measurement and store it inside
     * the corresponding rfm69_driver. Moments after a valid syncword is obtained, the dio0
     * interrupt will fire to signal a payload ready. When this happens, the fifo will be read,
     * paired with the corresponding rssi, and pushed to the user via callback.
     * 
     * At a later time, these interrupts will most likely be configured at the time of transmission 
     * or reception to allow greater flexibility for other types of modes. The intent is to make this 
     * driver as fully-featured as possible, but as mentioned above, fsk reception will be the first
     * goal.
     * 
     * @note If the user doesn't specify a dio3 or dio3 interrupt, the only option will be to block or 
     * spawn a thread to continuously poll for a syncword/mode_ready/payload_ready. This will be a pretty big
     * resource hog and also probably result in poor rssi accuracy (TBD). This feature can be added later for
     * applications that are limited in available GPIO.
     */

    if(self.pins.dio0 >= 0) {
        wiringPiISR(self.pins.dio0, INT_EDGE_RISING, rfm69_dio0_callback_0_0);
    }
    if(self.pins.dio3 >= 0) {
        wiringPiISR(self.pins.dio3, INT_EDGE_RISING, rfm69_dio3_callback_0_0);
    }

    printf("read_id: %02x\n", silicon);

    return &self;
}

int rfm69_receive(struct rfm69_device* device,
                  const struct rfm69_config* config, 
                  rfm69_rx_cb callback) {

    if(device == NULL) {return -EINVAL;}
    if(config == NULL) {return -EINVAL;}

    if(device->state != RFM69_STATE_STANDBY){
        rfm69_standby(device);
    }

    rfm69_set_modulation(device, 
                         config->mode, 
                         config->modulation,
                         config->shaping);

    rfm69_set_bitrate(device, config->br);

    if(config->modulation == RFM69_MODULATION_FSK) {
        rfm69_set_fsk_frequency_deviation(device, config->fsk.freq_deviation_hz);
        rfm69_set_fsk_bandwidth(device, config->fsk.bw, RFM69_DC_CANCEL_PRXBW_4);
    }else {
        /** @todo ook settings */
    }

    /** Do some default packet configuration, @todo make these configurable */
    rfm69_set_packet_config1(device,
        RFM69_PACKETCONFIG1_PACKETFORMAT_FIXED, RFM69_PACKETCONFIG1_ENCODING_NONE,
        RFM69_PACKETCONFIG1_CRC_OFF, RFM69_PACKETCONFIG1_FILTER_BAD_CRC_ON,
        RFM69_PACKETCONFIG1_ADDRESSFILTERING_NONE);

    rfm69_set_packet_config2(device,
        RFM69_PACKETCONFIG2_INTERPACKETRXDELAY,
        RFM69_PACKETCONFIG2_DO_RESTART_RX,
        RFM69_PACKETCONFIG2_AUTO_RX_RESTART_OFF,
        RFM69_PACKETCONFIG2_AES_OFF);


    rfm69_set_preamble_length(device, config->preamble_length);
    rfm69_set_frequency(device, config->center_freq_hz);
    rfm69_set_sync_words(device, config->sync_words, config->sync_count);
    rfm69_set_digital_auto_gain_control(device, RFM69_TESTDAGC_CONTINUOUSDAGC_IMPROVED_LOWBETAOFF);
    rfm69_set_power_level(device, 0);

    /** Hardcode dio0 for rx payload ready, and dio3 for sync word. @todo later we will add a configuration
     * system so the user can map the interrupt lines.
    */
    rfm69_write_reg(device, RFM69_REG_25_DIOMAPPING1, RFM69_DIOMAPPING1_DIO0MAPPING_01 | RFM69_DIOMAPPING1_DIO3MAPPING_10);
    device->dio.pm_dio0_rx = RFM69_PM_DIO0_RX_PAYLOAD_READY;
    device->dio.pm_dio0_tx = RFM69_PM_DIO0_TX_PACKET_SENT;
    device->dio.pm_dio3_rx = RFM69_PM_DIO3_RX_SYNC_ADDRESS;
    device->dio.pm_dio3_tx = RFM69_PM_DIO3_TX_UNUSED;

    device->rx_cb = callback;
    device->payload_length = config->payload_length;
    rfm69_set_payload_length(device, config->payload_length);

    rfm69_set_rx(device);

    return 0;
}



int rfm69_print_registers(struct rfm69_device* device) {
    if(device == NULL) {
        return -EINVAL;
    }

    struct rfm69_reg_status {
        uint8_t address;
        uint8_t por;
        uint8_t value;
        uint8_t rw;
    };

    struct rfm69_reg_status table[] = {
        { .address = RFM69_REG_00_FIFO, .por = 0x00 },
        { .address = RFM69_REG_01_OPMODE, .por = 0x04 },
        { .address = RFM69_REG_02_DATAMODUL, .por = 0x00 },
        { .address = RFM69_REG_03_BITRATEMSB, .por = 0x1a },
        { .address = RFM69_REG_04_BITRATELSB, .por = 0x0b },
        { .address = RFM69_REG_05_FDEVMSB, .por = 0x00 },
        { .address = RFM69_REG_06_FDEVLSB, .por = 0x52 },
        { .address = RFM69_REG_07_FRFMSB, .por = 0xe4 },
        { .address = RFM69_REG_08_FRFMID, .por = 0xc0 },
        { .address = RFM69_REG_09_FRFLSB, .por = 0x00 },
        { .address = RFM69_REG_0A_OSC1, .por = 0x41 },
        { .address = RFM69_REG_0B_AFCCTRL, .por = 0x00 },
        { .address = RFM69_REG_0C_RESERVED, .por = 0x02 },
        { .address = RFM69_REG_0D_LISTEN1, .por = 0x92 },
        { .address = RFM69_REG_0E_LISTEN2, .por = 0xf5 },
        { .address = RFM69_REG_0F_LISTEN3, .por = 0x20 },
        { .address = RFM69_REG_10_VERSION, .por = 0x24 },
        { .address = RFM69_REG_11_PALEVEL, .por = 0x9f },
        { .address = RFM69_REG_12_PARAMP, .por = 0x09 },
        { .address = RFM69_REG_13_OCP, .por = 0x1a },
        { .address = RFM69_REG_14_RESERVED, .por = 0x40 },
        { .address = RFM69_REG_15_RESERVED, .por = 0xb0 },
        { .address = RFM69_REG_16_RESERVED, .por = 0x7b },
        { .address = RFM69_REG_17_RESERVED, .por = 0x9b },
        { .address = RFM69_REG_18_LNA, .por = 0x08 },
        { .address = RFM69_REG_19_RXBW, .por = 0x86 },
        { .address = RFM69_REG_1A_AFCBW, .por = 0x8a },
        { .address = RFM69_REG_1B_OOKPEAK, .por = 0x40 },
        { .address = RFM69_REG_1C_OOKAVG, .por = 0x80 },
        { .address = RFM69_REG_1D_OOKFIX, .por = 0x06 },
        { .address = RFM69_REG_1E_AFCFEI, .por = 0x10 },
        { .address = RFM69_REG_1F_AFCMSB, .por = 0x00 },
        { .address = RFM69_REG_20_AFCLSB, .por = 0x00 },
        { .address = RFM69_REG_21_FEIMSB, .por = 0x00 },
        { .address = RFM69_REG_22_FEILSB, .por = 0x00 },
        { .address = RFM69_REG_23_RSSICONFIG, .por = 0x02 },
        { .address = RFM69_REG_24_RSSIVALUE, .por = 0xff },
        { .address = RFM69_REG_25_DIOMAPPING1, .por = 0x00 },
        { .address = RFM69_REG_26_DIOMAPPING2, .por = 0x05 },
        { .address = RFM69_REG_27_IRQFLAGS1, .por = 0x80 },
        { .address = RFM69_REG_28_IRQFLAGS2, .por = 0x00 },
        { .address = RFM69_REG_29_RSSITHRESH, .por = 0xff },
        { .address = RFM69_REG_2A_RXTIMEOUT1, .por = 0x00 },
        { .address = RFM69_REG_2B_RXTIMEOUT2, .por = 0x00 },
        { .address = RFM69_REG_2C_PREAMBLEMSB, .por = 0x00 },
        { .address = RFM69_REG_2D_PREAMBLELSB, .por = 0x03 },
        { .address = RFM69_REG_2E_SYNCCONFIG, .por = 0x98 },
        { .address = RFM69_REG_2F_SYNCVALUE1, .por = 0x00 },
        { .address = RFM69_REG_38_PAYLOADLENGTH, .por = 0x40 },
        { .address = RFM69_REG_39_NODEADRS, .por = 0x00 },
        { .address = RFM69_REG_3A_BROADCASTADRS, .por = 0x00 },
        { .address = RFM69_REG_3B_AUTOMODES, .por = 0x00 },
        { .address = RFM69_REG_3C_FIFOTHRESH, .por = 0xf0 },
        { .address = RFM69_REG_3D_PACKETCONFIG2, .por = 0x02 },
        { .address = RFM69_REG_3E_AESKEY1, .por = 0x00 },
        { .address = RFM69_REG_4E_TEMP1, .por = 0x01 },
        { .address = RFM69_REG_4F_TEMP2, .por = 0x00 },
        { .address = RFM69_REG_58_TESTLNA, .por = 0x1b },
        { .address = RFM69_REG_5A_TESTPA1, .por = 0x55 },
        { .address = RFM69_REG_5C_TESTPA2, .por = 0x70 },
        { .address = RFM69_REG_6F_TESTDAGC, .por = 0x00 },
        { .address = RFM69_REG_71_TESTAFC, .por = 0x00 },
    };

    for(int i = 0; i < sizeof(table) / sizeof(table[0]); i++) {
        int ret = rfm69_read_reg(device, table[i].address, &table[i].value);
        if(ret < 0){
            continue;
        }
        printf("REG: %02x VALUE: %02x POR: %02x\n", table[i].address, table[i].value, &table[i].por);
    }

    return 0;
}
