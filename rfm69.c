/* SPDX-License-Identifier: GPL-3.0 */ 
/*
 * Raspberry PI Linux Driver for HopeRf RFM69
 *
 * Author: Sheldon Blackshire <sheldon.blackshire@gmail.com>
 */

#include <errno.h>
#include <rfm69.h>
#include <rfm69_priv.h>
#include <stdio.h>
#include <stdint.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>


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
 * @param address Register address (See Table 23 - Registers Summary for a complete list)
 * @param value Buffer to store the result of the read
 * @retval 0 On Success
 * @retval -EINVAL if `value` is `NULL`
 * @retval -EBADF on Invalid SPI parameter
 * @retval < 0 for an internal i/o error 
 */
static int rfm69_read_reg(uint8_t address, uint8_t* value) {
    if(value == NULL) { return -EINVAL; }

    uint8_t buffer[2] = {
        RFM69_READ_ADDRESS(address),
        0 /** Dummy byte, will be replaced with byte from device */
    };

    int ret = wiringPiSPIDataRW(0, buffer, sizeof(buffer) / sizeof(buffer[0]));
    if(ret < 0) {
        return ret;
    }

    *value = buffer[1];

    return 0;
}

/**
 * @brief Write an 8-bit register value over SPI
 * @param address Register address (See Table 23 - Registers Summary for a complete list)
 * @param value Contains the value to be written to the register
 * @retval 0 On Success
 * @retval -EBADF on Invalid SPI parameter
 * @retval < 0 for an internal i/o error 
 */
static int rfm69_write_reg(uint8_t address, uint8_t value) {

    uint8_t buffer[2] = {
        RFM69_WRITE_ADDRESS(address),
        value
    };

    int ret = wiringPiSPIDataRW(0, buffer, sizeof(buffer) / sizeof(buffer[0]));
    if(ret < 0) {
        return ret;
    }

    return 0;
}

/**
 * @brief Reads/Writes the rx/tx fifo
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
static int rfm69_fifo_transaction(uint8_t* buffer, uint8_t size) {
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

    int ret = wiringPiSPIDataRW(0, buffer, sizeof(buffer) / sizeof(buffer[0]));
    if(ret < 0) {
        return ret;
    }

    return size - 1;
}