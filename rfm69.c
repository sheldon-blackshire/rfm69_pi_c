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

enum rfm69_super_state {
    RFM69_STATE_IDLE = 0,
    RFM69_STATE_RX,
    RFM69_STATE_TX,
    RFM69_STATE_PAYLOAD_READY,
    RFM69_STATE_PACKET_SENT
};
struct rfm69_device {
    enum rfm69_super_state state;
    struct rfm69_pins pins;
};

/** todo Work-in-progress. Rpi has two SPI ports, and supports 
 * 2 select lines per port (More if SPI is be bitbanged). Need to
 * transition to a device array instead of a singleton.
*/
static struct rfm69_device self;


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

/**
 * @brief Update the mode bits of RFM69_REG_01_OPMODE. 
 * @param mode The transceivers operating mode. See RegOpMode(0x01) from
 * Table 24 (Common Configuration Registers) for more details.
 * @retval 0 on Success
 * @retval -EPROTONOSUPPORT for an invalid `mode` argument
 * @retval -ETIMEDOUT if the register update doesn't complete in time.
 * @retval -EIO if a reg r/w operation fails
 */
static int rfm69_set_mode(uint8_t mode) {

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
    ret = rfm69_read_reg(RFM69_REG_01_OPMODE, &reg_op_mode);
    if(ret < 0) {
        return -EIO;
    }

    /** Only bits 2-4 of the RegOpMode contain the mode. Replace those bits
     * with the input argument `mode`. */
    reg_op_mode &= ~RFM69_OPMODE_MODE;
    reg_op_mode |= (mode & RFM69_OPMODE_MODE);

    ret = rfm69_write_reg(RFM69_REG_01_OPMODE, &reg_op_mode);
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
        ret = rfm69_read_reg(RFM69_REG_27_IRQFLAGS1, &reg_irq_flags1);
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

static int rfm69_set_rx(void) {
    int ret = rfm69_set_mode(RFM69_OPMODE_MODE_RX);
    if(ret == 0){
        self.state = RFM69_STATE_RX;        
    }else{
        printf("rfm69_set_rx %d\n", ret);
    }
    return ret;
}

static int rfm69_set_tx(void) {
    int ret = rfm69_set_mode(RFM69_OPMODE_MODE_TX);
    if(ret == 0) {
        self.state = RFM69_STATE_TX;        
    }else{
        printf("rfm69_set_tx %d\n", ret);
    }
    return ret;
}

static int rfm69_sleep(void) {
    int ret = rfm69_set_mode(RFM69_OPMODE_MODE_SLEEP);
    if(ret == 0) {
        self.state = RFM69_STATE_IDLE;        
    }else{
        printf("rfm69_sleep %d\n", ret);
    }
    return ret;
}

static int rfm69_standby(void) {
    int ret = rfm69_set_mode(RFM69_OPMODE_MODE_STDBY);
    if(ret == 0) {
        self.state = RFM69_STATE_IDLE;        
    }else{
        printf("rfm69_sleep %d\n", ret);
    }
    return ret;
}

static int rfm69_reset(int pin) {
    if(pin < 0) { return -EINVAL; }

    pinMode(pin, OUTPUT);
    delay(10);
    digitalWrite(pin, HIGH);
    delay(10);
    digitalWrite(pin, LOW);
    delay(10);

    return 0;
}

int rfm69_init(const struct rfm69_pins* pins) {
    if(pins == NULL) { return -EINVAL; }
    if(pins->reset < 0) { return - EINVAL; }
    if(pins->spi_bus < 0 || pins->spi_bus > 1) {return -EINVAL;}

    self.pins = *pins;

    int ret = 0;
    wiringPiSetup();
    wiringPiSPISetup(self.pins.spi_bus, RFM69_SPI_BUS_CLOCK_HZ);
    
    ret = wiringPiSetupGpio();
    if(ret < 0){
        printf("wiringPiSetupGpio %d\n",ret);
    }
   
    ret = rfm69_reset(self.pins.reset);
    if(ret < 0){
        printf("rfm69_reset %d\n", ret);
    }

    ret = rfm69_standby();
        if(ret < 0){
        printf("rfm69_standby %d\n", ret);
    }

    uint8_t silicon = 0;
    ret = rfm69_read_reg(RFM69_REG_10_VERSION, &silicon);
    if(ret < 0){
        printf("can't read id %d", ret);
    }

    printf("read_id: %02x\n", silicon);

    return &self; 
}
