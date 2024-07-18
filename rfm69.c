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
    int spi;
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
 * @param device 
 * @param address Register address (See Table 23 - Registers Summary for a complete list)
 * @param value Buffer to store the result of the read
 * @retval 0 On Success
 * @retval -EINVAL if `value` is `NULL`
 * @retval -EBADF on Invalid SPI parameter
 * @retval < 0 for an internal i/o error 
 */
static int rfm69_read_reg(struct rfm69_device* device, uint8_t address, uint8_t* value) {
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
static int rfm69_write_reg(struct rfm69_device* device, uint8_t address, uint8_t value) {
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
static int rfm69_fifo_transaction(struct rfm69_device* device, uint8_t* buffer, uint8_t size) {
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

    int ret = wiringPiSPIDataRW(device->spi, buffer, sizeof(buffer) / sizeof(buffer[0]));
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
static int rfm69_set_mode(struct rfm69_device* device, uint8_t mode) {
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
        device->state = RFM69_STATE_IDLE;        
    }else{
        printf("rfm69_sleep %d\n", ret);
    }
    return ret;
}

static int rfm69_standby(struct rfm69_device* device) {
    if(device == NULL) { return -EINVAL; } 

    int ret = rfm69_set_mode(device, RFM69_OPMODE_MODE_STDBY);
    if(ret == 0) {
        device->state = RFM69_STATE_IDLE;        
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

static int rfm69_set_frequency_deviation(struct rfm69_device* device, uint16_t hz) {
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

static int rfm69_set_frequency(struct rfm69_device *device, uint32_t hz) {
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

    printf("read_id: %02x\n", silicon);

    return &self;
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
