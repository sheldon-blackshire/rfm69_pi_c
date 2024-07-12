#include <stdio.h>
#include <stdint.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#define SPI_BUS 0
#define SPI_BUS_CLOCK_HZ 10000000

#define GPIO_RFM_RESET  33 /** GPIO33 */
#define GPIO_RFM_DIO0   37
#define GPIO_RFM_DIO3   39
#define GPIO_RFM_NSS     8 /** Chip Select */
#define GPIO_RFM_MISO    9
#define GPIO_RFM_MOSI   10
#define GPIO_RFM_SCK    11


#define SX1231_REG_01_OPMODE 0x01
#define SX1231_OPMODE_MODE 0x1c
#define SX1231_REG_27_IRQFLAGS1 0x27
#define SX1231_IRQFLAGS1_MODEREADY 0x80


uint8_t main_read_reg(uint8_t address) {
    uint8_t data[2]={ address & 0x7F };
    wiringPiSPIDataRW(0, data, 2);
    return data[1];
}

uint8_t  main_write_reg(uint8_t address, uint8_t value) {
    char data[2]={ address | 0x80, value };
    wiringPiSPIDataRW(0, data, 2);
    return 0;
}

static int main_set_operation_mode(uint8_t mode) {

    uint8_t om = main_read_reg(SX1231_REG_01_OPMODE);

    om &= ~SX1231_OPMODE_MODE;
    om |= (mode & SX1231_OPMODE_MODE);

    main_write_reg(SX1231_REG_01_OPMODE, om);

    for(int i = 0; i < 5; i++) {
        uint8_t inc = main_read_reg(SX1231_REG_27_IRQFLAGS1);
        if(inc & SX1231_IRQFLAGS1_MODEREADY){
            return 0;
        }
        delay(10);
    }
}


int main() {
    int ret = 0;
    printf("Hello World\n");
    wiringPiSetup();
    wiringPiSPISetup(SPI_BUS, SPI_BUS_CLOCK_HZ);
    ret=wiringPiSetupGpio();
    if(ret < 0){
        printf("setup %d\n",ret);
    }
    int i = GPIO_RFM_RESET;

    printf("setting pin %d\n",i);
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH);
    printf("Pin %d state %d\n",i, digitalRead(i));
    delay(250);
    digitalWrite(i, LOW);
    delay(250);
    printf("Pin %d state %d\n",i, digitalRead(i));



    main_set_operation_mode(0);


    for(int i = 0; i < 100; i++){
        printf("read_id: %02x\n", main_read_reg(0x10));
        delay(250);
    }

    return 0;
}
