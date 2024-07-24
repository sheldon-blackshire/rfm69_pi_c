# rfm69_pi_c
The purpose of this repository is to provide a RaspberryPi, C implementation for the [HopeRf RFM69](https://www.adafruit.com/product/3071) that exposes as much functionality as possible while providing extensive documentation on how the device works. Below is a brief list of specifications for the transceiver.

* Low Cost: < 10$ from Adafruit and Sparkfun. Can be purchased in large quantites from HopeRf for a few dollars.
* Wide Frequency Range: 315 MHz, 433 MHz, 868 MHz, and 915 MHz (There are two part numbers for the low and high frequncy bands).
* FSK bit rates up to 300 kb/s
* FSK, GFSK, MSK, GMSK and OOK modulations
* +13 to +20 dBm up to 100 mW Power Output Capability (power output selectable in software)
* 50mA (+13 dBm) to 150mA (+20dBm) current draw for transmissions, ~30mA during active radio listening.
* Range of approx. 500 meters, depending on obstructions, frequency, antenna and power output (Reception of 20 km has been achieved with a directional antenna with a transmitter in flight!)
* Encrypted packet engine with AES-128
* Uses the amateur or license-free ISM band (ITU "Europe" license-free ISM or ITU "American" amateur with limitations)

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)

## Installation
1. Clone the repository:
```bash
 git clone https://github.com/sheldon-blackshire/rfm69_pi_c.git
```

2. Install dependencies:

[CMake](https://cmake.org/getting-started/) is used to build the project. This repository requires at least Version 3.20.0.
```bash
sudo apt install cmake
```
The RFM69 requires GPIO access and SPI communication. The [WiringPi](https://github.com/WiringPi/WiringPi) project is used due to its popularity and support for many variants of the RaspberryPi.
```bash
git clone https://github.com/WiringPi/WiringPi.git
```
Once the repository has been cloned, follow the instructions in the [INSTALL](https://github.com/WiringPi/WiringPi/blob/pwm/INSTALL) file. 
> [!WARNING]
> Release 3.6 doesn't support the [CM4S](https://www.raspberrypi.com/products/compute-module-4s/) without a small [modification](https://github.com/WiringPi/WiringPi/issues/262). Release 3.8 should
> correct this problem.
```bash
git clone https://github.com/WiringPi/WiringPi.git
```
## Usage

There is currently one example (more to come) that configures the radio to listen for packets. When a packet is detected, its broadcast over UDP. A common application for this example would be to detected wireless sensor nodes. A higher level application written in Python, Node.js, or similar would listen for the UDP broadcast and process incoming packets accordingly (save to files, databases, display on a web interface, etc...). 

1. Navigate to the source directory and edit the main.c file and configure the rfm69 radio pins as well as the radio settings.
2. Build and run the program with the following commands
```bash
cmake .
make
./app
```
