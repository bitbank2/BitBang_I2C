Bit Bang I2C library
Copyright (c) 2018-2019 BitBank Software, Inc.
Written by Larry Bank (bitbank@pobox.com)
Project started 10/12/2018

The purpose of this code is to provide a simple C library which can bit-bang
the I2C protocol on any 2 GPIO pins on any system. The I2C protocol doesn't
require any special functionality of the pins beyond standard GPIO features.
The reason I wrote it was for getting easy access to I2C devices on
various microcontrollers that don't necessarily have exposed I2C interfaces.
This has come in handy on a variety of projects including AVR, ESP32, and nRF5
micrcontrollers.

The pin access functions can be wrapper functions for the native versions (e.g. on the nRF5 SDK)
On AVR micros, the digitalWrite/digitalRead/pinMode functions are somewhat
slow because they check the pin numbers against tables and do other tasks.
This library includes logic to speed that up. By specifying pin numbers as the
port name + bit, the library will run considerably faster on AVR
microcontrollers. For example, On the Arduino Uno (ATmega328P), I/O pin 9 is
actually I/O Port B, bit 1. To use the direct pin method, you would specify
the pin number as 0xB1. On the ATtiny85, this is the only pin numbering
supported so that the Wire library doesn't get linked in (to save FLASH space). 

Usage:
-----
Start by initializing the library with the desired pin numbers for SDA/SCL
along with the desired clock frequency. Frequencies above 400Khz are 
possible, but not necessarily accurate. Luckily I2C devices don't really
care about the exact clock frequency, only that the signals are stable
within the given periods.
For example: I2CInit(10, 11, 100000); // SDA=pin 10, SCL=pin 11, 100K clock

Instead of exposing functions to start and stop I2C transactions, I decided
to make it simpler by providing composite functions that hide the details of
I2C protocol. For scanning the I2C bus for devices, I provide the I2CScan()
function which returns a bitmap (16 bytes x 8 bits) with a bit set for every
device it finds. Call it like this:

unsigned char ucMap[16];

I2CScan(ucMap);

To detect if a single address is active, use I2CTest(addr).

For reading and writing data to the I2C device, use the following functions:

I2CRead(uint8_t u8Address, uint8_t *pu8Data, int iLength);
I2CReadRegister(uint8_t iAddr, uint8_t u8Register, uint8_t *pData, int iLen);
I2CWrite(uint8_t iAddr, uint8_t *pData, int iLen); 


