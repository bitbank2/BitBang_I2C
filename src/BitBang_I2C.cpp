//
// Bit Bang I2C library
// Copyright (c) 2018-2019 BitBank Software, Inc.
// Written by Larry Bank (bitbank@pobox.com)
// Project started 10/12/2018
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#ifdef _LINUX_
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <pigpio.h>
#define PROGMEM
#define false 0
#define true 1
#define memcpy_P memcpy
#define INPUT 1
#define OUTPUT 2

// maps RPI pins to BCM GPIO numbers
const int iRPIPins[] = {-1,-1,-1,2,-1,3,-1,4,14,-1,
                        15,17,18,27,-1,22,23,-1,24,10,
                        -1,9,25,11,8,-1,7,0,1,5,
                        -1,6,12,13,-1,19,16,26,20,-1,
                        21};

#else // Arduino
#include <Arduino.h>
#ifndef __AVR_ATtiny85__
#include <Wire.h>
#endif
#ifdef W600_EV
#include <W600FastIO.h>
#define VARIANT_MCK 80000000ul
#endif
#endif // _LINUX_
#include <BitBang_I2C.h>
//static int iSCL, iSDA; // keep requested pin numbers in private statics
//static int iDelay; // bit delay in ms for the requested clock rate
#if defined ( __AVR__ ) && !defined( ARDUINO_ARCH_MEGAAVR )
volatile uint8_t *iDDR_SCL, *iPort_SCL_Out;
volatile uint8_t *iDDR_SDA, *iPort_SDA_In, *iPort_SDA_Out;
uint8_t iSDABit, iSCLBit;
#endif
#ifdef FUTURE
//#else // must be a 32-bit MCU
volatile uint32_t *iDDR_SCL, *iPort_SCL_Out;
volatile uint32_t *iDDR_SDA, *iPort_SDA_In, *iPort_SDA_Out;
uint32_t iSDABit, iSCLBit;
#endif

#if defined ( __AVR__ ) && !defined( ARDUINO_ARCH_MEGAAVR )
uint8_t getPinInfo(uint8_t pin, volatile uint8_t **iDDR, volatile uint8_t **iPort, int bInput)
{
  uint8_t port, bit;

  port = (pin & 0xf0); // hex port (A,B,D,E,F)
  bit = pin & 0x7;
  switch (port)
  {
#ifdef PORTE
    case 0xE0:
      *iPort = (bInput) ? &PINE : &PORTE;
      *iDDR = &DDRE;
      break;
#endif
#ifdef PORTF
    case 0xF0:
      *iPort = (bInput) ? &PINF : &PORTF;
      *iDDR = &DDRF;
      break;
#endif
#ifdef PORTG
    case 0xA0: // really port G
      *iPort = (bInput) ? &PING : &PORTG;
      *iDDR = &DDRG;
      break;
#endif
#ifdef PORTC
    case 0xC0:
      *iPort = (bInput) ? &PINC : &PORTC;
      *iDDR = &DDRC;
      break;
#endif
#ifdef PORTB
    case 0xB0:
      *iPort = (bInput) ? &PINB : &PORTB;
      *iDDR = &DDRB;
      break;
#endif
#ifdef PORTD
    case 0xD0:
      *iPort = (bInput) ? &PIND : &PORTD;
      *iDDR = &DDRD;
      break;
#endif
  }
  return bit;
} /* getPinInfo() */
#endif

//#else // 32-bit version
#ifdef FUTURE
uint32_t getPinInfo(uint8_t pin, volatile uint32_t **iDDR, volatile uint32_t **iPort, int bInput)
{
  uint32_t port, bit;

  if (pin <= 0xbf) // port 0
  {
    *iPort = (bInput) ? &REG_PORT_IN0 : &REG_PORT_OUT0;
    *iDDR = &REG_PORT_DIR0;
  }
  else if (pin <= 0xdf) // port 1
  {
    *iPort = (bInput) ? &REG_PORT_IN1 : &REG_PORT_OUT1;
    *iDDR = &REG_PORT_DIR1;
  }
  else return 0xffffffff; // invalid
  bit = pin & 0x1f;
  return bit;
} /* getPinInfo() */
#endif // __AVR__

inline uint8_t SDA_READ(uint8_t iSDA)
{
#if defined ( __AVR__ ) && !defined( ARDUINO_ARCH_MEGAAVR )
  if (iSDA >= 0xa0) // direct pin numbering
  {
    if (*iPort_SDA_In & iSDABit)
       return HIGH;
    else
       return LOW;
  }
  else
#endif
  {
#ifndef __AVR_ATtiny85__
#ifdef W600_EV
    return w600DigitalRead(iSDA);
#else
#ifdef _LINUX_
    return gpioRead(iSDA);
#else
    return digitalRead(iSDA);
#endif // _LINUX
#endif
#endif
  }
  return 0; // fall through?
}
inline void SCL_HIGH(uint8_t iSCL)
{
#if defined ( __AVR__ ) && !defined( ARDUINO_ARCH_MEGAAVR )
  if (iSCL >= 0xa0) // direct pin numbering
  {
    *iDDR_SCL &= ~iSCLBit;
  }
  else
#endif
  {
#ifndef __AVR_ATtiny85__
#ifdef W600_EV
    w600PinMode(iSCL, GPIO_INPUT);
#else
#ifdef _LINUX_
    gpioSetMode(iSCL, PI_INPUT);
#else
    pinMode(iSCL, INPUT);
#endif // _LINUX_
#endif
#endif
  }
}

inline void SCL_LOW(uint8_t iSCL)
{
#if defined ( __AVR__ ) && !defined( ARDUINO_ARCH_MEGAAVR )
  if (iSCL >= 0xa0) // direct pin numbering
  {
    *iDDR_SCL |= iSCLBit;
  }
  else
#endif
  {
#ifndef __AVR_ATtiny85__
#ifdef W600_EV
    w600PinMode(iSCL, GPIO_OUTPUT);
    w600DigitalWrite(iSCL, LOW);
#else
#ifdef _LINUX_
    gpioSetMode(iSCL, PI_OUTPUT);
#else
    pinMode(iSCL, OUTPUT);
#endif // _LINUX_
#endif
#endif
  }
}

inline void SDA_HIGH(uint8_t iSDA)
{
#if defined ( __AVR__ ) && !defined( ARDUINO_ARCH_MEGAAVR )
  if (iSDA >= 0xa0) // direct pin numbering
  {
    *iDDR_SDA &= ~iSDABit;
  }
  else
#endif
  {
#ifndef __AVR_ATtiny85__
#ifdef W600_EV
    w600PinMode(iSDA, GPIO_INPUT);
#else
#ifdef _LINUX_
    gpioSetMode(iSDA, PI_INPUT);
#else
    pinMode(iSDA, INPUT);
#endif // _LINUX_
#endif
#endif
  }
}

inline void SDA_LOW(uint8_t iSDA)
{
#if defined ( __AVR__ ) && !defined( ARDUINO_ARCH_MEGAAVR )
  if (iSDA >= 0xa0) // direct pin numbering
  {
    *iDDR_SDA |= iSDABit;
  }
  else
#endif
  {
#ifndef __AVR_ATtiny85__
#ifdef W600_EV
    w600PinMode(iSDA, GPIO_OUTPUT);
    w600DigitalWrite(iSDA, LOW);
#else
#ifdef _LINUX_
    gpioSetMode(iSDA, PI_OUTPUT);
#else
    pinMode(iSDA, OUTPUT);
#endif // _LINUX_
#endif
#endif
  }
}

void inline sleep_us(int iDelay)
{
#ifdef __AVR_ATtiny85__
  iDelay *= 2;
  while (iDelay)
  {
    __asm__ __volatile__ (
    "nop" "\n\t"
    "nop"); //just waiting 2 cycle
    iDelay--;
  }
#else
  if (iDelay > 0)
#ifdef _LINUX_
     gpioDelay(iDelay);
#else
     delayMicroseconds(iDelay);
#endif // _LINUX
#endif
}
#ifndef __AVR_ATtiny85__
// Transmit a byte and read the ack bit
// if we get a NACK (negative acknowledge) return 0
// otherwise return 1 for success
//

static inline int i2cByteOut(BBI2C *pI2C, uint8_t b)
{
uint8_t i, ack;
uint8_t iSDA = pI2C->iSDA;
uint8_t iSCL = pI2C->iSCL; // in case of bad C compiler
int iDelay = pI2C->iDelay;

  for (i=0; i<8; i++)
  {
      if (b & 0x80)
        SDA_HIGH(iSDA); // set data line to 1
      else
        SDA_LOW(iSDA); // set data line to 0
      SCL_HIGH(iSCL); // clock high (slave latches data)
      sleep_us(iDelay);
      SCL_LOW(iSCL); // clock low
      b <<= 1;
      sleep_us(iDelay);
  } // for i
// read ack bit
  SDA_HIGH(iSDA); // set data line for reading
  SCL_HIGH(iSCL); // clock line high
  sleep_us(iDelay); // DEBUG - delay/2
  ack = SDA_READ(iSDA);
  SCL_LOW(iSCL); // clock low
  sleep_us(iDelay); // DEBUG - delay/2
  SDA_LOW(iSDA); // data low
  return (ack == 0) ? 1:0; // a low ACK bit means success
} /* i2cByteOut() */
#endif

#if defined ( __AVR__ ) && !defined( ARDUINO_ARCH_MEGAAVR )
#define SDA_LOW_AVR *iDDR_sda |= sdabit;
#define SDA_HIGH_AVR *iDDR_sda &= ~sdabit;
#define SCL_LOW_AVR *iDDR_scl |= sclbit;
#define SCL_HIGH_AVR *iDDR_scl &= ~sclbit;
#define SDA_READ_AVR (*iPort_SDA_In & sdabit)
static inline int i2cByteOutAVR(BBI2C *pI2C, uint8_t b)
{
uint8_t i, ack;
uint8_t *iDDR_sda = (uint8_t *)iDDR_SDA; // Put in local variables to avoid reading
uint8_t *iDDR_scl = (uint8_t *)iDDR_SCL; // from volatile pointer vars each time
uint8_t sdabit = iSDABit;
uint8_t sclbit = iSCLBit;

     for (i=0; i<8; i++)
     {
         if (b & 0x80)
           SDA_HIGH_AVR // set data line to 1
         else
           SDA_LOW_AVR // set data line to 0
         SCL_HIGH_AVR // clock high (slave latches data)
         sleep_us(pI2C->iDelay);
         SCL_LOW_AVR // clock low
         b <<= 1;
     } // for i
// read ack bit
  SDA_HIGH_AVR // set data line for reading
  SCL_HIGH_AVR // clock line high
//  sleep_us(iDelay); // DEBUG - delay/2
  ack = SDA_READ_AVR;
  SCL_LOW_AVR // clock low
//  sleep_us(iDelay); // DEBUG - delay/2
  SDA_LOW_AVR // data low
  return (ack == 0) ? 1:0; // a low ACK bit means success
} /* i2cByteOutAVR() */

#define BOTH_LOW_FAST *iDDR = both_low;
#define BOTH_HIGH_FAST *iDDR = both_high;
#define SCL_HIGH_FAST *iDDR = scl_high; 
#define SDA_HIGH_FAST *iDDR = sda_high;
#define SDA_READ_FAST *iDDR & iSDABit;
static inline int i2cByteOutAVRFast(BBI2C *pI2C, uint8_t b)
{
uint8_t i, ack;
uint8_t *iDDR = (uint8_t *)iDDR_SDA; // Put in local variables to avoid reading
uint8_t bOld = *iDDR; // current value
uint8_t both_low = bOld | iSDABit | iSCLBit;
uint8_t both_high = bOld & ~(iSDABit | iSCLBit);
uint8_t scl_high = (bOld | iSDABit) & ~iSCLBit;
uint8_t sda_high = (bOld | iSCLBit) & ~iSDABit;

     BOTH_LOW_FAST // start with both lines set to 0
     for (i=0; i<8; i++)
     {
         if (b & 0x80)
         {
           SDA_HIGH_FAST // set data line to 1
           sleep_us(pI2C->iDelay);
           BOTH_HIGH_FAST // rising edge clocks data
         }
         else // more probable case (0) = shortest code path
         {
           SCL_HIGH_FAST // clock high (slave latches data)
         }
         sleep_us(pI2C->iDelay);
         BOTH_LOW_FAST // clock low
         b <<= 1;
     } // for i
// read ack bit
  SDA_HIGH_FAST // set data line for reading
  BOTH_HIGH_FAST // clock line high
  sleep_us(pI2C->iDelay); // DEBUG - delay/2
  ack = SDA_READ_FAST;
  BOTH_LOW_FAST // clock low
//  sleep_us(pI2C->iDelay); // DEBUG - delay/2
//  SDA_LOW_AVR // data low
  return (ack == 0) ? 1:0; // a low ACK bit means success
} /* i2cByteOutAVR() */
#endif // __AVR__

#ifndef __AVR_ATtiny85__
static inline int i2cByteOutFast(BBI2C *pI2C, uint8_t b)
{
uint8_t i, ack, iSDA, iSCL;
int iDelay;

     iSDA = pI2C->iSDA;
     iSCL = pI2C->iSCL;
     iDelay = pI2C->iDelay;

     if (b & 0x80)
        SDA_HIGH(iSDA); // set data line to 1
     else
        SDA_LOW(iSDA); // set data line to 0
     for (i=0; i<8; i++)
     {
         SCL_HIGH(iSCL); // clock high (slave latches data)
         sleep_us(iDelay);
         SCL_LOW(iSCL); // clock low
         sleep_us(iDelay);
     } // for i
// read ack bit
  SDA_HIGH(iSDA); // set data line for reading
  SCL_HIGH(iSCL); // clock line high
  sleep_us(pI2C->iDelay); // DEBUG - delay/2
  ack = SDA_READ(iSDA);
  SCL_LOW(iSCL); // clock low
  sleep_us(pI2C->iDelay); // DEBUG - delay/2
  SDA_LOW(iSDA); // data low
  return (ack == 0) ? 1:0; // a low ACK bit means success
} /* i2cByteOutFast() */
#endif
//
// Receive a byte and read the ack bit
// if we get a NACK (negative acknowledge) return 0
// otherwise return 1 for success
//
static inline uint8_t i2cByteIn(BBI2C *pI2C, uint8_t bLast)
{
uint8_t i;
uint8_t b = 0;

     SDA_HIGH(pI2C->iSDA); // set data line as input
     for (i=0; i<8; i++)
     {
         sleep_us(pI2C->iDelay); // wait for data to settle
         SCL_HIGH(pI2C->iSCL); // clock high (slave latches data)
         b <<= 1;
         if (SDA_READ(pI2C->iSDA) != 0) // read the data bit
           b |= 1; // set data bit
         SCL_LOW(pI2C->iSCL); // cloc low
     } // for i
     if (bLast)
        SDA_HIGH(pI2C->iSDA); // last byte sends a NACK
     else
        SDA_LOW(pI2C->iSDA);
     SCL_HIGH(pI2C->iSCL); // clock high
     sleep_us(pI2C->iDelay);
     SCL_LOW(pI2C->iSCL); // clock low to send ack
     sleep_us(pI2C->iDelay);
     SDA_LOW(pI2C->iSDA); // data low
  return b;
} /* i2cByteIn() */

//
// Send I2C STOP condition
//
static inline void i2cEnd(BBI2C *pI2C)
{
   SDA_LOW(pI2C->iSDA); // data line low
   sleep_us(pI2C->iDelay);
   SCL_HIGH(pI2C->iSCL); // clock high
   sleep_us(pI2C->iDelay);
   SDA_HIGH(pI2C->iSDA); // data high
   sleep_us(pI2C->iDelay);
} /* i2cEnd() */


static inline int i2cBegin(BBI2C *pI2C, uint8_t addr, uint8_t bRead)
{
   int rc;
   SDA_LOW(pI2C->iSDA); // data line low first
   sleep_us(pI2C->iDelay);
   SCL_LOW(pI2C->iSCL); // then clock line low is a START signal
   addr <<= 1;
   if (bRead)
      addr++; // set read bit
#ifdef __AVR_ATtiny85__
   rc = i2cByteOutAVR(pI2C, addr);
#else
   rc = i2cByteOut(pI2C, addr); // send the slave address and R/W bit
#endif
   return rc;
} /* i2cBegin() */

static inline int i2cWrite(BBI2C *pI2C, uint8_t *pData, int iLen)
{
uint8_t b;
int rc, iOldLen = iLen;

   rc = 1;
   while (iLen && rc == 1)
   {
      b = *pData++;
#ifdef __AVR_ATtiny85__
      rc = i2cByteOutAVRFast(pI2C, b);
#else
#if defined ( __AVR__ ) && !defined( ARDUINO_ARCH_MEGAAVR )
     if (pI2C->iSDA >= 0xa0)
     {
        rc = i2cByteOutAVRFast(pI2C, b);
     }
     else
#endif
     {
        if (b == 0xff || b == 0)
           rc = i2cByteOutFast(pI2C, b); // speed it up a bit more if all bits are ==
        else
           rc = i2cByteOut(pI2C, b);
     }
#endif
      if (rc == 1) // success
      {
         iLen--;
      }
   } // for each byte
   return (rc == 1) ? (iOldLen - iLen) : 0; // 0 indicates bad ack from sending a byte
} /* i2cWrite() */

static inline void i2cRead(BBI2C *pI2C, uint8_t *pData, int iLen)
{
   while (iLen--)
   {
      *pData++ = i2cByteIn(pI2C, iLen == 0);
   } // for each byte
} /* i2cRead() */
//
// Initialize the I2C BitBang library
// Pass the pin numbers used for SDA and SCL
// as well as the clock rate in Hz
//
void I2CInit(BBI2C *pI2C, uint32_t iClock)
{
#ifdef _LINUX_
   if (gpioInitialise() < 0)
   {
      printf("pigpio failed to initialize\n");
      return;
   }
#endif
   if (pI2C == NULL) return;

   if (pI2C->bWire) // use Wire library
   {
#if !defined( _LINUX_ ) && !defined( __AVR_ATtiny85__ )
#if defined(TEENSYDUINO) || defined( __AVR__ ) || defined( NRF52 ) || defined ( ARDUINO_ARCH_NRF52840 )
       Wire.begin();
#else
       if (pI2C->iSDA == 0xff || pI2C->iSCL == 0xff)
          Wire.begin();
       else
          Wire.begin(pI2C->iSDA, pI2C->iSCL);
#endif
       Wire.setClock(iClock);
#endif
#ifdef _LINUX_
       {
           char filename[32];
           sprintf(filename, "/dev/i2c-%d", pI2C->iBus);
           if ((pI2C->file_i2c = open(filename, O_RDWR)) < 0)
                 return;
       }
#endif // _LINUX_
       return;
   }
   if (pI2C->iSDA < 0xa0)
   {
#if !defined ( __AVR_ATtiny85__ ) && !defined( _LINUX_ )
#ifdef W600_EV
     w600PinMode(pI2C->iSDA, GPIO_OUTPUT);
     w600PinMode(pI2C->iSCL, GPIO_OUTPUT);
     w600DigitalWrite(pI2C->iSDA, LOW); // setting low = enabling as outputs
     w600DigitalWrite(pI2C->iSCL, LOW);
     w600PinMode(pI2C->iSDA, GPIO_INPUT); // let the lines float (tri-state)
     w600PinMode(pI2C->iSCL, GPIO_INPUT);
#else // generic
     pinMode(pI2C->iSDA, OUTPUT);
     pinMode(pI2C->iSCL, OUTPUT);
     digitalWrite(pI2C->iSDA, LOW); // setting low = enabling as outputs
     digitalWrite(pI2C->iSCL, LOW);
     pinMode(pI2C->iSDA, INPUT); // let the lines float (tri-state)
     pinMode(pI2C->iSCL, INPUT);
#endif
#endif
#ifdef _LINUX_
     // use PIGPIO
     // convert pin numbers to BCM numbers for PIGPIO
     pI2C->iSDA = iRPIPins[pI2C->iSDA];
     pI2C->iSCL = iRPIPins[pI2C->iSCL];
     gpioWrite(pI2C->iSDA, 0);
     gpioWrite(pI2C->iSCL, 0);
     gpioSetMode(pI2C->iSDA, PI_INPUT);
//     gpioSetPullUpDown(pI2C->iSDA, PI_PUD_UP);
     gpioSetMode(pI2C->iSCL, PI_INPUT);
//     gpioSetPullUpDown(pI2C->iSCL, PI_PUD_UP);
#endif // _LINUX_
   }
#if defined ( __AVR__ ) && !defined( ARDUINO_ARCH_MEGAAVR )
   else // direct pin mode, get port address and bit
   {
//      iSDABit = 1 << (pI2C->iSDA & 0x7);
      iSDABit = 1 << getPinInfo(pI2C->iSDA, &iDDR_SDA, &iPort_SDA_Out, 0);
      getPinInfo(pI2C->iSDA, &iDDR_SDA, &iPort_SDA_In, 1);
//      iSCLBit = 1 << (pI2C->iSCL & 0x7);
      iSCLBit = 1 << getPinInfo(pI2C->iSCL, &iDDR_SCL, &iPort_SCL_Out, 0);
      *iDDR_SDA &= ~iSDABit; // pinMode input
      *iDDR_SCL &= ~iSCLBit; // pinMode input
      *iPort_SDA_Out &= ~iSDABit; // digitalWrite SDA LOW
      *iPort_SCL_Out &= ~iSCLBit; // digitalWrite SCL LOW
   }
#endif // __AVR__
  // For now, we only support 100, 400 or 800K clock rates
  // all other values default to 100K
#ifdef _LINUX_
   pI2C->iDelay = 1000000 / iClock;
   if (pI2C->iDelay < 1) pI2C->iDelay = 1;
#else
   if (iClock >= 1000000)
      pI2C->iDelay = 0; // the code execution is enough delay
   else if (iClock >= 800000)
      pI2C->iDelay = 1;
   else if (iClock >= 400000)
      pI2C->iDelay = 2;
   else if (iClock >= 100000)
      pI2C->iDelay = 10;
   else pI2C->iDelay = (uint16_t)(1000000 / iClock);
#endif // _LINUX_
} /* i2cInit() */
//
// Test a specific I2C address to see if a device responds
// returns 0 for no response, 1 for a response
//
uint8_t I2CTest(BBI2C *pI2C, uint8_t addr)
{
uint8_t response = 0;

  if (pI2C->bWire)
  {
#if !defined( _LINUX_ ) && !defined( __AVR_ATtiny85__ )
  // We use the return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.
  Wire.beginTransmission(addr);
  response = !Wire.endTransmission();
#endif
#ifdef _LINUX_
    if (ioctl(pI2C->file_i2c, I2C_SLAVE, addr) >= 0)
        response = 1;
#endif
    return response;
  }
  if (i2cBegin(pI2C, addr, 0)) // try to write to the given address
  {
    response = 1;
  }
  i2cEnd(pI2C);
  return response;
} /* I2CTest() */
//
// Scans for I2C devices on the bus
// returns a bitmap of devices which are present (128 bits = 16 bytes, LSB first)
// A set bit indicates that a device responded at that address
//
void I2CScan(BBI2C *pI2C, uint8_t *pMap)
{
  int i;
  for (i=0; i<16; i++) // clear the bitmap
    pMap[i] = 0;
  for (i=1; i<128; i++) // try every address
  {
    if (I2CTest(pI2C, i))
    {
      pMap[i >> 3] |= (1 << (i & 7));
    }
  }
} /* I2CScan() */
//
// Write I2C data
// quits if a NACK is received and returns 0
// otherwise returns the number of bytes written
//
int I2CWrite(BBI2C *pI2C, uint8_t iAddr, uint8_t *pData, int iLen)
{
  int rc = 0;
  
  if (pI2C->bWire)
  {
#if !defined ( _LINUX_ ) && !defined( __AVR_ATtiny85__ )
    Wire.beginTransmission(iAddr);
    Wire.write(pData, (uint8_t)iLen);
    rc = !Wire.endTransmission();
#endif
#ifdef _LINUX_
    if (ioctl(pI2C->file_i2c, I2C_SLAVE, iAddr) >= 0)
    {
       if (write(pI2C->file_i2c, pData, iLen) >= 0)
          rc = 1;
    } 
#endif // _LINUX_
    return rc;
  }
  rc = i2cBegin(pI2C, iAddr, 0);
  if (rc == 1) // slave sent ACK for its address
  {
     rc = i2cWrite(pI2C, pData, iLen);
  }
  i2cEnd(pI2C);
  return rc; // returns the number of bytes sent or 0 for error
} /* I2CWrite() */
//
// Read N bytes starting at a specific I2C internal register
//
int I2CReadRegister(BBI2C *pI2C, uint8_t iAddr, uint8_t u8Register, uint8_t *pData, int iLen)
{
  int rc;
  
  if (pI2C->bWire) // use the wire library
  {
      int i = 0;
#if !defined( _LINUX_ ) && !defined( __AVR_ATtiny85__ )
      Wire.beginTransmission(iAddr);
      Wire.write(u8Register);
      Wire.endTransmission();
      Wire.requestFrom(iAddr, (uint8_t)iLen);
      while (i < iLen)
      {
          pData[i++] = Wire.read();
      }
#endif
#ifdef _LINUX_
    if (ioctl(pI2C->file_i2c, I2C_SLAVE, iAddr) >= 0)
    {
       write(pI2C->file_i2c, &u8Register, 1);
       i = read(pI2C->file_i2c, pData, iLen);
    } 
#endif // _LINUX_
      return (i > 0);
  }
  rc = i2cBegin(pI2C, iAddr, 0); // start a write operation
  if (rc == 1) // slave sent ACK for its address
  {
     rc = i2cWrite(pI2C, &u8Register, 1); // write the register we want to read from
     if (rc == 1)
     {
       i2cEnd(pI2C);
       rc = i2cBegin(pI2C, iAddr, 1); // start a read operation
       if (rc == 1)
       {
         i2cRead(pI2C, pData, iLen);
       }
     }
  }
  i2cEnd(pI2C);
  return rc; // returns 1 for success, 0 for error
} /* I2CReadRegister() */
//
// Read N bytes
//
int I2CRead(BBI2C *pI2C, uint8_t iAddr, uint8_t *pData, int iLen)
{
  int rc;
  
    if (pI2C->bWire) // use the wire library
    {
        int i = 0;
#if !defined( _LINUX_ ) && !defined( __AVR_ATtiny85__ )
        Wire.requestFrom(iAddr, (uint8_t)iLen);
        while (i < iLen)
        {
            pData[i++] = Wire.read();
        }
#endif
#ifdef _LINUX_
    if (ioctl(pI2C->file_i2c, I2C_SLAVE, iAddr) >= 0)
    {
       i = read(pI2C->file_i2c, pData, iLen);
    } 
#endif // _LINUX_
        return (i > 0);
    }
  rc = i2cBegin(pI2C, iAddr, 1);
  if (rc == 1) // slave sent ACK for its address
  {
     i2cRead(pI2C, pData, iLen);
  }
  i2cEnd(pI2C);
  return rc; // returns 1 for success, 0 for error
} /* I2CRead() */
//
// Figure out what device is at that address
// returns the enumerated value
//
int I2CDiscoverDevice(BBI2C *pI2C, uint8_t i)
{
uint8_t j, cTemp[8];
int iDevice = DEVICE_UNKNOWN;

  if (i == 0x3c || i == 0x3d) // Probably an OLED display
  {
    I2CReadRegister(pI2C, i, 0x00, cTemp, 1);
    cTemp[0] &= 0xbf; // mask off power on/off bit
    if (cTemp[0] == 0x8) // SH1106
       iDevice = DEVICE_SH1106;
    else if (cTemp[0] == 3 || cTemp[0] == 6)
       iDevice = DEVICE_SSD1306;
    return iDevice;
  }
  if (i >= 0x40 && i <= 0x4f) // check for TI INA219 power measurement sensor
  {
    I2CReadRegister(pI2C, i, 0x00, cTemp, 2);
    if (cTemp[0] == 0x39 && cTemp[1] == 0x9f)
       return DEVICE_INA219;
  }
//  else if (i == 0x5b) // MLX90615?
//  {
//    I2CReadRegister(pI2C, i, 0x10, cTemp, 3);
//    for (j=0; j<3; j++) Serial.println(cTemp[j], HEX);
//  }
  // try to identify it from the known devices using register contents
  {
    // Check for TI HDC1080
    I2CReadRegister(pI2C, i, 0xff, cTemp, 2);
    if (cTemp[0] == 0x10 && cTemp[1] == 0x50)
       return DEVICE_HDC1080;

    // Check for BME680
    if (i == 0x76 || i == 0x77)
    {
       I2CReadRegister(pI2C, i, 0xd0, cTemp, 1); // chip ID
       if (cTemp[0] == 0x61) // BME680
          return DEVICE_BME680;
    }
    // Check for VL53L0X
    I2CReadRegister(pI2C, i, 0xc0, cTemp, 3);
    if (cTemp[0] == 0xee && cTemp[1] == 0xaa && cTemp[2] == 0x10)
       return DEVICE_VL53L0X;

    // Check for CCS811
    I2CReadRegister(pI2C, i, 0x20, cTemp, 1);
    if (cTemp[0] == 0x81) // Device ID
       return DEVICE_CCS811;

    // Check for LIS3DSH accelerometer from STMicro
    I2CReadRegister(pI2C, i, 0x0f, cTemp, 1);
    if (cTemp[0] == 0x3f) // WHO_AM_I
       return DEVICE_LIS3DSH;

    // Check for LIS3DH accelerometer from STMicro
    I2CReadRegister(pI2C, i, 0x0f, cTemp, 1);
    if (cTemp[0] == 0x33) // WHO_AM_I
       return DEVICE_LIS3DH;

    // Check for LSM9DS1 magnetometer/gyro/accel sensor from STMicro
    I2CReadRegister(pI2C, i, 0x0f, cTemp, 1);
    if (cTemp[0] == 0x68) // WHO_AM_I
       return DEVICE_LSM9DS1;

    // Check for LPS25H pressure sensor from STMicro
    I2CReadRegister(pI2C, i, 0x0f, cTemp, 1);
    if (cTemp[0] == 0xbd) // WHO_AM_I
       return DEVICE_LPS25H;
    
    // Check for HTS221 temp/humidity sensor from STMicro
    I2CReadRegister(pI2C, i, 0x0f, cTemp, 1);
    if (cTemp[0] == 0xbc) // WHO_AM_I
       return DEVICE_HTS221;
    
    // Check for MAG3110
    I2CReadRegister(pI2C, i, 0x07, cTemp, 1);
    if (cTemp[0] == 0xc4) // WHO_AM_I
       return DEVICE_MAG3110;

    // Check for LM8330 keyboard controller
    I2CReadRegister(pI2C, i, 0x80, cTemp, 2);
    if (cTemp[0] == 0x0 && cTemp[1] == 0x84) // manufacturer code + software revision
       return DEVICE_LM8330;

    // Check for MAX44009
    if (i == 0x4a || i == 0x4b)
    {
      for (j=0; j<8; j++)
        I2CReadRegister(pI2C, i, j, &cTemp[j], 1); // check for power-up reset state of registers
      if ((cTemp[2] == 3 || cTemp[2] == 2) && cTemp[6] == 0 && cTemp[7] == 0xff)
         return DEVICE_MAX44009;
    }
       
    // Check for ADS1115
    I2CReadRegister(pI2C, i, 0x02, cTemp, 2); // Lo_thresh defaults to 0x8000
    I2CReadRegister(pI2C, i, 0x03, &cTemp[2], 2); // Hi_thresh defaults to 0x7fff
    if (cTemp[0] == 0x80 && cTemp[1] == 0x00 && cTemp[2] == 0x7f && cTemp[3] == 0xff)
       return DEVICE_ADS1115;

    // Check for MCP9808
    I2CReadRegister(pI2C, i, 0x06, cTemp, 2); // manufacturer ID && get device ID/revision
    I2CReadRegister(pI2C, i, 0x07, &cTemp[2], 2); // need to read them individually
    if (cTemp[0] == 0 && cTemp[1] == 0x54 && cTemp[2] == 0x04 && cTemp[3] == 0x00)
       return DEVICE_MCP9808;
       
    // Check for BMP280/BME280
    I2CReadRegister(pI2C, i, 0xd0, cTemp, 1);
    if (cTemp[0] == 0x55) // BMP180
       return DEVICE_BMP180;
    else if (cTemp[0] == 0x58)
       return DEVICE_BMP280;
    else if (cTemp[0] == 0x60) // BME280
       return DEVICE_BME280;

    // Check for LSM6DS3
    I2CReadRegister(pI2C, i, 0x0f, cTemp, 1); // WHO_AM_I
    if (cTemp[0] == 0x69)
       return DEVICE_LSM6DS3;
       
    // Check for ADXL345
    I2CReadRegister(pI2C, i, 0x00, cTemp, 1); // DEVID
    if (cTemp[0] == 0xe5)
       return DEVICE_ADXL345;
       
    // Check for MPU-60x0i, MPU-688x, and MPU-9250
    I2CReadRegister(pI2C, i, 0x75, cTemp, 1);
    if (cTemp[0] == (i & 0xfe)) // Current I2C address (low bit set to 0)
       return DEVICE_MPU6000;
    else if (cTemp[0] == 0x71)
       return DEVICE_MPU9250;
    else if (cTemp[0] == 0x19)
       return DEVICE_MPU6886;

    // Check for DS3231 RTC
    I2CReadRegister(pI2C, i, 0x0e, cTemp, 1); // read the control register
    if (i == 0x68 && cTemp[0] == 0x1c) // fixed I2C address and power on reset value  
       return DEVICE_DS3231;
  }
  return iDevice;
} /* I2CDiscoverDevice() */
