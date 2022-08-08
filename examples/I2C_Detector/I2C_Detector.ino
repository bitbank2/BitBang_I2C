//
// I2C Detector - scan and identify devices on an I2C bus using the BitBang_I2C library
// 
// The purpose of this code is to provide a sample sketch which can serve
// to detect not only the addresses of I2C devices, but what type of device each one is.
// So far, I've added the 25 devices I've personally used or found to be reliably detected
// based on their register contents. I encourage people to do pull requests to add support
// for more devices to make this code have wider appeal.

// There are plenty of I2C devices which appear at fixed addresses, yet don't have unique
// "Who_Am_I" registers or other data to reliably identify them. It's certainly possible to
// write code which initializes these devices and tries to verify their identity. This can
// potentially damage them and would necessarily increase the code size. I would like to keep
// the size of this code small enough so that it can be included in many microcontroller 
// projects where code space is scarce.

// Copyright (c) 2019 BitBank Software, Inc.
// Written by Larry Bank
// email: bitbank@pobox.com
// Project started 25/02/2019
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
// Uses my Bit Bang I2C library. You can find it here:
// https://github.com/bitbank2/BitBang_I2C
#include <BitBang_I2C.h>

// Arbitrary pins I used for testing with an ATmega328p
// Define as -1, -1 to use the Wire library over the default I2C interface
#define SDA_PIN -1
#define SCL_PIN -1
#define BITBANG false
// M5Stack Atom Grove connector pin assignments
//#define SDA_PIN 32 
//#define SCL_PIN 26
// M5Stack Atom internal I2C connected to the IMU
//#define SDA_PIN 25
//#define SCL_PIN 21
//
// If you don't need the explicit device names displayed, disable this code by
// commenting out the next line
//
BBI2C bbi2c;

void setup() {
  Serial.begin(115200);
  memset(&bbi2c, 0, sizeof(bbi2c));
  bbi2c.bWire = !BITBANG; // use bit bang, not wire library
  bbi2c.iSDA = SDA_PIN;
  bbi2c.iSCL = SCL_PIN;
  I2CInit(&bbi2c, 100000L);
  delay(100); // allow devices to power up
}

void loop() {
uint8_t map[16];
char szTemp[32];
uint8_t i;
int iDevice, iCount;
uint32_t u32Caps;

  Serial.println("Starting I2C Scan");
  I2CScan(&bbi2c, map); // get bitmap of connected I2C devices
  if (map[0] == 0xfe) // something is wrong with the I2C bus
  {
    Serial.println("I2C pins are not correct or the bus is being pulled low by a bad device; unable to run scan");
  }
  else
  {
    iCount = 0;
    for (i=1; i<128; i++) // skip address 0 (general call address) since more than 1 device can respond
    {
      if (map[i>>3] & (1 << (i & 7))) // device found
      {
        iCount++;
        Serial.print("Device found at 0x");
        Serial.print(i, HEX);
        iDevice = I2CDiscoverDevice(&bbi2c, i, &u32Caps);
        Serial.print(", type = ");
        I2CGetDeviceName(iDevice, szTemp);
        Serial.print(szTemp); // show the device name as a string
        Serial.print(", capability bits = 0x");
        Serial.println(u32Caps, HEX);
      }
    } // for i
    Serial.print(iCount, DEC);
    Serial.println(" device(s) found");
  }
  delay(5000);
}
