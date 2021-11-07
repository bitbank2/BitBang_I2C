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
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <BitBang_I2C.h>
#include <pigpio.h>

#define SHOW_NAME
#ifdef SHOW_NAME
const char *szNames[]  = {"Unknown","SSD1306","SH1106","VL53L0X","BMP180", "BMP280","BME280",
                "MPU-60x0", "MPU-9250", "MCP9808","LSM6DS3", "ADXL345", "ADS1115","MAX44009",
                "MAG3110", "CCS811", "HTS221", "LPS25H", "LSM9DS1","LM8330", "DS3231", "LIS3DH",
                "LIS3DSH","INA219","SHT3X","HDC1080","MPU6886","BME680", "AXP202", "AXP192", "24AA02XEXX", 
                "DS1307", "MPU688X", "FT6236G", "FT6336G", "FT6336U", "FT6436", "BM8563","BNO055"};
#endif

BBI2C bbi2c;

int main(int argc, char **argv)
{
int i, iSDA, iSCL;
uint8_t map[16];
int iDevice, iCount;

	if (argc != 3) {
		printf("Bit Bang I2C detector\nUsage: i2c_detect <SDA_PIN> <SCL_PIN>\n");
		return 0;
	}

  iSDA = atoi(argv[1]);
  iSCL = atoi(argv[2]);
  memset(&bbi2c, 0, sizeof(bbi2c));
  bbi2c.bWire = 0; // use bit bang, not wire library
  bbi2c.iSDA = iSDA;
  bbi2c.iSCL = iSCL;
  I2CInit(&bbi2c, 100000L);

  printf("Starting I2C Scan\n");
  I2CScan(&bbi2c, map); // get bitmap of connected I2C devices
  if (map[0] == 0xfe) // something is wrong with the I2C bus
  {
    printf("I2C pins are not correct or the bus is being pulled low by a bad device; unable to run scan\n");
  }
  else
  {
    iCount = 0;
    for (i=1; i<128; i++) // skip address 0 (general call address) since more than 1 device can respond
    {
      if (map[i>>3] & (1 << (i & 7))) // device found
      {
        iCount++;
	iDevice = I2CDiscoverDevice(&bbi2c, i);
        printf("Device found at 0x%02x, type = %s\n", i, szNames[iDevice]);
      }
    } // for i
    printf("%d device(s) found\n", iCount);
  }
  return 0;
}
