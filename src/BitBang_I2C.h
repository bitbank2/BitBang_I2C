//
// Bit Bang I2C library
// Copyright (c) 2018 BitBank Software, Inc.
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
#ifndef __BITBANG_I2C__
#define __BITBANG_I2C__

// On Linux, use it as C code, not C++
#if !defined(ARDUINO) && defined(__cplusplus)
extern "C" {
#endif

// supported devices
enum {
  DEVICE_UNKNOWN = 0,
  DEVICE_SSD1306,
  DEVICE_SH1106,
  DEVICE_VL53L0X,
  DEVICE_BMP180,
  DEVICE_BMP280,
  DEVICE_BME280,
  DEVICE_MPU6000,
  DEVICE_MPU9250,
  DEVICE_MCP9808,
  DEVICE_LSM6DS3,
  DEVICE_ADXL345,
  DEVICE_ADS1115,
  DEVICE_MAX44009,
  DEVICE_MAG3110,
  DEVICE_CCS811,
  DEVICE_HTS221,
  DEVICE_LPS25H,
  DEVICE_LSM9DS1,
  DEVICE_LM8330,
  DEVICE_DS3231,
  DEVICE_LIS3DH,
  DEVICE_LIS3DSH,
  DEVICE_INA219,
  DEVICE_SHT3X,
  DEVICE_HDC1080,
  DEVICE_MPU6886,
  DEVICE_BME680,
  DEVICE_AXP202,
  DEVICE_AXP192,
  DEVICE_24AAXXXE64,
  DEVICE_DS1307,
  DEVICE_MPU688X,
  DEVICE_FT6236G,
  DEVICE_FT6336G,
  DEVICE_FT6336U,
  DEVICE_FT6436,
  DEVICE_BM8563,
  DEVICE_BNO055,
  DEVICE_AHT20,
  DEVICE_TMF882X,
  DEVICE_SCD4X,
  DEVICE_ST25DV,
  DEVICE_LTR390,
  DEVICE_BMP388,
  DEVICE_COUNT
};

// Device capabilities
#define DEVICE_CAP_TEMPERATURE    0x00000001
#define DEVICE_CAP_HUMIDITY       0x00000002
#define DEVICE_CAP_PRESSURE       0x00000004
#define DEVICE_CAP_ACCELEROMETER  0x00000008
#define DEVICE_CAP_GYROSCOPE      0x00000010
#define DEVICE_CAP_MAGNETOMETER   0x00000020
#define DEVICE_CAP_VISIBLE_LIGHT  0x00000040
#define DEVICE_CAP_IR_LIGHT       0x00000080
#define DEVICE_CAP_UV_LIGHT       0x00000100
#define DEVICE_CAP_TOF_DISTANCE   0x00000200
#define DEVICE_CAP_GAS_VOCS       0x00000400
#define DEVICE_CAP_GAS_CO2        0x00000800
#define DEVICE_CAP_GAS_CO         0x00001000
#define DEVICE_CAP_GAS_ALCOHOL    0x00002000
#define DEVICE_CAP_RADAR_DISTANCE 0x00004000
#define DEVICE_CAP_PM25           0x00008000
#define DEVICE_CAP_RTC            0x00010000
#define DEVICE_CAP_NFC            0x00020000
#define DEVICE_CAP_1D_BARCODE     0x00040000
#define DEVICE_CAP_2D_BARCODE     0x00080000
#define DEVICE_CAP_DISPLAY_1BPP   0x00100000
#define DEVICE_CAP_DISPLAY_16BPP  0x00200000
#define DEVICE_CAP_POWER_MGMT     0x00400000
#define DEVICE_CAP_TOUCH_CTRL     0x00800000
#define DEVICE_CAP_POWER_MEASURE  0x01000000
#define DEVICE_CAP_EEPROM         0x02000000
#define DEVICE_CAP_KEYBOARD       0x04000000
#define DEVICE_CAP_ADC            0x08000000
#define DEVICE_CAP_DAC            0x10000000
#define DEVICE_CAP_FIFO           0x20000000

typedef struct mybbi2c
{
uint8_t iSDA, iSCL; // pin numbers (0xff = disabled)
uint8_t bWire, bAlign; // use the Wire library
uint8_t iSDABit, iSCLBit; // bit numbers of the ports
uint32_t iDelay;
#ifdef _LINUX_
int file_i2c;
int iBus;
#else
volatile uint32_t *pSDADDR, *pSDAPORT; // data direction and port register addr
volatile uint32_t *pSCLDDR, *pSCLPORT;
#endif
} BBI2C;
//
// Read N bytes
//
int I2CRead(BBI2C *pI2C, uint8_t iAddr, uint8_t *pData, int iLen);
//
// Read N bytes starting at a specific I2C internal register
//
int I2CReadRegister(BBI2C *pI2C, uint8_t iAddr, uint8_t u8Register, uint8_t *pData, int iLen);
//
// Write I2C data
// quits if a NACK is received and returns 0
// otherwise returns the number of bytes written
//
int I2CWrite(BBI2C *pI2C, uint8_t iAddr, uint8_t *pData, int iLen);
//
// Scans for I2C devices on the bus
// returns a bitmap of devices which are present (128 bits = 16 bytes, LSB first)
//
// Test if an address responds
// returns 0 if no response, 1 if it responds
//
uint8_t I2CTest(BBI2C *pI2C, uint8_t addr);

// A set bit indicates that a device responded at that address
//
void I2CScan(BBI2C *pI2C, uint8_t *pMap);
//
// Initialize the I2C BitBang library
// Pass the pin numbers used for SDA and SCL
// as well as the clock rate in Hz
//
void I2CInit(BBI2C *pI2C, uint32_t iClock);
//
// Figure out what device is at that address
// returns the enumerated value
//
int I2CDiscoverDevice(BBI2C *pI2C, uint8_t i, uint32_t *pCapabilities);
//
// Get the device's name as text
//
void I2CGetDeviceName(int iDevice, char *szName);

#if !defined(ARDUINO) && defined(__cplusplus)
}
#endif

#endif //__BITBANG_I2C__

