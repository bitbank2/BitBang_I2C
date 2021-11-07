//
// Simple program to display the info of the attached RPI HAT
// This info is stored in an I2C EEPROM attached to pins 27 & 28
// of the 40 pin RPI GPIO header.
//
// Info about the physical, electrical and software specifications
// of the RPI HAT standard are defined here:
// https://github.com/raspberrypi/hats/blob/master/eeprom-format.md
//
// Copyright (c) 2021 BitBank Software, Inc.
// Written by Larry Bank
// email: bitbank@pobox.com
// Project started 11/07/2021
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

BBI2C bbi2c;
#define SDA_ID 27
#define SCL_ID 28
// Fixed address of all HAT EEPROM chips
#define EEPROM_ADDR 0x50
//
// Read a block of 32 bytes at the given address
// or from the last read address if iAddr == -1
//
int eeReadBlock(int iAddr, unsigned char *pData)
{
unsigned char ucTemp[4];
int rc;

        if (iAddr != -1) // send the address
        {
                ucTemp[0] = (unsigned char)(iAddr >> 8);
                ucTemp[1] = (unsigned char)iAddr;
                rc = I2CWrite(&bbi2c, EEPROM_ADDR, ucTemp, 2);
        } // otherwise read from the last address and increment
        rc = I2CRead(&bbi2c, EEPROM_ADDR, pData, 32);
        return (rc == 32);
} /* eeReadBlock() */

const char *szTypes[] = {"invalid", "vendor info", "GPIO map", "Linux device tree blob", "manufacturer custom data", "reserved", "invalid"};

int main(int argc, char **argv)
{
int i, iAtoms, iLen, iOff;
uint8_t ucHAT[256], ucTemp[256]; // read some of the HAT info here

  memset(&bbi2c, 0, sizeof(bbi2c));
  bbi2c.bWire = 0; // use bit bang, not wire library
  bbi2c.iSDA = SDA_ID;
  bbi2c.iSCL = SCL_ID;
  I2CInit(&bbi2c, 100000L);
  // Read 256 bytes of the HAT data starting at offset 0
  for (i=0; i<256; i+=32) {
	  eeReadBlock(i, &ucHAT[i]);
  }
  // Print signature
  memcpy(ucTemp, ucHAT, 4);
  ucTemp[4] = 0;
  printf("EEPROM Header Info:\n");
  printf("signature: %s\n", ucHAT);
  printf("version: 0x%02x\n", ucHAT[4]);
  iAtoms = ucHAT[6];
  printf("total atoms: %d\n", iAtoms);
  iLen = *(uint32_t *)&ucHAT[8];
  printf("total length: %d bytes\n", iLen);
  // Display info for each Atom
  iOff = 12; // starting offset of first Atom
  for (i=0; i<iAtoms; i++) {
     uint32_t iType, iSize;
     int j, iPins, vslen, pslen;
     iType = *(uint16_t *)&ucHAT[iOff];
     if (iType > 5 && iType < 0xffff) iType = 5; // reserved
     if (iType == 0xffff) iType = 0; // invalid 
     iSize = *(uint32_t *)&ucHAT[iOff+4];
     printf("Atom Type: 0x%04x (%s)\n",iType, szTypes[iType]);
     printf("Atom Size: %d bytes\n",iSize);
     if (iType == 0x0001) { // vendor info
        printf("** Vendor Info **\n");
	printf("   UUID: %08x%08x%08x%08x\n", *(uint32_t *)&ucHAT[iOff+8], *(uint32_t *)&ucHAT[iOff+12], *(uint32_t *)&ucHAT[iOff+16], *(uint32_t *)&ucHAT[iOff+20]);
	printf("   Product ID: %d\n", *(uint16_t *)&ucHAT[iOff+24]);
	printf("   Product Ver: %d\n", *(uint16_t *)&ucHAT[iOff+26]);
	vslen = ucHAT[iOff+28]; // vendor name string length
	pslen = ucHAT[iOff+29]; // vendor product string length
	memcpy(ucTemp, &ucHAT[iOff+30], vslen);
	ucTemp[vslen] = 0;
	printf("   Vendor: %s\n", ucTemp);
	memcpy(ucTemp, &ucHAT[iOff+30+vslen], pslen);
	ucTemp[pslen] = 0;
	printf("   Product: %s\n", ucTemp);
     } else if (iType == 0x0002) { // GPIO map
	printf("** GPIO Map **\n");
        printf("   GPIO (BCM #) pins used:\n");
	iPins = 0;
        for (j=0; j<28; j++) {
           if (ucHAT[iOff+10+j] & 0x80) { // bit 7 indicates pin is used
              printf("%02d,",ucHAT[iOff+10+j]);
	      iPins++;
	   }
	}
        printf("\n   %d pins defined\n", iPins);	
     }
     iOff += iSize + 8; 
  }
  printf("** End of Atom list **\n");
  return 0;
}
