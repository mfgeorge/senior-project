/* mb4-driver.h
   Class for interfacing with an IC-MB4 master IC. 
   
   California Polytechnic State University, San Luis Obispo
   In partial fulfillment of the requirements for a bachelor's 
   degree from the department of Mechanical Engineering. 

   Michael George
   10/21/16

   This code comes without any warrenty or guarantee from the author. 
   Any usage is at the discretion of the user, and should be done 
   at their own risk. 

   This software is hereby licensed under the Modified BSD License.

   Copyright (c) 2016, Michael George
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
       * Redistributions of source code must retain the above copyright
         notice, this list of conditions and the following disclaimer.
       * Redistributions in binary form must reproduce the above copyright
         notice, this list of conditions and the following disclaimer in the
         documentation and/or other materials provided with the distribution.
       * Neither the name of the Accumulator Volume Sensing Team nor the
         names of its contributors may be used to endorse or promote products
         derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL ACCUMULATOR VOLUME SENSING TEAM BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <SPI.h>

// Conversion factor to go from raw position to physical
// this is 2^26 (26 bits max from encoder)
#define CONV_FAC     .000000244*39.3701 // inches

// The BREAK instruction stops all ongoing processes of the MB4
#define BREAK     0b10000000

// The INIT instruction sends out an MA pulse train on all MA clock lines
#define INIT      0b00010000

// Define Channel 1 as in use and Channel 2 as not active
#define CH1         0x01

// The correct setting for bit 4:0 of the 
// FREQ register for a 20/8 MHz clock
#define CLOCK_SPEED 0x03

// Setting for the BiSS C protocol to go in bit 6 of REGVERS
// and into bit 1:0 of CFGCH1
#define BISS_C       5

// Setting for automatically restarting read cycles
// to go into the FREQAGS register (set exactly to this)
#define AGSFREQ   0x81

// Setting for RS422 line levels to be or'd into 
// bit 3:2 of the CFGIF register
#define RS422     0x02

// Setting to enable Single Cycle Data (SCD) in bit 6 of ENSCD1
#define SCD_AVAIL    1

// Setting for data length of SCD (26+2 bits -1 since 0 is a length of 1)
// to go into register SCDLEN1 bit 5:0
#define DATA_LENGTH  27

// Setting for CRC Polynomial selection in SELCRCS1 bit 7
#define CRC_SELECT   0

// Setting for the CRC polynomial in SCRCLEN1 bit 6:0
#define CRC_POLY     6

// Setting for the initial start value of CRC in SCRCPOLY1 bit 15:0
#define CRC_START    0

// Setting for all slaves to be sensors
#define SLAVES    0x00

// Commands sent over SPI for the IC Haus MB4
#define WRITE_DATA   0x02
#define READ_DATA    0x03
#define READ_STATUS  0x05
#define WRITE_INSTRUCTION 0x07
#define READ_DATA0   0x09  // 0 Provides fast access to read 
#define WRITE_DATA0  0x0B

// Register addresses to read from the IC Haus MB4 Chip
// Please refer to the datasheet, as the same names for the 
// registers are used here that are in the datasheet
#define SCDATA1   0x00
#define SCDATA1_CRC  0x07
#define ENSCD1    0xC0
#define SCDLEN1   0xC0
#define SELCRCS1  0xC1 // bit 7
#define SCRCLEN1  0xC1 // bit 6:0
#define SCRCSTART1   0xC2 // bit 15:0
#define CHSEL     0xE4
#define REGVERS   0xE5 
#define FREQ      0xE6
#define FREQAGS   0xE8
#define REVISION  0xEA
#define VERSION   0xEB
#define CFGCH1    0xED
#define ACTnSENS  0xEF
#define STATUS_REG   0xF0
#define SVALID    0xF1
#define CDMTIMEOUT   0xF3
#define INSTR     0xF4
#define CFGIF     0xF5
#define CDS_STATUS0  0xF8
#define CDS_STATUS1  0xF9

// MB4Driver class: a class for communicating with the IC-MB4 master from 
//                iC Hause over SPI. This class also implements methods for
//                reading a Renishaw LMA10 absolute magnetic encoder that is 
//                connected to the IC-MB4 in the first slave position. However,
//                primitive readRegister and writeRegister methods are available,
//                allowing this code to be adapted for use in other applications.
//                It is strongly recommended that one is familiar with the IC-MB4
//                datasheet before attempting to interpret specific low level 
//                parts of this code, or before adapting this code to a different 
//                specific application than it was originally intended.
// 
// For all functions, see the comment above each one in the source file (.cpp) for a more in
// depth explanation. 
class MB4Driver {
   // Private methods are for use only within other methods in the MB4Driver
   // class.
   private:
      // The SPI chip select pin that the IC-MB4 is connected to
      uint8_t selectPin;

      // The different status states the MB4 can have. This status also contains
      // status interpretations that are specific to the Renishaw LMA10 encoder
      enum status
      {
         no_errors,        // All clear for data release
         encoder_alarm,    // Invalid position data from encoder
         encoder_warning,  // Warning from the encoder (close to overspeed?)
         invalid_crc       // Cyclic check sum reported incorrectly

      } currentStatus; // currentStatus will hold the status 

      // For descriptions of these two functions please see source file
      uint8_t checkStatus_unprotected();

      uint32_t currentRawPosition;

      // Class member variable that will hold the offset for the encoder
      float offset;
   public:
      // For descriptions of these functions please see the source file,
      // however effort has been made to make the function names self 
      // explanatory. 
      MB4Driver(uint8_t selectPin, float offset);

      uint32_t readRegister(uint8_t registerAddress, uint8_t numBytesToRead);

      void writeRegister(uint8_t registerAddress, uint8_t* data, uint8_t numBytesToWrite);

      void writeRegister(uint8_t registerAddress, uint8_t data);

      void writeInstruction(uint8_t instruction);

      uint32_t getRawPosition();

      float convertRawPosition(uint32_t rawPos, float offset);

      float getPosition();

      void printImportantRegisters();

      void printSCDATA1Registers();

      void printVersion();
};