// mb4-driver.h
// Class for interfacing with an IC-MB4 chip
// Michael George
// 10/21/16

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

class MB4Driver {
   private:
      uint8_t selectPin;

      enum status
      {
         no_errors,        // All clear for data release
         encoder_alarm,    // Invalid position data from encoder
         encoder_warning,  // Warning from the encoder (close to overspeed?)
         invalid_crc       // Cyclic check sum reported incorrectly

      } currentStatus;

      uint8_t checkStatus_unprotected();

      uint32_t currentRawPosition;

      float offset;
   public:
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