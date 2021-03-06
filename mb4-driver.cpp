/* mb4-driver.cpp
   Source code for a class for interfacing with an IC-MB4 master IC. 
   
   California Polytechnic State University, San Luis Obispo
   In partial fulfillment of the requirements for a bachelor's 
   degree from the department of Mechanical Engineering. 

   Michael George
   10/21/16

   This code comes without any warranty or guarantee from the author. 
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

// Include the header file which has all of the prototypes of the functions
// contained in this source file.
#include "mb4-driver.h"

// For class description, please refer to the header file. This is the first 
// place to start for a general overview of the methods available, and can 
// provide a general idea of the methods available in this class. 


// MB4Driver: Constructor for the MB4Driver class
// Parameters: 
// selectPin: the SPI chip select pin that the mb4 is connected to
// offset: (optional parameter) the offset in inches for the encoder readings
MB4Driver::MB4Driver(uint8_t selectPin, float offset=0){
   // Begin the SPI communication protocol that will be used to 
   // communicate with the IC-mb4 chip
   SPI.begin();

   // Setup the necessary serial communication for this library
   if(!Serial){
      Serial.begin(9600);
   }

   this->selectPin = selectPin;

   // Setup Pin 10 to be a digital output for slave select
   pinMode(this->selectPin, OUTPUT);

   // Set the select pin high so that communication is not yet enabled
   digitalWrite(this->selectPin, 1);

   // Store the offset from 0 for this encoder
   this->offset = offset;

   // Tell master to stop any previous processes and start fresh
   this->writeInstruction(BREAK);

   // Set the Channel 1 as the only active channel
   this->writeRegister(CHSEL, CH1);

   // Set up the channel as BiSS register configuration
   this->writeRegister(REGVERS, BISS_C << 6);

   // Set the FREQ register bit 4:0 to communicate with encoder
   uint8_t currentFREQ = this->readRegister(FREQ, 1);
   currentFREQ &= ~(0b00011111);
   currentFREQ |= CLOCK_SPEED;
   this->writeRegister(FREQ, currentFREQ);
   Serial.print("FREQ: \t\t");
   Serial.println(this->readRegister(FREQ, 1));

   // Set up the communication for BiSS C protocol
   uint8_t currentCFGCH1 = this->readRegister(CFGCH1, 1);
   currentCFGCH1 &= ~(0b00001111);
   currentCFGCH1 |= BISS_C;
   this->writeRegister(CFGCH1, currentCFGCH1);
   Serial.print("CFGCH1: \t");
   Serial.println(this->readRegister(CFGCH1, 1));

   // Set up for automatically starting read cycles
   this->writeRegister(FREQAGS, AGSFREQ);
   Serial.print("FREQAGS: \t");
   Serial.println(this->readRegister(FREQAGS, 1));

   // Set up for RS422 Line levels in CFGIF bit 3:2
   uint8_t currentCFGIF = this->readRegister(CFGIF, 1);
   currentCFGIF &= ~(0b00001111);
   currentCFGIF |= (RS422 << 2);
   // Enable the internal clock sou~rce 
   currentCFGIF |= (1);
   this->writeRegister(CFGIF, currentCFGIF);
   Serial.print("CFGIF: \t\t");
   Serial.println(this->readRegister(CFGIF, 1));

      // Configure the data length of the SCD. bit 5:0 SCDLEN1
   uint8_t currentSCDLEN1 = this->readRegister(SCDLEN1, 1);
   currentSCDLEN1 &= ~(0b11111111);
   currentSCDLEN1 |= (DATA_LENGTH);
   currentSCDLEN1 |= (SCD_AVAIL << 6);
   this->writeRegister(SCDLEN1, currentSCDLEN1);
   Serial.print("SCDLEN1 & ENSCD1: \t");
   Serial.println(this->readRegister(SCDLEN1, 1));

   // Configure the CRC info
   this->writeRegister(SELCRCS1, (CRC_SELECT << 7) | CRC_POLY);
   Serial.print("SELCRCS1: \t");
   Serial.println(this->readRegister(SELCRCS1, 1));

   // Configure the CRC start 
   uint8_t crcStartToSend[2];
   crcStartToSend[0] = CRC_START;
   crcStartToSend[1] = CRC_START;
   this->writeRegister(SCRCSTART1, crcStartToSend, 2);
   Serial.print("SCRCSTART1: \t");
   Serial.println(this->readRegister(SCRCSTART1, 2));

   // Configure all slaves to be sensors
   this->writeRegister(ACTnSENS, SLAVES);
   Serial.print("ACTnSENS: \t");
   Serial.println(this->readRegister(ACTnSENS, 1));

   // Enable the AGS (Automatic Get Sensor) bit so that the MB4 now polls
   // encoder
   uint8_t currentInstruction = this->readRegister(INSTR, 1);
   currentInstruction |= 1;
   this->writeInstruction(currentInstruction);
   Serial.print("INSTR: \t ");
   Serial.println(this->readRegister(INSTR, 1), BIN);

   // Notify user that MB4Driver is instantiated
   Serial.println("MB4Driver Instantiated");

   // Notify user of version of the MB4 IC
   this->printVersion();

   // Give Time to collect the first reading
   delay(1000);

   // Print out all the initial registers for SCDATA1
   Serial.print("00:\t");
   Serial.print(this->readRegister(0x00, 1), HEX);

   Serial.print("\t01: ");
   Serial.print(this->readRegister(0x01, 1), HEX);

   Serial.print("\t02: ");
   Serial.print(this->readRegister(0x02, 1), HEX);

   Serial.print("\t03: ");
   Serial.print(this->readRegister(0x03, 1), HEX);

   Serial.print("\t04: ");
   Serial.print(this->readRegister(0x04, 1), HEX);

   Serial.print("\t05: ");
   Serial.print(this->readRegister(0x05, 1), HEX);

   Serial.print("\t06: ");
   Serial.print(this->readRegister(0x06, 1), HEX);

   Serial.print("\t07: ");
   Serial.println(this->readRegister(0x07, 1), HEX);

   // Print the first raw position reading
   this->getRawPosition();

   //this->printImportantRegisters();

}

// readRegister: Function for reading a specific register on the MB4
// Parameters:
// registerAddress: the register starting address to read from
// numBytesToRead: the number of the bytes to read
// returns: the value read from the register address assuming that the 
//          first register read is the most significant byte
uint32_t MB4Driver::readRegister(uint8_t registerAddress, uint8_t numBytesToRead){
   // Drop the chip select pin low to select MB4 for output
   digitalWrite(this->selectPin, 0);

   // Configure the correct SPI settings to be used
   SPI.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE0));

   // Send the read command 
   SPI.transfer(READ_DATA);

   // Send the register address to read from 
   SPI.transfer(registerAddress);

   // Create a buffer to read bytes into
   uint8_t buffer = 0;

   // Create a value to return 
   uint32_t value = 0;

   // Read the bytes in a loop 
   for(int i=0; i<numBytesToRead; i++){
      buffer = SPI.transfer(0);
      // Arrage the bytes for a proper number to return
      value *= 2^8;

      value +=buffer;
   }

   // Bring chip select high to stop communication with MB4
   digitalWrite(this->selectPin, 1);

   // End the SPI transaction for nice cooperation with other 
   // SPI dependent libraries
   SPI.endTransaction();

   return value;

}

// writeRegister: A function to write data to a register on the MB4 
//                  (array version) 
// Parameters:
// registerAddress: the starting address of the register to write to
// data: an array of bytes to write 
// numBytesToWrite: the number of bytes in array pointer or data to write
void MB4Driver::writeRegister(uint8_t registerAddress, uint8_t* data, uint8_t numBytesToWrite){
   // Drop the chip select pin low to select MB4 for output
   digitalWrite(this->selectPin, 0);

   // Configure the correct SPI settings to be used
   SPI.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE0));

   // Send the read command 
   SPI.transfer(WRITE_DATA);

   // Send the register address to write to 
   SPI.transfer(registerAddress);

   // Write the bytes
   SPI.transfer(data, numBytesToWrite);

   // Bring chip select high to stop communication with MB4
   digitalWrite(this->selectPin, 1);

   // End the SPI transaction for nice cooperation with other 
   // SPI dependent libraries
   SPI.endTransaction();

}

// writeRegister: A function to write data to a register on the MB4 
//                  (byte version) 
// Parameters:
// registerAddress: the starting address of the register to write to
// data: the byte of data to be written to that address
void MB4Driver::writeRegister(uint8_t registerAddress, uint8_t data){
   // Drop the chip select pin low to select MB4 for output
   digitalWrite(this->selectPin, 0);

   // Configure the correct SPI settings to be used
   SPI.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE0));

   // Send the read command 
   SPI.transfer(WRITE_DATA);

   // Send the register address to write to 
   SPI.transfer(registerAddress);

   // Write the data
   SPI.transfer(data);

   // Bring chip select high to stop communication with MB4
   digitalWrite(this->selectPin, 1);

   SPI.endTransaction();

}

// writeInstruction: A function to quickly write data to the MB4's instruction 
//                  register.
// Parameters:
// instruction: The instruction to write to the MB4's instruction register
// returns: nothing
void MB4Driver::writeInstruction(uint8_t instruction){
   // Drop the chip select pin low to select MB4 for output
   digitalWrite(this->selectPin, 0);

   // Configure the correct SPI settings to be used
   SPI.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE0));

   // Send the read command 
   SPI.transfer(WRITE_INSTRUCTION);

   // Write the bytes
   SPI.transfer(instruction);

   // Bring chip select high to stop communication with MB4
   digitalWrite(this->selectPin, 1);

   SPI.endTransaction();

}

// getRawPosition:
// A function to get the raw position data from the MB4 chip. This is
// where SPI must be used to communicate with the MB4 chip. 
// Parameters: none
// Returns: raw position in a 0 to 2^26 number. 
uint32_t MB4Driver::getRawPosition() {

   // Lock the bank before reading SCDATA1 to prevent data corruption
   uint8_t currentInstruction = this->readRegister(INSTR, 1);
   currentInstruction |= (1 << 6);
   this->writeInstruction(currentInstruction);

   // // Check out how the instruction register responds
   // Serial.print("INSTR after lock: \t ");
   // Serial.println(this->readRegister(INSTR, 1), BIN);

   // Create an array of 4 x 8 bit numbers
   uint32_t readingBuffer;

   // Initialize a zero reading
   uint32_t reading = 0;

   // Read the data and unpack into correct order one register at a time
   for(uint8_t index = 0; index < 4; index++){
      readingBuffer = this->readRegister(SCDATA1 + index, 1);
      reading += readingBuffer << (index*8);
   }

   // Shift the status bits out of the reading
   reading = reading >> 2;

   // Check if the reading is valid 
   // Can use unprotected checkStatus_unprotected() since data registers are 
   // locked.
   if (this->checkStatus_unprotected() == no_errors){
      this->currentRawPosition = reading;

      // // Print out the raw encoder reading recieved
      // Serial.print("Raw Encoder Reading: \t");
      // Serial.println(reading);
   }

   // Unlock the bank after reading SCDATA1 to allow those registers to update
   currentInstruction = this->readRegister(INSTR, 1);
   currentInstruction &= ~(1 << 6);
   this->writeInstruction(currentInstruction);

   // // Check out how the instruction register responds
   // Serial.print("INSTR after unlock: \t ");
   // Serial.println(this->readRegister(INSTR, 1), BIN);

   return this->currentRawPosition;

}

// checkStatus_unprotected: a function for use after the data registers
//                          have been locked. Checks the error registers
//                          to make sure the encoder and the MB4 are not
//                          reporting any errors.
// Parameters: None 
// Returns: the currentStatus of the encoder. which can be
//          no_errors, invalid_crc, encoder_warning, or encoder_alarm.
uint8_t MB4Driver::checkStatus_unprotected(){

   // The encoder status (no warnings is 00, refer to LMA10 datasheet)
   uint32_t encoderStatus = this->readRegister(SCDATA1, 1);

   // Check if CRC is correct
   bool valid = (this->readRegister(SVALID,1) == 2) ? true : false;

   // Check for errors in this order of precedence (some errors trump others)
   if ((encoderStatus == 0) && valid && this->currentStatus != encoder_alarm) {
      this->currentStatus = no_errors;
   }
   else if (!valid && this->currentStatus != encoder_alarm) {               
   // Error in com between MB4 and encoder
      this->currentStatus = invalid_crc;
      Serial.println("INVALID CRC");
   }  
   else if (encoderStatus == 1 && this->currentStatus != encoder_alarm) {  
   // Close to overspeed, consult LMA10 datasheet
      this->currentStatus = encoder_warning; 
      Serial.println("ENCODER WARNING");
   }
   else if (encoderStatus == 2 || this->currentStatus == encoder_alarm) {   
   // Encoder invalid position data
      this->currentStatus = encoder_alarm;
      Serial.println("ENCODER ALARM");
   }

   return this->currentStatus;
}


// convertRawPosition: converts the raw position readings of the encoder (bits)
//                    into a decimal number in inches
// Parameters:
// rawPos: The raw position in bits 
// offset: The offset distance in inches to acheive 0 (some encoder strips don't
//          start at 0)
// returns: a float representing the position in inches on the encoder strip
float MB4Driver::convertRawPosition(uint32_t rawPos, float offset){
   return (float)(rawPos)*CONV_FAC - offset;
}

// getPosition: gets the current position of the encoder in inches. This function
//              will automate the call of getRawPosition() and put the output into
//              convertRawPosition. 
// Parameters: None
// returns: a float representing the position of the encoder in inches
float MB4Driver::getPosition(){
   // get the current position in inches from the encoder
   float position = this->convertRawPosition(this->getRawPosition(), this->offset);
   // For some reason, the position will suddenly jump to <100 if the encoder 
   // goes off the strip
   if (position > 10.0 && position < 100) {
      position -= 85.60; // cover the case of barely going off the strip
   }
   else if (position > 190) {
      position -= 200; // cover the case of the 200 range
   }
   return position;
}

// printImportantRegisters: A function to print all of the important registers that 
//                          must be configured in order for a single encoder sensor
//                          to be used in a polling scheme. This function is useful 
//                          for designing and debugging applications with the MB4 ic
// Parameters: None
// returns: Nothing (the print to standard out is what results)
void MB4Driver::printImportantRegisters(){
   Serial.println();
   Serial.println("------ Important Registers Print out ------");

   Serial.print("C0:\t");
   Serial.println(this->readRegister(0xC0, 1), HEX);

   Serial.print("C1:\t");
   Serial.println(this->readRegister(0xC1, 1), HEX);

   Serial.print("E4:\t");
   Serial.println(this->readRegister(0xE4, 1), HEX);

   Serial.print("E5:\t");
   Serial.println(this->readRegister(0xE5, 1), HEX);

   Serial.print("E6:\t");
   Serial.println(this->readRegister(0xE6, 1), HEX);

   Serial.print("E8:\t");
   Serial.println(this->readRegister(0xE8, 1), HEX);

   Serial.print("EA:\t");
   Serial.println(this->readRegister(0xEA, 1), HEX);

   Serial.print("EB:\t");
   Serial.println(this->readRegister(0xEB, 1), HEX);

   Serial.print("EC:\t");
   Serial.println(this->readRegister(0xEC, 1), HEX);

   Serial.print("ED:\t");
   Serial.println(this->readRegister(0xED, 1), HEX);

   Serial.print("F0:\t");
   Serial.println(this->readRegister(0xF0, 1), HEX);

   Serial.print("F1:\t");
   Serial.println(this->readRegister(0xF1, 1), HEX);

   Serial.print("F3:\t");
   Serial.println(this->readRegister(0xF3, 1), HEX);

   Serial.print("F5:\t");
   Serial.println(this->readRegister(0xF5, 1), HEX);

   Serial.print("F8:\t");
   Serial.println(this->readRegister(0xF8, 1), HEX);

   Serial.print("F9:\t");
   Serial.println(this->readRegister(0xF9, 1), HEX);

   Serial.println("------ End of Important Register Print out -------");
}

// printSCDATA1Registers: prints all of the registers associated with the 
//                      first slave device. This function is useful for 
//                      debugging and designing code for specific applications.
// Parameters: None
// Returns: nothing
void MB4Driver::printSCDATA1Registers(){

   // Lock the bank before reading SCDATA1 to prevent data corruption
   uint8_t currentInstruction = this->readRegister(INSTR, 1);
   currentInstruction |= (1 << 6);
   this->writeInstruction(currentInstruction);

   // Print out all the SCDATA1 registers for debugging 
   Serial.print("00: ");
   Serial.print(this->readRegister(0x00, 1), HEX);

   Serial.print("\t| 01: ");
   Serial.print(this->readRegister(0x01, 1), HEX);

   Serial.print("\t| 02: ");
   Serial.print(this->readRegister(0x02, 1), HEX);

   Serial.print("\t| 03: ");
   Serial.print(this->readRegister(0x03, 1), HEX);

   Serial.print("\t| 04: ");
   Serial.print(this->readRegister(0x04, 1), HEX);

   Serial.print("\t| 05: ");
   Serial.print(this->readRegister(0x05, 1), HEX);

   Serial.print("\t| 06: ");
   Serial.print(this->readRegister(0x06, 1), HEX);

   Serial.print("\t| 07: ");
   Serial.println(this->readRegister(0x07, 1), HEX);

   // Unlock the bank after reading SCDATA1 to allow the registers to update
   currentInstruction = this->readRegister(INSTR, 1);
   currentInstruction &= ~(1 << 6);
   this->writeInstruction(currentInstruction);

}

// printVersion: Function to use for printing the version of the MB4 ic
//              iC Haus reccomends this as the first step to see if your 
//              MB4 is wired correctly and initially establishing 
//              communication
// Parameters: None
// Returns: Nothing
void MB4Driver::printVersion() {

   // read the version and revision register
   uint32_t version = this->readRegister(VERSION, 1);
   uint32_t revision = this->readRegister(REVISION, 1);

   // Print them out to the user over serial 
   Serial.println("Version data of MB4 instantiated:");
   Serial.print("\n Version recieved is: \t");
   Serial.print(version);
   Serial.print("\t Revision recieved is: \t");
   Serial.print(revision);
   Serial.println();

}