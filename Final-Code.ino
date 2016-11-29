/*
  BiSS-Reader.ino
  A program to read a BiSS signal after it has been processed via 
  an IC Haus MB4 chip. The communication between the two chips
  occurs via SPI.

   California Polytechnic State University, San Luis Obispo
   In partial fulfillment of the requirements for a bachelor's 
   degree from the department of Mechanical Engineering. 

   Michael George
   10/21/16

   This code comes without any warrenty or guarantee from the author. 
   Any usage is at the discression of the user, and should be done 
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
#include "mb4-driver.h"
#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"
#include "Filters.h"

// Encoder Offset if reading does not start at 0 at the start of the encoder
#define ENCODER_OFFSET       124.9 // in inches

// Linear Pot offset if reading does not start at 0
#define LIN_POT_OFFSET       0
#define LIN_POT_SCALE        300/25.4*1/1023 // in inches for 200 mm length strip

// The connections from the Arduino Uno to the IC-MB4 chip are:

// Arduino Uno 		| 	IC-MB4
// 		10			->	1	(slave select)	
#define SELECT 		10
//		11 			->	3 	(Master Out Slave In) 
// 		12 			->	4 	(Master In Slave Out)
// 		13 			-> 	2 	(Serial Clock)

// Linear Pot
#define LIN_POT   A0

// Mode Selection switch input 
#define SWITCH    3 // Pin D3 

enum Position_Mode {
  linearPot,
  encoder
} mode;

// Function prototype for converting linear pot analog reading to inches
float linearPotToInches(uint16_t rawPosition, float offset);

// Function for setting the measurement mode to encoder or linear pot
uint8_t setMeasureMode();

// create a 7 segment display object for displaying the reading
Adafruit_7segment display = Adafruit_7segment();

void setup() {
	// enable serial communication
    Serial.begin(9600);
    Serial.println("---- Accumulator Position Testing Code Start Up ----");
    display.begin(0x70);
    mode = setMeasureMode();
}

void loop() {


   // Position in inches
   static float position;

   if (mode == encoder) {

     // create an MB4 master object that is connected to the encoder
     MB4Driver master = MB4Driver(SELECT, ENCODER_OFFSET);

     // Raw position in bits
     static uint32_t rawPosition;

     while (true) {
      // Get the raw encoder position
      rawPosition = master.getRawPosition();

      // Get the position in inches
      position = abs(master.getPosition());

      // Display the position
      display.println(position, 3); // Try to diplay to 3rd decimal point
      display.writeDisplay();

      // Print out position information to a serial terminal too for debugging
      Serial.print("Position [in] = \t");
      Serial.print(position, 4);

      // Print out the raw bit position as well for debugging 
      Serial.print("\t Raw Position [bits] = \t");
      Serial.println(rawPosition);
      // Check for a mode change
      if (!(digitalRead(SWITCH))){
        mode = setMeasureMode();
        break;
      }
      delay(100);
    }
   }
   else if (mode == linearPot) {

    // Raw position in bits
    static uint16_t rawPosition;

    // Filter objects 
    static const float testFrequency = 2;                     // test signal frequency (Hz)
    static const float windowLength = 20.0/testFrequency;     // how long to average the signal, for statistist
    static FilterOnePole filterOneLowpass( LOWPASS, testFrequency );   // create a one pole (RC) lowpass filter
    static RunningStatistics filterOneLowpassStats;                    // create running statistics to smooth these values
    filterOneLowpassStats.setWindowSecs( windowLength );

    while (true) {
      Serial.print("Digital Pin: ");
      Serial.println(digitalRead(SWITCH));
      // Read in the raw analog reading
      rawPosition = analogRead(LIN_POT);

      // Convert the reading to inches
      position = linearPotToInches(rawPosition, LIN_POT_OFFSET, &filterOneLowpass);

      // Display the position
      display.println(position, 3); // Try to diplay to 3rd decimal point
      display.writeDisplay();

      // Print out position information to a serial terminal too for debugging
      Serial.print("Position [in] = \t");
      Serial.print(position, 4);

      // Print out the raw bit position as well for debugging 
      Serial.print("\t Raw Position [bits] = \t");
      Serial.println(rawPosition);

      // Check for a mode change
      if (digitalRead(SWITCH)){
        mode = setMeasureMode();
        break;
      }
      delay(100);
    }
   }
}


float linearPotToInches(uint16_t rawPosition, float offset, FilterOnePole* filterOneLowpass) {

  filterOneLowpass->input(rawPosition);
  rawPosition = filterOneLowpass->output();
  return float(rawPosition)*LIN_POT_SCALE - offset;
}

uint8_t setMeasureMode(){
  pinMode(SWITCH, INPUT_PULLUP);
  if (digitalRead(SWITCH)) {
    mode = encoder;
    Serial.println("Encoder Mode Selected");
  } 
  else {
    mode = linearPot;
    Serial.println("Linear Pot Mode Selected");
  }
  return mode;
}
