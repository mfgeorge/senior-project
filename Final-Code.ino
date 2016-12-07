/*
  BiSS-Reader.ino
  The main file for a program to read a BiSS signal after it has 
  been processed via an IC Haus MB4 chip. 
  The communication between the two chips occurs via SPI.

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

// Includes for all of the relevant libraries needed for this main file to run:
// Basic arduino libraries for SPI communication and i2c communication
#include <SPI.h>
#include <Wire.h>

// Adafruit libraries required for using the 7 segment display
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

// Filter library required to digitally filter the analog readings from the 
// linear potentiometer to settle down the fluctuations on the display
#include "Filters.h"

// Library of original code written by AVST to communicate with the IC that 
// manages communication with the encoder
#include "mb4-driver.h"


// Encoder Offset if reading does not start at 0 at the start of the encoder
#define ENCODER_OFFSET       124.9 // in inches

// Linear Potentiometer calibration values
#define LIN_POT_OFFSET       -1.43+.640
#define LIN_POT_SCALE        2.2501*5/1023 // in inches for 200 mm length strip

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

// Display
// Arduino Uno   | 7 seg display
//    A5      ->    SCL
//    A4      ->    SDA

// enumeration for the different position measurement modes that 
// we can be in based upon the switch 
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

// This setup function is required by arduino and runs once upon startup 
// of the microcontroller or after a reset.
void setup() {
	  // enable serial communication for debugging if a computer is connected
    Serial.begin(9600);
    // Print out a statement over serial if a computer is connected
    Serial.println("---- Accumulator Position Testing Code Start Up ----");
    // Start the 7 segment display for displaying position readings
    // at the i2c address 0x70
    display.begin(0x70);
    // Set the measurement mode based upon the position of the external 
    // mode switch
    mode = setMeasureMode();
}

// This loop function is required by arduino, and basically wraps all of the code 
// within it in a while (true) loop. In this case it serves as the outermost while
// loop
void loop() {

   // Variable for holding position in inches
   static float position;

   // check what mode we are in and run the appropriate mode
   if (mode == encoder) {
     // If we reach this point in the code, the switch was read to be
     // in encoder measurement mode

     // Perform all of the setup necessary for reading the encoder

     // create an MB4 master object that is connected to the encoder
     MB4Driver master = MB4Driver(SELECT, ENCODER_OFFSET);

     // Raw position in bits
     static uint32_t rawPosition;

     // Inner while loop for simply updating the encoder readings and output
     // the readings to the disply
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
        // Set the measurement mode to the new state
        mode = setMeasureMode();
        // Break out of this inner while loop so that the other state can be reached
        break;
      }
      // Delay after breaking out of the while loop for 100ms
      delay(100);
    }
   }
   else if (mode == linearPot) {
    // If we reach this point in the cod, the switch was read to be in 
    // linear potentiometer measurement mode

    // Perform all of the necessary setup for obtaining readings from the linear 
    // potentiometer

    // Raw position in bits
    static uint16_t rawPosition;

    // Filter objects for filtering the linear potentiometer readings to prevent constant
    // flickering of the display
    // 
    static const float testFrequency = 2;                     // test signal frequency (Hz)
    static const float windowLength = 20.0/testFrequency;     // how long to average the signal, for statistist
    static FilterOnePole filterOneLowpass( LOWPASS, testFrequency );   // create a one pole (RC) lowpass filter
    static RunningStatistics filterOneLowpassStats;                    // create running statistics to smooth these values
    filterOneLowpassStats.setWindowSecs( windowLength );

    while (true) {
      // The inner while loop for getting the linear potentiometer readings, and constantly 
      // updating the display

      // Print the state of the digital pin connected to the switch
      Serial.print("Digital Pin: ");
      // Read the state of the switch and display for debugging
      Serial.println(digitalRead(SWITCH));
      // Read in the raw analog reading
      rawPosition = analogRead(LIN_POT);

      // Convert the reading to inches
      position = linearPotToInches(rawPosition, LIN_POT_OFFSET, &filterOneLowpass);

      // output the position to the buffer of the display
      display.println(position, 3); // Try to diplay to 3rd decimal point
      // refresh the display so that the new reading is seen on it
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

// Function for converting the raw position obtainted from the linear potentiometer 
// to readings in inches. 
// Parameters: 
// rawPosition: a number from the 10-bit adc ranging from 0-1023
// offset: a number in inches representing the offset of 0 from the start of the 
//          linear potentiometers active area
// filterOneLowpass: a pointer to a first order lowpass filter object
// returns: a float- the position in inches

float linearPotToInches(uint16_t rawPosition, float offset, FilterOnePole* filterOneLowpass) {

  // pass the raw position reading into the lowpass filter
  filterOneLowpass->input(rawPosition);
  // update the rawposition variable to be the output from the lowpass filter
  rawPosition = filterOneLowpass->output();
  // return the position in inches based upon a linear calibration
  return float(rawPosition)*LIN_POT_SCALE - offset;
}

// Function for setting the measurement mode based upon the position of an external 
// switch.
// Parameters: none
// returns: the measurement mode that the switch has selected

uint8_t setMeasureMode(){
  // Set the pin mode before a reading occurs to be an input connected 
  // to the internal pull up resistors of the arduino
  pinMode(SWITCH, INPUT_PULLUP);
  // Read the connected switch once 
  if (digitalRead(SWITCH)) {
    // If the switch is open, then the encoder mode is selected. 
    mode = encoder;
    // Print a debug statement
    Serial.println("Encoder Mode Selected");
  } 
  else {
    // If the switch is closed, then linear potentiometer 
    // mode is selected
    mode = linearPot;
    // print a debug statement 
    Serial.println("Linear Pot Mode Selected");
  }
  // Return the mode based upon the position of the switch
  return mode;
}
