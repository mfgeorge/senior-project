//BiSS-Reader.ino
// A program to read a BiSS signal after it has been processed via 
// an IC Haus MB4 chip. The communication between the two chips
// occurs via SPI.

#include <SPI.h>
#include "mb4-driver.h"
#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

// Offset if reading does not start at 0 at the start of the encoder
#define OFFSET       126.8997-0.0516-0.0479-0.0491 // in inches

// The connections from the Arduino Uno to the IC-MB4 chip are:

// Arduino Uno 		| 	IC-MB4
// 		10			->	1	(slave select)	
#define SELECT 		10
//		11 			->	3 	(Master Out Slave In) 
// 		12 			->	4 	(Master In Slave Out)
// 		13 			-> 	2 	(Serial Clock)

// create a 7 segment display object for displaying the reading
Adafruit_7segment display = Adafruit_7segment();

void setup() {
	// enable serial communication
    Serial.begin(9600);
    Serial.println("---- BiSS Reader Starting ----");
    display.begin(0x70);
}

void loop() {

  // create an MB4 master object that is connected to the encoder
  MB4Driver master = MB4Driver(SELECT, OFFSET);

   // Raw position in bits from encoder
   static uint32_t rawPosition;

   // Position in inches
   static float position;

   while (true) {
      // Get the raw encoder position
      rawPosition = master.getRawPosition();

      // Get the position in inches
      position = master.getPosition();

      // Display the position
      display.println(position, 3); // Try to diplay to 3rd decimal point
      display.writeDisplay();

      // Print out position information to a serial terminal too for debugging
      Serial.print("Position [in] = \t");
      Serial.print(position, 4);

      // Print out the raw bit position as well for debugging 
      Serial.print("\t Raw Position [bits] = \t");
      Serial.println(rawPosition);


   	delay(100);
   }
}



