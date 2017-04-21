/*************************************************** 
  This is a library for our I2C LED Backpacks

  Designed specifically to work with the Adafruit LED Matrix backpacks 
  ----> http://www.adafruit.com/products/872
  ----> http://www.adafruit.com/products/871
  ----> http://www.adafruit.com/products/870

  These displays use I2C to communicate, 2 pins are required to 
  interface. There are multiple selectable I2C addresses. For backpacks
  with 2 Address Select pins: 0x70, 0x71, 0x72 or 0x73. For backpacks
  with 3 Address Select pins: 0x70 thru 0x77

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

Adafruit_8x8matrix left_eye = Adafruit_8x8matrix();
Adafruit_8x8matrix right_eye = Adafruit_8x8matrix();
Adafruit_8x16matrix mouth = Adafruit_8x16matrix();


void setup() {
  Serial.begin(9600);
  Serial.println("8x8 LED Matrix Test");
  
  left_eye.begin(0x70);  // pass in the address
  right_eye.begin(0x71);
  mouth.begin(0x72);
}



void loop() {
  left_eye.clear();
  left_eye.drawCircle(3,4,3,LED_ON);
  left_eye.drawPixel(3,4,LED_ON);
  left_eye.writeDisplay();  // write the changes we just made to the display

  right_eye.clear();
  right_eye.drawCircle(4,4,3,LED_ON);
  right_eye.drawPixel(4,4,LED_ON);
  right_eye.writeDisplay();  // write the changes we just made to the display

  mouth.clear();
  mouth.drawLine(0,3,3,0,LED_ON);
  mouth.drawLine(7,3,7,12,LED_ON);
  mouth.drawLine(0,12,3,15,LED_ON);
  mouth.writeDisplay();
  delay(500);
}
