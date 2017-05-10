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
#include "FloFace.h"

FloFace floFace = FloFace(0x72,0x71,0x70,0,1,3);

void setup()
{

      Serial.begin(9600);
        Serial.println("8x8 LED Matrix Test");
    floFace.init();
    Serial.println("done with init");
}

void loop()
{   
    Serial.println("loop");
    // if(Serial.available()){
    //     parse_serial();
    // }
    // //INCOMING SERIAL DATA
    // while (Serial.available() > 1)
    // {
    //     byte byte1 = Serial.read();
    //     byte byte2 = Serial.read();
    //     int control = (byte1 & 0xF0) >> 4;
    //     int val = ((byte1 & 0x0F) << 8) + byte2;
    //     if (control == 1)
    //     {
    //         actP = val;
    //     }
    //     if (control == 2)
    //     {
    //         setP = val;
    //     }
    //     if (control == 3)
    //     {
    //         runningV = false;
    //     }
    //     if (control == 4)
    //     {
    //         runningV = true;
    //     }
    // }

    // Orientation test... should have one dot in x direc, 2 in y direc
    bool bob[64] = {1,1,1,0,0,0,0,0,1};
    bool sue[168] = {1,1,1,0,0,0,0,0,1};
    for (int i = 0; i < 64; i++)
    {
            Serial.print(bob[i]);
    }
    floFace.DrawLeftEye(bob);
    floFace.DrawRightEye(bob);
    floFace.DrawMouth(sue);
}
