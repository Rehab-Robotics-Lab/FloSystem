/**
 * \file
 * A test file to see if your matrices are hooked up properly. 
 * You should see eyes and mouth on the screens
 * You will likely need to change the addresses and orientation on line 14
 * Orientation: should have one dot in x direc, 2 in y direc on bottom left
 *
 * \copyright &copy; PCI 2017. All rights reserved
 */

#include "FloFace.h"

FloFace floFace = FloFace(0x72, 0x71, 0x70, 3, 1, 3);

void setup()
{

    Serial.begin(9600);
    floFace.init();
    Serial.println("done with init");
}

void loop()
{
    
if(Serial.available()){
        parse_serial();
    }
    //INCOMING SERIAL DATA
    while (Serial.available() > 0)
    {
        byte byte1 = Serial.read();
        byte byte2 = Serial.read();
        int control = (byte1 & 0xF0) >> 4;
        int val = ((byte1 & 0x0F) << 8) + byte2;
        if (control == 1)
        {
            actP = val;
        }
        if (control == 2)
        {
            setP = val;
        }
        if (control == 3)
        {
            runningV = false;
        }
        if (control == 4)
        {
            runningV = true;
        }
    }

    bool bob[64] = {1,1,1,0,0,0,0,0,1};
    bool sue[168] = {1,1,1,0,0,0,0,0,1};
    for (int i = 0; i < 64; i++)
    {
            Serial.print(bob[i]);
    }
    floFace.DrawLeftEye(bob);
    floFace.DrawRightEye(bob);
    floFace.DrawMouth(sue);

    bool small2d[8][8];
    small2d[0][0] = 1;
    small2d[1][0] = 1;
    small2d[2][0] = 1;
    small2d[0][1] = 1;
    small2d[3][3] = 1;


    bool small2d[8][8] =
        {
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 1, 1, 1, 1, 0, 0},
            {0, 1, 1, 1, 1, 1, 1, 0},
            {0, 1, 0, 0, 1, 1, 1, 0},
            {0, 1, 0, 0, 1, 1, 1, 0},
            {1, 1, 1, 1, 1, 1, 1, 0},
            {1, 0, 1, 1, 1, 1, 0, 0},
            {1, 1, 0, 0, 0, 0, 0, 0}};
    bool big2d[8][16] =
        {
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
            {1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
            {1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0},
            {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
    bool smallzero[8][8] = {0};
    bool bigzero[8][16] = {0};

    floFace.DrawLeftEye(small2d);
    floFace.DrawRightEye(small2d);
    floFace.DrawMouth(big2d);
    delay(1000);
    floFace.DrawLeftEye(smallzero);
    floFace.DrawRightEye(smallzero);
    floFace.DrawMouth(bigzero);
    delay(1000);

    bool happyMouth[168] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    bool eyeLeft[64] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1,
                        1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    floFace.DrawLeftEye(eyeLeft);
    floFace.DrawRightEye(eyeLeft);
    floFace.DrawMouth(happyMouth);
    delay(1000);
}



