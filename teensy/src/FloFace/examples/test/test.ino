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
