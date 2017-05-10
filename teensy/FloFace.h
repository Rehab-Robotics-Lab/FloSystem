/**
 * \file
 * Header file for FloFace class, which controls the matrices that make up Flo's face
 *
 * \copyright &copy; PCI 2017. All rights reserved
 */

#ifndef FloFace_h
#define FloFace_h

#include "Arduino.h" // Need the basic arduino libs
#include "Adafruit_LEDBackpack.h" // The library for the LED matrices
#include "Adafruit_GFX.h" // The adafruit graphics drawing library

/**
*   Class to control the matrices that make up Flo's face
*/
class FloFace
{
  public:
    FloFace(int mouth_channel, int left_eye_channel, int right_eye_channel,
            int mouth_rotation = 0, int left_eye_rotation = 0,
            int right_eye_rotation = 0);
    void DrawLeftEye(bool lights[8][8]);
    int SetLeftEyeBrightness(int brightness);
    void DrawRightEye(bool lights[8][8]);
    int SetRightEyeBrightness(int brightness);
    void DrawMouth(bool lights[8][16]);
    int SetMouthBrightness(int brightness);
    void DrawLeftEye(bool lights[64]);
    void DrawRightEye(bool lights[64]);
    void DrawMouth(bool lights[128]);
    int init();



  private:
    Adafruit_8x8matrix* _left_eye; ///<The adafruit object for the left eye matrix
    Adafruit_8x8matrix* _right_eye; ///<The adafruit object for the right eye matrix
    Adafruit_8x16matrix* _mouth; ///<The adafruit object for the mouth
    int _left_eye_channel;
    int _right_eye_channel;
    int _mouth_channel;
    int _left_eye_rot;
    int _right_eye_rot;
    int _mouth_rot;
};

#endif //FloFace_h