/**
 * \file
 * Source file for FloFace class, which controls the matrices that make up Flo's face
 *
 * \copyright &copy; PCI 2017. All rights reserved
 */

#include "FloFace.h"

/**
 * Constructor for Flo Face class
 * @param mouth_channel The  channel of the mouth
 * @param left_eye_channel The I2C channel of the left eye. Valid inputs:
 *                         0x70, 0x71, 0x72 or 0x73
 * @param right_eye_channel The I2C channel of the right eye. Valid inputs:
 *                          0x70, 0x71, 0x72 or 0x73
 * @param mouth_channel The I2C channel of the mouth. Valid inputs: 0x70 thru 0x77
 * @param left_eye_rotation The rotation of the left eye. Valid inputs: 0, 1, 2, 3
 * @param right_eye_rotation The rotation of the right eye. Valid inputs: 0, 1, 2, 3
 * @param mouth_rotation The rotation of the left eye. Valid inputs: 1, 3
 */
FloFace::FloFace(int mouth_channel, int left_eye_channel, int right_eye_channel,
                 int mouth_rotation, int left_eye_rotation,
                 int right_eye_rotation)
{
    // assign variables
    _left_eye = new Adafruit_8x8matrix();
    _right_eye = new Adafruit_8x8matrix();
    _mouth = new Adafruit_8x16matrix();

    _left_eye_channel = left_eye_channel;
    _right_eye_channel = right_eye_channel;
    _mouth_channel = mouth_channel;
    _left_eye_rot = left_eye_rotation;
    _right_eye_rot = right_eye_rotation;
    _mouth_rot = mouth_rotation;
}

/**
 * Initializes the face constructs so that they can start being displayed to
 * @return Status flag: 0=success
 */
int FloFace::init()
{
    int status = 0;
    // initialize hardware
    _left_eye->begin(_left_eye_channel); // pass in the address
    _right_eye->begin(_right_eye_channel);
    _mouth->begin(_mouth_channel);

    // Set rotations
    _left_eye->setRotation(_left_eye_rot);
    _right_eye->setRotation(_right_eye_rot);
    _mouth->setRotation(_mouth_rot);

    return status;
}

/**
 * Sets the brightness of the left eye
 * @param brightness The desired brightness, range 0-15
 * @return status, -1 indicates an out of range input
 */
int FloFace::SetLeftEyeBrightness(int brightness)
{
    if (brightness < 0 || brightness > 15)
    {
        return -1;
    }
    _left_eye->setBrightness(brightness);
    return 0;
}

/**
 * Sets the brightness of the right eye
 * @param brightness The desired brightness, range 0-15
 * @return status, -1 indicates an out of range input
 */
int FloFace::SetRightEyeBrightness(int brightness)
{
    if (brightness < 0 || brightness > 15)
    {
        return -1;
    }
    _right_eye->setBrightness(brightness);
    return 0;
}

/**
 * Sets the brightness of the mouth
 * @param brightness The desired brightness, range 0-15
 * @return status, -1 indicates an out of range input
 */
int FloFace::SetMouthBrightness(int brightness)
{
    if (brightness < 0 || brightness > 15)
    {
        return -1;
    }
    _mouth->setBrightness(brightness);
    return 0;
}

/**
 * Draws the left eye
 * @param lights The lights to turn on/off, is an array , first dimension is row
 */
void FloFace::DrawLeftEye(bool lights[8][8])
{
    Serial.println("in left eye matrix draw");
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            _left_eye->drawPixel(j, i, lights[i][j]);
        }
    }
    _left_eye->writeDisplay();
}

/**
 * Draws the right eye
 * @param lights The lights to turn on/off, is an array, first dimension is row
 */
void FloFace::DrawRightEye(bool lights[8][8])
{
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            _right_eye->drawPixel(j, i, lights[i][j]);
        }
    }
    _right_eye->writeDisplay();
}

/**
 * Draws the mouth
 * @param lights The lights to turn on/off, is an array, first dimension is row
 */
void FloFace::DrawMouth(bool lights[8][16])
{
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 16; j++)
        {
            _mouth->drawPixel(j, i, lights[i][j]);
        }
    }
    _mouth->writeDisplay();
}

/**
 * Draws the mouth
 * @param lights The lights to turn on/off, is an array, flattened: [row1,row2,...]
 */
void FloFace::DrawMouth(bool lights[168])
{
    bool(&rect)[8][16] = *reinterpret_cast<bool(*)[8][16]>(lights);
    DrawMouth(rect);
}

/**
 * Draws the left eye
 * @param lights The lights to turn on/off, is an array, flattened: [row1,row2,...]
 */
void FloFace::DrawLeftEye(bool lights[64])
{
    bool(&rect)[8][8] = *reinterpret_cast<bool(*)[8][8]>(lights);
        DrawLeftEye(rect);

}

/**
 * Draws the right eye
 * @param lights The lights to turn on/off, is an array, flattened: [row1,row2,...]
 */
void FloFace::DrawRightEye(bool lights[64])
{
    bool(&rect)[8][8] = *reinterpret_cast<bool(*)[8][8]>(lights);
    DrawRightEye(rect);
}
