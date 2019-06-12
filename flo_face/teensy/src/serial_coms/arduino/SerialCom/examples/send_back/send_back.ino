/**
 * \file
 * A test file for the serial communications library. Will send back whatever it
 * receives 
 *
 * \copyright &copy; PCI 2017. All rights reserved
 */

#include "SerialCom.h"

void reply(char* data, int length);

SerialCom communicator = SerialCom(reply);

void setup()
{
    communicator.init();
}

void loop()
{
    communicator.update();
}

void reply(char* data, int length){
    communicator.sendData(data, length);
}