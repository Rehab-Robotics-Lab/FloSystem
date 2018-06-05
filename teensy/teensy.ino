/**
 * \file
 * Running the matrices on flo faces
 *
 * \copyright &copy; PCI 2017. All rights reserved
 */

#include "src/FloFace/src/FloFace.h"
#include "src/serial_coms/arduino/SerialCom/src/SerialCom.h" // https://github.com/Rehab-Robotics-Lab/serial_coms

FloFace floFace = FloFace(0x72, 0x71, 0x70, 3, 1, 1);

void process_incoming(char* data, int length);
SerialCom communicator = SerialCom(process_incoming);
bool mouth[168];
bool leye[64];
bool reye[64];


void setup()
{
    communicator.init();
    floFace.init();
    char send[] = "Done with init";
    communicator.sendData(send, sizeof(send)/sizeof(send[0]));
}

void loop()
{
    communicator.update();
}
/**
 * 1) check first byte for what we are lighting up
 * 2) remaining bits are on off values for the matrices, so draw them...
 */
void process_incoming(char* data, int length){
    if(data[0]==0){
        for(int i = 0; i<168; i++){
            mouth[i] = (data[1 + i/8] >> (7 - i%8)) & 1;
        }
        floFace.DrawMouth(mouth);
    }else if(data[0]==1){
        for(int i = 0; i<64; i++){
            leye[i] = (data[1 + i/8] >> (7 - i%8)) & 1;
        }
        floFace.DrawLeftEye(leye);
    }else if(data[0]==2){
        for(int i = 0; i<64; i++){
            reye[i] = (data[1 + i/8] >> (7 - i%8)) & 1;
        }
        floFace.DrawRightEye(reye);
    }
}