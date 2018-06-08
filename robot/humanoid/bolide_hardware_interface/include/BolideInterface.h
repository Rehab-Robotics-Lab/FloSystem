#ifndef BOLIDE_INTEFACE_H
#define BOLIDE_INTERFACE_H


#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>

#include "serial/serial.h"

#define CMD_set_motor              0x02
#define CMD_capture_pos            0x03
#define CMD_relax_motor            0x04
#define CMD_SN_read		           0x05
#define CMD_capture_current        0x06
#define CMD_capture_torque         0x07
#define CMD_capture_battery        0x08

#define NUM_SERVOS 18

class BolideInterface{
    public:
        BolideInterface(std::string port);
        ~BolideInterface(void);

        // Positions are set and read as 0-2pi for a full rotation (internally 0-1023)
        float * ReadTorque(void);
        float * ReadPosition(void);
        float ReadBattery(void);

        void SetPosition(int motorID, float position);
        void TorqueOff(void);
        void TorqueOff(int motorID);

    private:
        void ReadData(int command);
        
        float set_positions_ [NUM_SERVOS]; ///< The positions the joints are set to
        float read_torque_ [NUM_SERVOS]; ///< The last read torque values
        float read_positions_ [NUM_SERVOS]; ///< The last read postions values
        float read_battery_; ///< The last read battery value
        serial::Serial *serial_con_; ///< The serial connection to the bolide
};

#endif