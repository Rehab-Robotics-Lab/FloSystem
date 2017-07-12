/*
  A1-16.h - Modified for XYZrobot ATmega 1280 control board.
  Copyright (c) 2015 Wei-Shun You. XYZprinting Inc.  All right reserved.
*/

#include <Arduino.h>

#ifndef A1_16_h
#define A1_16_h

#define MAX_SERVOS                         20
#define BUFFER_SIZE                        64

#define A1_16_Broadcast_ID				   254

//Requested and ACK package CMD Definition
#define CMD_EEP_WRITE                      0x01
#define CMD_ACK_EEP_WRITE                  0x41
#define CMD_EEP_READ                       0x02
#define CMD_ACK_EEP_READ                   0x42
#define CMD_RAM_WRITE                      0x03
#define CMD_ACK_RAM_WRITE                  0x43
#define CMD_RAM_READ                       0x04
#define CMD_ACK_RAM_READ                   0x44
#define CMD_I_JOG                          0x05
#define CMD_ACK_I_JOG                      0x45
#define CMD_S_JOG                          0x06
#define CMD_ACK_S_JOG                      0x46
#define CMD_STAT                           0x07
#define CMD_ACK_STAT                       0x47
#define CMD_ROLLBACK                       0x08
#define CMD_ACK_ROLLBACK                   0x48
#define CMD_REBOOT                         0x09
#define CMD_ACK_REBOOT                     0x49

//EEPROM and RAM Parameter Definition
//EEPROM
#define EEP_Model_NO                       0x00		//  R/W,1 byte
#define EEP_Year                           0x01		//  R/W,1 byte
#define EEP_Month                          0x02		//  R/W,1 byte
#define EEP_Day                            0x03		//  R/W,1 byte
#define EEP_Auto_Rollback                  0x04		//  R/W,1 byte
#define EEP_Baud_Rate	                   0x05		//  R/W,1 byte
#define EEP_sID                            0x06		//  R/W,1 byte
#define EEP_ACK_Policy                     0x07		//  R/W,1 byte
#define EEP_Alarm_LED_Policy               0x08		//  R/W,1 byte
#define EEP_Toqure_Policy                  0x09		//  R/W,1 byte
#define EEP_SPDctl_Policy                  0x0a		//  R/W,1 byte
#define EEP_Max_TEMP                       0x0b		//  R/W,1 byte
#define EEP_Min_Voltage                    0x0c		//  R/W,1 byte
#define EEP_Max_Voltage                    0x0d		//  R/W,1 byte
#define EEP_ACC_Ratio                      0x0e		//  R/W,1 byte
#define EEP_Max_Wheel_Ref_POS              0x12		//  R/W,2 byte
#define EEP_Min_PWM                        0x15		//  R/W,1 byte
#define EEP_Max_PWM                        0x16		//  R/W,2 byte
#define EEP_Overload_PWM                   0x18		//  R/W,2 byte
#define EEP_Min_POS                        0x1A		//  R/W,2 byte
#define EEP_Max_POS                        0x1C		//  R/W,2 byte
#define EEP_POS_Kp                         0x1E		//  R/W,2 byte
#define EEP_POS_Kd                         0x20		//  R/W,2 byte
#define EEP_POS_Ki                         0x22		//  R/W,2 byte
#define EEP_CtoO_Ref_POS                   0x24		//  R/W,2 byte
#define EEP_OtoC_Ref_POS                   0x26		//  R/W,2 byte
#define EEP_Max_Current                    0x28		//  R/W,2 byte
#define EEP_Ramp_Speed                     0x2a		//  R/W,2 byte
#define EEP_LED_Blink_Period               0x2c		//  R/W,1 byte
#define EEP_Package_Timeout_Period         0x2e		//  R/W,1 byte
#define EEP_Overload_Dection_Period        0x30		//  R/W,1 byte
#define EEP_Inposition_Margin              0x32		//  R/W,1 byte
#define EEP_Over_Voltage_Dection_Period    0x33		//  R/W,1 byte
#define EEP_Over_Temp_Dection_Period       0x34		//  R/W,1 byte
#define EEP_Calibration_Difference         0x35		//  R/W,1 byte
//RAM
#define RAM_sID                            0x00		//  R/W,1 byte
#define RAM_ACK_Policy                     0x01		//  R/W,1 byte
#define RAM_Alarm_LED_Policy               0x02		//  R/W,1 byte
#define RAM_Toqure_Policy                  0x03		//  R/W,1 byte
#define RAM_SPDctl_Policy                  0x04		//  R/W,1 byte
#define RAM_Max_TEMP                       0x05		//  R/W,1 byte
#define RAM_Min_Voltage                    0x06		//  R/W,1 byte
#define RAM_Max_Voltage                    0x07		//  R/W,1 byte
#define RAM_ACC_Ratio                      0x08		//  R/W,1 byte
#define RAM_Max_Wheel_Ref_POS              0x0c		//  R/W,2 byte
#define RAM_Min_PWM                        0x0f		//  R/W,1 byte
#define RAM_Max_PWM                        0x10		//  R/W,2 byte
#define RAM_Overload_PWM                   0x12		//  R/W,2 byte
#define RAM_Min_POS                        0x14		//  R/W,2 byte
#define RAM_Max_POS                        0x16		//  R/W,2 byte
#define RAM_POS_Kp                         0x18		//  R/W,2 byte
#define RAM_POS_Kd                         0x1a		//  R/W,2 byte
#define RAM_POS_Ki                         0x1c		//  R/W,2 byte
#define RAM_CtoO_Ref_POS                   0x1e		//  R/W,2 byte
#define RAM_OtoC_Ref_POS                   0x20		//  R/W,2 byte
#define RAM_Max_Current                    0x22		//  R/W,2 byte
#define RAM_Ramp_Speed                     0x24		//  R/W,2 byte
#define RAM_LED_Blink_Period               0x26		//  R/W,1 byte
#define RAM_Package_Timeout_Period         0x28		//  R/W,1 byte
#define RAM_Overload_Dection_Period        0x2a		//  R/W,1 byte
#define RAM_Inposition_Margin              0x2c		//  R/W,1 byte
#define RAM_Over_Voltage_Dection_Period    0x2d		//  R/W,1 byte
#define RAM_Over_Temp_Dection_Period       0x2e		//  R/W,1 byte
#define RAM_Calibration_Difference         0x2f		//  R/W,1 byte
#define RAM_Status_Error                   0x30		//  R/W,1 byte
#define RAM_Status_Detail                  0x31		//  R/W,1 byte
#define RAM_LED_Control                    0x35		//  R/W,1 byte
#define RAM_Voltage                        0x36		//  RO,1 byte
#define RAM_Temp                           0x37		//  RO,1 byte
#define RAM_Current_Control_Mode           0x38		//  RO,1 byte
#define RAM_Tick                           0x39		//  RO,1 byte
#define RAM_Calibrated_Position            0x3a		//  RO,2 byte
#define RAM_Joint_Position                 0x3c		//  RO,2 byte
#define RAM_PWM_Output_Duty                0x40		//  RO,2 byte
#define RAM_Bus_Current                    0x42		//  RO,2 byte
#define RAM_Position_Goal                  0x44		//  RO,2 byte
#define RAM_Position_Ref                   0x46		//  RO,2 byte
#define RAM_Omega_Goal                     0x48		//  RO,2 byte
#define RAM_Omega_Ref                      0x4a		//  RO,2 byte
#define RAM_Requested_Counts               0x4c		//  RO,2 byte
#define RAM_ACK_Counts                     0x4e		//  RO,2 byte

void A1_16_Ini(unsigned long baud);
void A1_16_SetPosition(unsigned char _pID, unsigned char _CMD,  unsigned char _playtime, unsigned int _position);
void A1_16_SetSpeed(unsigned char _pID, unsigned char _playtime, int _speed);
void A1_16_TorqueOff(unsigned char _pID);
int A1_16_ReadData(unsigned char _pID, unsigned char _CMD, unsigned char _addr_start, unsigned char _data_length);
int A1_16_ReadPacket(unsigned char _data_length);
void A1_16_WriteData(unsigned char _pID, unsigned char _CMD, unsigned char _addr_start, char _data_write);
void A1_16_WriteData2(unsigned char _pID, unsigned char _CMD, unsigned char _addr_start, int _data_write);
void A1_16_Basic(unsigned char _pID, unsigned char _CMD);


#define SetPositionS_JOG(id, time, pos) (A1_16_SetPosition(id, CMD_S_JOG,  time, pos))
#define SetPositionI_JOG(id, time, pos) (A1_16_SetPosition(id, CMD_I_JOG,  time, pos))

#define ReadDataEEP1(id, _addr) A1_16_ReadData(id, CMD_EEP_READ, _addr, 0x01)
#define ReadDataEEP2(id, _addr) A1_16_ReadData(id, CMD_EEP_READ, _addr, 0x02)
#define ReadDataRAM1(id, _addr) A1_16_ReadData(id, CMD_RAM_READ, _addr, 0x01)
#define ReadDataRAM2(id, _addr) A1_16_ReadData(id, CMD_RAM_READ, _addr, 0x02)
#define ReadPosition(id) A1_16_ReadData(id, CMD_RAM_READ, RAM_Joint_Position, 0x02)

#define WriteDataEEP1(id, _addr, _data) A1_16_WriteData(id, CMD_EEP_WRITE, _addr, _data)
#define WriteDataRAM1(id, _addr, _data) A1_16_WriteData(id, CMD_RAM_WRITE, _addr, _data)
#define WriteDataEEP2(id, _addr, _data) A1_16_WriteData2(id, CMD_EEP_WRITE, _addr, _data)
#define WriteDataRAM2(id, _addr, _data) A1_16_WriteData2(id, CMD_RAM_WRITE, _addr, _data)
#define SetID(id, newid) A1_16_WriteData(id, CMD_EEP_WRITE, EEP_sID, newid)
#define SetKpRAM(id, Kp) A1_16_WriteData2(id, CMD_RAM_WRITE, RAM_POS_Kp, Kp)
#define SetKdRAM(id, Kd) A1_16_WriteData2(id, CMD_RAM_WRITE, RAM_POS_Kd, Kd)
#define SetKiRAM(id, Ki) A1_16_WriteData2(id, CMD_RAM_WRITE, RAM_POS_Ki, Ki)
#define SetRampSpeedRAM(id, RampSpeed) A1_16_WriteData2(id, CMD_RAM_WRITE, RAM_Ramp_Speed, RampSpeed)

#define ReadStatus(id) A1_16_Basic(id, CMD_STAT)
#define RollBack(id) A1_16_Basic(id, CMD_ROLLBACK)
#define RollBackAll() A1_16_Basic(254, CMD_ROLLBACK)
#define Reboot(id) A1_16_Basic(id, CMD_REBOOT)
#define RebootAll() A1_16_Basic(254, CMD_REBOOT)

#endif
