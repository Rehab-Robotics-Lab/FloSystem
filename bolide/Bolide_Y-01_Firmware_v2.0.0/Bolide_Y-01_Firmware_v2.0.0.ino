//Include Library
#include <EEPROM.h>
#include <Wire.h>
#include "Y01_BOLIDE_Player.h"
#include "Y01_A1_16.h"
#include "BOLIDE_Y-01_Board.h"
#include "Y-01_Mask_Definition.h"
#include "Y-01_USER_MOTION.h"
#include "Scratch.h"

#define FW_VERSION  "V2.0.0"

//== Declare Global Parameters ==
BOLIDE_Player XYZrobot;
unsigned char packet[7];
static boolean cmd_torque_off = false, torque_released = false;
//Motion Editor Parameter
static boolean seq_trigger = false, seq_loop_trigger = false;
static int seq_pSeqCnt = 0xFF, SeqPos = 0x00;
static int poses[max_pose_index][MAX_SERVOS];      // poses [index][servo_id-1], check for the motion index!!
static int pose_index[max_pose_index];
sp_trans_t sequence[max_seq_index];// sequence

//========================= Set up =======================================
void setup()
{
    //Configure all basic setting
    Serial.begin(115200);
    AIM_Task_Setup(); // Sets up motor connection
    LED_Setup(); // Turns of LEDs

    Serial.println((String)"Bolide FW Version " + FW_VERSION);
    Serial.println((String)"Start Playing!!");
}
//========================= Main =======================================
void loop()
{
    USB_Task();                          // USB Communcation motion
    vExeMode_MotionEditor();            // play a sequence if it is setup
    // if(Timer_PowerLow.Timer_Task(1000)) {
    //     Power_Detection_Task();
    // }
    // #TODO: need to check power level:
    //     PWR_Voltage = analogRead(PWRDET_PIN);   // vol*0.0124
    // if (PWR_Voltage < Power_Voltage_Alarm)
}

//=========================== Function ================================

// USB Task
boolean USB_Task(void)
{
    unsigned char temp;
    if(Serial.available() < 2) return false;

    if(temp = Serial.read() == packet_header) {
            Motion_Editor_Packet_Task();
    }
    else {
        USB_Flush();
        return false;
    }
    return true;
}

//== Setup function ==
//Configure A1-16 servo motor
void AIM_Task_Setup(void) {
    XYZrobot.setup(115200, 18);
    eeprom_init();
}

//Configure eye led board pin
void LED_Setup(void) {
    // Eye LED
    pinMode(LED_BLUE_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    // Chest LED
    pinMode(LSA_LED_BLUE_PIN, OUTPUT);
    pinMode(LSA_LED_GREEN_PIN, OUTPUT);
    pinMode(LSA_LED_RED_PIN, OUTPUT);

    OCR5A = OCR5B = OCR5C = 0; // Turn off
}

// EEPROM Function
void eeprom_init(void) {
    uint8_t flag = EEPROM.read(EEP_ADDR_FLAG);
    if(flag != 0x01)
    {
        // EEPROM INIT
        Serial.println("EEP INIT");
        for(int i = 0; i < 18; i++) {
            eeprom_write_short(EEP_ADDR_ZEROPOINT + (i << 1), 0);
        }
        eeprom_write(EEP_ADDR_FLAG, 0x01);
    }
    else
    {
        short zeroPoint[18];
        for(int i = 0; i < 18; i++) {
            zeroPoint[i] = eeprom_read_short(EEP_ADDR_ZEROPOINT + (i << 1));
        }
        XYZrobot.setZeroPoint(zeroPoint);
    }
}
void eeprom_write(int addr, uint8_t data) {
    if(EEPROM.read(addr) != data) EEPROM.write(addr, data);
}
void eeprom_write_short(int addr, short data) {
    static uint8_t _temp = 0x00;
    _temp = data & 0xFF;
    eeprom_write(addr, _temp);
    _temp = (data >> 8) & 0xFF;
    eeprom_write(addr + 1, _temp);
}
short eeprom_read_short(int addr) {
    return (int16_t)(EEPROM.read(addr) + (EEPROM.read(addr + 1) << 8));
}

void robot_torque_off(void)
{
	A1_16_TorqueOff(A1_16_Broadcast_ID);
	torque_released = true;
}


void USB_Flush(void) {
    while(Serial.available() > 0) { Serial.read(); };
}
//Motion Editor Task
boolean Motion_Editor_Packet_Task(void)
{
    static int pBuffer[128] = {0};
    static unsigned char pIndex = 0, pLength = 0xFF, pCMD = 0x00;
    static unsigned char motor_ID = 0;
    static unsigned char playtime=0;
    static int position_ID = 0;
    static int seq_pPoseCnt = 0xFF, PoseCnt = 0;
    static int SeqCnt = 0;
    static int SeqProcessCnt = 0;
    static int _temp;
    boolean motion_editor_mode = false;

    _temp = packet_header;//Serial.read();
    if(_temp == packet_header) goto _packet_start;
    else {return false;}
    _packet_start:
    pBuffer[header_address] = _temp; pIndex = 1; _reset_timer4(timeout_limit);
    while((_temp = Serial.read()) == -1) {
        if(packet_timeout_status) {Packet_Error_Feedback(0x00); return false;}
    }
    pBuffer[length_address] = _temp; pLength = _temp; pIndex++; _reset_timer4(timeout_limit);
    while((_temp = Serial.read()) == -1) {
        if(packet_timeout_status) {Packet_Error_Feedback(0x00); return false;}
    }
    pBuffer[CMD_address] = _temp; pCMD = _temp; pIndex++; _reset_timer4(timeout_limit);
    while(1) {
        while((_temp = Serial.read()) == -1) {
            if(packet_timeout_status) {Packet_Error_Feedback(0x00); return false;}
        }
        pBuffer[pIndex] = _temp; pIndex++; _reset_timer4(timeout_limit);
        if(pIndex == pLength) {_reset_timer4(timeout_limit); goto _packet_finish;}
    }
    _packet_finish:
    motion_editor_mode = true;
    if(pBuffer[pIndex-1] == packet_tail)
    {
        //initial motion editor setting, must tell the robot how many motors there are.
        // send packet: [header, size, command (0x01), num motors, tail]
        if(pCMD == CMD_init_motor) {
            XYZrobot.poseSize = pBuffer[motor_num_address];
            XYZrobot.readPose();
            Packet_Init(pBuffer[motor_num_address]);
        }
        else if(pCMD == CMD_set_motor) {        //set motor position
            motor_ID = pBuffer[motor_ID_address];
            position_ID = (pBuffer[motor_pos_msb] << 8) + pBuffer[motor_pos_lsb];
            // playtime = pBuffer[0x06] // time in units of 10ms
            // SetPositionI_JOG(motor_ID, playtime, position_ID);
            SetPositionI_JOG(motor_ID, 0, position_ID);
            Packet_Set(motor_ID, position_ID);
        }
        else if(pCMD == CMD_capture_pos) {    //get motor position
            Packet_Pos_Read();
        }
        else if(pCMD == CMD_relax_motor) {      //relax motor
            motor_ID = pBuffer[motor_ID_address];
            A1_16_TorqueOff(motor_ID);
            Packet_Relax(motor_ID);
        }
        else if(pCMD == CMD_SN_read) {          // serial number read
            Packet_SN();
        }
        else if(pCMD == CMD_SEQ_relax) {                //relax servo
            robot_torque_off(); //A1_16_TorqueOff(A1_16_Broadcast_ID);
        }
        else if(pCMD == CMD_version_read) {
            Packet_Version_Read();
        }
        else if(pCMD == CMD_capture_current){
            Packet_Current_Read();
        }
        else if(pCMD == CMD_capture_torque){
            Packet_Torque_Read();
        }
        else if(pCMD == CMD_capture_battery){
            Packet_Battery_Read();
        }
        /// Work with loading then playing a sequence of poses:
        //load total pose number, how many unique poses are in the sequence which you will play?
        // Send packet: [header, size, command (0x10), number of sequences, tail]
        else if(pCMD == CMD_SEQ_load_PoseCnt) {
            seq_trigger = false; SeqPos = 0;
            seq_pPoseCnt = pBuffer[seq_pose_cnt_address];
            PoseCnt = 0;
            if(seq_pPoseCnt > max_pose_index) Packet_Error_Feedback(0x00);
            else {Packet_Error_Feedback(0x01); SeqProcessCnt = SEQ_Process_load_PoseCnt; seq_loop_trigger = false;}
        }
        //load pose in sequence. you should send a command here for eachpose in the sequence
        // Send packet: [header, size, command (0x11), poseID, msb of motor0, lsb of motor0, msb of motor1, lsb of motor1, ... , msb of motorn, lsb of motorn, tail]
        else if(pCMD == CMD_SEQ_load_Pose) {
            static int PoseID = 0, _i = 0;
            seq_trigger = false; SeqPos = 0;
            if(SeqProcessCnt == SEQ_Process_load_PoseCnt) {
                PoseID = pBuffer[seq_pose_ID_address];
                for(_i = 0; _i < XYZrobot.poseSize; _i++) {
                    poses[PoseCnt][_i] = (pBuffer[seq_pose_start_address + 2*_i] << 8) + pBuffer[seq_pose_start_address + 1 + 2*_i];
                    pose_index[PoseCnt] = PoseID;
                }
                PoseCnt++;
                if(PoseCnt == seq_pPoseCnt){Packet_Error_Feedback(0x02); PoseCnt = 0; SeqProcessCnt = SEQ_Process_load_Pose;}
                else Packet_Error_Feedback(0x01);
            }
            else Packet_Error_Feedback(0x00);
        }
        // how many steps are in the sequence?
        // send packet: [head, size, command (0x12), number of steps, tail]
        else if(pCMD == CMD_SEQ_load_SEQCnt) {          //load total sequence number
            seq_trigger = false; SeqPos = 0;
            if(SeqProcessCnt == SEQ_Process_load_Pose) {
                seq_pSeqCnt = pBuffer[seq_seq_cnt_address];
                if(seq_pSeqCnt > max_seq_index) Packet_Error_Feedback(0x00);
                else {Packet_Error_Feedback(0x01); SeqProcessCnt = SEQ_Process_load_SEQCnt;}
            }
            else Packet_Error_Feedback(0x00);
        }
        // Tell the order of poses and time for each. Once all sequence steps are sent, the sequence begins
        // send packet: [header, size, command (0x13), poseID (loaded above), transition time msb, transition time lsb, tail]
        else if(pCMD == CMD_SEQ_load_SEQ) {             //load sequence
            static int sPoseID = 0, sTime = 0, _i;
            seq_trigger = false; SeqPos = 0;
            if(SeqProcessCnt == SEQ_Process_load_SEQCnt) {
                sPoseID = pBuffer[seq_pose_name_address];
                sTime = (pBuffer[seq_pose_time_MSB_address] << 8) + pBuffer[seq_pose_time_LSB_address];
                for(_i = 0;_i < max_pose_index;_i++){
                    if(pose_index[_i] == sPoseID) {
                        sequence[SeqCnt].pose = _i; sequence[SeqCnt].time = sTime;
                        SeqCnt++; break;
                    }
                }
                if(SeqCnt == seq_pSeqCnt) {
                    Packet_Error_Feedback(0x02);
                    SeqCnt = 0; seq_trigger = true;
                    XYZrobot.readPose();
                    SeqProcessCnt = SEQ_Process_load_SEQ;
                }
                else Packet_Error_Feedback(0x01);
            }
            else Packet_Error_Feedback(0x00);
        }
        else if(pCMD == CMD_SEQ_halt) {                 //halt sequence
            seq_trigger = false;
            Packet_Error_Feedback(0x03);
            //halt sequence
        }
    }
    else if(pCMD == CMD_SEQ_relax) {                //relax servo
        seq_trigger = false;
        robot_torque_off(); //A1_16_TorqueOff(A1_16_Broadcast_ID);
        Packet_Error_Feedback(0x03);
    }
    else{Packet_Error_Feedback(0x00); pLength = 0xFF;}
    return motion_editor_mode;
}

void Packet_Init(unsigned char motor_num) {
    Serial.write(packet_header);
    Serial.write(0x05);
    Serial.write(CMD_init_motor);
    Serial.write(motor_num);
    Serial.write(packet_tail);
}
void Packet_Set(unsigned char motor_ID, int pos_set) {
    Serial.write(packet_header);
    Serial.write(0x07);
    Serial.write(CMD_set_motor);
    Serial.write(motor_ID);
    Serial.write(((pos_set & 0xFF00) >> 8));
    Serial.write((pos_set & 0x00FF));
    Serial.write(packet_tail);
}
void Packet_Pos_Read(void) {
    static int position_buffer[19] = {0};      // position buffer
    static int _i = 0;
    for(_i = 1;_i < 19;_i++) position_buffer[_i] = ReadPosition(_i);
    Serial.write(packet_header);
    Serial.write(0x28);
    Serial.write(CMD_capture_pos);
    for(_i = 1;_i < 19;_i++) {
        Serial.write(((position_buffer[_i] & 0xFF00) >> 8));
        Serial.write((position_buffer[_i] & 0x00FF));
    }
    Serial.write(packet_tail);
}
void Packet_Current_Read(void){
    static int current_buffer[19] = {0};      // velocity buffer
    static int _i = 0;
    for(_i = 1;_i < 19;_i++) current_buffer[_i] = ReadCurrent(_i);
    Serial.write(packet_header);
    Serial.write(0x28);
    Serial.write(CMD_capture_current);
    for(_i = 1;_i < 19;_i++) {
        Serial.write(((current_buffer[_i] & 0xFF00) >> 8));
        Serial.write((current_buffer[_i] & 0x00FF));
    }
    Serial.write(packet_tail);
}
void Packet_Torque_Read(void){
    static int current_buffer[19] = {0};      // velocity buffer
    static int _i = 0;
    for(_i = 1;_i < 19;_i++) current_buffer[_i] = ReadTorque(_i);
    Serial.write(packet_header);
    Serial.write(0x28);
    Serial.write(CMD_capture_torque);
    for(_i = 1;_i < 19;_i++) {
        Serial.write(((current_buffer[_i] & 0xFF00) >> 8));
        Serial.write((current_buffer[_i] & 0x00FF));
    }
    Serial.write(packet_tail);
}
void Packet_Battery_Read(void){
    int volt = analogRead(PWRDET_PIN);
    Serial.write(packet_header);
    Serial.write(0x06);
    Serial.write(CMD_capture_battery);
    Serial.write(((volt & 0xFF00) >> 8));
    Serial.write(((volt & 0x00FF)));
    Serial.write(packet_tail);
}
void Packet_Relax(unsigned char motor_ID) {
    Serial.write(packet_header);
    Serial.write(0x05);
    Serial.write(CMD_relax_motor);
    Serial.write(motor_ID);
    Serial.write(packet_tail);
}
void Packet_SN(void) {
    static int _i = 0;
    Serial.write(packet_header);
    Serial.write(0x12);
    Serial.write(CMD_SN_read);
    for(_i = 10;_i < 24;_i++) Serial.write(EEPROM.read(_i));
    Serial.write(packet_tail);
}
void Packet_Version_Read(void) {
    Serial.write(packet_header);
    Serial.write(0x0A);
    Serial.write(CMD_version_read);
    Serial.write(model_Bolide);                 //Model
    Serial.write(type_Y01);                     //Type
    Serial.write(application_default);          //Application
    Serial.write(main_version_number);          //Main Version
    Serial.write(secondary_version_number);     //Secondary Version
    Serial.write(revision_number);              //Revision
    Serial.write(packet_tail);
}
void Packet_Error_Feedback(unsigned char CMD_reaction) {
    Serial.write(packet_header);
    Serial.write(0x04);
    Serial.write(CMD_reaction);
    Serial.write(packet_tail);
}
void Seq_Pos_Feedback(unsigned char Seq){
    Serial.write(packet_header);
    Serial.write(seq_number_comp);
    Serial.write(seq_number_comp);
    Serial.write(Seq);
    Serial.write(packet_tail);
}


// SEQUENCE PLAYING CODE:
void vExeMode_MotionEditor(void)
{
    if(seq_trigger) {                    // play sequence edited by motion editor
        Motion_Editor_Seq_Play();
    }
}

void Motion_Editor_Seq_Play(void) {
    static int _i = 0;
    static int pose_index = 0;
    pose_index = sequence[SeqPos].pose;
    for(_i = 0; _i < XYZrobot.poseSize; _i++) XYZrobot.setNextPose(_i+1, poses[pose_index][_i]);
    XYZrobot.interpolateSetup(sequence[SeqPos].time);
    while(XYZrobot.interpolating) XYZrobot.interpolateStep();
    Seq_Pos_Feedback(SeqPos);
    SeqPos++;
    if(SeqPos == seq_pSeqCnt) {
        SeqPos = 0;
        if(seq_loop_trigger);
        else{seq_trigger = false; seq_pSeqCnt = 0xFF;}
    }
}
