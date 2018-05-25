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
Constant_Timer Timer_Serial, Timer_PowerLow, Timer_Idle, Timer_Calibrate;
BOLIDE_Player XYZrobot;
static float ax, ay, az;
static int I2C_Address = 0x3B >> 1; 
static int distance;
static int SN_packet[9] = {0}, SN_packet_index = 0, inByte = 0;
unsigned char packet[7];
static int LED_mode;
static int g_packet[8], ir_packet[4], ir_msb, ir_lsb, ir_rowdata;
static boolean cmd_torque_off = false, torque_released = false, BT_update = false, Scratch_update = false;
//Motion Editor Parameter
static boolean seq_trigger = false, seq_loop_trigger = false;
static int seq_pSeqCnt = 0xFF, SeqPos = 0x00;
static int poses[max_pose_index][MAX_SERVOS];      // poses [index][servo_id-1], check for the motion index!!
static int pose_index[max_pose_index];
sp_trans_t sequence[max_seq_index];// sequence
Joystick mJoystick;
boolean bScratchMode = false;
unsigned char last_walking = 0; // 0: not walking, 1: left foot, 2: right foort
unsigned char exe_mode = 0;
boolean bInAutoMode = false;

char eyeled_delay_cnt = 0;
void set_eye_led(char eyemode, char _delay_cnt = 0);

#define EXE_MODE_RCU            0
#define EXE_MODE_MOTION_EDITOR  1
#define EXE_MODE_SCRATCH        2
#define EXE_MODE_AUTOMODE       3

#define MODE_NUMBER             4

//========================= Set up =======================================
void setup() 
{
    //Configure all basic setting
    Serial.begin(115200);
    AIM_Task_Setup();
    Speaker_Task_Setup();
    LED_Setup();
    Buzzer_Setup();
    Button_Setup();
    Analog_Input_Setup();
    Timer_Task_Setup();

    //Start motion
    G_SENSOR_Task_Setup();
    Initial_Pose_Setup();
    AutoMode_Setup();

    Serial.println((String)"Bolide FW Version " + FW_VERSION);
    Serial.println((String)"Start Playing!!");
}
//========================= Main =======================================
void loop()
{ 
    USB_Task();                          // USB Communcation motion
    BT_Task();                           // BT Communcation motion

    // Execution Mode
    switch(exe_mode)
    {
        case EXE_MODE_RCU:
            vExeMode_RCUTask();
            break;
        case EXE_MODE_MOTION_EDITOR:
            vExeMode_MotionEditor();
            break;
        case EXE_MODE_SCRATCH:
            vExeMode_Scratch();
            break;
        case EXE_MODE_AUTOMODE:
            vExeMode_AutoMode();
            break;	    
        default:
            // Execution Mode Error
            exe_mode = EXE_MODE_RCU;
            break;
    }

    if(Timer_PowerLow.Timer_Task(1000)) {
        Power_Detection_Task();
    }
}

//=========================== Function ================================
// BT Task
void BT_Task(void)
{
    if(Serial2.available() > 0)
    {
        set_eye_led(EYE_MODE_BLUE, 20);
        Timer_Serial.SetTimer(100);
        if(Serial2.peek() == 0x00) {         // RCU & APP
            while(Serial2.available() < 7) { if(Timer_Serial.TimerEvent()) {BT_Flush(); return;} }
            BT_Packet_Task();
            if(BT_update == true) {
                if((exe_mode != EXE_MODE_RCU) && (exe_mode != EXE_MODE_AUTOMODE)) exe_mode = EXE_MODE_RCU;
            }
        }
        else if((Serial2.peek() == 0xFF)) {  // BT SCRATCH
            if(Scratch_Packet_Task(packet, true)) {
                exe_mode = EXE_MODE_SCRATCH;
                Scratch_update = true;
            }
        }    
        BT_Flush();
    }
}

// USB Task
boolean USB_Task(void)
{
    unsigned char temp;   
    if(Serial.available() < 2) return false;
    
    if(temp = Serial.read() == packet_header) 
    {
        if(Serial.peek() == 0x55) {      // USB Scratch
            if(Scratch_Packet_Task(packet, false)) {
                exe_mode = EXE_MODE_SCRATCH;
                Scratch_update = true;
            }
        }
        else {    // Motion Editor Packet
            if(Motion_Editor_Packet_Task()) exe_mode = EXE_MODE_MOTION_EDITOR;
        }
    }
    else {
        USB_Flush();
        return false;
    }
    return true;
}

// ===================== Execution Mode ======================= //
void vExeMode_RCUTask(void)
{
    if(BUTTON_Task() == 0) set_led_mode(BLE_LED_BREATH);
    
    if(BT_update) {                      // BT Command
        RCU_CMD_Task();
    }
    else {                               // Execution Mode
        RCU_Task();
    }
}
void vExeMode_AutoMode(void)
{
    set_led_mode(RED_LED_BREATH);
    set_eye_led(EYE_MODE_GREEN);

    BUTTON_Task();
    if(BT_update) {                      // BT Command
        if(RCU_CMD_Task() == true) {
            exe_mode = EXE_MODE_RCU;
        }
    }
    Idle_Task();
}
void vExeMode_MotionEditor(void)
{
    if(seq_trigger) {                    // play sequence edited by motion editor
        Motion_Editor_Seq_Play();
    }  
}
void vExeMode_Scratch(void)
{
    if(Scratch_update) {
        set_led_mode(LED_SCRATCH);
        SCRATCH_CMD_Task(packet);
        Scratch_update = false;
    }
}

boolean RCU_CMD_Task(void)
{
    boolean _isBtCmd = false;
    //==== RCU Command ====
    if(packet[1]!=255 & packet[2]!=1) 
    {
        boolean bButtonCmd = true;
        mJoystick.RCU_LJX = mJoystick.RCU_LJY = mJoystick.RCU_RJX = mJoystick.RCU_RJY = 0;

        set_led_mode(ONE_LED_ON);
        if(packet[5] & RCU_mask_release) {           // Release Button
            robot_torque_off();
        }        
        else if(packet[5] & RCU_mask_BT) {           // Bluetooth Button
        }
        else if(packet[5] & RCU_mask_power) {        // Power Button
            Action(RCU_HOME, Adjustment_index);
        }        
        else if(packet[6] & RCU_mask_L1) {           // L1 Button
            Action(RCU_L1, Adjustment_index);
        }        
        else if(packet[6] & RCU_mask_L2) {           // L2 Button
            Action(RCU_L2, Adjustment_index);
        }
        else if(packet[6] & RCU_mask_L3) {           // L3 Button
            Action(RCU_L3, Adjustment_index);
        }        
        else if(packet[5] & RCU_mask_R1) {           // R1 Button
            Action(RCU_R1, Adjustment_index);
        }
        else if(packet[5] & RCU_mask_R2) {           // R2 Button
            Action(RCU_R2, Adjustment_index);
        }        
        else if(packet[5] & RCU_mask_R3) {           // R3 Button
            Action(RCU_R3, Adjustment_index);
        }
        else {    // Joystick
            bButtonCmd = false;
            mJoystick.RCU_LJX = packet[1]-128;
            mJoystick.RCU_LJY = packet[2]-128;
            mJoystick.RCU_RJX = packet[3]-128;
            mJoystick.RCU_RJY = packet[4]-128;
        }
        set_led_mode(BLE_LED_BREATH);

        if ((bButtonCmd == true) || (last_walking != 0) || (abs(mJoystick.RCU_LJX) > 30 || abs(mJoystick.RCU_LJY) > 30 || abs(mJoystick.RCU_RJX) > 30 || abs(mJoystick.RCU_RJY) > 30)) {
            Timer_Idle.Reset();
            _isBtCmd = true;
        }
            
        BT_update = false;
    }
    //==== App Command ==== 
    else if((packet[1] == 255) && (packet[2] == 1)) 
    {
    	if (packet[3] == 101) Action(RCU_HOME, Adjustment_index);
    	else if (packet[3] == 102) robot_torque_off(); //A1_16_TorqueOff(254);
        else if(packet[3] == 251) BT_Gsensor_Data();
        else if(packet[3] == 252) BT_IR_Data();
        else if(packet[3] == 253) BT_FW();
        else 
        {
            if((Adjustment_index == true) && (Falling_Task() != 5)) {
                Getup_Task(Falling_Task());
            }
            else {            
                Serial.println(packet[3]);
                if (packet[3] == 1)  { if (ir_check_distance() == true) { Action(A_WalkForward); Action(A_Standup); } }
                else if (packet[3] == 2) { Action(A_WalkBackward); Action(A_Standup); }
                else if(packet[3] == 3) { Action(A_WalkLeftward); }
                else if(packet[3] == 4) { Action(A_WalkRightward); }
                else if (packet[3] == 5) { Action(A_TurnLeft); Action(A_Standup); }
                else if (packet[3] == 6) { Action(A_TurnRight); Action(A_Standup); }
                else  Action(packet[3]); 
            }
        }   
        BT_update = false;
        _isBtCmd = true;
    }
    return _isBtCmd;
}

void RCU_Task(void)
{
    // App Task
    if (packet[1] == 255 & packet[2] == 1) { }
    // Robot Button Task 
    else if((last_walking == 0) && (abs(mJoystick.RCU_LJX) < 30) && (abs(mJoystick.RCU_LJY) < 30) && (abs(mJoystick.RCU_RJX) < 30) && (abs(mJoystick.RCU_RJY) < 30)) {
        // Do Nothing
    }
    // RCU Task
    else                                     
    {   
        set_led_mode(ONE_LED_ON);        
        if((mJoystick.RCU_LJX > 30) && (abs(mJoystick.RCU_LJX) > abs(mJoystick.RCU_LJY))) {            // Left Joystick Rightside
            Action(RCU_LJR, Adjustment_index);
        }
        else if((mJoystick.RCU_LJX < -30) && (abs(mJoystick.RCU_LJX) > abs(mJoystick.RCU_LJY))) {      // Left Joystick Leftside
            Action(RCU_LJL, Adjustment_index);
        }
        else if(mJoystick.RCU_LJY > 30) {       // Left Joystick Upside
            if(ir_check_distance() == true) { 
                if(mJoystick.RCU_RJX > 30) Action(RCU_WFTR, Adjustment_index);         // WalkForward & Turn Right
                else if(mJoystick.RCU_RJX < -30) Action(RCU_WFTL, Adjustment_index);   // WalkForward & Turn Left
                else Action(RCU_LJU, Adjustment_index); 
            }
        }
        else if(mJoystick.RCU_LJY < -30) {      // Left Joystick Downside
            if(mJoystick.RCU_RJX > 30) Action(RCU_WBTR, Adjustment_index);             // WalkBackward & Turn Right
            else if(mJoystick.RCU_RJX < -30) Action(RCU_WBTL, Adjustment_index);       // WalkBackward & Turn Left
            else Action(RCU_LJD, Adjustment_index);
        }
        else if((mJoystick.RCU_RJX > 30) && (abs(mJoystick.RCU_RJX) > abs(mJoystick.RCU_RJY))) {       // Right Joystick Rightside
            Action(RCU_RJR, Adjustment_index);
        }
        else if((mJoystick.RCU_RJX < -30) && (abs(mJoystick.RCU_RJX) > abs(mJoystick.RCU_RJY))) {      // Right Joystick Leftside
            Action(RCU_RJL, Adjustment_index);
        }
        else if((mJoystick.RCU_RJY > 30) && (abs(mJoystick.RCU_RJY) > abs(mJoystick.RCU_RJX))) {       // Right Joystick Upside
            Action(RCU_RJU, Adjustment_index);
        }
        else if((mJoystick.RCU_RJY < -30) && (abs(mJoystick.RCU_RJY) > abs(mJoystick.RCU_RJX))) {      // Right Joystick Downside
            Action(RCU_RJD, Adjustment_index);
        }
        else {
			if ((last_walking != 0) && (torque_released == false)) Action(A_Standup);
            last_walking = 0;
        }
        set_led_mode(BLE_LED_BREATH);
    }
}

void Idle_Task(void)
{
	if (Timer_Idle.Timer_Task(5000))
	{
		if ((Falling_Task() == 5) && (XYZrobot.playing == false) && (torque_released == false))
		{
            if(exe_mode == EXE_MODE_AUTOMODE)
            {
                switch (random(20)) {
                case 0:  Action(A_Confuse);     break;
                case 1:  Action(A_Relax);       break;
                case 2:  Action(A_Stretch);     break;
                case 3:  Action(A_Muscle);      break;
                case 4:  Action(A_Fart);        break;
                case 5:  Action(A_HeadSquat);   break;
                case 6:  
                    XYZ_Play_Action(D_InitWalkForward); XYZ_Play_Action(D_WalkForward); XYZ_Play_Action(D_Standup); delay(600); 
                    Action(A_LookAround);
                    break;
                case 7:
                    Action(A_LookAround); delay(200); Action(A_Point);
                    XYZ_Play_Action(D_InitWalkForward); Action(A_WalkForward); Action(A_Standup);
                    break;
                default:    break;
                }   
            }
		}
		Timer_Idle.Reset();
	}	
}

//== Setup function ==
//Configure A1-16 servo motor
void AIM_Task_Setup(void) {
    XYZrobot.setup(115200, 18);
    eeprom_init();
}

//Configure speaker Board
void Speaker_Task_Setup(void) {
    Serial3.begin(115200);
    MusicPlaying_wav_volume(0x80);	// Set Music Volume
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
    
    set_led_mode(LED_OFF);
}

//Configure onboard buzzer pin
void Buzzer_Setup(void) {
    pinMode(BUZZER_PIN, OUTPUT);
}

//Configure onboard button pin
void Button_Setup(void) {
    pinMode(BUTTON1_PIN, INPUT);
    pinMode(BUTTON2_PIN, INPUT);
    pinMode(BUTTON3_PIN, INPUT);
    pinMode(BUTTON4_PIN, INPUT);
}

//Configure Analog input pin
void Analog_Input_Setup(void) {
    pinMode(PWRDET_PIN, INPUT);
    pinMode(DISTANCE_SENSOR_PIN, INPUT);
    analogReference(EXTERNAL); 
}

void AutoMode_Setup(void) {
    randomSeed(analogRead(2));
    Timer_Idle.Reset();
}

// Timer Setup
void Timer_Task_Setup(void) {
    //Set Timer3 as a normal timer for LED task
    TCCR3A = 0x00;
    TCCR3B |= _BV(CS32); TCCR3B &= ~_BV(CS31); TCCR3B |= _BV(CS30);
    _enable_timer3();
    _reset_timer3(timer_10ms);

    //Set Timer4 as a normal timer for communcation timeout
    TCCR4A = 0x00;
    TCCR4B |= _BV(CS42); TCCR4B &= ~_BV(CS41); TCCR4B |= _BV(CS40);
    _enable_timer4();
    _reset_timer4(timeout_limit); 
    //Set Timer5 as a Fast PWM generator for chest LED driver
    TCCR5A = _BV(COM5A1) | _BV(COM5B1) | _BV(COM5C1) | _BV(WGM51) | _BV(WGM50);  
    TCCR5B = _BV(WGM52) | _BV(CS52);  
    OCR5A = 0; OCR5B = 0; OCR5C = 0;
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

//G-Sensor Setup
void G_SENSOR_Task_Setup(void) {
    Wire.begin();
    setReg(0x2D, 0xA);
}
void G_Sensor_Task(void)
{
    ax = ((g_sensor_read_reg(0x33) << 8)  + g_sensor_read_reg(0x32)) / 256.0;
    ay = ((g_sensor_read_reg(0x35) << 8)  + g_sensor_read_reg(0x34)) / 256.0;
    az = ((g_sensor_read_reg(0x37) << 8)  + g_sensor_read_reg(0x36)) / 256.0;
}
void setReg(int reg, int data) {
    Wire.beginTransmission(I2C_Address);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}
int g_sensor_read_reg(int reg) {
    static int Gsensor_timer;
    Gsensor_timer = 0;
    Wire.beginTransmission(I2C_Address);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(I2C_Address,1);
    if(Wire.available() <= 1 ) return Wire.read();
}

//Falling Detection Task
int Falling_Task(void) {
    int posture_index;
    G_Sensor_Task();
  
    if ((az) < -0.75) posture_index=1; //Frontside Getup
    else if ((az) > 0.75) posture_index=2; // Backside Getup    
    else if ((ax) < -0.75) posture_index=3; // Rightside Getup  
    else if ((ax) > 0.75) posture_index=4; // Leftside Getup
    else if ((az) <= 0.75 && (az) >= -0.75) posture_index=5;// Stand Status
  
    return posture_index;
}
void robot_torque_off(void)
{
	A1_16_TorqueOff(A1_16_Broadcast_ID);
	torque_released = true;
}

//Getup Detection Task
void Getup_Task(int posture_index) 
{
    if (posture_index == 1)     // Fall Forward
    {	
        switch(random(3)){
        case 0:
            Action(A_FrontFall);
            robot_torque_off();
            break;
        case 1:
            Action(A_FrontGetUp);   delay(400);
            Action(A_FrontupHead);
            break;
        default:
            Action(A_FrontGetUp);
            break;
        }	
    }
    else if (posture_index == 2)    // Fall Backward
    {
        switch(random(5))
        {
        case 0:
            Action(A_BackGetUp);    delay(400);
    		Action(A_Bottom);
            break;
        case 1:
            Action(A_Situp);
            break;
        case 2:
            Action(A_BackupFart);
            break;
        default:
            Action(A_BackGetUp);
        }
    }
    else if(posture_index == 3) Action(A_SideGetUp);
    else if(posture_index == 4) Action(A_SideGetUp); 
}

//IR Task
int IR_SENSOR_Task(void) {
    distance = (6787/(analogRead(DISTANCE_SENSOR_PIN)-3))-4;
    return distance;
}
boolean ir_check_distance(void)
{
    if(Avoidance_index == true) 
    {
        if((Falling_Task() == 5) && (IR_SENSOR_Task() < 35)) 
        {
            Action(A_Standup);
            for(int i = 0; i < 3; i++) {
                tone(BUZZER_PIN, pgm_read_word_near(&obstacle_alarm_frq[i]));
                delay(250);
                noTone(BUZZER_PIN);
            }
            switch (random(2)) {
            case 0:
                Action(A_HoldHead);
                break;
            case 1:
                Action(A_Block);
                break;
            }			
            return false;
        }
    } 
    else {
        return true;
    }
    return true;
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
        if(pCMD == CMD_init_motor) {        //initial motion editor setting
            seq_trigger = false; SeqPos = 0;
            XYZrobot.poseSize = pBuffer[motor_num_address];
            XYZrobot.readPose();
            Packet_Init(pBuffer[motor_num_address]);
        }
        else if(pCMD == CMD_set_motor) {        //set motor position
            seq_trigger = false; SeqPos = 0;
            motor_ID = pBuffer[motor_ID_address];
            position_ID = (pBuffer[motor_pos_msb] << 8) + pBuffer[motor_pos_lsb];
            SetPositionI_JOG(motor_ID, 0x00, position_ID);
            Packet_Set(motor_ID, position_ID);
        }
        else if(pCMD == CMD_capture_motor) {    //get motor position
            seq_trigger = false; SeqPos = 0;
            motor_ID = pBuffer[motor_ID_address];
            Packet_Capture(motor_ID);
        }
        else if(pCMD == CMD_relax_motor) {      //relax motor
            seq_trigger = false; SeqPos = 0;
            motor_ID = pBuffer[motor_ID_address];
            A1_16_TorqueOff(motor_ID);
            Packet_Relax(motor_ID);
        }
        else if(pCMD == CMD_SN_read) {          // serial number read
            seq_trigger = false; SeqPos = 0;
            Packet_SN();
        }
        else if(pCMD == CMD_SEQ_load_PoseCnt) { //load total pose number
            seq_trigger = false; SeqPos = 0;
            seq_pPoseCnt = pBuffer[seq_pose_cnt_address];
            if(seq_pPoseCnt > max_pose_index) Packet_Error_Feedback(0x00);
            else {Packet_Error_Feedback(0x01); SeqProcessCnt = SEQ_Process_load_PoseCnt; seq_loop_trigger = false;}
        }
        else if(pCMD == CMD_SEQ_loop_load_PoseCnt) {    //load loop sequence pose number
            seq_trigger = false; SeqPos = 0;
            seq_pPoseCnt = pBuffer[seq_pose_cnt_address];
            if(seq_pPoseCnt > max_pose_index) Packet_Error_Feedback(0x00);
            else {Packet_Error_Feedback(0x01); SeqProcessCnt = SEQ_Process_load_PoseCnt; seq_loop_trigger = true;}
        }
        else if(pCMD == CMD_SEQ_load_Pose) {            //load pose in sequence
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
        else if(pCMD == CMD_SEQ_load_SEQCnt) {          //load total sequence number
            seq_trigger = false; SeqPos = 0;
            if(SeqProcessCnt == SEQ_Process_load_Pose) {
                seq_pSeqCnt = pBuffer[seq_seq_cnt_address];
                if(seq_pSeqCnt > max_seq_index) Packet_Error_Feedback(0x00);
                else {Packet_Error_Feedback(0x01); SeqProcessCnt = SEQ_Process_load_SEQCnt;}
            }
            else Packet_Error_Feedback(0x00);
        }
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
            //halt sequence
        }
        else if(pCMD == CMD_SEQ_relax) {                //relax servo
            seq_trigger = false;
            robot_torque_off(); //A1_16_TorqueOff(A1_16_Broadcast_ID);
        }
        else if(pCMD == CMD_version_read) {
            seq_trigger = false; SeqPos = 0;
            Packet_Version_Read();
        }
    }
    else{Packet_Error_Feedback(0x00); pLength = 0xFF;}
    return motion_editor_mode;
}

void Motion_Editor_Seq_Play(void) {
    static int _i = 0;
    static int pose_index = 0;
    pose_index = sequence[SeqPos].pose;
    for(_i = 0; _i < XYZrobot.poseSize; _i++) XYZrobot.setNextPose(_i+1, poses[pose_index][_i]);
    XYZrobot.interpolateSetup(sequence[SeqPos].time);
    while(XYZrobot.interpolating) XYZrobot.interpolateStep();
    SeqPos++;
    if(SeqPos == seq_pSeqCnt) {
        SeqPos = 0;
        if(seq_loop_trigger);
        else{seq_trigger = false; seq_pSeqCnt = 0xFF;}
    }
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
void Packet_Capture(unsigned char motor_ID) {
    static int position_buffer[19] = {0};      // position buffer
    static int _i = 0;
    for(_i = 1;_i < 19;_i++) position_buffer[_i] = ReadPosition(_i);
    Serial.write(packet_header);
    Serial.write(0x29);
    Serial.write(CMD_capture_motor);
    Serial.write(motor_ID);
    for(_i = 1;_i < 19;_i++) {
        Serial.write(((position_buffer[_i] & 0xFF00) >> 8));
        Serial.write((position_buffer[_i] & 0x00FF));
    }
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

void Init_AutoMode(void) 
{
    set_led_mode(RED_LED_BREATH);
    XYZrobot.readPose();
    if(Falling_Task() == 5) 
    {
        char ran_pos = random(5);
        switch(ran_pos) {
            case 0: 
                Action(A_Relax); break;
            case 1: 
                Action(A_Stretch); break;
            case 2: 
                Action(A_Muscle); break;
            case 3: 
                Action(A_FlyFart, Adjustment_index); break;
            case 4: 
                Action(A_Shake);
                Action(A_DefaultInitial, Adjustment_index);
                break;
        }
    }
}

//Initial Pose Task
void Initial_Pose_Setup(void) 
{
    XYZrobot.readPose();
    if(MCUSR & 0x01 != 0) {     // if Robot Power-On Reset
        Action(A_PowerOn);
    }
    else {                      // if Robot External Reset
        Action(A_Shake);
        Action(A_DefaultInitial);
    }    
    MCUSR = 0;          // Clear Reset Flag  
}
void XYZ_Update_ZeroPoint(void)
{
    // Read Pose
    unsigned int pose[18];
    for(int i = 0; i < 18; i++) {
        if(i < 6) { // don't modify hand pose
            pose[i] = pgm_read_word_near(DefaultPose1 + i+1);
        }
        else {
            pose[i] = ReadPosition(i+1);
        }
    }
    // Set Robot's Zero Point
    short zeroPoint[18];
    for(int i = 0; i < 18; i++) {
        zeroPoint[i] = (pose[i] - pgm_read_word_near(DefaultPose1 + i+1));
        for(int i = 0; i < 18; i++) {
            eeprom_write_short(EEP_ADDR_ZEROPOINT + (i << 1), zeroPoint[i]);
        }
    }
    XYZrobot.setZeroPoint(zeroPoint);    
}
void XYZ_Play_Action(const transition_t *addr)
{
    XYZrobot.playSeq(addr);    
    while(XYZrobot.playing) {      
        if(Serial2.available() > 0) {
            BT_Packet_Task();
            if(check_torque_release(packet)) { BT_Flush(); break; }
        }
        XYZrobot.play();
    }
    if(cmd_torque_off) {
        robot_torque_off(); 
        MusicPlaying_wav_stop();
        cmd_torque_off = false;
    }
}
void XYZ_Init_Walking(unsigned char nextStep, const transition_t *addr)
{
    if(last_walking == 0) {
        XYZ_Play_Action(addr);
    } 
    else if(last_walking != nextStep) {
        XYZ_Play_Action(D_Standup);
        XYZ_Play_Action(addr);
    }
    last_walking = nextStep;
}
//Action Task
void Action(int N, boolean falling_mode)
{
    if(torque_released == true) XYZrobot.readPose();

    if((falling_mode == true) && (Falling_Task() != 5))  Getup_Task(Falling_Task());
    else if(N == 0) {MusicPlaying_wav_play("none");if(ActionNo_0 != None) XYZrobot.playSeq(ActionNo_0); last_walking = 0;}
    else if(N == 1) {MusicPlaying_wav_play(Music_1);if(ActionNo_1 != None) { XYZ_Init_Walking(1, D_InitWalkForward);  XYZrobot.playSeq(ActionNo_1);}}
    else if(N == 2) {MusicPlaying_wav_play(Music_2);if(ActionNo_2 != None) { XYZ_Init_Walking(1, D_InitWalkBackward); XYZrobot.playSeq(ActionNo_2);}}
    else if(N == 3) {MusicPlaying_wav_play(Music_3);if(ActionNo_3 != None) XYZrobot.playSeq(ActionNo_3); last_walking = 0;}
    else if(N == 4) {MusicPlaying_wav_play(Music_4);if(ActionNo_4 != None) XYZrobot.playSeq(ActionNo_4); last_walking = 0;}
    else if(N == 5) {MusicPlaying_wav_play(Music_5);if(ActionNo_5 != None) { XYZ_Init_Walking(1, D_InitTurnLeft); XYZrobot.playSeq(ActionNo_5);}}
    else if(N == 6) {MusicPlaying_wav_play(Music_6);if(ActionNo_6 != None) { XYZ_Init_Walking(2, D_InitTurnRight); XYZrobot.playSeq(ActionNo_6);}}
    else if(N == 7) {MusicPlaying_wav_play(Music_7);if(ActionNo_7 != None) { XYZ_Init_Walking(1, D_InitWalkForward); XYZrobot.playSeq(ActionNo_7);}}
    else if(N == 8) {MusicPlaying_wav_play(Music_8);if(ActionNo_8 != None) { XYZ_Init_Walking(1, D_InitWalkForward); XYZrobot.playSeq(ActionNo_8);}}
    else if(N == 9) {MusicPlaying_wav_play(Music_9);if(ActionNo_9 != None) { XYZ_Init_Walking(1, D_InitWalkBackward); XYZrobot.playSeq(ActionNo_9);}}
    else if(N == 10) {MusicPlaying_wav_play(Music_10);if(ActionNo_10 != None) { XYZ_Init_Walking(1, D_InitWalkBackward); XYZrobot.playSeq(ActionNo_10);}}
    else if(N == 11) {MusicPlaying_wav_play(Music_11);if(ActionNo_11 != None) XYZrobot.playSeq(ActionNo_11);}
    else if(N == 12) {MusicPlaying_wav_play(Music_12);if(ActionNo_12 != None) XYZrobot.playSeq(ActionNo_12);}
    else if(N == 13) {MusicPlaying_wav_play(Music_13);if(ActionNo_13 != None) XYZrobot.playSeq(ActionNo_13);}
    else if(N == 14) {MusicPlaying_wav_play(Music_14);if(ActionNo_14 != None) XYZrobot.playSeq(ActionNo_14);}
    else if(N == 15) {MusicPlaying_wav_play(Music_15);if(ActionNo_15 != None) XYZrobot.playSeq(ActionNo_15);}
    else if(N == 16) {MusicPlaying_wav_play(Music_16);if(ActionNo_16 != None) XYZrobot.playSeq(ActionNo_16);}
    else if(N == 17) {MusicPlaying_wav_play(Music_17);if(ActionNo_17 != None) XYZrobot.playSeq(ActionNo_17);}
    else if(N == 18) {MusicPlaying_wav_play(Music_18);if(ActionNo_18 != None) XYZrobot.playSeq(ActionNo_18);}
    else if(N == 19) {MusicPlaying_wav_play(Music_19);if(ActionNo_19 != None) XYZrobot.playSeq(ActionNo_19);}
    else if(N == 20) {MusicPlaying_wav_play(Music_20);if(ActionNo_20 != None) XYZrobot.playSeq(ActionNo_20);}
    else if(N == 21) {MusicPlaying_wav_play(Music_21);if(ActionNo_21 != None) XYZrobot.playSeq(ActionNo_21);}
    else if(N == 22) {MusicPlaying_wav_play(Music_22);if(ActionNo_22 != None) XYZrobot.playSeq(ActionNo_22);}
    else if(N == 23) {MusicPlaying_wav_play(Music_23);if(ActionNo_23 != None) XYZrobot.playSeq(ActionNo_23);}
    else if(N == 24) {MusicPlaying_wav_play(Music_24);if(ActionNo_24 != None) XYZrobot.playSeq(ActionNo_24);}
    else if(N == 25) {MusicPlaying_wav_play(Music_25);if(ActionNo_25 != None) XYZrobot.playSeq(ActionNo_25);}
    else if(N == 26) {MusicPlaying_wav_play(Music_26);if(ActionNo_26 != None) XYZrobot.playSeq(ActionNo_26);}
    else if(N == 27) {MusicPlaying_wav_play(Music_27);if(ActionNo_27 != None) XYZrobot.playSeq(ActionNo_27);}
    else if(N == 28) {MusicPlaying_wav_play(Music_28);if(ActionNo_28 != None) XYZrobot.playSeq(ActionNo_28);}
    else if(N == 29) {MusicPlaying_wav_play(Music_29);if(ActionNo_29 != None) XYZrobot.playSeq(ActionNo_29);}
    else if(N == 30) {MusicPlaying_wav_play(Music_30);if(ActionNo_30 != None) XYZrobot.playSeq(ActionNo_30);}
    else if(N == 31) {MusicPlaying_wav_play(Music_31);if(ActionNo_31 != None) XYZrobot.playSeq(ActionNo_31);}
    else if(N == 32) {MusicPlaying_wav_play(Music_32);if(ActionNo_32 != None) XYZrobot.playSeq(ActionNo_32);}  
    else if(N == 33) {MusicPlaying_wav_play(Music_33);if(ActionNo_33 != None) XYZrobot.playSeq(ActionNo_33);}
    else if(N == 34) {MusicPlaying_wav_play(Music_34);if(ActionNo_34 != None) XYZrobot.playSeq(ActionNo_34);}
    else if(N == 35) {MusicPlaying_wav_play(Music_35);if(ActionNo_35 != None) XYZrobot.playSeq(ActionNo_35);}
    else if(N == 36) {MusicPlaying_wav_play(Music_36);if(ActionNo_36 != None) XYZrobot.playSeq(ActionNo_36);}
    else if(N == 37) {MusicPlaying_wav_play(Music_37);if(ActionNo_37 != None) XYZrobot.playSeq(ActionNo_37);}
    else if(N == 38) {MusicPlaying_wav_play(Music_38);if(ActionNo_38 != None) XYZrobot.playSeq(ActionNo_38);}
    else if(N == 39) {MusicPlaying_wav_play(Music_39);if(ActionNo_39 != None) XYZrobot.playSeq(ActionNo_39);}
    else if(N == 40) {MusicPlaying_wav_play(Music_40);if(ActionNo_40 != None) XYZrobot.playSeq(ActionNo_40);}
    else if(N == 41) {MusicPlaying_wav_play(Music_41);if(ActionNo_41 != None) XYZrobot.playSeq(ActionNo_41);}
    else if(N == 42) {MusicPlaying_wav_play(Music_42);if(ActionNo_42 != None) XYZrobot.playSeq(ActionNo_42);}
    else if(N == 43) {MusicPlaying_wav_play(Music_43);if(ActionNo_43 != None) XYZrobot.playSeq(ActionNo_43);}
    else if(N == 44) {MusicPlaying_wav_play(Music_44);if(ActionNo_44 != None) XYZrobot.playSeq(ActionNo_44);}
    else if(N == 45) {MusicPlaying_wav_play(Music_45);if(ActionNo_45 != None) XYZrobot.playSeq(ActionNo_45);}
    else if(N == 46) {MusicPlaying_wav_play(Music_46);if(ActionNo_46 != None) XYZrobot.playSeq(ActionNo_46);}
    else if(N == 47) {MusicPlaying_wav_play(Music_47);if(ActionNo_47 != None) XYZrobot.playSeq(ActionNo_47);}
    else if(N == 48) {MusicPlaying_wav_play(Music_48);if(ActionNo_48 != None) XYZrobot.playSeq(ActionNo_48);}
    else if(N == 49) {MusicPlaying_wav_play(Music_49);if(ActionNo_49 != None) XYZrobot.playSeq(ActionNo_49);}
    else if(N == 50) {MusicPlaying_wav_play(Music_50);if(ActionNo_50 != None) XYZrobot.playSeq(ActionNo_50);}
    else if(N == 51) {MusicPlaying_wav_play(Music_51);if(ActionNo_51 != None) XYZrobot.playSeq(ActionNo_51);}
    else if(N == 52) {MusicPlaying_wav_play(Music_52);if(ActionNo_52 != None) XYZrobot.playSeq(ActionNo_52);}
    else if(N == 53) {MusicPlaying_wav_play(Music_53);if(ActionNo_53 != None) XYZrobot.playSeq(ActionNo_53);}
    else if(N == 54) {MusicPlaying_wav_play(Music_54);if(ActionNo_54 != None) XYZrobot.playSeq(ActionNo_54);}
    else if(N == 55) {MusicPlaying_wav_play(Music_55);if(ActionNo_55 != None) XYZrobot.playSeq(ActionNo_55);}
    else if(N == 56) {MusicPlaying_wav_play(Music_56);if(ActionNo_56 != None) XYZrobot.playSeq(ActionNo_56);}
    else if (N == 57) { MusicPlaying_wav_play(Music_57); if (ActionNo_57 != None) XYZrobot.playSeq(ActionNo_57); }  
    else if (N == 101) { MusicPlaying_wav_play(Music_Y01); if (ActionNo_Y01 != None) XYZrobot.playSeq(ActionNo_Y01); }
    else if (N == 102) { MusicPlaying_wav_play(Music_Y02); if (ActionNo_Y02 != None) XYZrobot.playSeq(ActionNo_Y02); }
    else if (N == 103) { MusicPlaying_wav_play(Music_Y03); if (ActionNo_Y03 != None) XYZrobot.playSeq(ActionNo_Y03); }
    else if (N == 104) { MusicPlaying_wav_play(Music_Y04); if (ActionNo_Y04 != None) XYZrobot.playSeq(ActionNo_Y04); }
    else if (N == 109) { MusicPlaying_wav_play(Music_Y09); if (ActionNo_Y09 != None) XYZrobot.playSeq(ActionNo_Y09); }
    else if (N == 110) { MusicPlaying_wav_play(Music_Y10); if (ActionNo_Y10 != None) XYZrobot.playSeq(ActionNo_Y10); }
    else if (N == 111) { MusicPlaying_wav_play(Music_Y11); if (ActionNo_Y11 != None) XYZrobot.playSeq(ActionNo_Y11); }
    else if (N == 112) { MusicPlaying_wav_play(Music_Y12); if (ActionNo_Y12 != None) XYZrobot.playSeq(ActionNo_Y12); }
    else if (N == 115) { MusicPlaying_wav_play(Music_Y15); if (ActionNo_Y15 != None) XYZrobot.playSeq(ActionNo_Y15); }
    else if (N == 118) { MusicPlaying_wav_play(Music_Y18); if (ActionNo_Y18 != None) XYZrobot.playSeq(ActionNo_Y18); }
    else if (N == 120) { MusicPlaying_wav_play(Music_Y20); if (ActionNo_Y20 != None) XYZrobot.playSeq(ActionNo_Y20); }
    else if (N == 122) { MusicPlaying_wav_play(Music_Y22); if (ActionNo_Y22 != None) XYZrobot.playSeq(ActionNo_Y22); }
    else if (N == 123) { MusicPlaying_wav_play(Music_Y23); if (ActionNo_Y23 != None) XYZrobot.playSeq(ActionNo_Y23); }
    else if (N == 124) { MusicPlaying_wav_play(Music_Y24); if (ActionNo_Y24 != None) { XYZ_Play_Action(D_Point); XYZrobot.playSeq(ActionNo_Y24); XYZ_Play_Action(D_Standup);}}
    else if (N == 128) { MusicPlaying_wav_play(Music_Y28); if (ActionNo_Y28 != None) XYZrobot.playSeq(ActionNo_Y28); }
    else if (N == 135) { MusicPlaying_wav_play(Music_Y35); if (ActionNo_Y35 != None) XYZrobot.playSeq(ActionNo_Y35); }
    else if (N == 136) { MusicPlaying_wav_play(Music_Y36); if (ActionNo_Y36 != None) XYZrobot.playSeq(ActionNo_Y36); }
    else if (N == 137) { MusicPlaying_wav_play(Music_Y37); if (ActionNo_Y37 != None) XYZrobot.playSeq(ActionNo_Y37); }
    else if (N == 138) { MusicPlaying_wav_play(Music_Y38); if (ActionNo_Y38 != None) XYZrobot.playSeq(ActionNo_Y38); }
    else if (N == 141) { MusicPlaying_wav_play(Music_Y41); if (ActionNo_Y41 != None) XYZrobot.playSeq(ActionNo_Y41); }
    else if (N == 143) { MusicPlaying_wav_play(Music_Y43); if (ActionNo_Y43 != None) XYZrobot.playSeq(ActionNo_Y43); }
    else if (N == 146) { MusicPlaying_wav_play(Music_Y46); if (ActionNo_Y46 != None) XYZrobot.playSeq(ActionNo_Y46); }
    else if (N == 148) { MusicPlaying_wav_play(Music_Y48); if (ActionNo_Y48 != None) XYZrobot.playSeq(ActionNo_Y48); }
    else if (N == 150) { MusicPlaying_wav_play(Music_Y50); if (ActionNo_Y50 != None) { XYZ_Init_Walking(1, D_InitWalkForward); for(int i=0;i<5;i++) {XYZ_Play_Action(ActionNo_Y50);} XYZ_Play_Action(D_Standup); }}
    else if (N == 160) { MusicPlaying_wav_play(Music_Y60); if (ActionNo_Y60 != None) XYZrobot.playSeq(ActionNo_Y60); }
    else if (N == 161) { MusicPlaying_wav_play(Music_Y61); if (ActionNo_Y61 != None) XYZrobot.playSeq(ActionNo_Y61); }
    else if (N == 162) { MusicPlaying_wav_play(Music_Y62); if (ActionNo_Y62 != None) XYZrobot.playSeq(ActionNo_Y62); }
    else if (N == 163) { MusicPlaying_wav_play(Music_Y63); if (ActionNo_Y63 != None) XYZrobot.playSeq(ActionNo_Y63); }
    else if (N == 164) { MusicPlaying_wav_play(Music_Y64); if (ActionNo_Y64 != None) XYZrobot.playSeq(ActionNo_Y64); }

    if(N > 10) last_walking = 0;
    torque_released = false;

    while(XYZrobot.playing) {      
      if(Serial2.available() > 0) {
          BT_Packet_Task();
          if(check_torque_release(packet)) { BT_Flush(); break; }
      }
      XYZrobot.play();
    }
    if(cmd_torque_off) {
      robot_torque_off(); 
      MusicPlaying_wav_stop();
      cmd_torque_off = false;
    }
    Timer_Idle.Reset();
}

boolean check_torque_release(unsigned char* _packet)
{
    if((_packet[1] != 255 && _packet[2] != 1) && ((_packet[5] & 0x10) >> 3)) {
        cmd_torque_off = true;
    }
    else if(_packet[1] == 255 && _packet[2] == 1 && _packet [3] == 102) {
        cmd_torque_off = true;
    }
    else{
        cmd_torque_off = false;
    }

    if(cmd_torque_off == true) {
        mJoystick.RCU_LJX = 0;
        mJoystick.RCU_LJY = 0;
        mJoystick.RCU_RJX = 0;
        mJoystick.RCU_RJY = 0;
    }
    return cmd_torque_off;
}

//BT Reading Task
boolean BT_Packet_Task(void)
{
    //return torque_relase button status
    static int temp_packet[7] = {0};
    static char _i = 0;
    if(Serial2.available() >= 7) {
        if((temp_packet[0] = Serial2.read()) == 0) ; else {find_header_BT(); return false;}
        if((temp_packet[1] = Serial2.read()) == 0) {find_header_BT(); return false;}
        if((temp_packet[2] = Serial2.read()) == 0) {find_header_BT(); return false;}
        if((temp_packet[3] = Serial2.read()) == 0) {find_header_BT(); return false;}
        if((temp_packet[4] = Serial2.read()) == 0) {find_header_BT(); return false;}
        if((temp_packet[5] = Serial2.read()) == 0) {find_header_BT(); return false;}
        if((temp_packet[6] = Serial2.read()) == 0) {find_header_BT(); return false;}
        if(temp_packet[1] != 255 && temp_packet[2] != 1) {
            Serial2.write((temp_packet[6]&0x00F0)>>4);
        } 

        for(_i = 0;_i < 7 ;_i++) packet[_i] = temp_packet[_i];
        BT_update = true;
        return true;
    }
    return false;
}

// BT G-sensor Data Feedback
void BT_Gsensor_Data(void)
{
    g_packet[0]=0;
    if(g_sensor_read_reg(0x32) == 0xFF) {
        g_packet[1] = 0xFF;
        g_packet[7] = 0xC0;
    }
    else { 
        g_packet[1] = g_sensor_read_reg(0x32)+(0x01);
        g_packet[7] = 0x80;
    }   
    if(g_sensor_read_reg(0x33) == 0xFF) {
        g_packet[2] = 0xFF;
        g_packet[7] = g_packet[7]+(0x20);
    }
    else { 
        g_packet[2] = g_sensor_read_reg(0x33)+(0x01);
    }  
    if(g_sensor_read_reg(0x34) == 0xFF) {
        g_packet[3] = 0xFF;
        g_packet[7] = g_packet[7]+(0x10);
    }
    else { 
        g_packet[3] = g_sensor_read_reg(0x34)+(0x01);
    }   
    if(g_sensor_read_reg(0x35) == 0xFF) {
        g_packet[4] = 0xFF;
        g_packet[7] = g_packet[7]+(0x08);
    }
    else { 
        g_packet[4] = g_sensor_read_reg(0x35)+(0x01);
    }         
    if(g_sensor_read_reg(0x36) == 0xFF) {
        g_packet[5] = 0xFF;
        g_packet[7] = g_packet[7]+(0x04);
    }
    else { 
        g_packet[5] = g_sensor_read_reg(0x36)+(0x01);
    }  
    if(g_sensor_read_reg(0x37) == 0xFF) {
        g_packet[6] = 0xFF;
        g_packet[7] = g_packet[7]+(0x02);
    }
    else { 
        g_packet[6] = g_sensor_read_reg(0x37)+(0x01);
    }  

    Serial2.write(g_packet[0]); // packet head
    delay(50);
    Serial2.write(g_packet[1]); // AX_MSB +1
    delay(50);
    Serial2.write(g_packet[2]); // AX_LSB +1
    delay(50);
    Serial2.write(g_packet[3]); // AY_MSB +1
    delay(50);
    Serial2.write(g_packet[4]); // AY_LSB +1
    delay(50);
    Serial2.write(g_packet[5]); // AZ_MSB +1
    delay(50);
    Serial2.write(g_packet[6]); // AZ_LSB +1  
    delay(50);
    Serial2.write(g_packet[7]);
    delay(50);   
}

// BT IR Data Feedback
void BT_IR_Data(void) 
{
    ir_rowdata = analogRead(DISTANCE_SENSOR_PIN);
    ir_msb = ir_rowdata>>8;
    ir_lsb = ir_rowdata&0xFF;
    ir_packet[0] = 0;
    if(ir_lsb == 0xFF) {
        ir_lsb = 0xFF;
        ir_packet[3] = 0x81;
    }
    else {
        ir_lsb = ir_lsb + 0x01;
        ir_packet[3] = 0x80;
    } 
    ir_packet[1] = ir_msb +0x01;
    ir_packet[2] = ir_lsb;  
    Serial2.write(ir_packet[0]); // packet head
    delay(50);
    Serial2.write(ir_packet[1]); // IR_MSB+1
    delay(50);
    Serial2.write(ir_packet[2]); // IR_LSB+1 
    delay(50);
    Serial2.write(ir_packet[3]);
    delay(50);  
}

// Ending_Message Feedback
void BT_ActionEnding(int ActionNum) {
    Serial2.write(0);  // packet head
    delay(50);
    Serial2.write(ActionNum); // Action Number 
    delay(50);
}

// BT FW Feedback 
void BT_FW()
{
    Serial2.write(0xFF);                      // packet head
    delay(50);
    Serial2.write(model_Bolide);              //Model
    delay(50);
    Serial2.write(type_Y01);                 //Type
    delay(50);
    Serial2.write(application_default);       //Application
    delay(50);
    Serial2.write(main_version_number);       //Main Version
    delay(50);
    Serial2.write(secondary_version_number);  //Secondary Version
    delay(50);
    Serial2.write(revision_number);           //Revison
    delay(50);
}

// Clean BT Buffer
void BT_Flush(void) {  
    while(Serial2.available() > 0) { Serial2.read(); };
}
void find_header_BT(void) {
    static int cb = 0x00;
    while(Serial2.available() > 0) {
        if(Serial2.peek() == 0) return;
        cb = Serial2.read();
    }
}

// Speaker function
void MusicPlaying_wav_play(char song_name[]) {
    Serial3.write(0);
    Serial3.print("P");
    Serial3.write(song_name); //set the filename of song : 0000 ~ 9999    
}
void MusicPlaying_wav_stop() {
    Serial3.write(0);
    Serial3.print("S0000");
}
void MusicPlaying_wav_volume(int volume) {
    Serial3.write(0);
    Serial3.write('V');  
    Serial3.write(volume);// volume : 0x01 ~ 0x7F 
    Serial3.print("000");
}

// Buzzer function : play start music
void Start_Music(void) {
    int _i = 0x00;
    for(_i = 0 ; _i < 7; _i++) {
        tone(BUZZER_PIN, pgm_read_word_near(&start_music_frq[_i]));
        delay(200);
        noTone(BUZZER_PIN);    
    }
}

// Button function
uint8_t BUTTON_Task(void)
{
    static unsigned char button_timer = 0x00;
    static uint8_t key = 0x00, last_key = 0x00;
    key = !digitalRead(BUTTON1_PIN) + ((!digitalRead(BUTTON2_PIN))<<1) + ((!digitalRead(BUTTON3_PIN))<<2) + ((!digitalRead(BUTTON4_PIN))<<3);
    if(key != last_key) button_timer++;
    else button_timer = 0;

    if(button_timer > 20) {
        button_timer = 0;
        last_key = key;
        if(key != 0) {
            set_led_mode(ONE_LED_ON);
            if(key == key_mask_button1) Action(RB_1);           // Button_1
            else if(key == key_mask_button2) Action(RB_2);      // Button_2
            else if(key == key_mask_button3) {                  // Button_3
                // Auto Mode Control
                if(exe_mode != EXE_MODE_AUTOMODE) {                    
                    exe_mode = EXE_MODE_AUTOMODE;
                    Init_AutoMode();
                }
                else {
                    exe_mode = EXE_MODE_RCU;
                }
                bInAutoMode = 1 - bInAutoMode;
            }
            else if(key == key_mask_button4) {                  // Button_4
                // Calibration Task
                exe_mode = EXE_MODE_RCU;
            }
        }
    }

    // Calibration Task
    if(key == key_mask_button4) Calibration_Task(true);
    else Calibration_Task(false);

    return key;
}

void Calibration_Task(boolean key_pressed)
{
    static int zp_task = 0;
    if(key_pressed == true) {
        // Button_4        
        if(zp_task < 50) {
            set_led_mode(RED_LED_ON);
            if(Timer_Calibrate.Timer_Task(100))   zp_task++;
        }
        else if(zp_task == 50) {
            if(torque_released == true) {   // Update ZeroPoint
                XYZ_Update_ZeroPoint();
            }
            else {      // Reset ZeroPoint
                short zeroPoint[18] = {0};
                for(int i = 0; i < 18; i++) {
                    eeprom_write_short(EEP_ADDR_ZEROPOINT + (i << 1), 0);
                }
                XYZrobot.setZeroPoint(zeroPoint);
            }
            zp_task++;
            set_led_mode(WHITE_LED_ON);
            Action(A_DefaultInitial);
        }
        else {  // zp_task > 50
            set_led_mode(WHITE_LED_ON);
        }
    }
    else {
        zp_task = 0;
    }
}

// LED function
void set_eye_led(char eyemode, char _delay_cnt)
{   
    if(eyeled_delay_cnt > 0) { eyeled_delay_cnt--; return; }

    if(eyemode == EYE_MODE_OFF) {
        EYE_LED_OFF;
    }
    else if(eyemode == EYE_MODE_GREEN) {
        EYE_LED_GRN;
    }
    else if(eyemode == EYE_MODE_BLUE) {
        EYE_LED_BLE;
    }
    if(_delay_cnt > 0)  eyeled_delay_cnt = _delay_cnt;
}

void set_led_mode(char mode)
{
    if(mode == LED_mode)  return;
    
    if(LED_mode == LED_SCRATCH && mode != LED_SCRATCH) {
        Timer_Task_Setup();
    }
    
    if(mode == LED_OFF) {         // LED OFF
        EYE_LED_OFF;
        OCR5A = OCR5B = OCR5C = 0;
    }
    else if(mode == LED_SCRATCH) {  // Scratch Mode
        if(LED_mode != LED_SCRATCH) {
            EYE_LED_BLE;                       // Eye LED
            analogWrite(LSA_LED_RED_PIN, 0);   // Red
            analogWrite(LSA_LED_GREEN_PIN, 0); // Green
            analogWrite(LSA_LED_BLUE_PIN, 0);  // Blue
        }
    }
    else if(mode == BLUE_LED_ON) {
        OCR5C = 1023;
        OCR5A = OCR5B = 0;
    }
    else if(mode == GREEN_LED_ON) {
        OCR5B = 1023;
        OCR5A = OCR5C = 0;
    }
    else if(mode == RED_LED_ON) {
        OCR5A = 1023;
        OCR5B = OCR5C = 0;
    }
    else if(mode == WHITE_LED_ON) {
        OCR5A = OCR5B = OCR5C = 1023;
    }
    else if(mode == BLE_LED_BREATH) {    // BLUE LED Blink
    }
    else if(mode == GRN_LED_BREATH) {    // GREEN LED Blink
    }
    else if(mode == RED_LED_BREATH) {    // RED LED Blink
    }
    else if(mode == ONE_LED_ON) {    // Single LED On
        if(OCR5A > 1) OCR5A = 1023;
        else if(OCR5B > 1)  OCR5B = 1023;
        else  OCR5C = 1023;
        //_disable_timer3();
    }
    LED_mode = mode;
}
void Led_Task(void)
{
    // 10ms Task
    static int R = 0, G = 0, B = 0;
    static int _R = 41, _G = 41, _B = 41;
    static unsigned char timer_cnt = 0;

    // EYE LED
    if(eyeled_delay_cnt > 0)    eyeled_delay_cnt--;

    // CHEST LED
    if(LED_mode == BLE_LED_BREATH) {       
        // LED_BLUE
        if(timer_cnt > 0) { timer_cnt--; return; }
        timer_cnt = 4;  // 10ms*4

        if(B < 40){B++; OCR5C = pgm_read_word_near(&log_light_40[B]); }
        else if(_B > 0){_B--; OCR5C = pgm_read_word_near(&log_light_40[_B]); }
        else{
            R = 0;G = 0;B = 0;
            _R = 41;_G = 41;_B = 41;
        }
        OCR5A = OCR5B = 0;
    }
    else if (LED_mode == GRN_LED_BREATH) { 
        // LED_GREEN
        if(G < 40){G++; OCR5B = pgm_read_word_near(&log_light_40[G]); }
        else if(_G > 0){_G--; OCR5B = pgm_read_word_near(&log_light_40[_G]); }
        else{
            R = 0;G = 0;B = 0;
            _R = 41;_G = 41;_B = 41;
        }
        OCR5A = OCR5C = 0;
    }
    else if(LED_mode == RED_LED_BREATH) {
        // LED_RED
        if(R < 40){R++; OCR5A = pgm_read_word_near(&log_light_40[R]);}
        else if(_R > 0){_R--; OCR5A = pgm_read_word_near(&log_light_40[_R]);}
        else{
        R = 0;G = 0;B = 0;
        _R = 41;_G = 41;_B = 41;
        }
        OCR5B = OCR5C = 0;
    }
    else {
        return;
    }
/*    else if(LED_mode == 3) {
        if(timer_cnt > 0) { timer_cnt--; return; }
        timer_cnt = 4;  // 10ms*4

        if(R < 40){R++; OCR5A = pgm_read_word_near(&log_light_40[R]);}
        else if(_R > 0){_R--; OCR5A = pgm_read_word_near(&log_light_40[_R]);}
        else if(G < 40){G++; OCR5B = pgm_read_word_near(&log_light_40[G]);}
        else if(_G > 0){_G--; OCR5B = pgm_read_word_near(&log_light_40[_G]);}
        else if(B < 40){B++; OCR5C = pgm_read_word_near(&log_light_40[B]);}
        else if(_B > 0){_B--; OCR5C = pgm_read_word_near(&log_light_40[_B]);}
        else{
        R = 0;G = 0;B = 0;
        _R = 41;_G = 41;_B = 41;
        }
    }*/
}

ISR(TIMER3_OVF_vect) 
{
    Led_Task();

    // Reset Timer
    _reset_timer3(timer_10ms);
}

// Power Detection function
void Power_Detection_Task(void)
{
    static short PWR_Voltage;
    static boolean robot_tired = false;
    static int cnt = 0;

    PWR_Voltage = analogRead(PWRDET_PIN);   // vol*0.0124
    if (PWR_Voltage < Power_Voltage_Alarm) 
    {
        if(cnt > 10)
        {
            static boolean buzzer_tone = false;
            if(!buzzer_tone)
                tone(BUZZER_PIN, 1000);
            else 
                noTone(BUZZER_PIN);
            buzzer_tone = !buzzer_tone;

            if(robot_tired == false) {
                if(exe_mode == EXE_MODE_AUTOMODE) { Action(A_WFLow); Action(A_WFLow); } 
                Action(A_Tired); robot_torque_off(); 
            }
            robot_tired = true;
        }
        else cnt++;
    }
    else {
        robot_tired = false;
        cnt = 0;
    }
}
ISR(TIMER4_OVF_vect){
  Power_Detection_Task();
  packet_timeout_status = true;
  _reset_timer4(timeout_limit);
}

// ======= Timer ========
boolean Constant_Timer::Timer_Task(unsigned long _time_ms) {
  time_tick = _time_ms;
  return TimerEvent();
}
boolean Constant_Timer::TimerEvent(void) {  
  if((millis() - _last_time) >= time_tick){
    _last_time = millis();
    return true;
  }
  return false;
}
void Constant_Timer::Reset(void) {
    _last_time = millis();
}
void Constant_Timer::SetTimer(unsigned long _timetick) {
    time_tick = _timetick;
    _last_time = millis();
}
