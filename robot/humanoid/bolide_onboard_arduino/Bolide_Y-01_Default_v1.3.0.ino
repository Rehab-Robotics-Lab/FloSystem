//Include Library
#include <BOLIDE_Player.h>
#include <A1_16.h>
#include <EEPROM.h>
#include <Wire.h>
#include "BOLIDE_Y-01_Board.h"
#include "Y-01_Mask_Definition.h"
#include "Y-01_USER_MOTION.h"

//== Declare Global Parameters ==
//Normal Operation Pararmeter
BOLIDE_Player XYZrobot;
static float ax, ay, az;
static int I2C_Address = 0x3B >> 1;
static int distance;
static int SN_packet[9] = {0}, SN_packet_index = 0, inByte = 0;
static int packet[7], LED_mode, g_packet[8], ir_packet[4], ir_msb, ir_lsb, ir_rowdata;
static boolean torque_release = false, BT_update = false;
static int joystick_status[4] = {125, 125, 125, 125};
//Motion Editor Parameter
static boolean packet_timeout_status = false;
static boolean seq_trigger = false, seq_loop_trigger = false;
static int seq_pSeqCnt = 0xFF, SeqPos = 0x00;
static int poses[max_pose_index][MAX_SERVOS]; // poses [index][servo_id-1], check for the motion index!!
static int pose_index[max_pose_index];
sp_trans_t sequence[max_seq_index]; // sequence

//========================= Set up =======================================
void setup()
{
  //Configure all basic setting
  Serial.begin(115200);
  AIM_Task_Setup();
  BT_Task_Setup();
  Speaker_Task_Setup();
  Eye_LED_Setup();
  Buzzer_Setup();
  Button_Setup();
  Analog_Input_Setup();
  Timer_Task_Setup();
  _enable_timer4();

  //Start motion
  LED_Task(2);
  Start_Music();
  G_SENSOR_Task_Setup();
  Initial_Pose_Setup();
  delay(1000);
  LED_Task(0);
}
//========================= Main =======================================
void loop()
{
  //USB Communcation motion
  if (Serial.available() > 0)
  {
    Motion_Editor_Packet_Task();
  }

  //play sequence edited by motion editor
  else if (seq_trigger)
  {
    Motion_Editor_Seq_Play();
  }

  else
  {
    //BT Communcation motion
    if (Serial2.available() > 0)
    {
      BT_Packet_Task();
      if (BT_update)
      {
        joystick_status[0] = packet[1];
        joystick_status[1] = packet[2];
        joystick_status[2] = packet[3];
        joystick_status[3] = packet[4];

        //==== RCU Command ====
        if (packet[1] != 255 & packet[2] != 1)
        {
          LED_Task(1);

          //Release Button
          if (packet[5] & RCU_mask_release)
          {
            A1_16_TorqueOff(A1_16_Broadcast_ID);
            //          cb_BT();
          }

          //Bluetooth Button
          else if (packet[5] & RCU_mask_BT)
          {
            //          cb_BT();
          }

          //Power Button
          else if (packet[5] & RCU_mask_power)
          {
            XYZrobot.playSeq(DefaultInitial);
            while (XYZrobot.playing)
              XYZrobot.play();
            //          cb_BT();
          }

          //L1 Button
          else if (packet[6] & RCU_mask_L1)
          {
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
                Action(RCU_L1);
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(RCU_L1);
          }

          //L2 Button
          else if (packet[6] & RCU_mask_L2)
          {
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
                Action(RCU_L2);
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(RCU_L2);
          }

          //L3 Button
          else if (packet[6] & RCU_mask_L3)
          {
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
                Action(RCU_L3);
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(RCU_L3);
          }

          //R1 Button
          else if (packet[5] & RCU_mask_R1)
          {
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
                Action(RCU_R1);
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(RCU_R1);
          }

          //R2 Button
          else if (packet[5] & RCU_mask_R2)
          {
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
                Action(RCU_R2);
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(RCU_R2);
          }

          //R3 Button
          else if (packet[5] & RCU_mask_R3)
          {
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
                Action(RCU_R3);
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(RCU_R3);
          }

          //LeftJoystick_Rightside
          else if ((packet[1] > 155 & packet[2] > 155 & packet[1] > packet[2]) | (packet[1] > 155 & packet[2]<95 & (packet[1] - 155)>(95 - packet[2])) | (packet[1] > 155 & packet[2] >= 95 & packet[2] <= 155))
          {
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
                Action(RCU_LJR);
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(RCU_LJR);
          }

          //LeftJoystick_Leftside
          else if ((packet[1]<95 & packet[2]> 155 & (95 - packet[1]) > (packet[2] - 155)) | (packet[1] < 95 & packet[2]<155 & (95 - packet[1])>(95 - packet[2])) | (packet[1] < 95 & packet[2] >= 95 & packet[2] <= 155))
          {
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
                Action(RCU_LJL);
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(RCU_LJL);
          }

          //LeftJoystick_Upside
          else if ((packet[1] > 155 & packet[2] > 155 & packet[1] < packet[2]) | (packet[1]<95 & packet[2]> 155 & (95 - packet[1]) < (packet[2] - 155)) | (packet[2] > 155 & packet[1] >= 95 & packet[1] <= 155))
          {
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
              {
                if (Avoidance_index & IR_SENSOR_Task() < 20)
                {
                  for (int i = 0; i < 3; i++)
                  {
                    tone(BUZZER_PIN, pgm_read_word_near(&obstacle_alarm_frq[i]));
                    delay(250);
                    noTone(BUZZER_PIN);
                  }
                }
                else
                  Action(RCU_LJU);
              }
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(RCU_LJU);
          }

          //LeftJoystick_Downside
          else if ((packet[1] > 155 & packet[2] < 95 & (packet[1] - 155) < (95 - packet[2])) | (packet[1] < 95 & packet[2] < 95 & (95 - packet[1]) < (155 - packet[2])) | (packet[2] < 95 & packet[1] >= 95 & packet[1] <= 155))
          {
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
                Action(RCU_LJD);
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(RCU_LJD);
          }

          //RightJoystick_Rightside
          else if ((packet[3] > 155 & packet[4] > 155 & packet[3] > packet[4]) | (packet[3] > 155 & packet[4]<95 & (packet[3] - 155)>(95 - packet[4])) | (packet[3] > 155 & packet[4] >= 95 & packet[4] <= 155))
          {
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
                Action(RCU_RJR);
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(RCU_RJR);
          }

          //RightJoystick_Leftside
          else if ((packet[3]<95 & packet[4]> 155 & (95 - packet[3]) > (packet[4] - 155)) | (packet[3] < 95 & packet[4]<95 & (95 - packet[3])>(95 - packet[4])) | (packet[3] < 95 & packet[4] >= 95 & packet[4] <= 155))
          {
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
                Action(RCU_RJL);
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(RCU_RJL);
          }

          //RightJoystick_Upside
          else if ((packet[3] > 155 & packet[4] > 155 & packet[3] < packet[4]) | (packet[3]<95 & packet[4]> 155 & (95 - packet[3]) < (packet[4] - 155)) | (packet[4] > 155 & packet[3] >= 95 & packet[3] <= 155))
          {
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
                Action(RCU_RJU);
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(RCU_RJU);
          }

          //RightJoystick_Downside
          else if ((packet[3] > 155 & packet[4] < 95 & (packet[3] - 155) < (95 - packet[4])) | (packet[3] < 155 & packet[4] < 95 & (95 - packet[3]) < (95 - packet[4])) | (packet[4] < 95 & packet[3] >= 95 & packet[3] <= 155))
          {
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
                Action(RCU_RJD);
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(RCU_RJD);
          }

          LED_Task(0);
          BT_update = false;
        }
        //==== App Command ====
        else if (packet[1] == 255 & packet[2] == 1)
        {
          LED_Task(3);
          if (packet[3] == 101)
          {
            XYZrobot.playSeq(DefaultInitial);
            while (XYZrobot.playing)
              XYZrobot.play();
          }
          else if (packet[3] == 102)
            A1_16_TorqueOff(254);
          else if (packet[3] == 251)
            BT_Gsensor_Data();
          else if (packet[3] == 252)
            BT_IR_Data();
          else if (packet[3] == 253)
            BT_FW();
          else
          {
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
              {
                if (packet[3] == 1)
                { //WalkForward
                  if (Avoidance_index & IR_SENSOR_Task() < 20)
                  {
                    for (int i = 0; i < 3; i++)
                    {
                      tone(BUZZER_PIN, pgm_read_word_near(&obstacle_alarm_frq[i]));
                      delay(250);
                      noTone(BUZZER_PIN);
                    }
                  }
                  else
                    Action(packet[3]);
                }
                else
                  Action(packet[3]);
              }
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(packet[3]);
          }
          LED_Task(0);
          BT_update = false;
        }
      }
    }
    else
    {
      if (packet[1] != 255 & packet[2] != 1)
      {
        if (joystick_status[0] <= 155 & joystick_status[0] >= 95 & joystick_status[1] <= 155 & joystick_status[1] >= 95 & joystick_status[2] <= 155 & joystick_status[2] >= 95 & joystick_status[3] <= 155 & joystick_status[3] >= 95)
        {
          // Button task
          BUTTON_Task();
        }
        else
        {

          if ((joystick_status[0] > 155 & joystick_status[1] > 155 & joystick_status[0] > joystick_status[1]) | (joystick_status[0] > 155 & joystick_status[1]<95 & (joystick_status[0] - 155)>(95 - joystick_status[1])) | (joystick_status[0] > 155 & joystick_status[1] >= 95 & joystick_status[1] <= 155))
          {
            //LeftJoystick_Rightside
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
                Action(RCU_LJR);
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(RCU_LJR);
          }

          else if ((joystick_status[0]<95 & joystick_status[1]> 155 & (95 - joystick_status[0]) > (joystick_status[1] - 155)) | (joystick_status[0] < 95 & joystick_status[1]<155 & (95 - joystick_status[0])>(95 - joystick_status[1])) | (joystick_status[0] < 95 & joystick_status[1] >= 95 & joystick_status[1] <= 155))
          {
            //LeftJoystick_Leftside
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
                Action(RCU_LJL);
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(RCU_LJL);
          }

          else if ((joystick_status[0] > 155 & joystick_status[1] > 155 & joystick_status[0] < joystick_status[1]) | (joystick_status[0]<95 & joystick_status[1]> 155 & (95 - joystick_status[0]) < (joystick_status[1] - 155)) | (joystick_status[1] > 155 & joystick_status[0] >= 95 & joystick_status[0] <= 155))
          {
            //LeftJoystick_Upside
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
              {
                if (Avoidance_index & IR_SENSOR_Task() < 20)
                {
                  for (int i = 0; i < 3; i++)
                  {
                    tone(BUZZER_PIN, pgm_read_word_near(&obstacle_alarm_frq[i]));
                    delay(250);
                    noTone(BUZZER_PIN);
                  }
                }
                else
                  Action(RCU_LJU);
              }
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(RCU_LJU);
          }

          else if ((joystick_status[0] > 155 & joystick_status[1] < 95 & (joystick_status[0] - 155) < (95 - joystick_status[1])) | (joystick_status[0] < 95 & joystick_status[1] < 95 & (95 - joystick_status[0]) < (155 - joystick_status[1])) | (joystick_status[1] < 95 & joystick_status[0] >= 95 & joystick_status[0] <= 155))
          {
            //LeftJoystick_Downside
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
                Action(RCU_LJD);
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(RCU_LJD);
          }

          else if ((joystick_status[2] > 155 & joystick_status[3] > 155 & joystick_status[2] > joystick_status[3]) | (joystick_status[2] > 155 & joystick_status[3]<95 & (joystick_status[2] - 155)>(95 - joystick_status[3])) | (joystick_status[2] > 155 & joystick_status[3] >= 95 & joystick_status[3] <= 155))
          {
            //RightJoystick_Rightside
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
                Action(RCU_RJR);
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(RCU_RJR);
          }

          else if ((joystick_status[2]<95 & joystick_status[3]> 155 & (95 - joystick_status[2]) > (joystick_status[3] - 155)) | (joystick_status[2] < 95 & joystick_status[3]<95 & (95 - joystick_status[2])>(95 - joystick_status[3])) | (joystick_status[2] < 95 & joystick_status[3] >= 95 & joystick_status[3] <= 155))
          {
            //RightJoystick_Leftside
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
                Action(RCU_RJL);
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(RCU_RJL);
          }

          else if ((joystick_status[2] > 155 & joystick_status[3] > 155 & joystick_status[2] < joystick_status[3]) | (joystick_status[2]<95 & joystick_status[3]> 155 & (95 - joystick_status[2]) < (joystick_status[3] - 155)) | (joystick_status[3] > 155 & joystick_status[2] >= 95 & joystick_status[2] <= 155))
          {
            //RightJoystick_Upside
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
                Action(RCU_RJU);
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(RCU_RJU);
          }

          else if ((joystick_status[2] > 155 & joystick_status[3] < 95 & (joystick_status[2] - 155) < (95 - joystick_status[3])) | (joystick_status[2] < 155 & joystick_status[3] < 95 & (95 - joystick_status[2]) < (95 - joystick_status[3])) | (joystick_status[3] < 95 & joystick_status[2] >= 95 & joystick_status[2] <= 155))
          {
            //RightJoystick_Downside
            if (Adjustment_index)
            {
              if (Falling_Task() == 5)
                Action(RCU_RJD);
              else
                Getup_Task(Falling_Task());
            }
            else
              Action(RCU_RJD);
          }
        }
      }
    }
  }
}
//=========================== Function ================================
//== Setup function ==
//Configure A1-16 servo motor
void AIM_Task_Setup(void)
{
  XYZrobot.setup(115200, 18);
}

//Configure BT board
void BT_Task_Setup(void)
{
  Serial2.begin(9600);
}

//Configure speaker Board
void Speaker_Task_Setup(void)
{
  Serial3.begin(115200);
  pinMode(LSA_LED_BLUE_PIN, OUTPUT);
  pinMode(LSA_LED_GREEN_PIN, OUTPUT);
  pinMode(LSA_LED_RED_PIN, OUTPUT);
}

//Configure eye led board pin
void Eye_LED_Setup(void)
{
  pinMode(LED_BLUE_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
}

//Configure onboard buzzer pin
void Buzzer_Setup(void)
{
  pinMode(BUZZER_PIN, OUTPUT);
}

//Configure onboard button pin
void Button_Setup(void)
{
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);
  pinMode(BUTTON3_PIN, INPUT);
  pinMode(BUTTON4_PIN, INPUT);
}

//Configure Analog input pin
void Analog_Input_Setup(void)
{
  pinMode(PWRDET_PIN, INPUT);
  pinMode(DISTANCE_SENSOR_PIN, INPUT);
  analogReference(EXTERNAL);
}

// Timer Setup
void Timer_Task_Setup(void)
{
  //Set Timer3 as a normal timer for LED task
  TCCR3A = 0x00;
  TCCR3B |= _BV(CS32);
  TCCR3B &= ~_BV(CS31);
  TCCR3B |= _BV(CS30);
  //Set Timer4 as a normal timer for communcation timeout
  TCCR4A = 0x00;
  TCCR4B |= _BV(CS42);
  TCCR4B &= ~_BV(CS41);
  TCCR4B |= _BV(CS40);
  //Set Timer5 as a Fast PWM generator for chest LED driver
  TCCR5A = _BV(COM5A1) | _BV(COM5B1) | _BV(COM5C1) | _BV(WGM51) | _BV(WGM50);
  TCCR5B = _BV(WGM52) | _BV(CS52);
  OCR5A = 0;
  OCR5B = 0;
  OCR5C = 0;
}

//G-Sensor Setup
void G_SENSOR_Task_Setup(void)
{
  Wire.begin();
  setReg(0x2D, 0xA);
}
void setReg(int reg, int data)
{
  Wire.beginTransmission(I2C_Address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}
int getData(int reg)
{
  static int Gsensor_timer;
  Gsensor_timer = 0;
  Wire.beginTransmission(I2C_Address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(I2C_Address, 1);
  if (Wire.available() <= 1)
    return Wire.read();
}

//Falling Detection Task
int Falling_Task(void)
{
  int posture_index;
  ax = ((getData(0x33) << 8) + getData(0x32)) / 256.0;
  ay = ((getData(0x35) << 8) + getData(0x34)) / 256.0;
  az = ((getData(0x37) << 8) + getData(0x36)) / 256.0;

  if ((az) < -0.75)
    posture_index = 1; //Frontside Getup
  else if ((az) > 0.75)
    posture_index = 2; // Backside Getup
  else if ((ax) < -0.75)
    posture_index = 3; // Rightside Getup
  else if ((ax) > 0.75)
    posture_index = 4; // Leftside Getup
  else if ((az) <= 0.75 && (az) >= -0.75)
    posture_index = 5; // Stand Status

  return posture_index;
}

//Getup Detection Task
void Getup_Task(int posture_index)
{
  if (posture_index == 1)
    Action(52);
  else if (posture_index == 2)
    Action(53);
  else if (posture_index == 3)
    Action(54);
  else if (posture_index == 4)
    Action(54);
}

//IR Task
int IR_SENSOR_Task(void)
{
  distance = (6787 / (analogRead(DISTANCE_SENSOR_PIN) - 3)) - 4;
  return distance;
}

//Motion Editor Task
void Motion_Editor_Packet_Task(void)
{
  static int pBuffer[128] = {0};
  static unsigned char pIndex = 0, pLength = 0xFF, pCMD = 0x00;
  static unsigned char motor_ID = 0;
  static int position_ID = 0;
  static int seq_pPoseCnt = 0xFF, PoseCnt = 0;
  static int SeqCnt = 0;
  static int SeqProcessCnt = 0;
  static int _temp;

  _temp = Serial.read();
  if (_temp == packet_header)
    goto _packet_start;
  else
  {
    return;
  }
_packet_start:
  pBuffer[header_address] = _temp;
  pIndex = 1;
  _reset_timer4(timeout_limit);
  packet_timeout_status = false;
  while ((_temp = Serial.read()) == -1)
  {
    if (packet_timeout_status)
    {
      Packet_Error_Feedback(0x00);
      return;
    }
  }
  pBuffer[length_address] = _temp;
  pLength = _temp;
  pIndex++;
  _reset_timer4(timeout_limit);
  packet_timeout_status = false;
  while ((_temp = Serial.read()) == -1)
  {
    if (packet_timeout_status)
    {
      Packet_Error_Feedback(0x00);
      return;
    }
  }
  pBuffer[CMD_address] = _temp;
  pCMD = _temp;
  pIndex++;
  _reset_timer4(timeout_limit);
  packet_timeout_status = false;
  while (1)
  {
    while ((_temp = Serial.read()) == -1)
    {
      if (packet_timeout_status)
      {
        Packet_Error_Feedback(0x00);
        return;
      }
    }
    pBuffer[pIndex] = _temp;
    pIndex++;
    _reset_timer4(timeout_limit);
    packet_timeout_status = false;
    if (pIndex == pLength)
    {
      cb_USB();
      _reset_timer4(timeout_limit);
      goto _packet_finish;
    }
  }
_packet_finish:
  if (pBuffer[pIndex - 1] == packet_tail)
  {
    if (pCMD == CMD_init_motor)
    { //initial motion editor setting
      seq_trigger = false;
      SeqPos = 0;
      XYZrobot.poseSize = pBuffer[motor_num_address];
      XYZrobot.readPose();
      Packet_Init(pBuffer[motor_num_address]);
    }
    else if (pCMD == CMD_set_motor)
    { //set motor position
      seq_trigger = false;
      SeqPos = 0;
      motor_ID = pBuffer[motor_ID_address];
      position_ID = (pBuffer[motor_pos_msb] << 8) + pBuffer[motor_pos_lsb];
      SetPositionI_JOG(motor_ID, 0x00, position_ID);
      Packet_Set(motor_ID, position_ID);
    }
    else if (pCMD == CMD_capture_motor)
    { //get motor position
      seq_trigger = false;
      SeqPos = 0;
      motor_ID = pBuffer[motor_ID_address];
      Packet_Capture(motor_ID);
    }
    else if (pCMD == CMD_relax_motor)
    { //relax motor
      seq_trigger = false;
      SeqPos = 0;
      motor_ID = pBuffer[motor_ID_address];
      A1_16_TorqueOff(motor_ID);
      Packet_Relax(motor_ID);
    }
    else if (pCMD == CMD_SN_read)
    { // serial number read
      seq_trigger = false;
      SeqPos = 0;
      Packet_SN();
    }
    else if (pCMD == CMD_SEQ_load_PoseCnt)
    { //load total pose number
      seq_trigger = false;
      SeqPos = 0;
      seq_pPoseCnt = pBuffer[seq_pose_cnt_address];
      if (seq_pPoseCnt > max_pose_index)
        Packet_Error_Feedback(0x00);
      else
      {
        Packet_Error_Feedback(0x01);
        SeqProcessCnt = SEQ_Process_load_PoseCnt;
        seq_loop_trigger = false;
      }
    }
    else if (pCMD == CMD_SEQ_loop_load_PoseCnt)
    { //load loop sequence pose number
      seq_trigger = false;
      SeqPos = 0;
      seq_pPoseCnt = pBuffer[seq_pose_cnt_address];
      if (seq_pPoseCnt > max_pose_index)
        Packet_Error_Feedback(0x00);
      else
      {
        Packet_Error_Feedback(0x01);
        SeqProcessCnt = SEQ_Process_load_PoseCnt;
        seq_loop_trigger = true;
      }
    }
    else if (pCMD == CMD_SEQ_load_Pose)
    { //load pose in sequence
      static int PoseID = 0, _i = 0;
      seq_trigger = false;
      SeqPos = 0;
      if (SeqProcessCnt == SEQ_Process_load_PoseCnt)
      {
        PoseID = pBuffer[seq_pose_ID_address];
        for (_i = 0; _i < XYZrobot.poseSize; _i++)
        {
          poses[PoseCnt][_i] = (pBuffer[seq_pose_start_address + 2 * _i] << 8) + pBuffer[seq_pose_start_address + 1 + 2 * _i];
          pose_index[PoseCnt] = PoseID;
        }
        PoseCnt++;
        if (PoseCnt == seq_pPoseCnt)
        {
          Packet_Error_Feedback(0x02);
          PoseCnt = 0;
          SeqProcessCnt = SEQ_Process_load_Pose;
        }
        else
          Packet_Error_Feedback(0x01);
      }
      else
        Packet_Error_Feedback(0x00);
    }
    else if (pCMD == CMD_SEQ_load_SEQCnt)
    { //load total sequence number
      seq_trigger = false;
      SeqPos = 0;
      if (SeqProcessCnt == SEQ_Process_load_Pose)
      {
        seq_pSeqCnt = pBuffer[seq_seq_cnt_address];
        if (seq_pSeqCnt > max_seq_index)
          Packet_Error_Feedback(0x00);
        else
        {
          Packet_Error_Feedback(0x01);
          SeqProcessCnt = SEQ_Process_load_SEQCnt;
        }
      }
      else
        Packet_Error_Feedback(0x00);
    }
    else if (pCMD == CMD_SEQ_load_SEQ)
    { //load sequence
      static int sPoseID = 0, sTime = 0, _i;
      seq_trigger = false;
      SeqPos = 0;
      if (SeqProcessCnt == SEQ_Process_load_SEQCnt)
      {
        sPoseID = pBuffer[seq_pose_name_address];
        sTime = (pBuffer[seq_pose_time_MSB_address] << 8) + pBuffer[seq_pose_time_LSB_address];
        for (_i = 0; _i < max_pose_index; _i++)
        {
          if (pose_index[_i] == sPoseID)
          {
            sequence[SeqCnt].pose = _i;
            sequence[SeqCnt].time = sTime;
            SeqCnt++;
            break;
          }
        }
        if (SeqCnt == seq_pSeqCnt)
        {
          Packet_Error_Feedback(0x02);
          SeqCnt = 0;
          seq_trigger = true;
          XYZrobot.readPose();
          SeqProcessCnt = SEQ_Process_load_SEQ;
        }
        else
          Packet_Error_Feedback(0x01);
      }
      else
        Packet_Error_Feedback(0x00);
    }
    else if (pCMD == CMD_SEQ_halt)
    { //halt sequence
      seq_trigger = false;
      //halt sequence
    }
    else if (pCMD == CMD_SEQ_relax)
    { //relax servo
      seq_trigger = false;
      A1_16_TorqueOff(A1_16_Broadcast_ID);
    }
    else if (pCMD == CMD_version_read)
    {
      seq_trigger = false;
      SeqPos = 0;
      Packet_Version_Read();
    }
  }
  else
  {
    Packet_Error_Feedback(0x00);
    pLength = 0xFF;
  }
}
void cb_USB(void)
{
  while (Serial.read() != -1)
    ;
  //clear USB serial buffer
}
void Motion_Editor_Seq_Play(void)
{
  static int _i = 0;
  static int pose_index = 0;
  pose_index = sequence[SeqPos].pose;
  for (_i = 0; _i < XYZrobot.poseSize; _i++)
    XYZrobot.setNextPose(_i + 1, poses[pose_index][_i]);
  XYZrobot.interpolateSetup(sequence[SeqPos].time);
  while (XYZrobot.interpolating)
    XYZrobot.interpolateStep();
  SeqPos++;
  if (SeqPos == seq_pSeqCnt)
  {
    SeqPos = 0;
    if (seq_loop_trigger)
      ;
    else
    {
      seq_trigger = false;
      seq_pSeqCnt = 0xFF;
    }
  }
}
void Packet_Init(unsigned char motor_num)
{
  Serial.write(packet_header);
  Serial.write(0x05);
  Serial.write(CMD_init_motor);
  Serial.write(motor_num);
  Serial.write(packet_tail);
}
void Packet_Set(unsigned char motor_ID, int pos_set)
{
  Serial.write(packet_header);
  Serial.write(0x07);
  Serial.write(CMD_set_motor);
  Serial.write(motor_ID);
  Serial.write(((pos_set & 0xFF00) >> 8));
  Serial.write((pos_set & 0x00FF));
  Serial.write(packet_tail);
}
void Packet_Capture(unsigned char motor_ID)
{
  static int position_buffer[19] = {0}; // position buffer
  static int _i = 0;
  for (_i = 1; _i < 19; _i++)
    position_buffer[_i] = ReadPosition(_i);
  Serial.write(packet_header);
  Serial.write(0x29);
  Serial.write(CMD_capture_motor);
  Serial.write(motor_ID);
  for (_i = 1; _i < 19; _i++)
  {
    Serial.write(((position_buffer[_i] & 0xFF00) >> 8));
    Serial.write((position_buffer[_i] & 0x00FF));
  }
  Serial.write(packet_tail);
}
void Packet_Relax(unsigned char motor_ID)
{
  Serial.write(packet_header);
  Serial.write(0x05);
  Serial.write(CMD_relax_motor);
  Serial.write(motor_ID);
  Serial.write(packet_tail);
}
void Packet_SN(void)
{
  static int _i = 0;
  Serial.write(packet_header);
  Serial.write(0x12);
  Serial.write(CMD_SN_read);
  for (_i = 10; _i < 24; _i++)
    Serial.write(EEPROM.read(_i));
  Serial.write(packet_tail);
}
void Packet_Version_Read(void)
{
  Serial.write(packet_header);
  Serial.write(0x0A);
  Serial.write(CMD_version_read);
  Serial.write(model_Bolide);             //Model
  Serial.write(type_Y01);                 //Type
  Serial.write(application_default);      //Application
  Serial.write(main_version_number);      //Main Version
  Serial.write(secondary_version_number); //Secondary Version
  Serial.write(revision_number);          //Revision
  Serial.write(packet_tail);
}
void Packet_Error_Feedback(unsigned char CMD_reaction)
{
  Serial.write(packet_header);
  Serial.write(0x04);
  Serial.write(CMD_reaction);
  Serial.write(packet_tail);
}

//Initial Pose Task
void Initial_Pose_Setup(void)
{
  XYZrobot.readPose();
  XYZrobot.playSeq(DefaultInitial);
  while (XYZrobot.playing)
    XYZrobot.play();
}

//Action Task
void Action(int N)
{
  if (N == 0)
    MusicPlaying_wav_play("none");
  else if (N == 1)
  {
    MusicPlaying_wav_play(Music_1);
    if (ActionNo_1 != None)
      XYZrobot.playSeq(ActionNo_1);
  }
  else if (N == 2)
  {
    MusicPlaying_wav_play(Music_2);
    if (ActionNo_2 != None)
      XYZrobot.playSeq(ActionNo_2);
  }
  else if (N == 3)
  {
    MusicPlaying_wav_play(Music_3);
    if (ActionNo_3 != None)
      XYZrobot.playSeq(ActionNo_3);
  }
  else if (N == 4)
  {
    MusicPlaying_wav_play(Music_4);
    if (ActionNo_4 != None)
      XYZrobot.playSeq(ActionNo_4);
  }
  else if (N == 5)
  {
    MusicPlaying_wav_play(Music_5);
    if (ActionNo_5 != None)
      XYZrobot.playSeq(ActionNo_5);
  }
  else if (N == 6)
  {
    MusicPlaying_wav_play(Music_6);
    if (ActionNo_6 != None)
      XYZrobot.playSeq(ActionNo_6);
  }
  else if (N == 7)
  {
    MusicPlaying_wav_play(Music_7);
    if (ActionNo_7 != None)
      XYZrobot.playSeq(ActionNo_7);
  }
  else if (N == 8)
  {
    MusicPlaying_wav_play(Music_8);
    if (ActionNo_8 != None)
      XYZrobot.playSeq(ActionNo_8);
  }
  else if (N == 9)
  {
    MusicPlaying_wav_play(Music_9);
    if (ActionNo_9 != None)
      XYZrobot.playSeq(ActionNo_9);
  }
  else if (N == 10)
  {
    MusicPlaying_wav_play(Music_10);
    if (ActionNo_10 != None)
      XYZrobot.playSeq(ActionNo_10);
  }
  else if (N == 11)
  {
    MusicPlaying_wav_play(Music_11);
    if (ActionNo_11 != None)
      XYZrobot.playSeq(ActionNo_11);
  }
  else if (N == 12)
  {
    MusicPlaying_wav_play(Music_12);
    if (ActionNo_12 != None)
      XYZrobot.playSeq(ActionNo_12);
  }
  else if (N == 13)
  {
    MusicPlaying_wav_play(Music_13);
    if (ActionNo_13 != None)
      XYZrobot.playSeq(ActionNo_13);
  }
  else if (N == 14)
  {
    MusicPlaying_wav_play(Music_14);
    if (ActionNo_14 != None)
      XYZrobot.playSeq(ActionNo_14);
  }
  else if (N == 15)
  {
    MusicPlaying_wav_play(Music_15);
    if (ActionNo_15 != None)
      XYZrobot.playSeq(ActionNo_15);
  }
  else if (N == 16)
  {
    MusicPlaying_wav_play(Music_16);
    if (ActionNo_16 != None)
      XYZrobot.playSeq(ActionNo_16);
  }
  else if (N == 17)
  {
    MusicPlaying_wav_play(Music_17);
    if (ActionNo_17 != None)
      XYZrobot.playSeq(ActionNo_17);
  }
  else if (N == 18)
  {
    MusicPlaying_wav_play(Music_18);
    if (ActionNo_18 != None)
      XYZrobot.playSeq(ActionNo_18);
  }
  else if (N == 19)
  {
    MusicPlaying_wav_play(Music_19);
    if (ActionNo_19 != None)
      XYZrobot.playSeq(ActionNo_19);
  }
  else if (N == 20)
  {
    MusicPlaying_wav_play(Music_20);
    if (ActionNo_20 != None)
      XYZrobot.playSeq(ActionNo_20);
  }
  else if (N == 21)
  {
    MusicPlaying_wav_play(Music_21);
    if (ActionNo_21 != None)
      XYZrobot.playSeq(ActionNo_21);
  }
  else if (N == 22)
  {
    MusicPlaying_wav_play(Music_22);
    if (ActionNo_22 != None)
      XYZrobot.playSeq(ActionNo_22);
  }
  else if (N == 23)
  {
    MusicPlaying_wav_play(Music_23);
    if (ActionNo_23 != None)
      XYZrobot.playSeq(ActionNo_23);
  }
  else if (N == 24)
  {
    MusicPlaying_wav_play(Music_24);
    if (ActionNo_24 != None)
      XYZrobot.playSeq(ActionNo_24);
  }
  else if (N == 25)
  {
    MusicPlaying_wav_play(Music_25);
    if (ActionNo_25 != None)
      XYZrobot.playSeq(ActionNo_25);
  }
  else if (N == 26)
  {
    MusicPlaying_wav_play(Music_26);
    if (ActionNo_26 != None)
      XYZrobot.playSeq(ActionNo_26);
  }
  else if (N == 27)
  {
    MusicPlaying_wav_play(Music_27);
    if (ActionNo_27 != None)
      XYZrobot.playSeq(ActionNo_27);
  }
  else if (N == 28)
  {
    MusicPlaying_wav_play(Music_28);
    if (ActionNo_28 != None)
      XYZrobot.playSeq(ActionNo_28);
  }
  else if (N == 29)
  {
    MusicPlaying_wav_play(Music_29);
    if (ActionNo_29 != None)
      XYZrobot.playSeq(ActionNo_29);
  }
  else if (N == 30)
  {
    MusicPlaying_wav_play(Music_30);
    if (ActionNo_30 != None)
      XYZrobot.playSeq(ActionNo_30);
  }
  else if (N == 31)
  {
    MusicPlaying_wav_play(Music_31);
    if (ActionNo_31 != None)
      XYZrobot.playSeq(ActionNo_31);
  }
  else if (N == 32)
  {
    MusicPlaying_wav_play(Music_32);
    if (ActionNo_32 != None)
      XYZrobot.playSeq(ActionNo_32);
  }
  else if (N == 33)
  {
    MusicPlaying_wav_play(Music_33);
    if (ActionNo_33 != None)
      XYZrobot.playSeq(ActionNo_33);
  }
  else if (N == 34)
  {
    MusicPlaying_wav_play(Music_34);
    if (ActionNo_34 != None)
      XYZrobot.playSeq(ActionNo_34);
  }
  else if (N == 35)
  {
    MusicPlaying_wav_play(Music_35);
    if (ActionNo_35 != None)
      XYZrobot.playSeq(ActionNo_35);
  }
  else if (N == 36)
  {
    MusicPlaying_wav_play(Music_36);
    if (ActionNo_36 != None)
      XYZrobot.playSeq(ActionNo_36);
  }
  else if (N == 37)
  {
    MusicPlaying_wav_play(Music_37);
    if (ActionNo_37 != None)
      XYZrobot.playSeq(ActionNo_37);
  }
  else if (N == 38)
  {
    MusicPlaying_wav_play(Music_38);
    if (ActionNo_38 != None)
      XYZrobot.playSeq(ActionNo_38);
  }
  else if (N == 39)
  {
    MusicPlaying_wav_play(Music_39);
    if (ActionNo_39 != None)
      XYZrobot.playSeq(ActionNo_39);
  }
  else if (N == 40)
  {
    MusicPlaying_wav_play(Music_40);
    if (ActionNo_40 != None)
      XYZrobot.playSeq(ActionNo_40);
  }
  else if (N == 41)
  {
    MusicPlaying_wav_play(Music_41);
    if (ActionNo_41 != None)
      XYZrobot.playSeq(ActionNo_41);
  }
  else if (N == 42)
  {
    MusicPlaying_wav_play(Music_42);
    if (ActionNo_42 != None)
      XYZrobot.playSeq(ActionNo_42);
  }
  else if (N == 43)
  {
    MusicPlaying_wav_play(Music_43);
    if (ActionNo_43 != None)
      XYZrobot.playSeq(ActionNo_43);
  }
  else if (N == 44)
  {
    MusicPlaying_wav_play(Music_44);
    if (ActionNo_44 != None)
      XYZrobot.playSeq(ActionNo_44);
  }
  else if (N == 45)
  {
    MusicPlaying_wav_play(Music_45);
    if (ActionNo_45 != None)
      XYZrobot.playSeq(ActionNo_45);
  }
  else if (N == 46)
  {
    MusicPlaying_wav_play(Music_46);
    if (ActionNo_46 != None)
      XYZrobot.playSeq(ActionNo_46);
  }
  else if (N == 47)
  {
    MusicPlaying_wav_play(Music_47);
    if (ActionNo_47 != None)
      XYZrobot.playSeq(ActionNo_47);
  }
  else if (N == 48)
  {
    MusicPlaying_wav_play(Music_48);
    if (ActionNo_48 != None)
      XYZrobot.playSeq(ActionNo_48);
  }
  else if (N == 49)
  {
    MusicPlaying_wav_play(Music_49);
    if (ActionNo_49 != None)
      XYZrobot.playSeq(ActionNo_49);
  }
  else if (N == 50)
  {
    MusicPlaying_wav_play(Music_50);
    if (ActionNo_50 != None)
      XYZrobot.playSeq(ActionNo_50);
  }
  else if (N == 51)
  {
    MusicPlaying_wav_play(Music_51);
    if (ActionNo_51 != None)
      XYZrobot.playSeq(ActionNo_51);
  }
  else if (N == 52)
  {
    MusicPlaying_wav_play(Music_52);
    if (ActionNo_52 != None)
      XYZrobot.playSeq(ActionNo_52);
  }
  else if (N == 53)
  {
    MusicPlaying_wav_play(Music_53);
    if (ActionNo_53 != None)
      XYZrobot.playSeq(ActionNo_53);
  }
  else if (N == 54)
  {
    MusicPlaying_wav_play(Music_54);
    if (ActionNo_54 != None)
      XYZrobot.playSeq(ActionNo_54);
  }

  while ((XYZrobot.playing) && !(BT_Packet_Task()))
  {
    XYZrobot.play();
    if (Serial2.available() > 0)
    {
      if (BT_Packet_Task())
      {
        cb_BT();
        break;
      }
      else
      {
        joystick_status[0] = packet[1];
        joystick_status[1] = packet[2];
        joystick_status[2] = packet[3];
        joystick_status[3] = packet[4];
      }
    }
  }
  if (torque_release)
  {
    A1_16_TorqueOff(A1_16_Broadcast_ID);
    MusicPlaying_wav_stop();
    torque_release = false;
  }
}

//BT Reading Task
boolean BT_Packet_Task(void)
{
  //return torque_relase button status
  static int temp_packet[7] = {0};
  static char _i = 0;
  if (Serial2.available() >= 7)
  {
    if ((temp_packet[0] = Serial2.read()) == 0)
      ;
    else
    {
      find_header_BT();
      return false;
    }
    if ((temp_packet[1] = Serial2.read()) == 0)
    {
      find_header_BT();
      return false;
    }
    if ((temp_packet[2] = Serial2.read()) == 0)
    {
      find_header_BT();
      return false;
    }
    if ((temp_packet[3] = Serial2.read()) == 0)
    {
      find_header_BT();
      return false;
    }
    if ((temp_packet[4] = Serial2.read()) == 0)
    {
      find_header_BT();
      return false;
    }
    if ((temp_packet[5] = Serial2.read()) == 0)
    {
      find_header_BT();
      return false;
    }
    if ((temp_packet[6] = Serial2.read()) == 0)
    {
      find_header_BT();
      return false;
    }
    if (temp_packet[1] != 255 && temp_packet[2] != 1)
    {
      Serial2.write((temp_packet[6] & 0x00F0) >> 4);
    }

    for (_i = 0; _i < 7; _i++)
      packet[_i] = temp_packet[_i];
    BT_update = true;
    if ((packet[1] != 255 && packet[2] != 1) && ((packet[5] & 0x0010) >> 3))
    {
      torque_release = true;
      return true;
    }
    else if (packet[1] == 255 && packet[2] == 1 && packet[3] == 102)
    {
      torque_release = true;
      return true;
    }
    else
    {
      torque_release = false;
      return false;
    }
  }
  return false;
}

// BT G-sensor Data Feedback
void BT_Gsensor_Data(void)
{
  g_packet[0] = 0;
  if (getData(0x32) == 0xFF)
  {
    g_packet[1] = 0xFF;
    g_packet[7] = 0xC0;
  }
  else
  {
    g_packet[1] = getData(0x32) + (0x01);
    g_packet[7] = 0x80;
  }
  if (getData(0x33) == 0xFF)
  {
    g_packet[2] = 0xFF;
    g_packet[7] = g_packet[7] + (0x20);
  }
  else
  {
    g_packet[2] = getData(0x33) + (0x01);
  }
  if (getData(0x34) == 0xFF)
  {
    g_packet[3] = 0xFF;
    g_packet[7] = g_packet[7] + (0x10);
  }
  else
  {
    g_packet[3] = getData(0x34) + (0x01);
  }
  if (getData(0x35) == 0xFF)
  {
    g_packet[4] = 0xFF;
    g_packet[7] = g_packet[7] + (0x08);
  }
  else
  {
    g_packet[4] = getData(0x35) + (0x01);
  }
  if (getData(0x36) == 0xFF)
  {
    g_packet[5] = 0xFF;
    g_packet[7] = g_packet[7] + (0x04);
  }
  else
  {
    g_packet[5] = getData(0x36) + (0x01);
  }
  if (getData(0x37) == 0xFF)
  {
    g_packet[6] = 0xFF;
    g_packet[7] = g_packet[7] + (0x02);
  }
  else
  {
    g_packet[6] = getData(0x37) + (0x01);
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
  ir_msb = ir_rowdata >> 8;
  ir_lsb = ir_rowdata & 0xFF;
  ir_packet[0] = 0;
  if (ir_lsb == 0xFF)
  {
    ir_lsb = 0xFF;
    ir_packet[3] = 0x81;
  }
  else
  {
    ir_lsb = ir_lsb + 0x01;
    ir_packet[3] = 0x80;
  }
  ir_packet[1] = ir_msb + 0x01;
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
void BT_ActionEnding(int ActionNum)
{
  Serial2.write(0); // packet head
  delay(50);
  Serial2.write(ActionNum); // Action Number
  delay(50);
}

// BT FW Feedback
void BT_FW()
{
  Serial2.write(0xFF); // packet head
  delay(50);
  Serial2.write(model_Bolide); //Model
  delay(50);
  Serial2.write(type_Y01); //Type
  delay(50);
  Serial2.write(application_default); //Application
  delay(50);
  Serial2.write(main_version_number); //Main Version
  delay(50);
  Serial2.write(secondary_version_number); //Secondary Version
  delay(50);
  Serial2.write(revision_number); //Revison
  delay(50);
}

// Clean BT Buffer
void cb_BT(void)
{
  while ((Serial2.read()) != -1)
    ;
}
void find_header_BT(void)
{
  static int cb = 0x00;
  while (Serial2.available() > 0)
  {
    if (Serial2.peek() == 0)
      return;
    cb = Serial2.read();
  }
}

// Speaker function
void MusicPlaying_wav_play(char song_name[])
{
  Serial3.write(0);
  Serial3.print("P");
  Serial3.write(song_name); //set the filename of song : 0000 ~ 9999
}
void MusicPlaying_wav_stop()
{
  Serial3.write(0);
  Serial3.print("S0000");
}
void MusicPlaying_wav_volume(int volume)
{
  Serial3.write(0);
  Serial3.write('V');
  Serial3.write(volume); // volume : 0x01 ~ 0x7F
  Serial3.print("000");
}

// Buzzer function : play start music
void Start_Music(void)
{
  int _i = 0x00;
  for (_i = 0; _i < 7; _i++)
  {
    tone(BUZZER_PIN, pgm_read_word_near(&start_music_frq[_i]));
    delay(250);
    noTone(BUZZER_PIN);
  }
}

// Button function
void BUTTON_Task(void)
{
  static unsigned char button_timer = 0x00;
  static int key = 0x00, last_key = 0x00;
  key = !digitalRead(BUTTON1_PIN) + ((!digitalRead(BUTTON2_PIN)) << 1) + ((!digitalRead(BUTTON3_PIN)) << 2) + ((!digitalRead(BUTTON4_PIN)) << 3);
  if (key != last_key)
    button_timer++;
  else
    button_timer = 0;
  if (button_timer > 20)
  {
    button_timer = 0;
    last_key = key;
    if (key != 0)
    {
      LED_Task(2);
      if (last_key == key_mask_button1)
        Action(RB_1);
      else if (last_key == key_mask_button2)
        Action(RB_2);
      else if (last_key == key_mask_button3)
        Action(RB_3);
      else if (last_key == key_mask_button4)
        Action(RB_4);
      LED_Task(0);
    }
  }
}

// LED function
void LED_Task(char mode)
{
  if (mode != 0)
  {
    TCNT3 = -1;
    _enable_timer3();
    LED_mode = mode;
  }
  else
  {
    EYE_LED_OFF;
    _disable_timer3();
    LED_mode = 0;
  }
}

ISR(TIMER3_OVF_vect)
{
  static int R = 0, G = 0, B = 0;
  static int _R = 41, _G = 41, _B = 41;
  static boolean blink_LED = true;
  if (LED_mode == 1)
  {
    if (blink_LED)
      EYE_LED_BLE;
    else
      EYE_LED_GRN;
    blink_LED = !blink_LED;
    _reset_timer3(4500);
  }
  else if (LED_mode == 2)
  {
    if (R < 40)
    {
      R++;
      OCR5A = pgm_read_word_near(&log_light_40[R]);
    }
    else if (_R > 0)
    {
      _R--;
      OCR5A = pgm_read_word_near(&log_light_40[_R]);
    }
    else if (G < 40)
    {
      G++;
      OCR5B = pgm_read_word_near(&log_light_40[G]);
      EYE_LED_BLE;
    }
    else if (_G > 0)
    {
      _G--;
      OCR5B = pgm_read_word_near(&log_light_40[_G]);
    }
    else if (B < 40)
    {
      B++;
      OCR5C = pgm_read_word_near(&log_light_40[B]);
      EYE_LED_GRN;
    }
    else if (_B > 0)
    {
      _B--;
      OCR5C = pgm_read_word_near(&log_light_40[_B]);
    }
    else
    {
      R = 0;
      G = 0;
      B = 0;
      _R = 41;
      _G = 41;
      _B = 41;
    }
    _reset_timer3(200);
  }
  else if (LED_mode == 3)
  {
    if (R < 40)
    {
      R++;
      OCR5A = pgm_read_word_near(&log_light_40[R]);
    }
    else if (_R > 0)
    {
      _R--;
      OCR5A = pgm_read_word_near(&log_light_40[_R]);
    }
    else if (G < 40)
    {
      G++;
      OCR5B = pgm_read_word_near(&log_light_40[G]);
    }
    else if (_G > 0)
    {
      _G--;
      OCR5B = pgm_read_word_near(&log_light_40[_G]);
    }
    else if (B < 40)
    {
      B++;
      OCR5C = pgm_read_word_near(&log_light_40[B]);
    }
    else if (_B > 0)
    {
      _B--;
      OCR5C = pgm_read_word_near(&log_light_40[_B]);
    }
    else
    {
      R = 0;
      G = 0;
      B = 0;
      _R = 41;
      _G = 41;
      _B = 41;
    }
    _reset_timer3(200);
  }
}

// Power Detection function
void Power_Detection_Task(void)
{
  static double PWR_Voltage;
  PWR_Voltage = analogRead(PWRDET_PIN) * 0.0124;
  if (PWR_Voltage < Power_Voltage_Alarm)
    tone(BUZZER_PIN, 1000);
}
ISR(TIMER4_OVF_vect)
{
  Power_Detection_Task();
  packet_timeout_status = true;
  _reset_timer4(timeout_limit);
}
