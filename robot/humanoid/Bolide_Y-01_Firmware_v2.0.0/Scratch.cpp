/*
  Scratch Function
 
  by: Steven Chi
  last updated: 2017/01/24
*/

#include <Arduino.h>
#include "Y01_BOLIDE_Player.h"
#include "Y01_A1_16.h"
#include "Scratch.h"
#include "BOLIDE_Y-01_Board.h"
#include "Y-01_USER_MOTION.h"
//#include "BOLIDE_Y-01_Board.h"

/*== USB Receive Variables ==*/
static unsigned char USB_packet[10] = {0};
static uint8_t USB_pN = 0;
static uint8_t USB_statement = 0;
static uint8_t SCRATCH_status = 0x00;
union{
	byte byteVal[4];
	float floatVal;
	long longVal;
} val;
union{
	byte byteVal[8];
	double doubleVal;
} valDouble;
union{
	byte byteVal[2];
	short shortVal;
} valShort;

boolean bIsBluetooth = false;

/*===== Local Function ======*/
// USB
//void USB_Buffer_Flush(void);
// BT 
//void BT_Flush(void);
void Serial_Flush(void);
void BT_Packet_Header_Finder(void);
// Scratch
void Scratch_Run_Module(unsigned char *_packet);
void scratch_read_sensor(unsigned char *_packet);

void Scratch_Wrtie(unsigned char _c);
void Scratch_WriteHead(void);
void Scratch_WriteEnd(void);
void Scratch_CallOK(void);
void Scratch_SendByte(uint8_t _c);
void Scratch_SendShort(short _value);
void Scratch_SendFloat(float _value);

short bytes_to_short(unsigned char *buffer, int idx);
float bytes_to_float(unsigned char *buffer, int idx);

/*=========================================================*/

void SCRATCH_CMD_Task(unsigned char *_packet)
{
  unsigned char idx = _packet[3];
  unsigned char action = _packet[4];
  switch(action) 
  {
      case 0x01:      // GET
          //Scratch_WriteHead();
          //Scratch_Wrtie(idx);
          //delay(100);	// delay for BT Module
          scratch_read_sensor(_packet);
          //Scratch_WriteEnd();
          break;
      case 0x02:    // RUN
          Scratch_Run_Module(_packet);
          Scratch_CallOK();
          break;
      case 0x04:    // RESTART
          break;
      case 0x05:    // START
          Scratch_CallOK();
          break;
  }  
}

void Scratch_Run_Module(unsigned char *_packet)
{
    //0xff 0x55 0x6 0x0 0x1 0xa 0x9 0x0 0x0 0xa
    unsigned char device = _packet[5];
    static boolean last_motion_walk = false;
    
    if(device != XYZMOTION_WALK) last_motion_walk = false;
    
    switch(device) 
    {
    case XYZMOTION_WALK: {
        int motion = _packet[6];

        if(motion == 1) {Action(A_WalkForward); Action(A_DefaultInitial);}
        else if(motion == 2) {Action(A_WalkBackward); Action(A_DefaultInitial);}
        else if(motion == 3) {Action(A_WalkLeftward);}
        else if(motion == 4) {Action(A_WalkRightward);}
        else if(motion == 5) {Action(A_TurnLeft); Action(A_DefaultInitial);}
        else if(motion == 6) {Action(A_TurnRight); Action(A_DefaultInitial);}
        last_motion_walk = true;
        break;
    }
    case XYZMOTION_POSE: {
        int motion = _packet[6];
        if(motion == 1) Action(A_SaluteLefthand);
        else if(motion == 2) Action(A_SaluteRighthand);
        else if(motion == 3) Action(A_PushUp);
        else if(motion == 4) Action(A_RightHandPushUp);
        else if(motion == 5) Action(A_LeftHandPushUp);
        else if(motion == 6) Action(A_Bow);
        else if(motion == 7) Action(A_WaveHand);
        else if(motion == 8) Action(A_WaveDance);
        else if(motion == 9) Action(A_DefaultInitial);
        break;
    }
    case XYZMOTION_GETUP: {
        int motion = _packet[6];        
        if(motion == 1) Action(A_FrontGetUp);
        else if(motion == 2) Action(A_BackGetUp);
        break;
    }
    case MOTORPOSITION: {
        int motorID = _packet[6];
        int motorPos = bytes_to_short(_packet, 7);
        int motorTime = _packet[9];
        SetPositionI_JOG(motorID, motorTime, motorPos);
        break;
    }
    case MOTORSPEED: {
        int motorID = _packet[6];
        int motorDir = _packet[7];
        int motorSpeed = bytes_to_short(_packet, 8);
        if(motorDir == 2)  motorSpeed = motorSpeed * 1;
        else if(motorDir == 1)  motorSpeed = motorSpeed * -1;
        A1_16_SetSpeed(motorID, 0, motorSpeed);
        break;
    }
    case MOTOR_TORQUE_OFF: {
        int motorID = _packet[6];
        A1_16_TorqueOff(motorID);
        break;
    }
    case CHEST_LED: {
        int pin = _packet[6];
        int led_on = _packet[7];
        if(led_on == 1) {
            if(pin == 1) {
                digitalWrite(LSA_LED_RED_PIN, HIGH);
                digitalWrite(LSA_LED_GREEN_PIN, LOW);
                digitalWrite(LSA_LED_BLUE_PIN, LOW);
            }
            else if(pin == 2) {
                digitalWrite(LSA_LED_RED_PIN, LOW);
                digitalWrite(LSA_LED_GREEN_PIN, HIGH);
                digitalWrite(LSA_LED_BLUE_PIN, LOW);
            }
            else if(pin == 3) { 
                digitalWrite(LSA_LED_RED_PIN, LOW);
                digitalWrite(LSA_LED_GREEN_PIN, LOW);
                digitalWrite(LSA_LED_BLUE_PIN, HIGH);
            }
        }
        else {
        	if(pin == 1) digitalWrite(LSA_LED_RED_PIN, LOW);
        	else if(pin == 2) digitalWrite(LSA_LED_GREEN_PIN, LOW);
        	else if(pin == 3) digitalWrite(LSA_LED_BLUE_PIN, LOW);
        }
        break;
    }
    case CHEST_PAINTER: {
    	analogWrite(LSA_LED_RED_PIN, _packet[6]);	// Red
    	analogWrite(LSA_LED_GREEN_PIN, _packet[7]);	// Green
    	analogWrite(LSA_LED_BLUE_PIN, _packet[8]);	// Blue
    	break;
    }
    case EYE_LED: {
        int pin = _packet[6];
        int led_on = _packet[7];
        if(led_on == 1) {
            if(pin == 2) {
                digitalWrite(LED_GREEN_PIN,HIGH);
                digitalWrite(LED_BLUE_PIN,LOW);
            }
            else if(pin == 3) {
                digitalWrite(LED_GREEN_PIN,LOW);
                digitalWrite(LED_BLUE_PIN,HIGH);
            }
        }
        else if(led_on == 0) {
            if(pin == 2) {
                digitalWrite(LED_GREEN_PIN,LOW); 
            }
            else if(pin == 3) {
                digitalWrite(LED_BLUE_PIN,LOW); 
            }       
        }
        break;
    }
    case PLAYTONE: {
        int hz = bytes_to_short(_packet, 7);
        int ms = bytes_to_short(_packet, 9);
        if(ms > 0) {
            tone(BUZZER_PIN,hz);
            delay(ms);
            noTone(BUZZER_PIN);    
        }
        else {
            noTone(BUZZER_PIN);  
        }
        break;
    }
    default:
        break;
    }
}

/**************************************************
    ff 55 len idx action device port slot data a
    0  1  2   3   4      5      6    7    8
***************************************************/
void scratch_read_sensor(unsigned char *_packet)
{
  float value=0.0;
//  int port,slot,pin;
  unsigned char device = _packet[5];
  unsigned char pin = _packet[6];
  switch(device) {
   case READ_IR: {     
     int distance = (6787/(analogRead(DISTANCE_SENSOR_PIN)-3))-4;
     Scratch_SendFloat((float)distance);
     break; 
   }
   case  READ_BUTTON: {
     pinMode(pin,INPUT);
     int btn = digitalRead(pin);
     if(btn == 1) { btn = 0; }
     else if(btn == 0) { btn = 1; }
     Scratch_SendFloat(btn);
     break;
   }
   case READ_GSENSROR: {   
     if(pin == 1) {  
        float ax = ((g_sensor_read_reg(0x33) << 8)  + g_sensor_read_reg(0x32)) / 256.0;
        Scratch_SendFloat((float)ax);
     }
     else if(pin == 2) {
        float ay = ((g_sensor_read_reg(0x35) << 8)  + g_sensor_read_reg(0x34)) / 256.0;
        Scratch_SendFloat((float)ay);
     }
     else if(pin == 3) {
        float az = ((g_sensor_read_reg(0x37) << 8)  + g_sensor_read_reg(0x36)) / 256.0;   
        Scratch_SendFloat((float)az);  
     }     
     else {
        Scratch_SendFloat(0);
     }
     break; 
   }
   case READ_FALL: {   
     if(pin == 1) {  
        float az = ((g_sensor_read_reg(0x37) << 8)  + g_sensor_read_reg(0x36)) / 256.0;  
        if(az <= -0.8) { Scratch_SendFloat(1); }
        else { Scratch_SendFloat(0); }
     }
     else if(pin == 2) {
        float az = ((g_sensor_read_reg(0x37) << 8)  + g_sensor_read_reg(0x36)) / 256.0;  
        if(az >= 0.8) { Scratch_SendFloat(1); }
        else { Scratch_SendFloat(0); }
     }
     else {
          Scratch_SendFloat(0);       
     }
   }
   break; 
   case READ_MOTOR_POS: {     
     int motor_pos = ReadPosition(pin);
     Scratch_SendFloat((float)motor_pos);
   }
   break; 
  } 
}

boolean Scratch_Packet_Task(unsigned char *_packet, boolean _bIsBT)
{
    static unsigned char temp_packet[7] = {0};
    boolean scratch_mode = false;
    bIsBluetooth = _bIsBT;
    
    HardwareSerial &mSerial = Serial;
    if(_bIsBT) { mSerial = Serial2; }
     
    Timer_Serial.Reset();
    while(mSerial.available() < 5) { if(Timer_Serial.Timer_Task(100)) {Serial_Flush();return false;} }

    if(_bIsBT) {// BT
        if((temp_packet[0] = mSerial.read()) != 0xFF) {Serial_Flush(); return false;}
    }
    else {      // USB
        temp_packet[0] = 0xFF;
    }
    
    if((temp_packet[1] = mSerial.read()) != 0x55) {Serial_Flush(); return false;}
    temp_packet[2] = mSerial.read();
    int len = temp_packet[2];

    for(int i = 3; i < len+3; i++) { 
        Timer_Serial.Reset();
        while(mSerial.available() == 0) { if(Timer_Serial.Timer_Task(30)) {Serial_Flush(); return false;} }    // wait for BT transfer
        temp_packet[i] = mSerial.read();
    }
    //Serial_Flush();
    for(int i = 0; i < len+3; i++) { _packet[i] = temp_packet[i]; }        
    scratch_mode = true;

    return scratch_mode;
}
void Serial_Flush(void) {
    if(bIsBluetooth)  BT_Flush();
    else USB_Flush();
//    while(mSerial.available() > 0) { mSerial.read(); };  // Clean BT Buffer
}
/*
void USB_Buffer_Flush(void) {    	
    while(Serial.available() > 0) { Serial.read(); };  // Clear USB Buffer
}
void BT_Flush(void) {    	
    while(Serial2.available() > 0) { Serial2.read(); };  // Clean BT Buffer
}*/
void BT_Packet_Header_Finder(void) 
{
    static int _cb = 0x00;
    while(Serial2.available() > 0) {
        if((Serial2.peek() == 0) || (Serial2.peek() == 0xFF)) return;
        _cb = Serial2.read();
    }
}

void Scratch_Wrtie(unsigned char _c) 
{
    if(bIsBluetooth) { Serial2.write(_c); }
    else { Serial.write(_c); } 
}
void Scratch_WriteHead(void)
{
    Scratch_Wrtie(0xFF);
    Scratch_Wrtie(0x55);
}
void Scratch_WriteEnd(void)
{
    Scratch_Wrtie(0x0D);
    Scratch_Wrtie(0x0A);
}
void Scratch_CallOK(void) 
{
    Scratch_Wrtie(0xFF);
    Scratch_Wrtie(0x55);
    Scratch_Wrtie(0x0D);
    Scratch_Wrtie(0x0A);
}
void Scratch_SendByte(uint8_t _c) 
{
    Scratch_Wrtie(0xFF);  // header_1
    Scratch_Wrtie(0x55);  // header_2
    Scratch_Wrtie(0x00);  // extId
    Scratch_Wrtie(0x01);  // return type (byte)
    Scratch_Wrtie(_c);    // val
    Scratch_Wrtie(0x0D);  // end_1
    Scratch_Wrtie(0x0A);  // end_2
}
void Scratch_SendShort(short _value) 
{
    valShort.shortVal = _value;
    Scratch_Wrtie(0xFF);  // header_1
    Scratch_Wrtie(0x55);  // header_2
    Scratch_Wrtie(0x00);  // extId
    Scratch_Wrtie(0x03);  // return type (short)
    Scratch_Wrtie(valShort.byteVal[0]);  // val_l
    Scratch_Wrtie(valShort.byteVal[1]);  // val_h
    delay(60);
    Scratch_Wrtie(0x0D);  // end_1
    Scratch_Wrtie(0x0A);  // end_2
}
void Scratch_SendFloat(float _value)
{ 
    val.floatVal = _value;
    Scratch_Wrtie(0xFF);  // header_1
    Scratch_Wrtie(0x55);  // header_2
    Scratch_Wrtie(0x00);  // extId
    Scratch_Wrtie(0x02);  // return type (float)
    Scratch_Wrtie(val.byteVal[0]);  // val_0
    Scratch_Wrtie(val.byteVal[1]);  // val_1
    delay(60);
    Scratch_Wrtie(val.byteVal[2]);  // val_2
    Scratch_Wrtie(val.byteVal[3]);  // val_3   
    Scratch_Wrtie(0x0D);  // end_1
    Scratch_Wrtie(0x0A);  // end_2
}

short bytes_to_short(unsigned char *buffer, int idx)
{
  valShort.byteVal[0] = buffer[idx];
  valShort.byteVal[1] = buffer[idx+1];
  return valShort.shortVal; 
}
float bytes_to_float(unsigned char *buffer, int idx)
{
  val.byteVal[0] = buffer[idx];
  val.byteVal[1] = buffer[idx+1];
  val.byteVal[2] = buffer[idx+2];
  val.byteVal[3] = buffer[idx+3];
  return val.floatVal;
}
