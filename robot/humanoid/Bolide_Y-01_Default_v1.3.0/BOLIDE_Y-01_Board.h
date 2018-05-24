#ifndef BOLIDE_BOARD_H
#define BOLIDE_BOARD_H

#include <avr/pgmspace.h>

//definition of struct
typedef struct{
    unsigned char pose;         // index of pose to transition to 
    int time;                   // time for transition
} sp_trans_t;

typedef struct{
    char RCU_LJX, RCU_LJY, RCU_RJX, RCU_RJY;
    boolean BTN_L3, BTN_R3;
} Joystick;

extern uint16_t ROBOT_ZERO_POINT[19];

// EEPROM ADDR 
#define EEP_ADDR_RESERVERD       0    // [0:9]
#define EEP_ADDR_SN             10    // [10:23]
#define EEP_ADDR_FLAG           30    // [30]
#define EEP_ADDR_ZEROPOINT      31    // [31:66]

//Pin Definition
//Digital output Pin
#define BUZZER_PIN 				4
#define LED_BLUE_PIN 			26
#define LED_GREEN_PIN 			25
#define LSA_LED_BLUE_PIN 		44
#define LSA_LED_GREEN_PIN 		45
#define LSA_LED_RED_PIN 		46

enum {
  LED_OFF = 0,
  BLUE_LED_ON = 1,
  GREEN_LED_ON = 2,
  RED_LED_ON = 3,
  WHITE_LED_ON = 4,
  BLE_LED_BREATH = 5,
  GRN_LED_BREATH = 6,
  RED_LED_BREATH = 7,
  ONE_LED_ON = 8,
  LED_SCRATCH = 9,
};

//Digital input Pin
#define BUTTON1_PIN 			32
#define BUTTON2_PIN 			33
#define BUTTON3_PIN				34
#define BUTTON4_PIN				35
//Analog input pin
#define PWRDET_PIN				0
#define DISTANCE_SENSOR_PIN 	3

//Eye LED function definition
#define EYE_LED_BLE digitalWrite(LED_BLUE_PIN, HIGH), digitalWrite(LED_GREEN_PIN, LOW)
#define EYE_LED_GRN digitalWrite(LED_BLUE_PIN, LOW), digitalWrite(LED_GREEN_PIN, HIGH)
#define EYE_LED_OFF digitalWrite(LED_BLUE_PIN, LOW), digitalWrite(LED_GREEN_PIN, LOW)

#define EYE_MODE_OFF      0
#define EYE_MODE_GREEN    1
#define EYE_MODE_BLUE     2

//Timer function definition
#define _enable_timer3() TIMSK3 |= _BV(TOIE3)
#define _disable_timer3() TIMSK3 &= ~_BV(TOIE3)
#define _enable_timer4() TIMSK4 |= _BV(TOIE4)
#define _disable_timer4() TIMSK4 &= ~_BV(TOIE4)

#define _reset_timer3(t) TCNT3 = -t
#define _reset_timer4(t) { TCNT4 = -t; packet_timeout_status = false; }
#define timeout_limit				15625      // Ticks for 1 sec @16 MHz,prescale=1024
#define timer_10ms          164        // 10ms * (32.768kHz/2) 

static boolean packet_timeout_status = false;

//Button
#define key_mask_button1			0x01
#define key_mask_button2			0x02
#define key_mask_button3			0x04
#define key_mask_button4			0x08

//RCU mask
#define RCU_mask_release			0x0010
#define RCU_mask_BT					0x0020
#define RCU_mask_power				0x0040
#define RCU_mask_L1					0x0001
#define RCU_mask_L2					0x0002
#define RCU_mask_L3					0x0004
#define RCU_mask_R1					0x0001
#define RCU_mask_R2					0x0002
#define RCU_mask_R3					0x0004
#define RCU_packet_header_address	0x00
#define RCU_packet_LJ_X_address		0x01
#define RCU_packet_LJ_Y_address		0x02
#define RCU_packet_RJ_X_address		0x03
#define RCU_packet_RJ_Y_address		0x04

// Status_Detail Bits
#define  MOTOR_MOVING_MASK  		0x10	// Servo_On/Wheel_On/In_Position/Omega_Goal
#define  IN_POSITION_MASK   		0x20	// Servo_On/!Wheel_On/Inposition_Margin
#define  TORQUE_ON_MASK				0x40	// Torque_On/Servo_On
#define  MOTOR_BRAKE_MASK			0x80	// Brake On

// Status_Error Bits
#define  POT_ERROR_MASK				0x01	// Invalid_POT (Position Control Mode Only)
#define  VOLTAGE_ERROR_MASK			0x02	// Over Voltage Limits
#define  TEMPERATURE_ERROR_MASK		0x04	// Over Temperature
#define  OVERLOAD_ERROR_MASK		0x08	// Over IBus Current
#define  DRIVER_ERROR_MASK			0x10	// to be ADDED
#define  CHECKSUM_ERROR_MASK		0x20	// Requested Packet Checksum Error
#define  PACKET_ERROR_MASK			0x40	// Request Packet Header/Data/Timeout Error
#define  RXFIFO_ERROR_MASK			0x80	// Requested Packet FIFO Error

//Power Detection
#define  Power_Voltage_Alarm		734		// 9.1v / 0.0124   
//================================================================================================================================
//=== LED Frequency ===
const PROGMEM uint16_t log_light_40[41] = {1,
                               1, 1, 2, 2, 2, 3, 3, 4, 5, 6,
                               7, 8, 10, 11, 13, 16, 19, 23, 27, 32,
                               38, 45, 54, 64, 76, 90, 108, 128, 152, 181,
                               215, 256, 304, 362, 430, 512, 608, 723, 860, 1023};
//=== Music Frequency ===
const PROGMEM uint16_t start_music_frq[7] = {262, 262, 392, 392, 440, 440, 392};
const PROGMEM uint16_t obstacle_alarm_frq[3] = {262, 550, 392};

// Timer 
class Constant_Timer
{  
public:
    boolean Timer_Task(unsigned long _time_ms);
    boolean TimerEvent(void);
    void Reset(void);
    void SetTimer(unsigned long _time);
private:
    unsigned long _last_time;
    unsigned long time_tick;
};

// Robot Function
int g_sensor_read_reg(int reg);    // G Sensor
void BT_Flush(void);
void USB_Flush(void);
void Action(int N, boolean bFallAction = false);
void XYZ_Play_Action(const transition_t *addr);

// Global Variable
extern BOLIDE_Player XYZrobot;
extern Constant_Timer Timer_Serial;

#endif
