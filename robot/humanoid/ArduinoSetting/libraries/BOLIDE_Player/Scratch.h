
/*==== SCRATCH Constant ====*/
#define PACKET_HEADER_SC				0xFF
#define PACKET_MIDTERM_SC				0x55
#define PACKET_TAIL_SC					0xFE

#define STATMENT_HEADER1_SC				0x00
#define STATMENT_HEADER2_SC				0x01
#define STATMENT_DATA_SC				0x02

#define CMD_DC_CONTROL_SC				0x01
#define CMD_DC_SPEED_SC					0x02
#define CMD_DC_SPEED_RE_SC				0x52
#define CMD_RC_CONTROL_SC				0x03
#define CMD_LINE_CAL_SC					0x04
#define CMD_LINE_RAW_DATA_SC			0x05
#define CMD_LINE_RAW_DATA_RE_SC			0x55
#define CMD_FORT_POS_SC                         0x06
#define CMD_LINE_CAL_DATA_RE_SC			0x56
#define CMD_LED_SC				0x07
#define CMD_IR_DATA_SC					0x08
#define CMD_IR_DATA_RE_SC				0x58
#define CMD_LSA_SC						0x09

#define CMD2_LSA_VOLUME_SC				0x01
#define CMD2_LSA_PLAY_SC				0x02
#define CMD2_LSA_STOP_SC				0x03
#define CMD2_LSA_PAUSE_SC				0x04
#define CMD2_LSA_CONTINUE_SC			0x05


#define XYZMOTION_WALK    	1
#define XYZMOTION_POSE    	2
#define READ_IR           	4
#define READ_BUTTON       	5
#define CHEST_LED         	6
#define EYE_LED           	7
#define READ_GSENSROR     	8
#define READ_FALL         	9
#define XYZMOTION_GETUP   	10
#define PLAYTONE          	11
#define MOTORPOSITION     	12
#define MOTORSPEED        	13
#define READ_MOTOR_POS    	14
#define MOTOR_TORQUE_OFF	15
#define CHEST_PAINTER		16

// Scratch
boolean Scratch_Packet_Task(unsigned char *_packet, boolean _bIsBT = false);
void SCRATCH_CMD_Task(unsigned char *packet);

