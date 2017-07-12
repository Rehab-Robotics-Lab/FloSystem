/*
  A1-16.h - Modified for XYZrobot ATmega 1280 control board.
  Copyright (c) 2015 Wei-Shun You. XYZprinting Inc.  All right reserved.
*/
#include <Arduino.h>
#include "A1_16.h"

unsigned char packet_received[BUFFER_SIZE];

unsigned int checksum_1;
unsigned int checksum_2;

void A1_16_Ini(unsigned long baud){
    DDRD &= ~_BV(DDD2);			//set the RXD input
	PORTD |= _BV(PORTD2);		//pull-high the RXD pinout
	Serial1.begin(baud,SERIAL_8N1);
}

void A1_16_SetPosition(unsigned char _pID, unsigned char _CMD,  unsigned char _playtime, unsigned int _position){
  static unsigned int _data[5];
  static int _i = 0;
  
  if(_CMD == CMD_S_JOG){
    _data[0] = _playtime;
    _data[1] = _position&0xff;
    _data[2] = (_position&0xff00)>>8;
    _data[3] = 0;					//set:0(position control), 1(speed control), 2(torque off), 3(position servo on)
    _data[4] = _pID;
  }
  else if (_CMD == CMD_I_JOG){
    _data[0] = _position&0xff;
    _data[1] = (_position&0xff00)>>8;
    _data[2] = 0;					//set:0(position control), 1(speed control), 2(torque off), 3(position servo on)
    _data[3] = _pID;
    _data[4] = _playtime;
  }
  checksum_1 = (0x0c)^_pID^_CMD;		//package_size^pID^CMD
  for(_i = 0;_i < 5;_i++) checksum_1 ^= _data[_i];
  checksum_1 &= 0xfe;
  checksum_2 = (~checksum_1)&0xfe;
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0x0c);				//package size
  Serial1.write(_pID);
  Serial1.write(_CMD);
  Serial1.write(checksum_1);
  Serial1.write(checksum_2);
  for(_i = 0;_i < 5;_i++) Serial1.write(_data[_i]);  
}

void A1_16_SetSpeed(unsigned char _pID, unsigned char _playtime, int _speed){
  static unsigned int _data[5];
  static int _i = 0;
  _data[0] = _speed&0xff;
  _data[1] = (_speed&0xff00)>>8;
  _data[2] = 1;					//set:0(position control), 1(speed control), 2(torque off), 3(position servo on)
  _data[3] = _pID;
  _data[4] = _playtime;
  checksum_1 = (0x0c)^_pID^CMD_I_JOG;		//package_size^pID^CMD
  for(_i = 0;_i < 5;_i++) checksum_1 ^= _data[_i];
  checksum_1 &= 0xfe;
  checksum_2 = (~checksum_1)&0xfe;
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0x0c);				//package size
  Serial1.write(_pID);
  Serial1.write(CMD_I_JOG);
  Serial1.write(checksum_1);
  Serial1.write(checksum_2);
  for(_i = 0;_i < 5;_i++) Serial1.write(_data[_i]);
}

void A1_16_TorqueOff(unsigned char _pID){
  static unsigned int _data[5];
  static int _i = 0;
  _data[0] = 0;
  _data[1] = 512&0xff;
  _data[2] = (512&0xff00)>>8;
  _data[3] = 2;					//set:0(position control), 1(speed control), 2(torque off), 3(position servo on)
  _data[4] = _pID;
  checksum_1 = (0x0c)^_pID^CMD_S_JOG;		//package_size^pID^CMD
  for(_i = 0;_i < 5;_i++) checksum_1 ^= _data[_i];
  checksum_1 &= 0xfe;
  checksum_2 = (~checksum_1)&0xfe;
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0x0c);				//package size
  Serial1.write(_pID);
  Serial1.write(CMD_S_JOG);
  Serial1.write(checksum_1);
  Serial1.write(checksum_2);
  for(_i = 0;_i < 5;_i++) Serial1.write(_data[_i]);  
}

int A1_16_ReadData(unsigned char _pID, unsigned char _CMD, unsigned char _addr_start, unsigned char _data_length){
  while(Serial1.read() != -1);
  checksum_1 = (9^_pID^_CMD^_addr_start^_data_length)&0xfe;
  checksum_2 = (~checksum_1)&0xfe;
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0x09);						//packet size
  Serial1.write(_pID);
  Serial1.write(_CMD);
  Serial1.write(checksum_1);
  Serial1.write(checksum_2);
  Serial1.write(_addr_start);
  Serial1.write(_data_length);			//length of data
  int value = A1_16_ReadPacket(_data_length);
  return value;
}

int A1_16_ReadPacket(unsigned char _data_length){
  unsigned char packet_length = 11 + _data_length;
  unsigned char packet_pointer = 0;
  unsigned char _i;
  unsigned int timeout_counter;
  unsigned char header_check = 0;
  
  while (packet_pointer < packet_length){
	timeout_counter = 0;
	while(Serial1.available() <= 0){
		timeout_counter++;
		if(timeout_counter > 1000L) return -1;
	}
	packet_received[packet_pointer] = Serial1.read();
	if((packet_received[packet_pointer] == 0xff) && (header_check == 0)){
		packet_pointer++;
		header_check = 1;
	}
	else if((packet_received[packet_pointer] == 0xff) && (header_check == 1)){
		packet_pointer++;
		header_check = 2;
	}
	else if(header_check == 2){
		packet_pointer++;
	}
  }
  
  checksum_1 = packet_received[2]^packet_received[3]^packet_received[4];
  for(_i = 7;_i < packet_length;_i++) checksum_1 ^= packet_received[_i];
  checksum_1 &= 0xfe;
  checksum_2 = (~checksum_1)&0xfe;
  if(checksum_1 != packet_received[5]) return -2;
  if(checksum_2 != packet_received[6]) return -3;
  if(_data_length != 1) return (packet_received[11] + (packet_received[12]<<8));
  else return packet_received[11];
}

void A1_16_WriteData(unsigned char _pID, unsigned char _CMD, unsigned char _addr_start, char _data_write){
  checksum_1 = (10^_pID^_CMD^_addr_start^0x01^_data_write)&0xfe;
  checksum_2 = (~checksum_1)&0xfe;
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(10);			//package size
  Serial1.write(_pID);
  Serial1.write(_CMD);
  Serial1.write(checksum_1);
  Serial1.write(checksum_2);
  Serial1.write(_addr_start);
  Serial1.write(0x01);			//length of data
  Serial1.write(_data_write);
}

void A1_16_WriteData2(unsigned char _pID, unsigned char _CMD, unsigned char _addr_start, int _data_write){
  unsigned char BYTE_1 = _data_write&0xff;
  unsigned char BYTE_2 = (_data_write&0xff00)>>8;
  checksum_1 = (11^_pID^_CMD^_addr_start^0x02^BYTE_1^BYTE_2)&0xfe;
  checksum_2 = (~checksum_1)&0xfe;
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(11);			//package size
  Serial1.write(_pID);
  Serial1.write(_CMD);
  Serial1.write(checksum_1);
  Serial1.write(checksum_2);
  Serial1.write(_addr_start);
  Serial1.write(0x02);			//length of data
  Serial1.write(BYTE_1);
  Serial1.write(BYTE_2);
}

void A1_16_Basic(unsigned char _pID, unsigned char _CMD){
  checksum_1 = (7^_pID^_CMD)&0xfe;
  checksum_2 = (~checksum_1)&0xfe;
  Serial1.write(0xff);          //header
  Serial1.write(0xff);          //header
  Serial1.write(7);				//package size
  Serial1.write(_pID);
  Serial1.write(_CMD);
  Serial1.write(checksum_1);
  Serial1.write(checksum_2);
}
