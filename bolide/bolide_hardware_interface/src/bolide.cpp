#include "BolideInterface.h"

BolideInterface::BolideInterface(std::string port){
    serial_con_ = new serial::Serial(port, 115200, serial::Timeout::simpleTimeout(200));
}

BolideInterface::~BolideInterface(void){
    delete serial_con_;
}

BolideInterface::ReadData(int command){
    serial_con_->flushInput();
    uint8_t byte_command[4] = {0xFF,0x04,command,0xfe};
    serial_con_->write(byte_command, 4)
}
    # print('new packet for: {}'.format(type))
    ser.flushInput()
    ser.write(bytearray([0xFF,0x04,commands[com],0xfe]))
    ret = ser.read(40)
    if len(ret) != 40:
        print('wrong length returned')
        return
    header = ord(ret[0])
    if header != 0xFF:
        print('first byte read did not match header: {}'.format(header))
        return
    len_bit = ord(ret[1])
    if len_bit != 40:
        print('wrong length bit sent')
        return
    command = ord(ret[2])
    if not command==commands[com]:
        print('incorrect command type: {}'.format(command))
        return
    end = ord(ret[-1])
    if end!=0xFE:
        print('bad end bit')
        return
    data = ret[3:-1]
    final_joint_pos = [0]*18
    for i in range(18):
        final_joint_pos[i] = (ord(data[2*i])<<8) + ord(data[2*i + 1])
    if com=='current':
        final_joint_pos = [fjp / 200.0 for fjp in final_joint_pos]
    print('{}:{}'.format(com,final_joint_pos))
    return final_joint_pos 