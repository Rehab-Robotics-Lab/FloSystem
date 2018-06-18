/**
 * \file
 * Source file for Serial Communications class
 *
 * \copyright &copy; PCI 2017. All rights reserved
 */

#include "SerialCom.h"

const char SerialCom::RESERVED_DICT[] = {253, 254, 255};

/**
 * Constructor for the serial com object
 * @param baud_rate       The baud rate to run the communications at
 * @param process_data    Pointer to the function which will be used to process 
 *                        incoming data
 * @param max_iters       The maximum number of iterations to run each loop when 
 *                        accepting data
 * @param buffer_oversize The amount to oversize buffers by when resizing. 
 *                        Should be equal to or great than one.
 * @param handle_error    Pointer to the function to handle errors
 */
SerialCom::SerialCom(void (*process_data)(char*, int), int baud_rate, int max_iters,
                     int buffer_oversize, void (*handle_error)(int)){
    _data_buffer = new char[1];
    _data_buffer_length = 1;
    _process_data = process_data;
    _write_pos = 0;
    _baud_rate = baud_rate;
    if(buffer_oversize < 1){buffer_oversize = 1;}
    _buffer_oversize = buffer_oversize;
    if(handle_error == NULL){ _handle_error = &_defaultErrorHandler; }
}

/**
 * Initialization function
 */
void SerialCom::init(){
    Serial.begin(_baud_rate);
}

/**
 * The update function that should be run every loop
 */
void SerialCom::update(){
    int iters_left = _max_iters;
    while(Serial.available() && (iters_left>0 || _max_iters==0)){
        iters_left--;
        byte just_read = Serial.read();
        if(just_read == MESSAGE_START){
            if(_in_message){ _handle_error(start_when_in_message); }
            _in_message = true;
            _write_pos = -2; // the first two bytes which are read are for the size
            _next_reserved = false;
        } else if(just_read == MESSAGE_END){
            if(!_in_message){ _handle_error(end_when_not_in_message); }
            else if(_write_pos == _expected_data_length){
                _process_data(_data_buffer, _write_pos); 
            }else{_handle_error(early_end);}
            _in_message = false;
        }else if(_in_message){
            if(just_read == RESERVED_BYTE){
                if(_next_reserved){
                    _in_message = false;
                    _handle_error(reserved_when_reserved);
                }
                _next_reserved = true;
            }else if(_write_pos == -2){ 
                _first_length_byte = just_read; 
                _write_pos++;
            }else if(_write_pos == -1){
                _second_length_byte = just_read;
                _expected_data_length = int(
                    (unsigned char)((_first_length_byte & 0x7F)<<7) |
                    (unsigned char)((_second_length_byte & 0x7F)));
                if(_expected_data_length > _data_buffer_length){
                    delete[] _data_buffer;
                    _data_buffer_length = _expected_data_length * _buffer_oversize;
                    _data_buffer = new char[_data_buffer_length];
                }
                _write_pos++;
            }else{
                if(_next_reserved){
                    _next_reserved = false;
                    int num_reserved = sizeof(RESERVED_DICT) / sizeof(RESERVED_DICT[0]);
                    if(just_read >= num_reserved){
                        _handle_error(invalid_reserved);
                        _in_message = false;
                    }else{ 
                        just_read = RESERVED_DICT[just_read]; }
                }

                if(_write_pos >= _data_buffer_length){
                    _handle_error(longer_than_expected);
                    _in_message = false;
                }else{
                    *(_data_buffer + _write_pos) = just_read;
                    _write_pos++;
                }
            }
        }else{
            _handle_error(outside_message);
        }
    }
}

/**
 * Sends data to the recipient
 * @param data   Pointer to the byte array of data
 * @param length The length of the array (if incorrect, will run under or over)
 *
 * # TODO: Make this more efficient
 */
void SerialCom::sendData(char* data, int length){
    Serial.write(MESSAGE_START);
    Serial.write((length >> 7) & 0x7F); // write the first length byte
    Serial.write( length & 0x7F ); // write the second length byte
    for(int j = 0; j<length; j++){
        bool found_reserved = false;
        for(int i=0; i<(int)(sizeof(RESERVED_DICT)/sizeof(RESERVED_DICT[0])); i++){
            if(data[j] == RESERVED_DICT[i]){
                char to_send[] = {RESERVED_BYTE, (char)i};
                found_reserved = true;
                Serial.write(to_send, 2);
            }
        }
        if(!found_reserved){
            Serial.write(&data[j],1);
        }
    }
    // Serial.write(data, length);
    Serial.write(MESSAGE_END);
}

/**
 * The default error handler that will be used if none other is specified
 * @param error_code The error code to handle
 */
void SerialCom::_defaultErrorHandler(int error_code){
    switch(error_code){
        case outside_message:{
            char send[] = "[0] Received invalid byte outside of message";
            sendData(send, sizeof(send)/sizeof(send[0]));
            break;
        }case invalid_reserved:{
            char send[] = "[1] Invalid value received as a reserved index after reserved flag";
            sendData(send, sizeof(send)/sizeof(send[0]));
            break;
        }case longer_than_expected:{
            char send[] = "[2] Received more bytes than indicated in size";
            sendData(send, sizeof(send)/sizeof(send[0]));
            break;
        }case end_when_not_in_message:{
            char send[] = "[3] Received an end of message when not in a message";
            sendData(send, sizeof(send)/sizeof(send[0]));
            break;
        }case start_when_in_message:{
            char send[] = "[4] Received start byte when already in message";
            sendData(send, sizeof(send)/sizeof(send[0]));
            break;
        }case reserved_when_reserved:{
            char send[] = "[5] Received a reserved byte next flag when one had already been received";
            sendData(send, sizeof(send)/sizeof(send[0]));
            break;
        }case early_end:{
            char send[] = "[6] Received an end command before expected end of message";
            sendData(send, sizeof(send)/sizeof(send[0]));
            break;
        }
    }
}
