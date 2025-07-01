/**
 * \file
 * Header file for Serial Communications class, which allows safe serial
 * communications of any type of data
 *
 * \copyright &copy; PCI 2017. All rights reserved
 */

#ifndef SesrialCom_h
#define SesrialCom_h

#include "Arduino.h" // Need the basic arduino libs

/**
*   Class to handle serial communications of arbitrary data length on an arduino
*/
class SerialCom
{
    public:
    SerialCom(void (*process_data)(char*, int), int baud_rate = 9600, 
             int max_iters = 0, int buffer_oversize = 1.25, 
             void (*handle_error)(int)=NULL);
    void init();
    static void sendData(char* data, int length);
    void update();

    /// Enum of error codes
    enum Error{
        outside_message, ///< Received a byte other than start, outside of a message
        invalid_reserved, ///< Received an index for a reserved byte that was out of range
        longer_than_expected,///< Reeived more data than expected, given length bytes
        end_when_not_in_message, ///< Received an end byte when not currently in a message
        start_when_in_message, ///< Received a start byte when already in a message
        reserved_when_reserved, ///< Received a next byte reserved flag twice in a row
        early_end ///< Received an end command before expected end of message
    };
    

    private:
    static void _defaultErrorHandler(int error_code);

    void (*_handle_error)(int); ///< Pointer to error handling function
    int _write_pos; ///< The position in the message buffer which we are 
                    ///< currently writing to
    bool _in_message; ///< Denotes whether we are in a message or not
    int _data_buffer_length; ///< The length of the data buffer
    char* _data_buffer; ///< Pointer to the message buffer
    int _max_iters; ///< The maximum number of interactions to grab serial data 
                    ///< per loop. If zero, no limit
    static const char MESSAGE_START = 254; ///< Signals the start of a message
    static const char MESSAGE_END = 255; ///< Signals the end of a message
    static const char RESERVED_BYTE = 253; ///< Signals a reserved byte is needed
    bool _next_reserved; ///< Indicates that the next byte is a reserved byte
    void (*_process_data)(char*, int); ///< Pointer to function which will 
                                       ///< process the data, it will be passed 
                                       ///< a pointer to the the data array and 
                                       ///< the length of the data
    const static char RESERVED_DICT[]; ///< An array defining the values of the 
                                       ///< reserved bytes
    int _buffer_oversize; ///<Factor by which to oversize buffers
    char _first_length_byte; ///< The first byte to be read indicating the 
                             ///< length of the upcoming data
    char _second_length_byte; ///< The second byte to be read indicating the 
                              ///< length of the upcoming data
    int _expected_data_length; ///< The expected length of data
    int _baud_rate; ///< The baud rate for communications
};

#endif //SesrialCom_h