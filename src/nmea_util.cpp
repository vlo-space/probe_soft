#include "nmea_util.hpp"

// used to translate to hexadecimal
const char HEX_SYMBOLS[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

namespace nmea_util {
    void writeCheckSum(Uart* serial, const char* command) {
        char sum = command[1];
        u_int16_t i;
        for( i=2; command[i] != '\0'; i++ ) {
            sum = sum ^ command[i]; 
        }
        serial->write(HEX_SYMBOLS[sum/16%16]);
        serial->write(HEX_SYMBOLS[sum%16]);
    }

    void writeCommand(Uart* serial, const char* command) {
        serial->write('\r');
        serial->write('\n');

        for (uint32_t i = 0; command[i] != '\0'; i++) {
            serial->write(command[i]);
        }

        serial->write('*');
        writeCheckSum(serial, command);
        serial->write('\r');
        serial->write('\n');
    }
}