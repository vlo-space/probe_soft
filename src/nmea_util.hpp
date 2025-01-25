#ifndef H_PCAS_UTIL
#define H_PCAS_UTIL

#include <Uart.h>

namespace nmea_util {

    /**
     * Calculates and writes the checksum.  
     * 
     * The checksum is a XOR of all the bytes between the $ and the * , written in hexadecimal.
     */
    void writeCheckSum(Uart* serial, const char* command);

    /**
     * Writes a PCA command to serial, automatically calculating and inserting the checksum.
     */
    void writeCommand(Uart* serial, const char* command);

}

#endif