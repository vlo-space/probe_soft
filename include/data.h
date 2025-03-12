#ifndef H_CANSAT_SENSORS
#define H_CANSAT_SENSORS

#include <cstdint>

struct SensedData {
    uint32_t index;

    uint32_t uptime;
    uint16_t micros;

    float temperature;
    float pressure;
    uint16_t vibrations;
    uint32_t altitude;

    float acceleration[3];
    uint8_t accelerationStatus;
    float gyroscope[3];
    uint8_t gyroscopeStatus;

    uint32_t gpsTime;
    double gpsLatitude;
    double gpsLongitude;
    double gpsAltitude;
};

struct __attribute__((packed)) Frame {
    char signature[3];
    SensedData data;
    uint32_t checksum;
};

#endif
