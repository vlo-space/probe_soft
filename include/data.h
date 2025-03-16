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

    void print(Print* output) const {
        output->print(this->index);
        output->print('\t');
        output->print(this->uptime);
        output->print('\t');
        output->print(this->micros);
        output->print('\t');
        output->print(this->temperature);
        output->print('\t');
        output->print(this->pressure);
        output->print('\t');
        output->print(this->vibrations);
        output->print('\t');
        output->print(this->acceleration[0], 6);
        output->print('\t');
        output->print(this->acceleration[1], 6);
        output->print('\t');
        output->print(this->acceleration[2], 6);
        output->print('\t');
        output->print(this->accelerationStatus);
        output->print('\t');
        output->print(this->gyroscope[0], 6);
        output->print('\t');
        output->print(this->gyroscope[1], 6);
        output->print('\t');
        output->print(this->gyroscope[2], 6);
        output->print('\t');
        output->print(this->gyroscopeStatus);
        output->print('\t');
        output->print(this->altitude);
        output->print('\t');
        output->print(this->gpsTime);
        output->print('\t');
        output->print(this->gpsLatitude, 6);
        output->print('\t');
        output->print(this->gpsLongitude, 6);
        output->print('\t');
        output->print(this->gpsAltitude, 6);
        output->println();
    }
};

struct __attribute__((packed)) Frame {
    char signature[3];
    SensedData data;
    uint32_t checksum;
};

#endif
