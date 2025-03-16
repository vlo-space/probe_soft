#ifndef H_CANSAT_SENSORS
#define H_CANSAT_SENSORS

#include <Arduino.h>
#include <cstdint>

struct __attribute__((packed)) SensedData {
    uint32_t index;

    uint32_t uptime;
    uint16_t micros;

    float temperature;
    float pressure;
    uint16_t vibrations;

    float acceleration[3];
    uint8_t accelerationStatus;
    float gyroscope[3];
    uint8_t gyroscopeStatus;

    uint32_t gpsTime;
    double gpsLatitude;
    double gpsLongitude;

    double altitude;

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
        output->print(this->gpsTime);
        output->print('\t');
        output->print(this->gpsLatitude, 6);
        output->print('\t');
        output->print(this->gpsLongitude, 6);
        output->print('\t');
        output->print(this->altitude, 6);
        output->println();
    }
};

struct __attribute__((packed)) Frame {
    char signature[3];
    SensedData data;
    uint32_t checksum;
};

static_assert(sizeof(Frame) < 85, "Radio frame too big");

#endif
