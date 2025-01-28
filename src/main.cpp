#include "angles_util.hpp"
#include "data.h"
#include "nmea_util.hpp"
#include "pins.h"

#include <Adafruit_BMP280.h>
#include <Adafruit_BNO08x.h>
#include <Arduino.h>
#include <CanSatKit.h>
#include <CanSatKitRadio.h>
#include <SD.h>
#include <SPI.h>
#include <squeue.hpp>
#include <TinyGPS++.h>

#define DEBUG_SERIAL_BAUD_RATE   115200
#define GPS_BAUD_RATE            9600
#define GPS_READ_BUFFER_SIZE     32
#define GPS_READ_TIMEOUT         100
#define WRITE_PERIOD             75
#define READ_DELAY               3
#define SENSED_DATA_BUFFER_SIZE  3
#define RADIO_PACKET_FRAME_COUNT 2

#define ACCEL_OFFSET_X (0)
#define ACCEL_OFFSET_Y (0)
#define ACCEL_OFFSET_Z (0)

#if GPS_READ_BUFFER_SIZE <= 2
    #error GPS_READ_BUFFER_SIZE must be at least 2
#endif

SQueue<SensedData, SENSED_DATA_BUFFER_SIZE> collectedData;
SDLib::File logFile;

TinyGPSPlus gps;
Adafruit_BNO08x bno08x(-1);
Adafruit_BMP280 bmp280;

CanSatKit::Radio radio(
    CanSatKit::Pins::Radio::ChipSelect, CanSatKit::Pins::Radio::DIO0, 433.0,
    CanSatKit::Bandwidth_500000_Hz, CanSatKit::SpreadingFactor_7,
    CanSatKit::CodingRate_4_8
);

uint32_t sensedDataIndex = 0;

uint32_t lastWrite = 0;

void fatalError(const char* error) {
    while (true) {
        SerialUSB.print("ERROR: ");
        SerialUSB.println(error);
        digitalWrite(PIN_LED, true);
        delay(1000);
        digitalWrite(PIN_LED, false);
        delay(1000);
    }
}

void setupBNO08x() {
    if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, 100)) {
        fatalError("BNO08x accelerometer init failed");
    }
    if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 5000)) {
        fatalError("BNO08x gyroscope init failed");
    }
}

void setup() {
    SerialUSB.begin(DEBUG_SERIAL_BAUD_RATE);

    Serial.begin(GPS_BAUD_RATE);
    Serial.setTimeout(GPS_READ_TIMEOUT);

    if (!bno08x.begin_I2C()) {
        fatalError("BNO08x init failed");
    }

    if (!bmp280.begin(0x76)) {
        fatalError("BMP280 init failed");
    }

    if (!radio.begin()) {
        fatalError("radio init failed");
    }
    radio.disable_debug();

    setupBNO08x();

    // Set the baud Rate to 115200 on GNSS serial
    nmea_util::writeCommand(&Serial, "$PCAS01,115200");
    // Set the GPS + BeiDou + GLONASS mode on GNSS
    nmea_util::writeCommand(&Serial, "$PCAS04,7");
    // Set the time between GNSS outputs and the type of data to send
    nmea_util::writeCommand(&Serial, "$PCAS03,5,0,0,0,0,0,0,0,0,0,,,0,0");

    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_VIBRATION_SENSOR, INPUT);

    if (SD.begin(PIN_SD_SELECT)) {
        logFile = SD.open("latest.log", O_APPEND | O_CREAT | O_WRITE);
        logFile.print('\n');
        logFile.println("--- STARTUP ---");
        logFile.flush();
    }
}

SensedData readSensors() {
    tone(4, 131);
    char buffer[GPS_READ_BUFFER_SIZE] = {0};

    while (Serial.available() > 0) {
        Serial.readBytes(buffer, GPS_READ_BUFFER_SIZE);
        for (size_t i = 0; i < GPS_READ_BUFFER_SIZE; i++) {
            if (buffer[i] == 0)
                break;
            gps.encode(buffer[i]);
        }
    }

    float pressure = bmp280.readPressure();
    float temperature = bmp280.readTemperature();
    uint16_t vibrations = analogRead(PIN_VIBRATION_SENSOR);
    uint8_t readEventCount = 0;

    float acceleration[3] = {NAN, NAN, NAN};
    uint8_t accelerationStatus = 0;
    float gyroscope[3] = {NAN, NAN, NAN};
    uint8_t gyroscopeStatus = 0;

    if (bno08x.wasReset()) {
        setupBNO08x();
    }

    sh2_SensorValue sensorData;
    while (readEventCount < 5 && bno08x.getSensorEvent(&sensorData)) {
        switch (sensorData.sensorId) {
            case SH2_LINEAR_ACCELERATION:
                accelerationStatus = sensorData.status & 0b11;
                acceleration[0] =
                    sensorData.un.linearAcceleration.x - ACCEL_OFFSET_X;
                acceleration[1] =
                    sensorData.un.linearAcceleration.y - ACCEL_OFFSET_Y;
                acceleration[2] =
                    sensorData.un.linearAcceleration.z - ACCEL_OFFSET_Z;
                break;

            case SH2_ARVR_STABILIZED_RV: {
                gyroscopeStatus = sensorData.status & 0b11;
                angles_util::Euler angles = angles_util::quaternionToEuler(
                    sensorData.un.arvrStabilizedRV.real,
                    sensorData.un.arvrStabilizedRV.i,
                    sensorData.un.arvrStabilizedRV.j,
                    sensorData.un.arvrStabilizedRV.k
                );

                gyroscope[0] = angles.roll;
                gyroscope[1] = angles.pitch;
                gyroscope[2] = angles.yaw;
                break;
            }

            default: break;
        }

        readEventCount++;
    }

    return {
        sensedDataIndex++,

        millis(),
        (uint16_t) (micros() % 1000),

        temperature,
        pressure,
        vibrations,

        {acceleration[0], acceleration[1], acceleration[2]},
        accelerationStatus,
        {gyroscope[0],    gyroscope[1],    gyroscope[2]   },
        gyroscopeStatus,

        gps.time.value(),
        gps.date.value(),
        (gps.location.isValid()) ? gps.location.lat() : NAN,
        (gps.location.isValid()) ? gps.location.lng() : NAN,
        gps.altitude.meters()
    };
}

void loop() {

    collectedData.push(readSensors());
    delay(READ_DELAY);

    if (collectedData.size() == SENSED_DATA_BUFFER_SIZE ||
        millis() - lastWrite >= WRITE_PERIOD) {
        lastWrite = millis();
        digitalWrite(PIN_LED, true);

        Frame radioBuffer[RADIO_PACKET_FRAME_COUNT] = {{0}};
        uint8_t radioBufferedCount = 0;

        while (!collectedData.empty()) {
            const SensedData* data = collectedData.front();

            SerialUSB.print(data->index);
            SerialUSB.print('\t');
            SerialUSB.print(data->uptime);
            SerialUSB.print('\t');
            SerialUSB.print(data->micros);
            SerialUSB.print('\t');
            SerialUSB.print(data->temperature);
            SerialUSB.print('\t');
            SerialUSB.print(data->pressure);
            SerialUSB.print('\t');
            SerialUSB.print(data->vibrations);
            SerialUSB.print('\t');
            SerialUSB.print(data->acceleration[0], 6);
            SerialUSB.print('\t');
            SerialUSB.print(data->acceleration[1], 6);
            SerialUSB.print('\t');
            SerialUSB.print(data->acceleration[2], 6);
            SerialUSB.print('\t');
            SerialUSB.print(data->accelerationStatus);
            SerialUSB.print('\t');
            SerialUSB.print(data->gyroscope[0], 6);
            SerialUSB.print('\t');
            SerialUSB.print(data->gyroscope[1], 6);
            SerialUSB.print('\t');
            SerialUSB.print(data->gyroscope[2], 6);
            SerialUSB.print('\t');
            SerialUSB.print(data->gyroscopeStatus);
            SerialUSB.print('\t');
            SerialUSB.print(data->gpsDate);
            SerialUSB.print('\t');
            SerialUSB.print(data->gpsTime);
            SerialUSB.print('\t');
            SerialUSB.print(data->gpsLatitude, 6);
            SerialUSB.print('\t');
            SerialUSB.print(data->gpsLongitude, 6);
            SerialUSB.print('\t');
            SerialUSB.print(data->gpsAltitude, 6);
            SerialUSB.println();

            logFile.print(data->index);
            logFile.print('\t');
            logFile.print(data->uptime);
            logFile.print('\t');
            logFile.print(data->micros);
            logFile.print('\t');
            logFile.print(data->temperature);
            logFile.print('\t');
            logFile.print(data->pressure);
            logFile.print('\t');
            logFile.print(data->vibrations);
            logFile.print('\t');
            logFile.print(data->acceleration[0], 6);
            logFile.print('\t');
            logFile.print(data->acceleration[1], 6);
            logFile.print('\t');
            logFile.print(data->acceleration[2], 6);
            logFile.print('\t');
            logFile.print(data->accelerationStatus);
            logFile.print('\t');
            logFile.print(data->gyroscope[0], 6);
            logFile.print('\t');
            logFile.print(data->gyroscope[1], 6);
            logFile.print('\t');
            logFile.print(data->gyroscope[2], 6);
            logFile.print('\t');
            logFile.print(data->gyroscopeStatus);
            logFile.print('\t');
            logFile.print(data->gpsDate);
            logFile.print('\t');
            logFile.print(data->gpsTime);
            logFile.print('\t');
            logFile.print(data->gpsLatitude, 6);
            logFile.print('\t');
            logFile.print(data->gpsLongitude, 6);
            logFile.print('\t');
            logFile.print(data->gpsAltitude, 6);
            logFile.println();

            radioBuffer[radioBufferedCount] = (Frame) {
                .signature = {'V', 'L', 'O'}
            };
            memcpy(&radioBuffer[radioBufferedCount].data, data, sizeof(SensedData));

            if (++radioBufferedCount >= RADIO_PACKET_FRAME_COUNT) {
                radio.transmit((uint8_t*) &radioBuffer, sizeof(radioBuffer));
                radioBufferedCount = 0;
            }

            collectedData.pop();
        }

        logFile.flush();
        digitalWrite(PIN_LED, false);
    }
}