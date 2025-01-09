#include <Arduino.h>

#include <squeue.hpp>

#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO08x.h>
#include <TinyGPS++.h>

#include "pins.h"
#include "sensors.hpp"
#include "angles_util.hpp"

#define GPS_READ_BUFFER_SIZE    32
#define WRITE_PERIOD            150
#define READ_DELAY              3
#define SENSED_DATA_BUFFER_SIZE 256

#define ACCEL_OFFSET_X          (0)
#define ACCEL_OFFSET_Y          (0)
#define ACCEL_OFFSET_Z          (0)

#if GPS_READ_BUFFER_SIZE <= 2
    #error GPS_READ_BUFFER_SIZE must be at least 2
#endif

SQueue<SensedData, SENSED_DATA_BUFFER_SIZE> collectedData;
SDLib::File logFile;

TinyGPSPlus gps;
Adafruit_BNO08x bno08x(-1);
Adafruit_BMP280 bmp280;

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
    if (!bno08x.enableReport(SH2_ACCELEROMETER, 10000)) {
        fatalError("BNO08x accelerometer enable failed");
    }
    if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 5000)) {
        fatalError("BNO08x gyroscope enable failed");
    }
}

void setup() {
    SerialUSB.begin(115200);

    Serial1.begin(9600);

    if (!bno08x.begin_I2C()) {
        fatalError("BNO08x init failed");
    }

    if (!bmp280.begin(0x76)) {
        fatalError("BMP280 init failed");
    }

    setupBNO08x();

    pinMode(PIN_LED, OUTPUT);

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

    while (Serial1.available() > 0) {
        Serial1.readBytes(buffer, GPS_READ_BUFFER_SIZE);

        for (size_t i = 0; i < GPS_READ_BUFFER_SIZE; i++) {
            if (buffer[i] == 0) break;
            gps.encode(buffer[i]);
        }
    }

    float pressure = bmp280.readPressure();
    float temperature = bmp280.readTemperature();

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
            case SH2_ACCELEROMETER:
                accelerationStatus = sensorData.status & 0b11;
                acceleration[0] = sensorData.un.accelerometer.x - ACCEL_OFFSET_X;
                acceleration[1] = sensorData.un.accelerometer.y - ACCEL_OFFSET_Y;
                acceleration[2] = sensorData.un.accelerometer.z - ACCEL_OFFSET_Z;
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

        {acceleration[0], acceleration[1], acceleration[2]},
        accelerationStatus,
        {gyroscope[0], gyroscope[1], gyroscope[2]},
        gyroscopeStatus,

        gps.time.value(),
        gps.date.value(),
        (gps.location.isValid())? gps.location.lat(): NAN,
        (gps.location.isValid())? gps.location.lng(): NAN,
        gps.altitude.meters()
    };
}

void loop() {

    collectedData.push(readSensors());
    delay(READ_DELAY);

    if (collectedData.size() == SENSED_DATA_BUFFER_SIZE || millis() - lastWrite >= WRITE_PERIOD) {
        lastWrite = millis();
        digitalWrite(PIN_LED, true);

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

            collectedData.pop();
        }

        logFile.flush();
        digitalWrite(PIN_LED, false);
    }
}