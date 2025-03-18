#include "angles_util.hpp"
#include "data.h"
#include "data_collector.hpp"
#include "nmea_util.hpp"
#include "pins.h"

#include <ace_crc/crc32_nibble.hpp>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO08x.h>
#include <Arduino.h>
#include <CanSatKit.h>
#include <CanSatKitRadio.h>
#include <SD.h>
#include <Servo.h>
#include <SPI.h>
#include <squeue.hpp>
#include <TinyGPS++.h>

#define DEBUG_SERIAL_BAUD_RATE   115200
#define GPS_BAUD_RATE            115200
#define GPS_READ_BUFFER_SIZE     32
#define GPS_READ_TIMEOUT         100
#define WRITE_PERIOD             75
#define READ_DELAY               3
#define SENSED_DATA_BUFFER_SIZE  3
#define RADIO_PACKET_FRAME_COUNT 2

#define SERVO_ROTATION_TIME 6800
#define SERVO_MICROSECONDS  1600

#define ALTITUDE_DELTA_SAMPLE_TIME 3000
#define LANDING_ACTIVATION_SPEED   -3.0

#define SEA_LEVEL_PRESSURE 1013.25F

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

Servo landingServo;

CanSatKit::Radio radio(
    CanSatKit::Pins::Radio::ChipSelect, CanSatKit::Pins::Radio::DIO0, 433.0,
    CanSatKit::Bandwidth_500000_Hz, CanSatKit::SpreadingFactor_7,
    CanSatKit::CodingRate_4_8
);

uint32_t sensedDataIndex = 0;

uint32_t lastWrite = 0;
uint32_t servoRotationStart = 0;

float lastAltitude = 0;
uint32_t lastAltitudeRead = 0;
DataCollector altitude;

bool landingStandDeployed = false;

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

void startServoRotation() {
    landingServo.attach(PIN_SERVO);
    landingServo.writeMicroseconds(SERVO_MICROSECONDS);
    servoRotationStart = millis();
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
    nmea_util::writeCommand(&Serial, "$PAIR864,0,0,115200");
    // Set the GPS + BeiDou + GLONASS mode on GNSS
    nmea_util::writeCommand(&Serial, "$PAIR066,1,1,1,0,0,0");
    // Set the time between GNSS outputs and the type of data to send
    nmea_util::writeCommand(&Serial, "$PAIR062,5,0");
    nmea_util::writeCommand(&Serial, "$PAIR062,0,5");
    nmea_util::writeCommand(&Serial, "$PAIR062,1,0");
    nmea_util::writeCommand(&Serial, "$PAIR062,2,0");
    nmea_util::writeCommand(&Serial, "$PAIR062,3,0");
    nmea_util::writeCommand(&Serial, "$PAIR062,4,0");

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

        {acceleration[0], acceleration[1], acceleration[2]},
        accelerationStatus,
        {gyroscope[0],    gyroscope[1],    gyroscope[2]   },
        gyroscopeStatus,

        gps.time.value(),
        (gps.location.isValid()) ? gps.location.lat() : NAN,
        (gps.location.isValid()) ? gps.location.lng() : NAN,
        bmp280.readAltitude()
    };
}

void loop() {

    collectedData.push(readSensors());
    delay(READ_DELAY);

    altitude.addReading(bmp280.readAltitude(SEA_LEVEL_PRESSURE), micros());

    if (altitude.timeSum() >= ALTITUDE_DELTA_SAMPLE_TIME * 1000) {
        double speed = altitude.averageSpeed() * 1000000.0;

        altitude = DataCollector();

        if (speed < LANDING_ACTIVATION_SPEED && !landingStandDeployed) {
            startServoRotation();
            landingStandDeployed = true;
        }
    }

    if (servoRotationStart != 0 &&
        millis() - servoRotationStart >= SERVO_ROTATION_TIME) {
        landingServo.detach();
        servoRotationStart = 0;
    }

    if (collectedData.size() == SENSED_DATA_BUFFER_SIZE ||
        millis() - lastWrite >= WRITE_PERIOD) {
        lastWrite = millis();
        digitalWrite(PIN_LED, true);

        Frame radioBuffer[RADIO_PACKET_FRAME_COUNT] = {{0}};
        uint8_t radioBufferedCount = 0;

        while (!collectedData.empty()) {
            const SensedData* data = collectedData.front();

            data->print(&SerialUSB);
            data->print(&logFile);

            radioBuffer[radioBufferedCount] = (Frame) {
                .signature = {'V',  'L', 'O'},
                .data = {0},
                .checksum =
                    ace_crc::crc32_nibble::crc_calculate(data, sizeof(SensedData))
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