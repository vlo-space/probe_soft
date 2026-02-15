#ifndef H_INS
#define H_INS

#include "math.hpp"

#include <algorithm>
#include <Arduino.h>
#include <array>

namespace ins {
    struct Previous {
        Vec3 pos;
        Vec3 velo;
        Quat oreientation;
    };

    Vec3 calculate(
        float accelometer[3], float gyroscope[4], unsigned long samplePeriod,
        Previous& previous
    ) {
        float dt = (float) samplePeriod / 1000.0;
        Quat orientation = orientation =
            Quat(gyroscope[0], gyroscope[1], gyroscope[2], gyroscope[3]);

        for (int i = 0; i < 3; i++) {
            if (accelometer[i] != accelometer[i]) {
                return previous.pos;
            }
        }

        for (int i = 0; i < 4; i++) {
            if (gyroscope[i] != gyroscope[i]) {
                orientation = previous.oreientation;
                break;
            }
        }

        Vec3 acceleration = Vec3(accelometer[0], accelometer[1], accelometer[2]);
        Vec3 rotated_accel = orientation.rotateVector(acceleration);
        Vec3 velo = previous.velo + (rotated_accel * dt);
        Vec3 pos = previous.pos + (velo * dt);

        previous.pos = pos;
        previous.velo = velo;
        previous.oreientation = orientation;

        return pos;
    }
}

#endif