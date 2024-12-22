#include "angles_util.hpp"
#include <cmath>
#include <api/Common.h>
#include <Arduino.h>

namespace angles_util {
    Euler quaternionToEuler(float qr, float qi, float qj, float qk) {
        Euler angles;

        float sqr = qr*qr;
        float sqi = qi*qi;
        float sqj = qj*qj;
        float sqk = qk*qk;

        angles.yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
        angles.pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
        angles.roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

        return angles;
    }
}