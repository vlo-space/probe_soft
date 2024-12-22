#ifndef H_ANGLES_UTIL
#define H_ANGLES_UTIL

namespace angles_util {
    class Euler {
        public:
            float roll;
            float pitch;
            float yaw;
    };

    Euler quaternionToEuler(float qr, float qi, float qj, float qk);
}

#endif