#ifndef H_MATH
#define H_MATH

#include <Arduino.h>
#include <cmath>

class Vec3 {
  private:
    float x;
    float y;
    float z;

  public:
    Vec3() {
        x = 0.0;
        y = 0.0;
        z = 0.0;
    };

    Vec3(float mx, float my, float mz) {
        x = mx;
        y = my;
        z = mz;
    }

    const Vec3 operator+(const Vec3& other) {
        return Vec3(this->x + other.x, this->y + other.y, this->z + other.z);
    };

    const Vec3 operator*(const float& scalar) {
        return Vec3(this->x * scalar, this->y * scalar, this->z * scalar);
    }

    Vec3& operator=(const Vec3& other) {
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }

    Vec3& operator+=(const Vec3& q) {
        x += q.x;
        y += q.y;
        z += q.z;
        return *this;
    }

    float get_x() { return this->x; };

    float get_y() { return this->y; };

    float get_z() { return this->z; };
};

class Quat {
  private:
    float x;
    float y;
    float z;
    float w;

  public:
    Quat() {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        w = 1.0;
    };

    Quat(float mx, float my, float mz, float mw) {
        x = mx;
        y = my;
        z = mz;
        w = mw;
    }

    Quat operator*(const Quat& b) {
        return Quat(
            w * b.x + x * b.w + y * b.z - z * b.y,
            w * b.y - x * b.z + y * b.w + z * b.x,
            w * b.z + x * b.y - y * b.x + z * b.w,
            w * b.w - x * b.x - y * b.y - z * b.z
        );
    }

    Quat operator*(const float& scalar) {
        return Quat(x * scalar, y * scalar, z * scalar, w * scalar);
    }

    Quat& operator+=(const Quat& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        w += other.w;
        return *this;
    };

    Quat& operator=(const Quat& other) {
        x = other.x;
        y = other.y;
        z = other.z;
        w = other.w;
        return *this;
    };

    Quat conjugate() { return Quat(-x, -y, -z, w); }

    Quat normalize() {
        return *this * (1 / std::sqrt(
                                this->w * this->w + this->x * this->x +
                                this->y * this->y + this->z * this->z
                            ));
    }

    Vec3 rotateVector(Vec3 vector) {
        Quat vq = Quat(vector.get_x(), vector.get_y(), vector.get_z(), 0.0);
        Quat norm = this->normalize();
        Quat result = norm * vq;

        return Vec3(result.x, result.y, result.z);
    }
};

#endif