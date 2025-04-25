#pragma once
#include <stdexcept>
#include <math.h>
#include <algorithm>

class Vector3D {
public:
    double x, y, z;

    Vector3D() : x(0), y(0), z(0) {}

    Vector3D(double a, double b, double c) : x(a), y(b), z(c) {}

    Vector3D operator+(const Vector3D& other) const {
        return Vector3D(x + other.x, y + other.y, z + other.z);
    }

    Vector3D& operator+=(const Vector3D& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vector3D operator-(const Vector3D& other) const {
        return Vector3D(x - other.x, y - other.y, z - other.z);
    }

    Vector3D& operator-=(const Vector3D& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vector3D operator*(double val) const {
        return Vector3D(val * x, val * y, val * z);
    }

    friend Vector3D operator*(double val, const Vector3D& vec) {
        return vec * val;
    }

    Vector3D& operator*=(double val) {
        x *= val;
        y *= val;
        z *= val;
        return *this;
    }

    Vector3D operator/(float scalar) const {
        if (scalar == 0) throw std::runtime_error("Division by zero!");
        return Vector3D(x / scalar, y / scalar, z / scalar);
    }

    Vector3D& operator/=(double val) {
        x /= val;
        y /= val;
        z /= val;
        return *this;
    }

    double dot(const Vector3D& other) const;

    Vector3D operator*(const Vector3D& other) const {
        return Vector3D(
            x * other.x,
            y * other.y,
            z * other.z
        );
    }

    Vector3D operator%(const Vector3D& other) const {
        return Vector3D(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    double magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    Vector3D normalize() const {
        double mag = magnitude();
        if (mag == 0) return Vector3D();

        return (*this) / mag;
    }

    Vector3D project(const Vector3D& other) {
        double lenght = this->dot(other);
        double otherLenghtSquared = other.dot(other);

        if (otherLenghtSquared == 0)
            return Vector3D();

        return other * (lenght / otherLenghtSquared);
    }

    Vector3D clamp(double minVal, double maxVal) const {
        return Vector3D(
            std::clamp(x, minVal, maxVal),
            std::clamp(y, minVal, maxVal),
            std::clamp(z, minVal, maxVal)
        );
    }
};

inline double dot(const Vector3D& self, const Vector3D& other) {
    return self.x * other.x + self.y * other.y + self.z * other.z;
}