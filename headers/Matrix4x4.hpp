#pragma once
#include "Vector.hpp"


class Matrix4x4 {
private:
    double m[4][4];
    double EPS = 1e-6;

public:
    Matrix4x4() {
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                m[i][j] = (i == j) ? 1.0 : 0.0;
    }

    Matrix4x4(double mat[4][4]) {
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                m[i][j] = mat[i][j];
    }

    double& operator()(int row, int col) { return m[row][col]; }
    const double& operator()(int row, int col) const { return m[row][col]; }

    Vector3D operator*(const Vector3D& vec) const {
        double x = m[0][0] * vec.x + m[0][1] * vec.y + m[0][2] * vec.z + m[0][3] * 1;
        double y = m[1][0] * vec.x + m[1][1] * vec.y + m[1][2] * vec.z + m[1][3] * 1;
        double z = m[2][0] * vec.x + m[2][1] * vec.y + m[2][2] * vec.z + m[2][3] * 1;
        double w = m[3][0] * vec.x + m[3][1] * vec.y + m[3][2] * vec.z + m[3][3] * 1;

        if (fabs(w) > EPS) {
            return Vector3D(x / w, y / w, z / w);
        }
        return Vector3D(x, y, z);
    }

    Matrix4x4 operator*(const Matrix4x4& other) const {
        Matrix4x4 result;

        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result.m[i][j] = m[i][0] * other.m[0][j] + m[i][1] * other.m[1][j] +
                                 m[i][2] * other.m[2][j] + m[i][3] * other.m[3][j];
            }
        }

        return result;
    }

    Matrix4x4 transpose() const {
        Matrix4x4 result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result.m[i][j] = m[j][i];
            }
        }
        return result;
    }



    static Matrix4x4 translate(double tx, double ty, double tz);
    static Matrix4x4 scale(double sx, double sy, double sz);
    static Matrix4x4 rotateX(double angle);
    static Matrix4x4 rotateY(double angle);
    static Matrix4x4 rotateZ(double angle);
    static Matrix4x4 rotate(const Vector3D& axis, double angle);
    static Matrix4x4 orthographic(double left, double right, double bottom, double top, double near, double far);
    static Matrix4x4 perspectiveProjection(double l, double r, double b, double t, double n, double f);
    static Matrix4x4 lookAt(const Vector3D& position, const Vector3D& target, const Vector3D& up);
};

inline Matrix4x4 Matrix4x4::translate(double tx, double ty, double tz) {
    Matrix4x4 mat;
    mat(0, 3) = tx;
    mat(1, 3) = ty;
    mat(2, 3) = tz;
    return mat;
}

inline Matrix4x4 Matrix4x4::scale(double sx, double sy, double sz) {
    Matrix4x4 mat;
    mat(0, 0) = sx;
    mat(1, 1) = sy;
    mat(2, 2) = sz;
    return mat;
}

inline Matrix4x4 Matrix4x4::rotateX(double angle) { 
    Matrix4x4 mat;
    double cosA = std::cos(angle);
    double sinA = std::sin(angle);

    mat(1, 1) = cosA;
    mat(1, 2) = -sinA;
    mat(2, 1) = sinA;
    mat(2, 2) = cosA;

    return mat;
}

inline Matrix4x4 Matrix4x4::rotateY(double angle) { 
    Matrix4x4 mat;
    double cosA = std::cos(angle);
    double sinA = std::sin(angle);

    mat(0, 0) = cosA;
    mat(0, 2) = -sinA;
    mat(2, 0) = sinA;
    mat(2, 2) = cosA;

    return mat;
}

inline Matrix4x4 Matrix4x4::rotateZ(double angle) { 
    Matrix4x4 mat;
    double cosA = std::cos(angle);
    double sinA = std::sin(angle);

    mat(0, 0) = cosA;
    mat(0, 1) = -sinA;
    mat(1, 0) = sinA;
    mat(1, 1) = cosA;

    return mat;
}

inline Matrix4x4 Matrix4x4::orthographic(double left, double right, double bottom, double top, double near, double far) {
    Matrix4x4 mat;
    mat(0, 0) = 2 / (right - left);
    mat(1, 1) = 2 / (top - bottom);
    mat(2, 2) = -2 / (far - near);
    mat(0, 3) = -(right + left) / (right - left);
    mat(1, 3) = -(top + bottom) / (top - bottom);
    mat(2, 3) = -(far + near) / (far - near);
    return mat;
}


inline Matrix4x4 Matrix4x4::perspectiveProjection(double l, double r, double b, double t, double n, double f) {
    Matrix4x4 mat;
    mat(0, 0) = 2 * n / (r - l);
    mat(0, 2) = (r + l) / (r - l);
    mat(1, 1) = 2 * n / (t - b);
    mat(1, 2) = (t + b) / (t - b);
    mat(2, 2) = (f + n) / (f - n);
    mat(2, 3) = 2 * f * n / (f - n);
    mat(3, 2) = -1;
    mat(3, 3) = 0;
    return mat;
}

inline Matrix4x4 Matrix4x4::lookAt(const Vector3D& position, const Vector3D& target, const Vector3D& up) {
    Vector3D forward = (target - position).normalize();
    Vector3D right = (up % forward).normalize(); // Правая система координат
    Vector3D newUp = forward % right;

    Matrix4x4 mat;
    mat(0, 0) = right.x;
    mat(0, 1) = right.y;
    mat(0, 2) = right.z;
    mat(0, 3) = -(dot(right, position));

    mat(1, 0) = newUp.x;
    mat(1, 1) = newUp.y;
    mat(1, 2) = newUp.z;
    mat(1, 3) = -(dot(newUp, position));

    mat(2, 0) = -forward.x;
    mat(2, 1) = -forward.y;
    mat(2, 2) = -forward.z;
    mat(2, 3) = dot(forward, position);

    mat(3, 0) = 0;
    mat(3, 1) = 0;
    mat(3, 2) = 0;
    mat(3, 3) = 1;

    return mat;
}


inline Matrix4x4 Matrix4x4::rotate(const Vector3D& axis, double angle) {
    Vector3D a = axis.normalize();
    double c = std::cos(angle);
    double s = std::sin(angle);
    double t = 1.0 - c;

    Matrix4x4 result;

    result.m[0][0] = t * a.x * a.x + c;
    result.m[0][1] = t * a.x * a.y - s * a.z;
    result.m[0][2] = t * a.x * a.z + s * a.y;
    result.m[0][3] = 0;
    
    result.m[1][0] = t * a.x * a.y + s * a.z;
    result.m[1][1] = t * a.y * a.y + c;
    result.m[1][2] = t * a.y * a.z - s * a.x;
    result.m[1][3] = 0;
    
    result.m[2][0] = t * a.x * a.z - s * a.y;
    result.m[2][1] = t * a.y * a.z + s * a.x;
    result.m[2][2] = t * a.z * a.z + c;
    result.m[2][3] = 0;
    
    result.m[3][0] = 0;
    result.m[3][1] = 0;
    result.m[3][2] = 0;
    result.m[3][3] = 1;

    return result;
}