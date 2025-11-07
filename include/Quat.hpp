#pragma once
#include "Matrix3x3.hpp"

struct Quat 
{
    double s = 1, x = 0, y = 0, z = 0;

    // Exercici 3
    Quat Normalized() const;
    Quat Multiply(const Quat& b, OpsCounter* op = nullptr) const;
    Vec3 Rotate(const Vec3& v, OpsCounter* op = nullptr) const;
    static Quat FromMatrix3x3(const Matrix3x3& R);
    Matrix3x3 ToMatrix3x3() const;
    static Quat FromAxisAngle(const Vec3& u, double phi);
    void ToAxisAngle(Vec3& axis, double& angle) const;
};