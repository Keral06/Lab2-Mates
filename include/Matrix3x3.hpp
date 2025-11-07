#pragma once
#include <vector>
#include <cstddef>
#include <cmath>
#include "OpsCounter.hpp"

struct Vec3 
{
    double x = 0, y = 0, z = 0;

    static double Dot(const Vec3& a, const Vec3& b);
    static Vec3 Cross(const Vec3& a, const Vec3& b);
    double Norm() const;
    Vec3 Normalize() const;
};

struct Matrix3x3 
{
    // Row-major
    double m[9] = { 0 };

    static Matrix3x3 Identity();
    double& At(std::size_t i, std::size_t j) { return m[i * 3 + j]; }
    double  At(std::size_t i, std::size_t j) const { return m[i * 3 + j]; }

    Vec3 Multiply(const Vec3& x, OpsCounter* op = nullptr) const;
    Matrix3x3 Multiply(const Matrix3x3& B, OpsCounter* op = nullptr) const;

    Vec3 operator*(const Vec3& x) const
    {
        return Multiply(x, nullptr);
    }
    Matrix3x3 operator*(const Matrix3x3& B) const
    {
        return Multiply(B, nullptr);
    }

    double Det() const;
    Matrix3x3 Transposed() const;
    double Trace() const;

    // Exercici 1
    bool IsRotation() const;
    static Matrix3x3 RotationAxisAngle(const Vec3& u, double phi);
    void ToAxisAngle(Vec3& axis, double& angle) const;
    Vec3 Rotate(const Vec3& v, OpsCounter* op = nullptr) const;

    // Exercici 2
    static Matrix3x3 FromEulerZYX(double yaw, double pitch, double roll);
    void ToEulerZYX(double& yaw, double& pitch, double& roll) const;
};
