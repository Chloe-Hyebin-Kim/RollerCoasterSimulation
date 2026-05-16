#pragma once
#include "stdafx.h"

class Vec3
{
public:
    float m_f32X;
    float m_f32Y;
    float m_f32Z;

    Vec3();
    Vec3(float x, float y, float z);

    Vec3 operator+(const Vec3& v) const;
    Vec3 operator-(const Vec3& v) const;
    Vec3 operator-() const;
    Vec3 operator*(float s) const;
    Vec3 operator/(float s) const;

    Vec3& operator+=(const Vec3& v);
};

Vec3 operator*(float s, const Vec3& v);

float DotVec3(const Vec3& a, const Vec3& b);
Vec3 CrossVec3(const Vec3& a, const Vec3& b);
float LengthVec3(const Vec3& v);
Vec3 NormalizeVec3(const Vec3& v);
float Clampf(float value, float min_val, float max_val);
Vec3 LerpVec3(const Vec3& a, const Vec3& b, float t);
Vec3 RotateAroundAxis(const Vec3& v, const Vec3& axisUnit, float angle);
