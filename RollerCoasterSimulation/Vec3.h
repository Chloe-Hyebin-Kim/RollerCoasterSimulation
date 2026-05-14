#pragma once

class Vec3
{
public:
    float x;
    float y;
    float z;

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

float dot(const Vec3& a, const Vec3& b);
Vec3 cross(const Vec3& a, const Vec3& b);
float length(const Vec3& v);
Vec3 normalize(const Vec3& v);
float clampf(float v, float lo, float hi);
Vec3 lerp(const Vec3& a, const Vec3& b, float t);
Vec3 rotateAroundAxis(const Vec3& v, const Vec3& axisUnit, float angle);
