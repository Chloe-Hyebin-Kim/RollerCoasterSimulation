#include "Vec3.h"

#include <algorithm>
#include <cmath>

namespace
{
    constexpr float EPS = 1e-6f;
}

Vec3::Vec3() : x(0.0f), y(0.0f), z(0.0f)
{
}

Vec3::Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_)
{
}

Vec3 Vec3::operator+(const Vec3& v) const
{
    return Vec3(x + v.x, y + v.y, z + v.z);
}

Vec3 Vec3::operator-(const Vec3& v) const
{
    return Vec3(x - v.x, y - v.y, z - v.z);
}

Vec3 Vec3::operator-() const
{
    return Vec3(-x, -y, -z);
}

Vec3 Vec3::operator*(float s) const
{
    return Vec3(x * s, y * s, z * s);
}

Vec3 Vec3::operator/(float s) const
{
    return Vec3(x / s, y / s, z / s);
}

Vec3& Vec3::operator+=(const Vec3& v)
{
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
}

Vec3 operator*(float s, const Vec3& v)
{
    return v * s;
}

float dot(const Vec3& a, const Vec3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3& a, const Vec3& b)
{
    return Vec3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

float length(const Vec3& v)
{
    return std::sqrt(dot(v, v));
}

Vec3 normalize(const Vec3& v)
{
    const float len = length(v);

    if (len < EPS)
        return Vec3(0.0f, 1.0f, 0.0f);

    return v / len;
}

float clampf(float v, float lo, float hi)
{
    return std::max(lo, std::min(hi, v));
}

Vec3 lerp(const Vec3& a, const Vec3& b, float t)
{
    return a * (1.0f - t) + b * t;
}

Vec3 rotateAroundAxis(const Vec3& v, const Vec3& axisUnit, float angle)
{
    const float c = std::cos(angle);
    const float s = std::sin(angle);

    return v * c
        + cross(axisUnit, v) * s
        + axisUnit * (dot(axisUnit, v) * (1.0f - c));
}
