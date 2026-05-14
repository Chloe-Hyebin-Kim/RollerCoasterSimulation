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
