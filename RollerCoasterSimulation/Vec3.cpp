#include "Vec3.h"

Vec3::Vec3() : m_f32X(0.0f), m_f32Y(0.0f), m_f32Z(0.0f)
{
}

Vec3::Vec3(float x_, float y_, float z_) : m_f32X(x_), m_f32Y(y_), m_f32Z(z_)
{
}

Vec3 Vec3::operator+(const Vec3& v) const
{
    return Vec3(m_f32X + v.m_f32X, m_f32Y + v.m_f32Y, m_f32Z + v.m_f32Z);
}

Vec3 Vec3::operator-(const Vec3& v) const
{
    return Vec3(m_f32X - v.m_f32X, m_f32Y - v.m_f32Y, m_f32Z - v.m_f32Z);
}

Vec3 Vec3::operator-() const
{
    return Vec3(-m_f32X, -m_f32Y, -m_f32Z);
}

Vec3 Vec3::operator*(float s) const
{
    return Vec3(m_f32X * s, m_f32Y * s, m_f32Z * s);
}

Vec3 Vec3::operator/(float s) const
{
    return Vec3(m_f32X / s, m_f32Y / s, m_f32Z / s);
}

Vec3& Vec3::operator+=(const Vec3& v)
{
    m_f32X += v.m_f32X;
    m_f32Y += v.m_f32Y;
    m_f32Z += v.m_f32Z;
    return *this;
}

Vec3 operator*(float scalar, const Vec3& vector)
{
    return vector * scalar;
}

float DotVec3(const Vec3& vectorA, const Vec3& vectorB)
{
    return vectorA.m_f32X * vectorB.m_f32X
        + vectorA.m_f32Y * vectorB.m_f32Y
        + vectorA.m_f32Z * vectorB.m_f32Z;
}

Vec3 CrossVec3(const Vec3& vectorA, const Vec3& vectorB)
{
    return Vec3(
        vectorA.m_f32Y * vectorB.m_f32Z - vectorA.m_f32Z * vectorB.m_f32Y,
        vectorA.m_f32Z * vectorB.m_f32X - vectorA.m_f32X * vectorB.m_f32Z,
        vectorA.m_f32X * vectorB.m_f32Y - vectorA.m_f32Y * vectorB.m_f32X
    );
}

float LengthVec3(const Vec3& vector)
{
    return sqrt(DotVec3(vector, vector));
}

Vec3 NormalizeVec3(const Vec3& vector)
{
    const float f32CurLength = LengthVec3(vector);

    if (f32CurLength < EPS)
        return Vec3(0.0f, 1.0f, 0.0f); // 0으로 나누기 방지용 기본 업벡터

    return vector / f32CurLength;
}

float Clampf(float value, float minLimit, float maxLimit)
{
    return max(minLimit, min(maxLimit, value));
}

Vec3 LerpVec3(const Vec3& start, const Vec3& end, float alpha)
{
    return start * (1.0f - alpha) + end * alpha;
}

Vec3 RotateAroundAxis(const Vec3& targetVector, const Vec3& normalizedAxis, float radians)
{
    const float cosAngle = cos(radians);
    const float sinAngle = sin(radians);

    return targetVector * cosAngle + CrossVec3(normalizedAxis, targetVector) * sinAngle + normalizedAxis * (DotVec3(normalizedAxis, targetVector) * (1.0f - cosAngle));
}