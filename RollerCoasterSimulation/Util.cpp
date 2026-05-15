#include "Util.h"

//Vec3::Vec3() : m_f32X(0.0f), m_f32Y(0.0f), m_f32Z(0.0f)
//{
//}

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



void ArcLengthTable::Sampling(const CatmullRomSpline& spline, int i32SampleCount)
{
    m_arrSamples.clear();
    m_f32TotalLength = 0.0f;

    if (i32SampleCount <= 0)
    {
        return;
    }

    int i32SegmentCount = spline.GetSegmentCount();

    if (i32SegmentCount <= 0)
    {
        return;
    }

    Vec3 previousPosition = spline.GetPoint(0.0f);

    for (int i32Index = 0; i32Index <= i32SampleCount; ++i32Index)
    {
        float f32Ratio = (float)(i32Index) / (float)(i32SampleCount);

        // 전체 spline parameter 범위:0 ~ (개수)
        float f32U = f32Ratio * (float)(i32SegmentCount);
        Vec3 position = spline.GetPoint(f32U);

        if (i32Index > 0)
        {
            float f32Distance = LengthVec3(position - previousPosition);
            m_f32TotalLength += f32Distance;
        }

        ArcLengthSample sample;
        sample.f32D = m_f32TotalLength;
        sample.f32Param = f32U;
        sample.vecPosition = position;

        m_arrSamples.push_back(sample);

        previousPosition = position;
    }
}

float ArcLengthTable::GetTotalLength() const
{
    return m_f32TotalLength;
}

ArcLengthSample ArcLengthTable::FrameAtArcLength(const CatmullRomSpline& spline, float s) const
{
    if (m_arrSamples.empty() || m_f32TotalLength <= EPS)
        return ArcLengthSample();

    s = std::fmod(s, m_f32TotalLength);

    if (s < 0.0f)
        s += m_f32TotalLength;

    auto it = std::lower_bound(
        m_arrSamples.begin(),
        m_arrSamples.end(),
        s,
        [](const ArcLengthSample& sample, float value)
        {
            return sample.f32D < value;
        }
    );

    if (it == m_arrSamples.begin())
        return m_arrSamples.front();

    if (it == m_arrSamples.end())
        return m_arrSamples.back();

    const ArcLengthSample& bSample = *it;
    const ArcLengthSample& aSample = *(it - 1);

    const float denom = max(EPS, bSample.f32D - aSample.f32D);
    const float alpha = Clampf((s - aSample.f32D) / denom, 0.0f, 1.0f);
    const float u = aSample.f32Param * (1.0f - alpha) + bSample.f32Param * alpha;

    ArcLengthSample out;
    out.f32D = s;
    out.f32Param = u;
    out.vecPosition = spline.GetPoint(u);
    out.vecTangent = NormalizeVec3(spline.GetDerivative(u));

  
    out.vecNormal = NormalizeVec3(LerpVec3(aSample.vecNormal, bSample.vecNormal, alpha));
    out.vecNormal = NormalizeVec3(out.vecNormal - out.vecTangent * DotVec3(out.vecNormal, out.vecTangent));
        
    out.vecBinormal = NormalizeVec3(CrossVec3(out.vecTangent, out.vecNormal));
    out.vecNormal = NormalizeVec3(CrossVec3(out.vecBinormal, out.vecTangent));

    return out;
}

/// <summary>
/// spline parameter 반환
/// </summary>
/// <param name="f32D"></param>
/// <returns></returns>
float ArcLengthTable::GetParamFromD(float f32D) const
{
    //binary search + linear interpolation
    if (m_arrSamples.empty())
    {
        return 0.0f;
    }

    if (m_f32TotalLength <= EPS)
    {
        return m_arrSamples.front().f32Param;
    }

    // closed track이면 D가 total length를 넘어도 한 바퀴 돌게 처리
    f32D = fmod(f32D, m_f32TotalLength);

    if (f32D < 0.0f)
    {
        f32D += m_f32TotalLength;
    }

    int i32Left = 0;
    int i32Right = static_cast<int>(m_arrSamples.size()) - 1;

    while (i32Left <= i32Right)
    {
        int i32Mid = (i32Left + i32Right) / 2;

        if (m_arrSamples[i32Mid].f32D < f32D)
        {
            i32Left = i32Mid + 1;
        }
        else
        {
            i32Right = i32Mid - 1;
        }
    }

    int i32UpperIndex = i32Left;

    if (i32UpperIndex <= 0)
    {
        return m_arrSamples.front().f32Param;
    }

    if (i32UpperIndex >= static_cast<int>(m_arrSamples.size()))
    {
        return m_arrSamples.back().f32Param;
    }

    const ArcLengthSample& lowerSample = m_arrSamples[i32UpperIndex - 1];
    const ArcLengthSample& upperSample = m_arrSamples[i32UpperIndex];

    float f32D0 = lowerSample.f32D;
    float f32D1 = upperSample.f32D;

    float f32Param0 = lowerSample.f32Param;
    float f32Param1 = upperSample.f32Param;


    //alpha = (D - D0) / (D1 - D0)

   //     u = (1 - alpha)u0 + alpha u1
    float f32Alpha = 0.0f;

    if ((f32D1 - f32D0) > EPS)
    {
        f32Alpha = (f32D - f32D0) / (f32D1 - f32D0);//D0~ D1사이에서 어디쯤?
    }

    return f32Param0 * (1.0f - f32Alpha) + f32Param1 * f32Alpha;//비율만큼 보간

    return 0.0f;
}
