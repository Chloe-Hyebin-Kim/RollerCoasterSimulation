#include "CatmullRomSpline.h"
#include "Util.h"



void CatmullRomSpline::Clear()
{
	m_vecControlPoints.clear();
}

int CatmullRomSpline::GetSegmentCount() const
{
    return static_cast<int>(m_vecControlPoints.size());
}
const vector<Vec3>& CatmullRomSpline::GetControlPoints() const
{
    return m_vecControlPoints;
}

void CatmullRomSpline::AddControlPoint(const Vec3& point)
{
	m_vecControlPoints.push_back(point);
}

float CatmullRomSpline::WrapParameter(float param) const
{
	int i32SegmentCount =  m_vecControlPoints.size();

	if (i32SegmentCount <= 0)
	{
		return 0.0f;
	}

	float f32SegmentCount = (float)(i32SegmentCount);

	param = fmod(param, f32SegmentCount);

	if (param < 0.0f)
	{
		param += f32SegmentCount;
	}

	return param;
}

void CatmullRomSpline::InitializeDefaultTrack()
{
    AddControlPoint(Vec3(0.0f, 16.0f, -38.0f));
    AddControlPoint(Vec3(22.0f, 12.0f, -30.0f));
    AddControlPoint(Vec3(40.0f, 8.0f, -8.0f));
    AddControlPoint(Vec3(30.0f, 22.0f, 18.0f));
    AddControlPoint(Vec3(5.0f, 30.0f, 32.0f));
    AddControlPoint(Vec3(-22.0f, 14.0f, 26.0f));
    AddControlPoint(Vec3(-42.0f, 7.0f, 2.0f));
    AddControlPoint(Vec3(-30.0f, 18.0f, -25.0f));
    AddControlPoint(Vec3(-10.0f, 24.0f, -44.0f));

    Rebuild(5000);
}

void CatmullRomSpline::Rebuild(int sampleCount)
{
    m_ArcLengthTable.Sampling(*this, sampleCount);
}

const CatmullRomSpline& CatmullRomSpline::Spline() const
{
    return *this;
}

const class ArcLengthTable& CatmullRomSpline::ArcLengthTable() const
{
    return m_ArcLengthTable;
}

Vec3 CatmullRomSpline::GetPoint(float param) const
{
	int i32ControlPointCount = m_vecControlPoints.size();

    if (i32ControlPointCount < 4)
    {
        return Vec3();
    }

    param = WrapParameter(param);

    int i32P1Index = static_cast<int>(std::floor(param));
    float f32LocalT = param - static_cast<float>(i32P1Index);

    int i32P0Index = (i32P1Index - 1 + i32ControlPointCount) % i32ControlPointCount;
    int i32P2Index = (i32P1Index + 1) % i32ControlPointCount;
    int i32P3Index = (i32P1Index + 2) % i32ControlPointCount;

    const Vec3& p0 = m_vecControlPoints[i32P0Index];
    const Vec3& p1 = m_vecControlPoints[i32P1Index];
    const Vec3& p2 = m_vecControlPoints[i32P2Index];
    const Vec3& p3 = m_vecControlPoints[i32P3Index];

    float f32T = f32LocalT;
    float f32T2 = f32T * f32T;
    float f32T3 = f32T2 * f32T;

    Vec3 result =
        0.5f *
        (
            2.0f * p1
            + (-p0 + p2) * f32T
            + (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * f32T2
            + (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * f32T3
            );

    return result;
}

Vec3 CatmullRomSpline::GetDerivative(float param) const
{
	//곡선 방향 알기위해 미분
	int i32ControlPointCount = m_vecControlPoints.size();

    if (i32ControlPointCount < 4)
    {
        return Vec3(1.0f, 0.0f, 0.0f);
    }

    param = WrapParameter(param);

    int i32P1Index = static_cast<int>(std::floor(param));
    float f32LocalT = param - static_cast<float>(i32P1Index);

    int i32P0Index = (i32P1Index - 1 + i32ControlPointCount) % i32ControlPointCount;
    int i32P2Index = (i32P1Index + 1) % i32ControlPointCount;
    int i32P3Index = (i32P1Index + 2) % i32ControlPointCount;

    const Vec3& p0 = m_vecControlPoints[i32P0Index];
    const Vec3& p1 = m_vecControlPoints[i32P1Index];
    const Vec3& p2 = m_vecControlPoints[i32P2Index];
    const Vec3& p3 = m_vecControlPoints[i32P3Index];

    float f32T  = f32LocalT;
    float f32T2 = f32T * f32T;

    Vec3 derivative =
        0.5f *
        (
            (-p0 + p2)
            + 2.0f * (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * f32T
            + 3.0f * (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * f32T2
        );

    return derivative;
}


Vec3 CatmullRomSpline::GetTangent(float param) const
{
	Vec3 derivative = GetDerivative(param);
	return NormalizeVec3(derivative);
}

class ArcLengthSample CatmullRomSpline::FrameAtArcLength(float s) const
{
    return m_ArcLengthTable.FrameAtArcLength(*this,s);
}

float CatmullRomSpline::TotalLength() const
{
    return m_ArcLengthTable.GetTotalLength();
}

bool CatmullRomSpline::IsReady() const
{
    return GetSegmentCount() >= 4 && TotalLength() > EPS;
}
