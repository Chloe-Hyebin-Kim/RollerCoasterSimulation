#pragma once

#include "bits/stdc++.h"
#include "Util.h"
#include "Vec3.h"
//#include <GL/freeglut.h>

using namespace std;

#define EPS 1e-6f






// spline으로 track 정의
// arc length table 또는 numerical integration 준비
// 매 프레임 속도 계산
// 실제 이동 거리 업데이트
// s에 해당하는 u 찾기
// 위치 계산
// 카메라 방향 계산
// frame 또는 quaternion으로 orientation 계산

class CatmullRomSpline
{

	//p0 - p1 - p2 - p3 - p4
	
	//P(t) = 0.5​{ 2P_1​ + (−P_0​ + P_2​)t 
	//			 + (2P_0​−5P_1​ + 4P_2​−P_3​)t ^ 2 
	//			 + (−P_0​ + 3P_1​−3P_2​ + P_3​)t ^ 3 }

	//t에서 곡선이 향하는 방향
	//p'(t) = 0.5​{ (−P_0​ + P_2​)
	//			+ 2(2P_0​−5P_1​ + 4P_2​−P_3​)t 
	//			+ 3(−P_0​ + 3P_1​−3P_2​ + P_3​)t ^ 2 }

public:
	CatmullRomSpline() = default;

	//param = segment index + local interpolation
public:
	void Clear();
	int GetSegmentCount() const;
	const vector<Vec3>& GetControlPoints() const;
	void AddControlPoint(const Vec3& point);

	Vec3 GetPoint(float param) const; //P(t) ==spline 위 위치
	Vec3 GetDerivative(float param) const; //P'(t) ==진행 방향 + 변화량
	Vec3 GetTangent(float param) const; //normalize(P'(t))==정규화된 방향

private:
	float WrapParameter(float f32U) const;

private:
	vector<Vec3> m_vecControlPoints;

public:
	void InitializeDefaultTrack();
	void Rebuild(int sampleCount);

	const CatmullRomSpline& Spline() const;
	const class ArcLengthTable& ArcLengthTable() const;

	class ArcLengthSample FrameAtArcLength(float s) const;
	float TotalLength() const;
	bool IsReady() const;

private:
	class ArcLengthTablef m_ArcLengthTable;
	//class CatmullRomSpline m_CatmullRomSpline;
};