#pragma once

#include "bits/stdc++.h"
//#include <GL/freeglut.h>

#include "RollerCoasterSimulation.h"
#include "CatmullRomSpline.h"
#include "Vec3.h"


struct ArcLengthSample
{
    float f32D;// arc length
    float f32Param;
    Vec3 vecPosition;
    Vec3 vecTangent;
    Vec3 vecNormal;
    Vec3 vecBinormal;
};

class ArcLengthTable
{
public:
    void Sampling(const class CatmullRomSpline& spline, int i32SampleCount);
    float GetTotalLength() const;

    ArcLengthSample FrameAtArcLength(const class CatmullRomSpline& spline, float s) const;
    float GetParamFromD(float f32D) const;

private:
    std::vector<ArcLengthSample> m_arrSamples;
    float m_f32TotalLength = 0.0f;
};