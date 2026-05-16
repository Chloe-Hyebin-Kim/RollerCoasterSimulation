#pragma once
#include "stdafx.h"
#include "ArcSample.h"

#include "RollerCoasterSimulation.h"
#include "CatmullRomSpline.h"

class ArcLengthTable
{
public:
	ArcLengthTable() = default;
    void Sampling(const class CatmullRomSpline& spline, int i32SampleCount);
    ArcSample FrameAtArcLength(const class CatmullRomSpline& spline, float s) const;
    float CalParamFromLength(float f32D) const;

public:
    const vector<ArcSample>& GetSampleArr() const { return m_arrSamples; };
    float GetTotalLength() const { return m_f32TotalLength; };
    bool IsEmpty() const;

private:
    std::vector<ArcSample> m_arrSamples;
    float m_f32TotalLength = 0.0f;
};