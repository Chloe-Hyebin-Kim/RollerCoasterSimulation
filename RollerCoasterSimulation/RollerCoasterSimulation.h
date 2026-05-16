#pragma once
#include "stdafx.h"

#include "ArcLengthTable.h"
#include "CatmullRomSpline.h"

class RollerCoasterSimulation
{
public:
    void Initialize();
    void Update(float dt);
    void Render();

    void ApplyFirstPersonCamera();

private:
    void DrawRails();
    void DrawCenterLine(const vector<ArcSample>& samples);
    void DrawSideLines(const vector<ArcSample>& samples);
    void DrawCrossLines(const vector<ArcSample>& samples);

private:
    CatmullRomSpline m_CatmullRomSpline;
    ArcLengthTable m_ArcLengthTable;

    // Energy conservation:
    // 1/2 v^2 + g h = constant = 1/2 v0^2 + g h0
    //float m_f32Gravity = GRAVITY;

    float m_f32InitialVelocity = 8.0f;
    float m_f32StartHeight = 0.0f;

    float m_f32CurrentS = 0.0f;
    float m_f32Velocity = 0.0f;//높이에 따라 계산//v = sqrt(v0^2 + 2 * g * (startHeight - currentHeight))
};