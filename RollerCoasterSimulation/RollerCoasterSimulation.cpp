#include "RollerCoasterSimulation.h"

void RollerCoasterSimulation::Initialize()
{
    m_CatmullRomSpline.Clear();

    // 닫힌 Catmull-Rom track용 control points
    m_CatmullRomSpline.AddControlPoint(Vec3(0.0f, 6.0f, 0.0f));
    m_CatmullRomSpline.AddControlPoint(Vec3(10.0f, 12.0f, 5.0f));
    m_CatmullRomSpline.AddControlPoint(Vec3(20.0f, 4.0f, 0.0f));
    m_CatmullRomSpline.AddControlPoint(Vec3(25.0f, 10.0f, -10.0f));
    m_CatmullRomSpline.AddControlPoint(Vec3(10.0f, 3.0f, -20.0f));
    m_CatmullRomSpline.AddControlPoint(Vec3(-5.0f, 9.0f, -10.0f));

    m_ArcLengthTable.Sampling(m_CatmullRomSpline, 1000);

    m_f32CurrentS = 0.0f;
    m_f32InitialVelocity = 8.0f;
    m_f32Velocity = m_f32InitialVelocity;

    ArcSample startFrame = m_ArcLengthTable.FrameAtArcLength(m_CatmullRomSpline, 0.0f);
    m_f32StartHeight = startFrame.vecPosition.m_f32Y;
}

void RollerCoasterSimulation::Update(float dt)
{
    if (m_ArcLengthTable.IsEmpty())
    {
        return;
    }

    ArcSample frame = m_ArcLengthTable.FrameAtArcLength(m_CatmullRomSpline, m_f32CurrentS);
    float curHeight = frame.vecPosition.m_f32Y;
    float velocitySquared =m_f32InitialVelocity * m_f32InitialVelocity + MIN_VELOCITY * GRAVITY * (m_f32StartHeight - curHeight);//높이에 따라 계산
	velocitySquared = max(MIN_V_POW2, velocitySquared); //최소속도 0 방지
    m_f32Velocity = sqrt(velocitySquared);

    m_f32CurrentS += m_f32Velocity * dt; //이동

    if (m_f32CurrentS > m_ArcLengthTable.GetTotalLength())
    {
        m_f32CurrentS = fmod(m_f32CurrentS, m_ArcLengthTable.GetTotalLength());
    }
}

void RollerCoasterSimulation::Render()
{
    DrawRails();


    //ArcSample frame = m_ArcLengthTable.FrameAtArcLength(m_CatmullRomSpline, m_f32CurrentS);

    //디버깅용
   /* std::cout
        << "s = " << m_f32CurrentS
        << " | v = " << m_f32Velocity
        << " | P = ("
        << frame.vecPosition.m_f32X << ", "
        << frame.vecPosition.m_f32Y << ", "
        << frame.vecPosition.m_f32Z << ")"
        << " | T = ("
        << frame.vecTangent.m_f32X << ", "
        << frame.vecTangent.m_f32Y << ", "
        << frame.vecTangent.m_f32Z << ")"
        << std::endl;
        

        //////////////
   //Initialize();
   //const float dt = 0.016f; // 약 60 FPS
   //for (int i = 0; i < 300; ++i)
   //{
   //    Update(dt);
   //    Render();
   //}
    */
}

void RollerCoasterSimulation::DrawRails()
{

    const vector<ArcSample>& samples = m_ArcLengthTable.GetSampleArr();

    if (samples.empty())
    {
        return;
    }

    DrawCenterLine(samples);
    DrawSideLines(samples);
    DrawCrossLines(samples);

}

void RollerCoasterSimulation::DrawCenterLine(const vector<ArcSample>& samples)
{
    glLineWidth(3.0f);
    glColor3f(1.0f, 1.0f, 0.0f); // 노란색 중심선

    glBegin(GL_LINE_STRIP);

    for (const ArcSample& sample : samples)
    {
        const Vec3& p = sample.vecPosition;
        glVertex3f(p.m_f32X,p.m_f32Y,p.m_f32Z);
    }

    // 닫힌 track처럼 마지막 점과 첫 점을 연결!!!
    const Vec3& first = samples.front().vecPosition;
    glVertex3f(first.m_f32X,first.m_f32Y,first.m_f32Z);

    glEnd();

    glLineWidth(1.0f);
}

void RollerCoasterSimulation::DrawSideLines(const vector<ArcSample>& samples)
{
    glLineWidth(4.0f);
    glColor3f(0.8f, 0.8f, 0.8f);

    glBegin(GL_LINE_STRIP);

    for (const ArcSample& sample : samples)
    {
        Vec3 leftRail = sample.vecPosition + sample.vecBinormal * RAIL_HALF;
        Vec3 rightRail = sample.vecPosition - sample.vecBinormal * RAIL_HALF;

        glVertex3f(leftRail.m_f32X, leftRail.m_f32Y, leftRail.m_f32Z);
        glVertex3f(rightRail.m_f32X, rightRail.m_f32Y, rightRail.m_f32Z);
    }

    //마지막 점과 첫 점 연결
    {
        const ArcSample& sample = samples.front();
        Vec3 leftRail = sample.vecPosition + sample.vecBinormal * RAIL_HALF;
        Vec3 rightRail = sample.vecPosition - sample.vecBinormal * RAIL_HALF;

        glVertex3f(leftRail.m_f32X, leftRail.m_f32Y, leftRail.m_f32Z);
        glVertex3f(rightRail.m_f32X, rightRail.m_f32Y, rightRail.m_f32Z);
    }

    glEnd();

    glLineWidth(1.0f);

}

void RollerCoasterSimulation::DrawCrossLines(const vector<ArcSample>& samples)
{
    glLineWidth(2.0f);
    glColor3f(0.5f, 0.3f, 0.1f);

    glBegin(GL_LINES);

    const int tieStep = 20;
    const int totalSamples = m_ArcLengthTable.GetSamplesArrCount();

    for (int i = 0; i < totalSamples;i += tieStep)
    {
        const ArcSample& sample = samples[i];

        Vec3 leftRail = sample.vecPosition + sample.vecBinormal * RAIL_HALF;
        Vec3 rightRail = sample.vecPosition - sample.vecBinormal * RAIL_HALF;

        glVertex3f(leftRail.m_f32X, leftRail.m_f32Y, leftRail.m_f32Z);
        glVertex3f(rightRail.m_f32X, rightRail.m_f32Y, rightRail.m_f32Z);
    }

    glEnd();

    glLineWidth(1.0f);
}
