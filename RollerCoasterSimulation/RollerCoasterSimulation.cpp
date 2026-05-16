#include "RollerCoasterSimulation.h"

void RollerCoasterSimulation::Initialize()
{
    m_CatmullRomSpline.Clear();

	//레일 만들기
    m_CatmullRomSpline.AddControlPoint(Vec3(0.0f, 6.0f, 0.0f));
    m_CatmullRomSpline.AddControlPoint(Vec3(10.0f, 12.0f, 5.0f));
    m_CatmullRomSpline.AddControlPoint(Vec3(22.0f, 16.0f, 0.0f));
    m_CatmullRomSpline.AddControlPoint(Vec3(35.0f, 7.0f, -8.0f));
    m_CatmullRomSpline.AddControlPoint(Vec3(42.0f, 3.0f, -22.0f));

    m_CatmullRomSpline.AddControlPoint(Vec3(35.0f, 10.0f, -38.0f));
    m_CatmullRomSpline.AddControlPoint(Vec3(18.0f, 18.0f, -48.0f));
    m_CatmullRomSpline.AddControlPoint(Vec3(0.0f, 8.0f, -52.0f));
    m_CatmullRomSpline.AddControlPoint(Vec3(-18.0f, 4.0f, -44.0f));
    m_CatmullRomSpline.AddControlPoint(Vec3(-32.0f, 13.0f, -30.0f));

    m_CatmullRomSpline.AddControlPoint(Vec3(-42.0f, 20.0f, -10.0f));
    m_CatmullRomSpline.AddControlPoint(Vec3(-35.0f, 9.0f, 10.0f));
    m_CatmullRomSpline.AddControlPoint(Vec3(-20.0f, 5.0f, 25.0f));
    m_CatmullRomSpline.AddControlPoint(Vec3(0.0f, 14.0f, 32.0f));
    m_CatmullRomSpline.AddControlPoint(Vec3(18.0f, 22.0f, 24.0f));

    m_CatmullRomSpline.AddControlPoint(Vec3(32.0f, 11.0f, 12.0f));
    m_CatmullRomSpline.AddControlPoint(Vec3(24.0f, 5.0f, -4.0f));
    m_CatmullRomSpline.AddControlPoint(Vec3(8.0f, 7.0f, -8.0f));

	m_ArcLengthTable.Sampling(m_CatmullRomSpline, 3000);//3000개 샘플링해서 길이 테이블 만들기

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

	ArcSample frame = m_ArcLengthTable.FrameAtArcLength(m_CatmullRomSpline, m_f32CurrentS);//현재 위치에 해당하는 프레임

    float curHeight = frame.vecPosition.m_f32Y;
    float velocitySquared =m_f32InitialVelocity * m_f32InitialVelocity + MIN_VELOCITY * GRAVITY * (m_f32StartHeight - curHeight);//높이에 따라 계산
	velocitySquared = max(MIN_V_POW2, velocitySquared); //최소속도 0 방지
	m_f32Velocity = sqrt(velocitySquared);//현재속도= sqrt(최초속도제곱 + 2 * g * (시작높이 - 현재높이))

    m_f32CurrentS += m_f32Velocity * dt; //이동
    //if (m_f32CurrentS > m_ArcLengthTable.GetTotalLength())// 전체 길이보다 이동거리가 크면 
    //{
    //	m_f32CurrentS = fmod(m_f32CurrentS, m_ArcLengthTable.GetTotalLength());// 전체를 반복해서 타도록
    //}

    float  totalLength = m_ArcLengthTable.GetTotalLength();
    if (m_ArcLengthTable.GetTotalLength() > EPS)
    {
        m_f32CurrentS = fmod(m_f32CurrentS, totalLength);

        if (m_f32CurrentS < 0.0f)
        {
            m_f32CurrentS += totalLength;
        }
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

void RollerCoasterSimulation::ApplyFirstPersonCamera()
{
    if (m_ArcLengthTable.IsEmpty())
    {
        return;
    }

    ArcSample frame = m_ArcLengthTable.FrameAtArcLength(m_CatmullRomSpline, m_f32CurrentS);

    Vec3 position = frame.vecPosition;
    Vec3 tangent = NormalizeVec3(frame.vecTangent);
    Vec3 normal = NormalizeVec3(frame.vecNormal);

    //P == 현재 롤코 위치
    //T == 진행 방향
    //N == 카메라의 정수리방향

    //eye = P + N * cameraHeight;
    //center = eye + T * lookAheadDistance;
    //up = N;

    Vec3 eye = position + normal * CAMERA_HEIGHT;
    Vec3 center = eye + tangent * LOOK_AHEAD_DISTANCE;
    Vec3 up = normal; //카메라 정수리 방향 그대로 씀

    gluLookAt(eye.m_f32X, eye.m_f32Y,eye.m_f32Z,center.m_f32X,center.m_f32Y,center.m_f32Z,up.m_f32X,up.m_f32Y,up.m_f32Z);
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
    glLineWidth(RAIL_THICKNESS);
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
    glLineWidth(RAIL_THICKNESS);
    glColor3f(0.8f, 0.8f, 0.8f);

    // 왼쪽 rail
    glBegin(GL_LINE_STRIP);

    for (const ArcSample& sample : samples)
    {
        Vec3 leftRail = sample.vecPosition + sample.vecBinormal * RAIL_HALF;

        glVertex3f(leftRail.m_f32X,leftRail.m_f32Y,leftRail.m_f32Z);
    }

    //마지막 점과 첫 점 연결
    {
        const ArcSample& sample = samples.front();
        Vec3 leftRail = sample.vecPosition + sample.vecBinormal * RAIL_HALF;

        glVertex3f(leftRail.m_f32X,leftRail.m_f32Y,leftRail.m_f32Z);
    }

    glEnd();

    // 오른쪽 rail
    glBegin(GL_LINE_STRIP);

    for (const ArcSample& sample : samples)
    {
        Vec3 rightRail = sample.vecPosition - sample.vecBinormal * RAIL_HALF;

        glVertex3f(rightRail.m_f32X,rightRail.m_f32Y,rightRail.m_f32Z);
    }

    //마지막 점과 첫 점 연결
    {
        const ArcSample& sample = samples.front();
        Vec3 rightRail = sample.vecPosition - sample.vecBinormal * RAIL_HALF;

        glVertex3f(rightRail.m_f32X,rightRail.m_f32Y,rightRail.m_f32Z);
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
