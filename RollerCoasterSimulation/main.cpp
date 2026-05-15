#include "RollerCoasterSimulation.h"
#include "CatmullRomSpline.h"

int main(int argc, char** argv)
{
    CatmullRomSpline spline;

    spline.AddControlPoint(Vec3(0.0f, 0.0f, 0.0f));
    spline.AddControlPoint(Vec3(10.0f, 5.0f, 0.0f));
    spline.AddControlPoint(Vec3(20.0f, 0.0f, 0.0f));
    spline.AddControlPoint(Vec3(30.0f, 10.0f, 0.0f));
    spline.AddControlPoint(Vec3(40.0f, 0.0f, 0.0f));

    for (float f32U = 0.0f; f32U < static_cast<float>(spline.GetSegmentCount()); f32U += 0.25f)
    {
        Vec3 position = spline.GetPoint(f32U);
        Vec3 tangent = spline.GetTangent(f32U);

        std::cout
            << "u = " << f32U
            << " | P(u) = ("
            << position.m_f32X << ", "
            << position.m_f32Y << ", "
            << position.m_f32Z << ")"
            << " | T(u) = ("
            << tangent.m_f32X << ", "
            << tangent.m_f32Y << ", "
            << tangent.m_f32Z << ")"
            << std::endl;
    }

    return 0;
}
