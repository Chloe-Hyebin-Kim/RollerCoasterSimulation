#pragma once

#include "Vec3.h"

class ArcSample
{
public:
    ArcSample();

    float f32D;// s == arc length
    float f32Param; //u
    class Vec3 vecPosition;
    class Vec3 vecTangent;
    class Vec3 vecNormal;
    class Vec3 vecBinormal;
};