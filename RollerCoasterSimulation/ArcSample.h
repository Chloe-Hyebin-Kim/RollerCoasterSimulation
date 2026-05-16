#pragma once

#include "Vec3.h"

class ArcSample
{
public:
    ArcSample();

    float f32Length;// s == arc length
    float f32Param; //u
    class Vec3 vecPosition;
    class Vec3 vecTangent; //레일 진행방향
    class Vec3 vecNormal;  //롤코 타는 위쪽 면
    class Vec3 vecBinormal;//왼쪽->오른쪽 레일 벌리는 방향
};