#pragma once

#include "bits/stdc++.h"
#include <GL/freeglut.h>

#include "Vec3.h"

using namespace std;

#define EPS 1e-6f

#define RAIL_HALF 2.0f

#define CAMERA_HEIGHT  0.7f
#define LOOK_AHEAD_DISTANCE 3.0f

#define MIN_VELOCITY 2.0f
#define MIN_V_POW2 MIN_VELOCITY*MIN_VELOCITY
#define GRAVITY 9.8f

//namespace
//{
//	constexpr float EPS = 1e-6f;
//}