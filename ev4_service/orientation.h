#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#define RAD2DEG 57.295779513082f
#define gyroMeasError 0
#define beta sqrt(3.0f / 4.0f) // compute beta

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q);

class OrientationSolver {
public:
	float yaw = 0, pitch = 0, roll = 0;
	float SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0;
	void Solve(float ax, float ay, float az, float gx, float gy, float gz, float deltat);
};
