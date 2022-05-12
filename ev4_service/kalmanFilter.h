#pragma once

class KalmanFilter {
public:
    float varVolt = 90;
    float varProcess = 0.2;

    float Pc = 0.0;
    float G = 0.0;
    float P = 1.0;
    float Xp = 0.0;
    float Zp = 0.0;
    float Xe = 0.0;
    KalmanFilter() {}
    float Filter(float val);
};