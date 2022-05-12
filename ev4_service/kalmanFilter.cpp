#include "kalmanFilter.h"

float KalmanFilter::Filter(float val) {
    Pc = P + varProcess;
    G = Pc/(Pc + varVolt);
    P = (1-G)*Pc;
    Xp = Xe;
    Zp = Xp;
    Xe = G*(val-Zp)+Xp;
    return Xe;
}