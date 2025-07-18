#include "PID.h"
#include <iostream>
using namespace std;
PID::PID(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

float PID::compute(float target, float current) {
    // 计算误差
    //Calculation error
    float error = target - current;
    // 误差的累计
    //Accumulation of errors
    intergral += error;
    // 本次误差和上一次误差的差异
    //The difference between this error and the previous error
    derivative = error - prevError;
    // 套用pid的公式
    //Apply the formula of pid
    targetpoint = kp * error + ki * intergral + kd * derivative;
    // 记录上一次的误差
    //Record the last error
    prevError = error;
    return targetpoint;
}

void PID::reset() {
    targetpoint = 0;
    intergral = 0;
    derivative = 0;
    prevError = 0;
}

void PID::Set_PID(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}
