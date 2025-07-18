#!/usr/bin/env python
# coding:utf-8
class SinglePID:
    def __init__(self, P=0.1, I=0.0, D=0.1):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.error = 0
        self.intergral = 0
        self.derivative = 0
        self.prevError = 0
        self.pid_reset()

    def Set_pid(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.pid_reset()

    def pid_compute(self, target, current):
        self.error = target - current
        self.intergral += self.error
        self.derivative = self.error - self.prevError
        self.targetpoint = self.Kp * self.error + self.Ki * self.intergral + self.Kd * self.derivative
        self.prevError = self.error
        return self.targetpoint

    def pid_reset(self):
        self.error = 0
        self.intergral = 0
        self.derivative = 0
        self.prevError = 0
        self.targetpoint = 0