//
// Created by zhouzhi on 2026/1/10.
//

#ifndef MOTOR_H
#define MOTOR_H

#include "PID.h"
#include "main.h"

enum ControlMode_e { CURRENT_MODE, SPEED_MODE, POSITION_MODE };

class MotorM2006 {
public:
    MotorM2006(const PID& pos_pid, const PID& speed_pid);
    void SetTarget(ControlMode_e mode, float val);
    void UpdateFeedback(uint8_t data[8]);
    int16_t ExecuteControl();
    float GetCurrentAngle() const { return total_angle; }

private:
    PID pos_controller;
    PID speed_controller;
    ControlMode_e ctrl_mode;
    float target_value;
    float current_speed, total_angle;
    int16_t last_raw_angle;
    int32_t rounds;
    bool initialized;
};

#endif
