//
// Created by zhouzhi on 2026/1/10.
//
#include "Motor.h"

MotorM2006::MotorM2006(const PID& pos_pid, const PID& speed_pid)
    : pos_controller(pos_pid), speed_controller(speed_pid),
      ctrl_mode(SPEED_MODE), target_value(0), initialized(false) {}

void MotorM2006::UpdateFeedback(uint8_t data[8]) {
    int16_t raw_angle = (data[0] << 8) | data[1];
    current_speed = (float)((int16_t)((data[2] << 8) | data[3]));

    if (!initialized) {
        last_raw_angle = raw_angle;
        initialized = true;
        return;
    }
    if (raw_angle - last_raw_angle > 4096) rounds--;
    else if (raw_angle - last_raw_angle < -4096) rounds++;
    last_raw_angle = raw_angle;
    total_angle = ((float)rounds * 360.0f + (float)raw_angle * 360.0f / 8192.0f) / 36.0f;
}

void MotorM2006::SetTarget(ControlMode_e mode, float val) {
    ctrl_mode = mode;
    target_value = val;
}

int16_t MotorM2006::ExecuteControl() {
    float final_current = 0;

    switch (ctrl_mode) {
        case POSITION_MODE: {
            // 位置环输出作为速度环期望
            float speed_target = pos_controller.Calculate(target_value, total_angle);
            final_current = speed_controller.Calculate(speed_target, current_speed);
            break;
        }
        case SPEED_MODE:
            final_current = speed_controller.Calculate(target_value, current_speed);
            break;
        case CURRENT_MODE:
            final_current = target_value;
            break;
    }
    return (int16_t)final_current;
}

bool MotorM2006::IsTargetReached(float const threshold) {

    if (ctrl_mode != POSITION_MODE) return true;
    float error = target_value - total_angle;

    if (error < 0) error = -error;

    return (error < threshold);
}