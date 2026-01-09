//
// Created by zhouzhi on 2026/1/10.
//

#include "../Inc/PID.h"

PID::PID(float kp, float ki, float kd, float kf, float max_out, float max_i, float d_alpha)
    : kp(kp), ki(ki), kd(kd), kf(kf), max_out(max_out), max_i(max_i), d_filter_alpha(d_alpha) {
    Reset();
}

void PID::Reset() {
    i_out = last_error = last_d_out = 0.0f;
}

float PID::Calculate(float target, float current, float ff_term) {
    float error = target - current;
    float p_out = kp * error;

    i_out += ki * error;
    if (i_out > max_i) i_out = max_i;
    else if (i_out < -max_i) i_out = -max_i;

    float d_raw = kd * (error - last_error);
    float d_filtered = (d_filter_alpha * last_d_out) + (1.0f - d_filter_alpha) * d_raw;
    last_error = error;
    last_d_out = d_filtered;

    float out = p_out + i_out + d_filtered + (target * kf) + ff_term;
    if (out > max_out) out = max_out;
    else if (out < -max_out) out = -max_out;

    return out;
}
