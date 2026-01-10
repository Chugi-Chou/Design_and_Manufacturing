//
// Created by zhouzhi on 2026/1/10.
//

#ifndef PID_H
#define PID_H

class PID {
public:
    PID(float kp, float ki, float kd, float kf, float max_out, float max_i, float d_alpha);
    float Calculate(float target, float current, float ff_term = 0.0f);
    void Reset();

private:
    float kp, ki, kd, kf;
    float max_out, max_i;
    float d_filter_alpha;
    float i_out, last_error, last_d_out;
};

#endif //PID_H
