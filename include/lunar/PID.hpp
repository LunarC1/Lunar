#pragma once
#include "math.h"

class PID{
    public:
        // float error = 0;
        float kp = 0, ki = 0, kd = 0;

        float antiWindup = 0;
        
        float settle_error = 0;
        float settle_time = 0;
        float timeout = 0;
        float accumulated_error = 0;
        float previous_error = 0;
        float output = 0;
        float time_spent_settled = 0;
        float time_spent_running = 0;
        float update_ms = 10;

        // PID(float error, float kp, float ki, float kd, float starti);

        // PID(float error, float kp, float ki, float kd, float starti, float settle_error, float settle_time, float timeout);
        PID(float kp, float ki, float kd, float antiWindup);
        
        PID(float kp, float ki, float kd, float antiWindup, float settle_error, float settle_time);

        PID(float kp, float ki, float kd, float antiWindup, float settle_error, float settle_time, float timeout);

        PID(float kp, float ki, float kd, float antiWindup, float settle_error, float settle_time, float timeout, float update_ms);

        float update(float error);

        void reset();

        bool is_settled();
};