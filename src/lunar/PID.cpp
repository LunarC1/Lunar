#include "math.h"
#include "lunar/PID.hpp"

PID::PID(float kp, float ki, float kd, float antiWindup, float settle_error, float settle_time, float timeout, float update_ms) :
    kp(kp),
    ki(ki),
    kd(kd),
    antiWindup(antiWindup),
    settle_error(settle_error),
    settle_time(settle_time),
    timeout(timeout),
    update_ms(update_ms)
{};

PID::PID(float kp, float ki, float kd, float antiWindup, float settle_error, float settle_time, float timeout) :
    kp(kp),
    ki(ki),
    kd(kd),
    antiWindup(antiWindup),
    settle_error(settle_error),
    settle_time(settle_time),
    timeout(timeout)
{};

PID::PID(float kp, float ki, float kd, float antiWindup, float settle_error, float settle_time) :
    kp(kp),
    ki(ki),
    kd(kd),
    antiWindup(antiWindup),
    settle_error(settle_error),
    settle_time(settle_time)
{};

PID::PID(float kp, float ki, float kd, float antiWindup) :
    kp(kp),
    ki(ki),
    kd(kd),
    antiWindup(antiWindup)
{};

float PID::update(float error){
    if (fabs(error) < antiWindup) accumulated_error+=error;
    // Checks if the error has crossed 0, and if it has, it eliminates the integral term.
    if ((error>0 && previous_error<0)||(error<0 && previous_error>0)) accumulated_error = 0; 

    output = kp*error + ki*accumulated_error + kd*(error-previous_error);

    previous_error = error;

    if(fabs(error)<settle_error) time_spent_settled+=10;
    else time_spent_settled = 0;

    time_spent_running+=10;

    return output;
}

void PID::reset(){
    time_spent_running = 0;
}

bool PID::is_settled(){
    if (time_spent_running>timeout && timeout != 0) return(true);
    if (time_spent_settled>settle_time) return(true);
    return(false);
}