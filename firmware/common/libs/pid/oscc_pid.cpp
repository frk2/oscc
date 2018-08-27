/**
 * @file oscc_pid.cpp
 *
 */


#include <debug.h>
#include <communications.h>
#include "oscc_pid.h"

#define MAX_TORQUE 0.22
#define INT_LIMIT 0.2

double curr_angle = 0;
double setpoint = 0;
int enabled = 0;
pid_s pid = {};

float p_term;
float i_term;
float d_term;
int flag;


void pid_zeroize(pid_s *pid, float integral_windup_guard) {
    // set prev and integrated error to zero
    pid->prev_input = 0;
    pid->int_error = 0;
    pid->prev_error = 0;
    pid->prev_steering_angle = 0;
    pid->windup_guard = integral_windup_guard;
    pid->control = 0;
    setpoint = 0;
}


int pid_update(pid_s *pid, float setpoint, float input, float dt) {
    float diff;
//    float p_term;
//    float i_term;
//    float d_term;

    float curr_error = setpoint - input;

    static int count = 0;

    if (dt <= 0) {
        return PID_ERROR;
    }

    // integration with windup guarding
    flag = 0;
    if (fabs(curr_error) < 2) {
        if (fabs((pid->int_error * pid->integral_gain < INT_LIMIT)))
        {
            pid->int_error += (curr_error * dt);
            flag = 1;
        }
        else if (pid->int_error * pid->integral_gain > INT_LIMIT && curr_error < 0) {
            pid->int_error += (curr_error * dt);
            flag = 2;
        }
        else if (pid->int_error * pid->integral_gain < -INT_LIMIT && curr_error > 0) {
            pid->int_error += (curr_error * dt);
            flag = 3;
        }
    }
    else if (fabs(curr_error) >= 2 && fabs(curr_error) <= 5) {
        pid->int_error += 0.5 * (curr_error * dt);
    }
    else if (fabs(curr_error) > 5) {
        pid->int_error = 0;
        flag = 4;
    }



//    if (pid->int_error < -(pid->windup_guard)) {
//        pid->int_error = -(pid->windup_guard);
//    } else if (pid->int_error > pid->windup_guard) {
//        pid->int_error = pid->windup_guard;
//    }

    if (count > 0) {
        // differentiation
        diff = ((curr_error - pid->prev_error) / dt);
    }

    // scaling
    p_term = (pid->proportional_gain * curr_error);
    i_term = (pid->integral_gain * pid->int_error);
    d_term = (pid->derivative_gain * diff);

    // summation of terms
    pid->control = p_term + i_term + d_term;

    if (pid->control < -MAX_TORQUE) {
        pid->control = -MAX_TORQUE;
    } else if (pid->control > MAX_TORQUE) {
        pid->control = MAX_TORQUE;
    }

    // save current error as previous error for next iteration
    pid->prev_error = curr_error;
    count++;

    return PID_SUCCESS;
}

void update_pid() {
//     if (enabled) {
//         pid_update(&pid, setpoint, curr_angle, 0.02);
//         publish_torque(pid.control);
//     } else {
//         pid_zeroize(&pid, 0);
//     }

//    DEBUG_PRINT(millis());
//    DEBUG_PRINT(",");
//    DEBUG_PRINT(curr_angle);
//    DEBUG_PRINT(",");
//    DEBUG_PRINT(pid.control);
//    DEBUG_PRINT(",");
//    DEBUG_PRINT(pid.prev_error  );
//    DEBUG_PRINT(",");
//    DEBUG_PRINT(p_term);
//    DEBUG_PRINT(",");
//    DEBUG_PRINT(i_term);
//    DEBUG_PRINT(",");
//    DEBUG_PRINT(d_term);
//    DEBUG_PRINT(",");
//    DEBUG_PRINT(setpoint);
//    DEBUG_PRINT(",");
//    DEBUG_PRINTLN(flag);
}
