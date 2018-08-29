/**
 * @file oscc_pid.cpp
 *
 */


#include <debug.h>
#include <communications.h>
#include "oscc_pid.h"

#define MAX_TORQUE 0.22
#define INT_LIMIT 0.25

double curr_angle = 0;
double setpoint = 0;
int enabled = 0;
int can_use_diff = 0;
int new_data = 0;
pid_s pid = {};

unsigned long last_time = 0;

float p_term;
float i_term;
float d_term;
int flag;

bool has_estimate= false;
BLA::Matrix<3, 1> latest_mean;
BLA::Matrix<3, 3> latest_covariance;

void KalmanUpdate(
    const BLA::Matrix<3, 1> &prev_mean,
    const BLA::Matrix<3, 3> &prev_covariance,
    const BLA::Matrix<3, 3>
        &F /* Dynamics based on previous estimate. */,
    const BLA::Matrix<3, 1> &G /* Random noise. */,
    const BLA::Matrix<1, 3> &observation_matrix,
    float observation_value, float observation_variance,
    BLA::Matrix<3, 1> &next_mean,
    BLA::Matrix<3, 3> &next_covariance) {
  // Predicted current value based only on previous value and velocity.
  // (DxD) * (Dx1) = (Dx1)
  const BLA::Matrix<3, 1> x_k_k_minus_1 = F * prev_mean;
  // Covariance estimate based only on previous value and velocity estimate,
  // without taking current observation into account.
  // First component: (DxD) * (DxD) * (DxD) = (DxD)
  // Second component: scalar * (Dx1) * (1xD) = (DxD)
  const BLA::Matrix<3, 3> P_k_k_minus_1 =
      F * prev_covariance * (~F) + G * (~G);
  // Observation residual.
  // First component: scalar
  // Second component: (1xD) * (Dx1) = (1x1) = scalar
  float y_k = observation_value - (observation_matrix * x_k_k_minus_1)(0,0);
  // Observation residual covariance.
  // First component: (1xD) * (DxD) * (Dx1) = (1x1) = scalar
  // Second component: scalar
  const float S =
      (observation_matrix * P_k_k_minus_1 * (~observation_matrix))(0,0) + observation_variance;
  // Optimal Kalman gain.
  // (DxD) * (Dx1) / scalar = (Dx1)
  const BLA::Matrix<3, 1> K =
      P_k_k_minus_1 * ~(observation_matrix) / S;
  // Final estimate is estimate from previous step, adjusted by the
  // observation residual.
  // First component: (Dx1)
  // Second component: scalar * (Dx1) = (Dx1)
  next_mean = x_k_k_minus_1 +  K * y_k;
  // Final estimate covariance.
  // ((DxD) - (Dx1) * (1xD)) * (DxD) = (DxD)
  BLA::Matrix<3,3> I;
  I << 	1.0, 0.0, 0.0,
	0.0, 1.0, 0.0,
	0.0, 0.0, 1.0;
  next_covariance = (I - K * observation_matrix) *  P_k_k_minus_1;
}

void pid_zeroize(pid_s *pid, float integral_windup_guard) {
    // set prev and integrated error to zero
    pid->prev_input = 0;
    pid->int_error = 0;
    pid->prev_error = 0;
    pid->prev_steering_angle = 0;
    pid->windup_guard = integral_windup_guard;
    pid->control = 0;
    pid->prev_diff = 0;
    setpoint = 0;
    can_use_diff = 0;
}


int pid_update(pid_s *pid, float setpoint, float input, float dt) {
    float diff = 0.0;
//    float p_term;
//    float i_term;
//    float d_term;

    float curr_error = setpoint - input;

    if (dt <= 0) {
        return PID_ERROR;
    }

    // integration with windup guarding
    flag = 0;
    if (fabs(curr_error) < 2) {
      
	float last_int_error = pid->int_error;
        pid->int_error += (curr_error * dt);
        flag = 1;
	    
	if (fabs(pid->int_error * pid->integral_gain) > INT_LIMIT)
        {
           pid->int_error = last_int_error; //reset integreation error under limit 
           flag = 2;
        }
        
    }
    else if (fabs(curr_error) >= 2 && fabs(curr_error) <= 5) {
      
	float last_int_error = pid->int_error;
        pid->int_error += 0.5*(curr_error * dt);
        flag = 3;
	    
	if (fabs(pid->int_error * pid->integral_gain) > INT_LIMIT)
        {
           pid->int_error = last_int_error; //reset integreation error under limit 
           flag = 7;
        }
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

    if (can_use_diff) {
        // differentiation
        float new_diff = ((curr_error - pid->prev_error) / dt);
	diff = (new_diff + pid->prev_diff) / 2.0;
	pid->prev_diff = new_diff;
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
    can_use_diff = 1;

    return PID_SUCCESS;
}

void update_pid() {
    if (new_data)
    {
      new_data = 0;
  
      float delta_t_sec = 0.0;
      unsigned long curr_time = millis();
      delta_t_sec = (curr_time - last_time)/1000.0;
    
      if (enabled) {
	
	  BLA::Matrix<3, 1> next_mean;
	  BLA::Matrix<3, 3> next_covariance;
	  
	  if (!has_estimate)
	  {
	    next_mean << curr_angle, 0.0, 0.0;
	    next_covariance <<  1.0, 0.0, 0.0,
			      0.0, 1.0, 0.0,
			      0.0, 0.0, 1.0;
	    has_estimate = true;
	  } else
	  {
	    
	    BLA::Matrix<3,3> F;
	    F << 1.0, delta_t_sec, (delta_t_sec * delta_t_sec * 0.5),
		0.0, 1.0,  delta_t_sec,
		0.0, 0.0, 0.8;
	  
	    BLA::Matrix<3, 1> G;
	    G << (delta_t_sec * delta_t_sec * delta_t_sec / 6.0), (delta_t_sec * delta_t_sec * 0.5), delta_t_sec;
	    
	    BLA::Matrix<1,3> obs_matr;
	    obs_matr << 1.0, 0.0, 0.0;
	    
	    float observ_var = 0.14; //в коде чувака было 2, но так как у нас угол это угол установки колеса то 2*37/520
	    
	    //KalmanUpdate(latest_mean, latest_covariance, F, G, obs_matr, curr_angle, observ_var, next_mean, next_covariance);
	    
	    pid_update(&pid, setpoint, curr_angle, delta_t_sec);
	    
	    publish_torque(pid.control);
	  }
	  latest_mean = next_mean;
	  latest_covariance = next_covariance;
	  
      } else {
	  publish_torque(0.0);
	  pid_zeroize(&pid, 0);
      }
      
      last_time = curr_time;

      int dt = (int)(delta_t_sec*1000);

      DEBUG_PRINT(dt);
      DEBUG_PRINT(",");
      DEBUG_PRINT(curr_angle);
      DEBUG_PRINT(",");
      DEBUG_PRINT(pid.control);
      DEBUG_PRINT(",");
      DEBUG_PRINT(pid.prev_error  );
      DEBUG_PRINT(",");
      DEBUG_PRINT(p_term);
      DEBUG_PRINT(",");
      DEBUG_PRINT(i_term);
      DEBUG_PRINT(",");
      DEBUG_PRINT(d_term);
      DEBUG_PRINT(",");
      DEBUG_PRINT(setpoint);
      DEBUG_PRINT(",");
      DEBUG_PRINTLN(flag);
    }
}
