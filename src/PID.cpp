#include "PID.h"
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, double max_i_error_, bool twiddle_active_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  max_i_error = max_i_error_;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  prev_cte = 0.0;  

  
  // Twiddle parameters
  twiddle_active = twiddle_active_;
  cumulative_err = 0.0;
  best_err = 0.0;
  //p = [0, 0, 0];
  //dp = [1, 1, 1];
}

void PID::UpdateError(double cte, double diff_time) {
  /**
   * TODO: Update PID errors based on cte.
   */
  p_error = cte;
  d_error = (cte - prev_cte) / (diff_time / 1000);
  i_error += cte;
  prev_cte = cte;
  
  // Clamp the i term to prevent integral windup
  if(abs(i_error) > max_i_error){
    if(i_error < 0.0) {
      i_error = -max_i_error;
    } else {
      i_error = max_i_error;
    }
  }
  
  std::cout << "p_err: " << p_error << "; i_err: " << i_error << "; d_err: " << d_error << std::endl;
  
  // Apply Twiddle Algorithm
  /*
  if(twiddle_active){
    if(cumulative_err < best_err){
      best_err = cumulative_err;
      
      if(
    }
  }
  */
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */  
  double p_component = -Kp * p_error;
  double d_component = -Kd * d_error;
  double i_component = -Ki * i_error;
  std::cout << "P: " << p_component << "; D: " << d_component << "; I: " << i_component << std::endl;
  return (-Kp * p_error) - (Kd * d_error) - (Ki * i_error);  // TODO: Add your total error calc here!
}