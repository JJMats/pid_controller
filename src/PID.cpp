#include "PID.h"
#include <iostream>
#include <limits>
#include <fstream>
#include <string>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

//void PID::Init(double Kp_, double Ki_, double Kd_, double max_i_error_, bool twiddle_active_, double initial_dp, int eval_steps) {
void PID::Init(){
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  
  /** 
   *  Read all parameters in from configuration text file
   *    Kp, Ki, and Kd are the initial gain parameters
   *    max_i_error is the maximum value that the I term will be clamped to
   *    twiddle_active is a boolean flag to determine whether or not to enable the Twiddle algorithm
   *    initial_dp_p, initial_dp_i, and initial_dp_d are the incremental adjustments to use for
   *      the Twiddle algorithm dp vector
   *    evaluation_steps is the quantity of steps to calculate a total error from
   */
  std::string line;
  std::ifstream param_file("../res/pid_params.txt");  
  if(param_file.is_open()){
    int line_count = 0;
    while(getline(param_file, line)){
      switch(line_count){
        case 0:
          Kp = std::stod(line);
          break;
        case 1:
          Ki = std::stod(line);
          break;
        case 2:
          Kd = std::stod(line);
          break;
        case 3:
          max_i_error = std::stod(line);
          break;
        case 4:
          twiddle_active = std::stoi(line);
          break;
        case 5:
          initial_dp_p = std::stod(line);
          break;
        case 6:
          initial_dp_i = std::stod(line);
          break;
        case 7:
          initial_dp_d = std::stod(line);
          break;
        case 8:
          evaluation_steps = std::stoi(line);
          break;
      }
      line_count += 1;
    }
    param_file.close();
  }
  
  // Initialize previous cross-track error to zero
  prev_cte = 0.0;
  
  // Initialize Twiddle parameters
  cumulative_err = 0.0;
  best_err = std::numeric_limits<double>::max();
  tuning_param_index = 0;
  step_count = 0;
  stabilization_steps = 100;
  update_count = 0;
  last_change = 0;
  
  p.push_back(Kp);
  p.push_back(Kd);
  p.push_back(Ki);
  dp.push_back(initial_dp_p);
  dp.push_back(initial_dp_d);
  dp.push_back(initial_dp_i);
}

void PID::UpdateError(double cte, double diff_time, double speed) {
  /**
   * TODO: Update PID errors based on cte.
   */
  p_error = cte;
  d_error = (cte - prev_cte) / (diff_time / 1000);
  i_error += cte;  
  
  // Clamp the i term to prevent integral windup
  if(abs(i_error) > max_i_error){
    if(i_error < 0.0) {
      i_error = -max_i_error;
    } else {
      i_error = max_i_error;
    }
  }  
    
  // Track error over a long period of time, such as an entire lap around the track.
  // Since messages are received at around 20Hz, the stabilize count should be around 20 * lap time (s)
  // Make small adjustments to the PID values and determine if the cumulative error has changed.
  step_count += 1;
  // Start accumulating the error value after the vehicle has time to stabliize from the last change
  if(step_count % evaluation_steps > stabilization_steps){
    cumulative_err += cte * cte;
  }
  
  //std::cout << "p_err: " << p_error << "; d_err: " << d_error << "; i_err: " << i_error << "; speed: " << speed << "; c_err: " << cumulative_err << "; c_err_n: " << cumulative_err / double(evaluation_steps - stabilization_steps) << std::endl;
        
  /*
  // Apply Twiddle Algorithm  
  if(twiddle_active && step_count % evaluation_steps == 0){
    update_count += 1;

    p[tuning_param_index] += dp[tuning_param_index];

    if(cumulative_err <= best_err){
      best_err = cumulative_err;
      dp[tuning_param_index] *= 1.1;
      std::cout << "BETTER: Cumulative error: " << cumulative_err << "; Best error: " << best_err << "; INCREASED dp for index " << tuning_param_index << " to: " << dp[tuning_param_index] << "; p: " << p[tuning_param_index] << std::endl;
      tuning_param_index = (tuning_param_index + 1) % 3;      
      index_changed = true;
    } else if(!index_changed){
      // Index did not change on last iteration, which means that the new error was worse than the best error
      // Decrease the dp value
      p[tuning_param_index] += dp[tuning_param_index];
      dp[tuning_param_index] *= 0.9;
      std::cout << "WORSE: Cumulative error: " << cumulative_err << "; Best error: " << best_err << "; DECREASED dp for index " << tuning_param_index << " to: " << dp[tuning_param_index] << "; p: " << p[tuning_param_index] << std::endl;

      tuning_param_index = (tuning_param_index + 1) % 3;
      index_changed = true;
    } else {
      // Error was worse than last iteration, so decrease the p value and try again with this same parameter
      p[tuning_param_index] -= 2 * dp[tuning_param_index];
      std::cout << "WORSE: Cumulative error: " << cumulative_err << "; Best error: " << best_err << "; INVERTED dp for index " << tuning_param_index << " to: " << dp[tuning_param_index] << "; p: " << p[tuning_param_index] << std::endl;
      index_changed = false;
    }
  */
  
  // Apply Algorithm "B"
  if(twiddle_active && step_count % evaluation_steps == 0){
    update_count += 1;

    if(index_changed || update_count == 1){
      // Store the cumulative error, then make a change to the parameter
      p[tuning_param_index] += dp[tuning_param_index];
      last_change = 1;
      index_changed = false;
    } else {
      if(cumulative_err <= best_err){
        best_err = cumulative_err;

        if(last_change == 1){
          // Since the value was increased last, increase it again
          p[tuning_param_index] += dp[tuning_param_index];
        } else if (last_change == -1){
          // Since the value was decreased last, decrease it again
          p[tuning_param_index] -= dp[tuning_param_index];
        }
      } else {
        if (last_change == 1){
          // It was incresaed last, and it did not improve the error, so decrease the value and see if it helps
          p[tuning_param_index] -= 2* dp[tuning_param_index];
          last_change = -1;
        } else {
          // The last decrease did not help, so use the last good value and move on to the next term.
          p[tuning_param_index] += dp[tuning_param_index];
          tuning_param_index = (tuning_param_index + 1) % 3;
          index_changed = true;
        }        
      }
    }
    
    // Assign new gain values
    Kp = p[0];
    Kd = p[1];
    Ki = p[2];
    
    // Write PID tuning results to output file for review
    std::string line;
    std::ofstream output_file;
    output_file.open("../res/pid_results.txt", std::ios::out | std::ios::app);
    if(output_file.is_open()){
      output_file << "Iteration " << update_count << ": Cumulative error: " << cumulative_err << "; Best error: " << best_err << "; P: " << p[0] << "; I: " << p[2] << "; D: " << p[1] << "\n";
      output_file.close();  
    }
    
    // Reset error and step count values
    cumulative_err = 0.0;
    step_count = 0;    
  }
  
  std::cout << "**** Iteration: " << step_count << "; P: " << p[0] << "; D: " << p[1] << "; I: " << p[2] << "; Sum: " << p[0] + p[1] + p[2] << std::endl;
  
  // Set previous cross-track error to new cross track error for next D term calculation
  prev_cte = cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */  
  
  //double p_component = -Kp * p_error;
  //double d_component = -Kd * d_error;
  //double i_component = -Ki * i_error;
  //std::cout << "P: " << p_component << "; D: " << d_component << "; I: " << i_component << std::endl;
  
  return (-Kp * p_error) - (Kd * d_error) - (Ki * i_error);  // TODO: Add your total error calc here!
}