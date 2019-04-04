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
  std::string line;
  std::ifstream param_file("pid_params.txt");  
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
  
  /*
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  max_i_error = max_i_error_;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  */
  prev_cte = 0.0;

  
  // Twiddle parameters
  //twiddle_active = twiddle_active_;
  cumulative_err = 0.0;
  best_err = std::numeric_limits<double>::max();
  tuning_param_index = 0;
  //stabilize_count = stabilize_steps;
  step_count = 0;
  stabilization_steps = 100;
  //evaluation_steps = eval_steps;
  update_count = 0;
  
  p.push_back(Kp);
  p.push_back(Kd);
  p.push_back(Ki);
  dp.push_back(initial_dp_p);
  dp.push_back(initial_dp_d);
  dp.push_back(initial_dp_i);
  
  /*
  for(int i = 0; i < 3; ++i){
    //p.push_back(0.0);    
    switch(i){
      case 0:
        p.push_back(Kp);
        break;
      case 1:
        p.push_back(Kd);
        break;
      case 2:
        p.push_back(Ki);
    }
    dp.push_back(initial_dp);
  }
  */
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
  // Since messages are received at around 20Hz, the stabilize count should be 20 * lap time (s)
  // Make small adjustments to the PID values and determine if the cumulative error has changed.
  step_count += 1;
  if(step_count % evaluation_steps > stabilization_steps){
    cumulative_err += cte * cte;
  }
  
  std::cout << "p_err: " << p_error << "; d_err: " << d_error << "; i_err: " << i_error << "; speed: " << speed << "; c_err: " << cumulative_err << "; c_err_n: " << cumulative_err / double(evaluation_steps - stabilization_steps) << std::endl;
  
  // Apply Twiddle Algorithm  
  if(twiddle_active && step_count % evaluation_steps == 0){
    update_count += 1;
    //cumulative_err /= evaulation_steps - stabilization_steps;
    
    p[tuning_param_index] += dp[tuning_param_index];
    //cumulative_err += cte * cte;
    // Divide the cumulative error by the message count
    //cumulative_err /= message_counter;

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
    
    Kp = p[0];
    Kd = p[1];
    Ki = p[2];
    
    std::string line;
    std::ofstream output_file;
    output_file.open("pid_results.txt", std::ios::out | std::ios::app);
    if(output_file.is_open()){
      output_file << "Iteration " << update_count << ": Cumulative error: " << cumulative_err << "; Best error: " << best_err << "; P: " << p[0] << "; I: " << p[2] << "; D: " << p[1] << "\n";
      output_file.close();  
    }
    
    
    cumulative_err = 0.0;
    step_count = 0;    
  }
  
  std::cout << "**** Iteration: " << step_count << "; P: " << p[0] << "; D: " << p[1] << "; I: " << p[2] << "; Sum: " << p[0] + p[1] + p[2] << std::endl;
  
  prev_cte = cte;
  //if(message_counter == 10000){
  //  cumulative_err = 0.0;
  //}
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