#ifndef PID_H
#define PID_H
#include <vector>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  //void Init(double Kp_, double Ki_, double Kd_, double max_i_error_, bool twiddle_active_, double initial_dp, int eval_steps);
  void Init();
  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte, double diff_time, double speed);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;
  double prev_cte;
  double max_i_error;  
  
  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
  
  /**
   * Twiddle Parameters
   */
  double cumulative_err;
  double best_err;
  double twiddle_active;
  int tuning_param_index;
  std::vector<double> dp;
  std::vector<double> p;
  bool index_changed;
  //int stabilize_count;
  int stabilization_steps;
  int evaluation_steps;
  int step_count;
  int update_count;
  double initial_dp_p;
  double initial_dp_d;
  double initial_dp_i;
  int last_change;
  
};

#endif  // PID_H