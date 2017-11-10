#ifndef PID_H
#define PID_H

#include <vector>

class PID {

public:
  /*
  * Errors
  */
  //double p_error_;
  //double i_error_;
  //double d_error_;

  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;

  //Error vector consisting of p_error, i_error_, d_error_ respectively
  std::vector<double> error_vector_;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  std::vector<double> UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
