#ifndef PID_H
#define PID_H

class PID {
public:
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
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
   * Get the control response
   */
  double getControlResponse() {return control_response_;}

  /*
   * Get the integration error term
   */
  double getCTE_Int() {return i_error_;}

 private:

  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;

  /*
  * Errors
  */
  double p_error_;
  double i_error_;
  double d_error_;

  bool initialized_;
  bool primed_;
  double cte_q_;
  double control_response_;
  double error_squared_;

};

#endif /* PID_H */
