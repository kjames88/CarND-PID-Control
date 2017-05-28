#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
  control_response_ = 0.0;
}

void PID::UpdateError(double cte) {
  double d = cte - d_error_;
  p_error_ = cte;
  i_error_ += cte;  // integral
  d_error_ = cte;   // prev for derivative
  control_response_ = (-Kp_ * cte) - (Ki_ * i_error_) - (Kd_ * d);
  
}

double PID::TotalError() {
}

