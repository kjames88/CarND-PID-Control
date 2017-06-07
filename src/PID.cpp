#include "PID.h"
#include <assert.h>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() : initialized_(false) {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
  cte_q_ = 0.0;
  control_response_ = 0.0;
  error_squared_ = 0.0;
  primed_ = false;
  initialized_ = true;
}

void PID::UpdateError(double cte) {
  assert(initialized_);
  if (primed_ == false) {
    cte_q_ = cte;
    primed_ = true;
  }
  p_error_ = cte;
  d_error_ = cte - cte_q_;
  i_error_ += cte;  // integral
  cte_q_ = cte;     // prev for derivative
}

double PID::TotalError() {
  control_response_ = (-Kp_ * p_error_) - (Ki_ * i_error_) - (Kd_ * d_error_);

  // if we get here we're probably already off the track anyway
  if (control_response_ < -1.0)
    control_response_ = -1.0;
  else if (control_response_ > 1.0)
    control_response_ = 1.0;

  error_squared_ += (p_error_ * p_error_);
  return control_response_;
}

