#include "PID.h"
#include <assert.h>

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
  for (int i=0; i<3; i++) {
    twiddle_dp_[i] = 1.0;
    twiddle_phase_[i] = 0;
  }
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
  cte_q_ = 0.0;
  cte_int_ = 0.0;
  control_response_ = 0.0;
  error_squared_ = 0.0;
  primed_ = false;
  initialized_ = true;
}

void PID::UpdateError(double cte) {
  assert(initialized_);
  if (primed_ == false) {
    cte_q_ = cte;
    p_error_ = cte;
    i_error_ = cte;
    d_error_ = cte;
    primed_ = true;
  }
  
  double d_cte = cte - cte_q_;
  cte_int_ += cte;  // integral
  cte_q_ = cte;     // prev for derivative
  control_response_ = (-Kp_ * cte) - (Ki_ * cte_int_) - (Kd_ * d_cte);

  // if we get here we're probably already off the track anyway
  if (control_response_ < -1.0)
    control_response_ = -1.0;
  else if (control_response_ > 1.0)
    control_response_ = 1.0;

  error_squared_ += (cte * cte);
}

double PID::TotalError() {
  return error_squared_;
}

