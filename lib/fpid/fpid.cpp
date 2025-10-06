#include "fpid.h"

Fpid::Fpid()
{
}

Fpid::Fpid(float min_out, float max_out)
    : min_out(min_out),
      max_out(max_out)
{
}

float Fpid::compute(float setpoint, float measure, float time_step)
{
  float pid;

  float error = setpoint - measure;
  float new_ki_term = ki_term + ki * error * time_step;

  pid = kf * setpoint;
  pid += kp * error;
  pid += new_ki_term;

  if (kd != 0.0f)
  {
    // derivative on measurement
    float kd_term = kd * (measure - prev_measure) / time_step; 
    pid -= kd_term;
  }

  if (pid > max_out)
  {
    pid = max_out;
  }
  else if (pid < min_out)
  {
    pid = min_out;
  }
  else
  {
    ki_term = new_ki_term; // only here, for windup protection
  }

  prev_measure = measure;

  return pid;
}

void Fpid::update_k_independent(float kf, float kp, float ki, float kd)
{
  Fpid::kf = kf;
  Fpid::kp = kp;
  Fpid::ki = ki;
  Fpid::kd = kd;
}

void Fpid::update_k_dependent_ideal(float kf, float kc, float Ti, float Td)
{
  Fpid::kf = kf;
  Fpid::kp = kc;
  Fpid::ki = kc / Ti;
  Fpid::kd = kc * Td;
}

void Fpid::reset_errors(float measure)
{
  ki_term = 0;
  prev_measure = measure;
}
