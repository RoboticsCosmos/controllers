/**
 * Author: Vamsi Kalagaturu
 * Contributors: Ravisankar Selvaraju, Wing Ki Lau
 *
 * Description: Library implmenting PID controller for the arm_actions package
 *
 * Copyright (c) [2023]
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "controllers/pid_controller.hpp"
#include <stdio.h>

// Adds a dead zone around the error to avoid small oscillations
double applyDeadZone(double error, double dead_zone)
{
  if (std::abs(error) < dead_zone)
  {
    return 0.0;
  }
  return error;
}

void computeProportionalTerm(double error, double Kp, double& signal)
{
  signal = Kp * error;
}

void computeIntegralTerm(double error, double Ki, double dt, double& error_sum,
                         double error_sum_tol, double& signal)
{
  error_sum += error * dt;

  if (error_sum > error_sum_tol)
  {
    error_sum = error_sum_tol;
  }
  else if (error_sum < -error_sum_tol)
  {
    error_sum = -error_sum_tol;
  }

  signal = Ki * error_sum;
}

void computeDerivativeTerm(double error, double Kd, double dt, double& last_error, double& signal)
{
  double derivative = (error - last_error) / dt;
  last_error = error;
  signal = Kd * derivative;
}

void pidController(double error, double Kp, double Ki, double Kd, double dt, double& error_sum,
                   double error_sum_tol, double& last_error, double& signal)
{
  double proportional, integral, derivative = 0.0;
  computeProportionalTerm(error, Kp, proportional);
  computeIntegralTerm(error, Ki, dt, error_sum, error_sum_tol, integral);
  computeDerivativeTerm(error, Kd, dt, last_error, derivative);
  signal = proportional + integral + derivative;
}

void computeIntegralTerm(double error, double Ki, double dt, double& error_sum,
                         double error_sum_tol, double decay_rate, double& signal)
{
  if (fabs(error) > 0.0)
  {
    // Accumulate the integral when error is non-zero
    error_sum += error * dt;

    // Clamp the integral term to prevent runaway accumulation
    if (error_sum > error_sum_tol)
    {
      error_sum = error_sum_tol;
    }
    else if (error_sum < -error_sum_tol)
    {
      error_sum = -error_sum_tol;
    }
  }
  else
  {
    // Decay the integral term when the error is zero
    error_sum = decay_rate * error_sum + (1.0 - decay_rate) * error;
  }

  // Compute the integral contribution to the control signal
  signal = Ki * error_sum;
}

// Compute Derivative Term with smoothing
void computeDerivativeTerm(double error, double Kd, double dt, double& last_error,
                           double smoothing_factor, double& signal)
{
  double derivative = (error - last_error) / dt;
  last_error = error;

  // // Apply a smoothing factor to reduce noise in derivative
  // static double smoothed_derivative = 0.0;
  // smoothed_derivative =
  //     smoothing_factor * smoothed_derivative + (1 - smoothing_factor) * derivative;

  // signal = Kd * smoothed_derivative;

  signal = Kd * derivative;
}

void pidController(double error, double Kp, double Ki, double Kd, double dt, double& error_sum,
                   double error_sum_tol, double& last_error, double decay_rate, double dead_zone,
                   double smoothing_factor, double& signal, bool debug)
{
  double proportional, integral, derivative = 0.0;

  // Apply dead zone to the error
  error = applyDeadZone(error, dead_zone);

  // Compute terms
  computeProportionalTerm(error, Kp, proportional);
  computeIntegralTerm(error, Ki, dt, error_sum, error_sum_tol, decay_rate, integral);
  computeDerivativeTerm(error, Kd, dt, last_error, smoothing_factor, derivative);

  if (debug)
  {
    printf("P: %f, I: %f, D: %f\n", proportional, integral, derivative);
    printf("error: %f, error_sum: %f, last_error: %f\n", error, error_sum, last_error);
  }

  // Compute final signal
  signal = proportional + integral + derivative;
}

void pidController(double* error, double Kp, double Ki, double Kd, double dt, double* error_sum,
                   double error_sum_tol, double* last_error, double* signal, int size)
{
  for (int i = 0; i < size; i++)
  {
    pidController(error[i], Kp, Ki, Kd, dt, error_sum[i], error_sum_tol, last_error[i], signal[i]);
  }
}

void impedanceController(double stiffnessError, double dampingError, double* stiffness_diag_mat,
                         double* damping_diag_mat, double& signal)
{
  // *Assumption: diagonal matrices are of size 1x1 - 1d control
  signal = stiffness_diag_mat[0] * stiffnessError + damping_diag_mat[0] * dampingError;
}

void computeEqualityError(double in1, double in2, double& out)
{
  out = in2 - in1;
}



void computeInBetweenError(double measured, double upperLimit, double lowerLimit, double& out)
{
  if (measured >= lowerLimit && measured <= upperLimit)
  {
    out = 0.0;
  }
  else if (measured < lowerLimit)
  {
    out = measured - lowerLimit;
  }
  else if (measured > upperLimit)
  {
    out = measured - upperLimit;
  }
}