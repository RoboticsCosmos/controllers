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

void computeProportionalTerm(double error, double Kp, double& signal)
{
  signal = Kp * error;
}

void computeIntegralTerm(double error, double Ki, double dt, double& error_sum, double& signal)
{
  error_sum += error * dt;
  signal = Ki * error_sum;
}

void computeDerivativeTerm(double error, double Kd, double dt, double& last_error, double& signal)
{
  double derivative = (error - last_error) / dt;
  last_error = error;
  signal = Kd * derivative;
}

void pidController(double error, double Kp, double Ki, double Kd, double dt, double& error_sum,
                   double& last_error, double& signal)
{
  double proportional, integral, derivative = 0.0;
  computeProportionalTerm(error, Kp, proportional);
  computeIntegralTerm(error, Ki, dt, error_sum, integral);
  computeDerivativeTerm(error, Kd, dt, last_error, derivative);
  signal = proportional + integral + derivative;
}

void computeError(double in1, double in2, double& out)
{
  out = in2 - in1;
}
