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

double computeProportionalTerm(const double current, const double target,
                               const double Kp, const double threshold) {
  return Kp * calc_error(current, target, threshold);
}

KDL::Vector computeProportionalTerm(const KDL::Vector& current,
                                    const KDL::Vector& target, const double Kp,
                                    const KDL::Vector& threshold) {
  return Kp * calc_error(current, target, threshold);
}

double computeIntegralTerm(const double current, const double target,
                           const double Ki, const double dt,
                           const double threshold, double& error_sum) {
  error_sum += calc_error(current, target, threshold) * dt;
  return Ki * error_sum;
}

KDL::Vector computeIntegralTerm(const KDL::Vector& current,
                                const KDL::Vector& target, const double Ki,
                                const double dt, const KDL::Vector& threshold,
                                KDL::Vector& error_sum) {
  error_sum += calc_error(current, target, threshold) * dt;
  return Ki * error_sum;
}

double computeDerivativeTerm(const double current, const double target,
                             const double Kd, const double dt,
                             const double threshold, double& last_error) {
  double current_error = calc_error(current, target, threshold);
  double derivative = (current_error - last_error) / dt;
  last_error = current_error;
  return Kd * derivative;
}

KDL::Vector computeDerivativeTerm(const KDL::Vector& current,
                                  const KDL::Vector& target, const double Kd,
                                  const double dt, const KDL::Vector& threshold,
                                  KDL::Vector& last_error) {
  KDL::Vector current_error = calc_error(current, target, threshold);
  KDL::Vector derivative = (current_error - last_error) / dt;
  last_error = current_error;
  return Kd * derivative;
}

KDL::Vector calc_error(const KDL::Vector& in1, const KDL::Vector& in2,
                       const KDL::Vector& threshold) {
  return in1 - in2 - threshold;
}

double calc_error(const double& in1, const double& in2,
                  const double& threshold) {
  return in1 - in2 - threshold;
}
