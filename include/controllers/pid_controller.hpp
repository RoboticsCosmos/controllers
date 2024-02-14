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

#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <array>
#include <iostream>
#include <tuple>
#include <vector>

#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"

/**
 * @brief Compute the proportional term of the PID controller.
 * @param current The current value of the system.
 * @param target The desired target value.
 * @param Kp The proportional gain.
 * @return The proportional term of the PID controller.
 */
double computeProportionalTerm(const double current, const double target,
                               const double Kp, const double threshold = 0.0);

/**
 * @brief Compute the proportional term of the PID controller.
 * @param current The current value of the system.
 * @param target The desired target value.
 * @param Kp The proportional gain.
 * @return The proportional term of the PID controller.
 */
KDL::Vector computeProportionalTerm(const KDL::Vector& current,
                                    const KDL::Vector& target, const double Kp,
                                    const KDL::Vector& threshold);

/**
 * @brief Compute the integral term of the PID controller.
 * @param current The current value of the system.
 * @param target The desired target value.
 * @param Ki The integral gain.
 * @param dt The time step.
 * @param error_sum The accumulated error.
 * @return The integral term of the PID controller.
 */
double computeIntegralTerm(const double current, const double target,
                           const double Ki, const double dt,
                           const double threshold, double& error_sum);

/**
 * @brief Compute the integral term of the PID controller.
 * @param current The current value of the system.
 * @param target The desired target value.
 * @param Ki The integral gain.
 * @param dt The time step.
 * @param error_sum The accumulated error.
 * @return The integral term of the PID controller.
 */
KDL::Vector computeIntegralTerm(const KDL::Vector& current,
                                const KDL::Vector& target, const double Ki,
                                const double dt, const double threshold,
                                KDL::Vector& error_sum);

/**
 * @brief Compute the derivative term of the PID controller.
 * @param current The current value of the system.
 * @param target The desired target value.
 * @param Kd The derivative gain.
 * @param dt The time step.
 * @param last_error The previous error.
 * @return The derivative term of the PID controller.
 */
double computeDerivativeTerm(const double current, const double target,
                             const double Kd, const double dt,
                             const double threshold, double& last_error);
/**
 * @brief Compute the derivative term of the PID controller.
 * @param current The current value of the system.
 * @param target The desired target value.
 * @param Kd The derivative gain.
 * @param dt The time step.
 * @param last_error The previous error.
 * @return The derivative term of the PID controller.
 */
KDL::Vector computeDerivativeTerm(const KDL::Vector& current,
                                  const KDL::Vector& target, const double Kd,
                                  const double dt, const KDL::Vector& threshold,
                                  KDL::Vector& last_error);

/**
 * @brief Calculates the error between two vectors.
 *
 * @param in1 The first vector.
 * @param in2 The second vector.
 * @param threshold The threshold vector for the error.
 * @return The error as a kdl vector.
 */
KDL::Vector calc_error(const KDL::Vector& in1, const KDL::Vector& in2,
                       const KDL::Vector& threshold);

/**
 * @brief Calculates the error between two vectors.
 *
 * @param in1 The first double.
 * @param in2 The second double.
 * @param threshold The threshold for the error.
 * @return The error as a doble.
 */
double calc_error(const double& in1, const double& in2,
                  const double& threshold = 0.0);

#endif /* PID_CONTROLLER_HPP */
