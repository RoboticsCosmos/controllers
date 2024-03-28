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

/**
 * @brief Compute the proportional term of the PID controller.
 * @param current The current value of the system.
 * @param target The desired target value.
 * @param Kp The proportional gain.
 * @param signal The proportional term of the PID controller.
 */
void computeProportionalTerm(double error, double Kp, double& signal);

/**
 * @brief Compute the integral term of the PID controller.
 * @param current The current value of the system.
 * @param target The desired target value.
 * @param Ki The integral gain.
 * @param dt The time step.
 * @param error_sum The accumulated error.
 * @param signal The integral term of the PID controller.
 */
void computeIntegralTerm(double error, double Ki, double dt, double& error_sum, double& signal);

/**
 * @brief Compute the derivative term of the PID controller.
 * @param current The current value of the system.
 * @param target The desired target value.
 * @param Kd The derivative gain.
 * @param dt The time step.
 * @param last_error The previous error.
 * @param signal The derivative term of the PID controller.
 */
void computeDerivativeTerm(double error, double Kd, double dt, double& last_error, double& signal);

void pidController(double error, double Kp, double Ki, double Kd, double dt, double& error_sum, double& last_error, double& signal);

/**
 * @brief Calculates the error between two vectors.
 *
 * @param in1 The first double.
 * @param in2 The second double.
 * @param threshold The threshold for the error.
 * @param out The error as a doble.
 */
void computeError(double in1, double in2, double threshold = 0.0, double& out);

#endif /* PID_CONTROLLER_HPP */
