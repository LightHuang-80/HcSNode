/*
 * scurve.h
 *
 *  Created on: Oct 28, 2020
 *      Author: Administrator
 */

#ifndef SRC_STEPPER_SCURVE_H_
#define SRC_STEPPER_SCURVE_H_

void calc_bezier_curve_coeffs(const int32_t v0, const int32_t v1, const uint32_t av);
int32_t eval_bezier_curve(const uint32_t curr_step);

#endif /* SRC_STEPPER_SCURVE_H_ */
