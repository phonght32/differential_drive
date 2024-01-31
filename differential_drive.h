// MIT License

// Copyright (c) 2024 phonght32

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef __DIFFERENTIAL_DRIVE_H__
#define __DIFFERENTIAL_DRIVE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "err_code.h"

typedef struct diff_drive *diff_drive_handle_t;

typedef struct
{
	float wheel_radius;		/*!< Wheel radius. Unit: mm */
	float wheel_seperation; /*!< Wheel seperation. Unit: mm */
	float min_lin_vel;		/*!< Min linear velocity. Unit: m/s */
	float max_lin_vel;		/*!< Max linear velocity. Unit: m/s */
	float min_ang_vel;		/*!< Min angular velocity. Unit: rad/s */
	float max_ang_vel;		/*!< Max angular velocity. Unit: rad/s */
	float tick_to_rad;		/*!< Convert encoder tick to wheel position */
} diff_drive_cfg_t;

/*
 * @brief   Initialize differential drive with default parameters.
 *
 * @note    This function must be called first.
 *
 * @param   None.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
diff_drive_handle_t diff_drive_init(void);

/*
 * @brief   Set configuration parameters.
 *
 * @param 	handle Handle structure.
 * @param   cfg Configuration structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t diff_drive_set_config(diff_drive_handle_t handle, diff_drive_cfg_t cfg);

/*
 * @brief   Configure differential drive to run.
 *
 * @param 	handle Handle structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t diff_drive_config(diff_drive_handle_t handle);

/*
 * @brief   Calculate velocity for each wheel based on expected linear and angular velocity.
 *
 * @param 	handle Handle structure.
 * @param 	lin_vel Expected linear velocity.
 * @param 	ang_vel Expected angular velocity.
 * @param 	left_wheel_vel Output velocity of left wheel.
 * @param 	right_wheel_vel Output velocity of right wheel.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t diff_drive_calc_wheel_vel(diff_drive_handle_t handle,
									 float lin_vel,
									 float ang_vel,
									 float *left_wheel_vel,
									 float *right_wheel_vel);

/*
 * @brief   Get encoder tick.
 *
 * @note 	This function gets the previous ticks set by "diff_drive_set_tick" called.
 *
 * @param 	handle Handle structure.
 * @param 	left_tick Left tick.
 * @param 	right_tick Right tick.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t diff_drive_get_tick(diff_drive_handle_t handle, int64_t *left_tick, int64_t *right_tick);

/*
 * @brief   Get position of wheels in radian.
 *
 * @note 	This function gets the previous wheel positions set by "diff_drive_set_rad" called.
 *
 * @param 	handle Handle structure.
 * @param 	left_rad Left radian.
 * @param 	right_rad Right radian.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t diff_drive_get_rad(diff_drive_handle_t handle, float *left_rad, float *right_rad);

/*
 * @brief   Get velocity of wheels in m/s.
 *
 * @note 	This function gets the previous wheel velocity set by "diff_drive_set_vel" called.
 *
 * @param 	handle Handle structure.
 * @param 	left_vel Left velocity.
 * @param 	right_vel Right velocity.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t diff_drive_get_vel(diff_drive_handle_t handle, float *left_vel, float *right_vel);

/*
 * @brief   Calculate odometry.
 *
 * @param 	handle Handle structure.
 * @param 	step_time Step time in ms.
 * @param 	left_tick Left tick.
 * @param 	right_tick Right tick.
 * @param 	theta Current yaw angel.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t diff_drive_calc_odom(diff_drive_handle_t handle,
								float step_time,
								int32_t left_tick,
								int32_t right_tick,
								float theta);

/*
 * @brief   Get odometry.
 *
 * @param 	handle Handle structure.
 * @param 	odom_pose_x Position x.
 * @param 	odom_pose_y Position y.
 * @param 	odom_pose_theta Yaw angle.
 * @param 	odom_vel_lin Linear velocity.
 * @param 	odom_vel_ang Angular velocity.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t diff_drive_get_odom(diff_drive_handle_t handle,
							   float *odom_pose_x,
							   float *odom_pose_y,
							   float *odom_pose_theta,
							   float *odom_vel_lin,
							   float *odom_vel_ang);

#ifdef __cplusplus
}
#endif

#endif /* __DIFFERENTIAL_DRIVE_H__ */
