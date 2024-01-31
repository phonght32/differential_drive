#include "stdlib.h"
#include "math.h"
#include "differential_drive/differential_drive.h"

#define NUM_WHEEL 2
#define LEFT_WHEEL 0
#define RIGHT_WHEEL 1

typedef struct diff_drive
{
	float wheel_radius;			  /*!< Wheel radius. Unit: mm */
	float wheel_seperation;		  /*!< Wheel seperation. Unit: mm */
	float min_lin_vel;			  /*!< Min linear velocity. Unit: m/s */
	float max_lin_vel;			  /*!< Max linear velocity. Unit: m/s */
	float min_ang_vel;			  /*!< Min angular velocity. Unit: rad/s */
	float max_ang_vel;			  /*!< Max angular velocity. Unit: rad/s */
	float tick_to_rad;			  /*!< Convert encoder tick to wheel position */
	int64_t prev_tick[NUM_WHEEL]; /*!< Previous encoder tick */
	float prev_rad[NUM_WHEEL];	  /*!< Previous position. Unit: radian */
	float prev_vel[NUM_WHEEL];	  /*!< Previous velocity. Unit: m/s */
	float prev_theta;			  /*!< Previous theta angle. Unit: rad */
	float odom_pose_x;			  /*!< Odom pose x */
	float odom_pose_y;			  /*!< Odom pose y */
	float odom_pose_z;			  /*!< Odom pose z */
	float odom_pose_theta;		  /*!< Odom pose theta */
	float odom_vel_lin;			  /*!< Odom linear velocity */
	float odom_vel_ang;			  /*!< Odom angular velocity */
} diff_drive_t;

static float constrain(float x, float low_val, float high_val)
{
	float value;
	if (x > high_val)
	{
		value = high_val;
	}
	else if (x < low_val)
	{
		value = low_val;
	}
	else
	{
		value = x;
	}
	return value;
}

diff_drive_handle_t diff_drive_init(void)
{
	diff_drive_handle_t handle = calloc(1, sizeof(diff_drive_t));
	if (handle == NULL)
	{
		return NULL;
	}

	return handle;
}

err_code_t diff_drive_set_config(diff_drive_handle_t handle, diff_drive_cfg_t cfg)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	handle->wheel_radius = cfg.wheel_radius;
	handle->wheel_seperation = cfg.wheel_seperation;
	handle->min_lin_vel = cfg.min_lin_vel;
	handle->max_lin_vel = cfg.max_lin_vel;
	handle->min_ang_vel = cfg.min_ang_vel;
	handle->max_ang_vel = cfg.max_ang_vel;
	handle->tick_to_rad = cfg.tick_to_rad;
	handle->prev_tick[LEFT_WHEEL] = 0;
	handle->prev_tick[RIGHT_WHEEL] = 0;
	handle->prev_rad[LEFT_WHEEL] = 0;
	handle->prev_rad[RIGHT_WHEEL] = 0;
	handle->prev_vel[LEFT_WHEEL] = 0;
	handle->prev_vel[RIGHT_WHEEL] = 0;
	handle->prev_theta = 0;
	handle->odom_pose_x = 0;
	handle->odom_pose_y = 0;
	handle->odom_pose_z = 0;
	handle->odom_pose_theta = 0;
	handle->odom_vel_ang = 0;
	handle->odom_vel_lin = 0;

	return ERR_CODE_SUCCESS;
}

err_code_t diff_drive_config(diff_drive_handle_t handle)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	/* Nothing to do */

	return ERR_CODE_SUCCESS;
}

err_code_t diff_drive_calc_wheel_vel(diff_drive_handle_t handle,
									 float lin_vel,
									 float ang_vel,
									 float *left_wheel_vel,
									 float *right_wheel_vel)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	float wheel_velocity_cmd[2];

	wheel_velocity_cmd[LEFT_WHEEL] = lin_vel - (ang_vel * handle->wheel_seperation / 2);
	wheel_velocity_cmd[RIGHT_WHEEL] = lin_vel + (ang_vel * handle->wheel_seperation / 2);

	wheel_velocity_cmd[LEFT_WHEEL] = constrain(wheel_velocity_cmd[LEFT_WHEEL], handle->min_lin_vel, handle->max_lin_vel);
	wheel_velocity_cmd[RIGHT_WHEEL] = constrain(wheel_velocity_cmd[RIGHT_WHEEL], handle->min_lin_vel, handle->max_lin_vel);

	*left_wheel_vel = wheel_velocity_cmd[LEFT_WHEEL];
	*right_wheel_vel = wheel_velocity_cmd[RIGHT_WHEEL];

	return ERR_CODE_SUCCESS;
}

err_code_t diff_drive_get_tick(diff_drive_handle_t handle, int64_t *left_tick, int64_t *right_tick)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	*left_tick = handle->prev_tick[LEFT_WHEEL];
	*right_tick = handle->prev_tick[RIGHT_WHEEL];

	return ERR_CODE_SUCCESS;
}

err_code_t diff_drive_get_rad(diff_drive_handle_t handle, float *left_rad, float *right_rad)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	*left_rad = handle->prev_rad[LEFT_WHEEL];
	*right_rad = handle->prev_rad[RIGHT_WHEEL];

	return ERR_CODE_SUCCESS;
}

err_code_t diff_drive_get_vel(diff_drive_handle_t handle, float *left_vel, float *right_vel)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	*left_vel = handle->prev_vel[LEFT_WHEEL];
	*right_vel = handle->prev_vel[RIGHT_WHEEL];

	return ERR_CODE_SUCCESS;
}

err_code_t diff_drive_calc_odom(diff_drive_handle_t handle,
								float step_time,
								int32_t left_tick,
								int32_t right_tick,
								float theta)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	float wheel_l = 0.0f, wheel_r = 0.0f; // rotation value of wheel [rad]
	float delta_s = 0.0f, delta_theta = 0.0f;
	float lin_vel = 0.0f, ang_vel = 0.0f; // v = translational velocity [m/s], w = rotational velocity [rad/s]

	if (step_time == 0)
		return ERR_CODE_FAIL;

	wheel_l = handle->tick_to_rad * (float)left_tick;
	wheel_r = handle->tick_to_rad * (float)right_tick;

	if (isnan(wheel_l))
		wheel_l = 0.0f;

	if (isnan(wheel_r))
		wheel_r = 0.0f;

	delta_s = handle->wheel_radius * (wheel_r + wheel_l) / 2.0f;

	/* theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION; */
	delta_theta = theta - handle->prev_theta;

	/* Compute odometric pose */
	handle->odom_pose_x += delta_s * cos(handle->odom_pose_theta + (delta_theta / 2.0));
	handle->odom_pose_y += delta_s * sin(handle->odom_pose_theta + (delta_theta / 2.0));
	handle->odom_pose_theta += delta_theta;

	/* Compute odometric instantaneouse velocity */
	lin_vel = delta_s / step_time;
	ang_vel = delta_theta / step_time;

	handle->odom_vel_lin = lin_vel;
	handle->odom_vel_ang = ang_vel;

	handle->prev_tick[LEFT_WHEEL] = left_tick;
	handle->prev_tick[RIGHT_WHEEL] = right_tick;
	handle->prev_rad[LEFT_WHEEL] += handle->tick_to_rad * (float)left_tick;
	handle->prev_rad[RIGHT_WHEEL] += handle->tick_to_rad * (float)right_tick;
	handle->prev_vel[LEFT_WHEEL] = wheel_l / step_time;
	handle->prev_vel[RIGHT_WHEEL] = wheel_r / step_time;
	handle->prev_theta = theta;

	return ERR_CODE_SUCCESS;
}

err_code_t diff_drive_get_odom(diff_drive_handle_t handle,
							   float *odom_pose_x,
							   float *odom_pose_y,
							   float *odom_pose_theta,
							   float *odom_vel_lin,
							   float *odom_vel_ang)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	*odom_pose_x = handle->odom_pose_x;
	*odom_pose_y = handle->odom_pose_y;
	*odom_pose_theta = handle->odom_pose_theta;
	*odom_vel_lin = handle->odom_vel_lin;
	*odom_vel_ang = handle->odom_vel_ang;

	return ERR_CODE_SUCCESS;
}
