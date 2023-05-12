/*
 * Created by Benjamin Riggs on 5/12/23.
 *
 * Copyright (c) 2023 Entropic Engineering. All rights reserved.
 */

#include "pid.h"

float pid_update( pid_control_t* pid, float error )
{
	pid->error_integral += error;
	pid->error_derivative_ema = ( error - pid->previous_error ) * pid->error_derivative_ema_weight +
								pid->error_derivative_ema * ( 1 - pid->error_derivative_ema_weight );
	pid->previous_error = error;

	return pid->K_p * (error + pid->error_integral / pid->T_i + pid->T_d * pid->error_derivative_ema)
}
