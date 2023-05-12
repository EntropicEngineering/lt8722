/*
 * Created by Benjamin Riggs on 5/12/23.
 *
 * Copyright (c) 2023 Entropic Engineering. All rights reserved.
 */

#ifndef LT8722_PID_H
#define LT8722_PID_H

#include <stdint.h>

typedef struct {
	// Config
	float K_p;							/// Proportional constant, applied to all terms
	float T_i;							/// Integration time, K_i =  K_p / T_i
	float T_d;							/// Derivative time, K_d = K_p * T_d
	float error_derivative_ema_weight;	/// Weight of new error delta in EMA, set to 1 for no EMA
	// Data
	float error_integral;		 /// Summed error values
	float previous_error;		 /// error value from last call to pid_update
	float error_derivative_ema;	 /// Error derivative, smoothed via EMA
} pid_control_t;

#define PID_INIT( K_P, T_I, T_D, WEIGHT )                                                          \
	( pid_control_t )                                                                              \
	{                                                                                              \
		.K_p = ( K_P ), .T_i = ( T_I ), .T_d = ( T_D ), .error_derivative_ema_weight = ( WEIGHT ), \
		.error_intrgral = 0, .previous_error = 0, .error_derivative_ema = 0                        \
	}

/**
 * Calculate the output of a PID controller.
 *
 * Ensure interval between calls is consistent.
 *
 * @param[in] pid 		Pointer to controller data, ensure it's not NULL
 * @param[in] error 	Current error value
 * @return 				Computed PID result
 */
float pid_update( pid_control_t* pid, float error );

#endif	// LT8722_PID_H
