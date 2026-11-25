#ifndef __PID_H
#define __PID_H
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/**
  * @brief  PID controller base Configuration Structure definition
  */
typedef struct
{
	float kp;
	float ki;
	float kd;
	int32_t rate;
	float output_max;
	float recent_error;
	float last_error;
	float integral;
	float integral_max;
	float differential;
	float output;
}pid_controller;

/* Control functions  ************************************************/
void pid_controller_create(pid_controller * pid_controller_ptr, float kp, float ki, float kd, int32_t rate, float output_max);
void pid_error_input(pid_controller * pid_controller_ptr, float error);
void pid_calculate(pid_controller * pid_controller_ptr);
float pid_output(pid_controller * pid_controller_ptr);

#ifdef __cplusplus
}
#endif

#endif
