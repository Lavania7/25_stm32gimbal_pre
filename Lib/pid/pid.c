#include "pid.h"
#include "main.h"

/**
  * @brief This function is creat a pid controller.
  */
void pid_controller_create(pid_controller * pid_controller_ptr, float kp, float ki, float kd, int32_t rate, float output_max)
{
	pid_controller_ptr->kp = kp;
	pid_controller_ptr->ki = ki;
	pid_controller_ptr->kd = kd;
	if (rate == 0) 
	{
		rate = 1;
	}
	pid_controller_ptr->rate = rate;
	pid_controller_ptr->output_max = output_max;
	pid_controller_ptr->recent_error = 0;
	pid_controller_ptr->last_error = 0;
	pid_controller_ptr->integral = 0;
	pid_controller_ptr->differential = 0;
	pid_controller_ptr->output = 0;
}

/**
  * @brief This function input the error to the pid controller.
  */
void pid_error_input(pid_controller * pid_controller_ptr, float error)
{
	pid_controller_ptr -> recent_error = error;
	pid_calculate(pid_controller_ptr);
}

/**
  * @brief This function calculate the result after error input.
  */
void pid_calculate(pid_controller * pid_controller_ptr)
{
	pid_controller_ptr -> integral += pid_controller_ptr->recent_error;
	if(pid_controller_ptr->integral>=20000)
	{
		pid_controller_ptr->integral=20000;
	}
	if(pid_controller_ptr->integral<=-20000)
	{
		pid_controller_ptr->integral=-20000;
	}
	pid_controller_ptr -> differential = pid_controller_ptr->recent_error - pid_controller_ptr->last_error;
	pid_controller_ptr -> last_error = pid_controller_ptr -> recent_error;
	pid_controller_ptr -> output = pid_controller_ptr->kp * pid_controller_ptr->recent_error 
															+ (pid_controller_ptr->ki * pid_controller_ptr->integral) / pid_controller_ptr->rate
															+ (pid_controller_ptr->kd * pid_controller_ptr->differential) * pid_controller_ptr->rate;
	if(pid_controller_ptr->output > pid_controller_ptr->output_max)pid_controller_ptr->output = pid_controller_ptr->output_max;
	else if(pid_controller_ptr->output < - pid_controller_ptr->output_max)pid_controller_ptr->output = - pid_controller_ptr->output_max;
}

/**
  * @brief This function output the result.
  */
float pid_output(pid_controller * pid_controller_ptr)
{
	return pid_controller_ptr->output;
}

