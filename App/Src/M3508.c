/**
******************************************************************************
* @file    M3508.c
* @author  李维中3296626485@qq.com
* @brief   RM motor app of the driver(2006 / 3508 / 6020)
*
******************************************************************************
* @attention
*
* Copyright(c) 2025 BNGU.
* All rights reserved.
* *****************************************************************************
*/

#include "M3508.h"
#include "pid.h"
#include "can.h"
#include "bsp_can.h"
#include "rm_motor.h"
#include <stdio.h>

DJI_Motor_ControlTypeDef M2006_Motor;
DJI_Motor_ControlTypeDef M3508_Motor;
DJI_Motor_ControlTypeDef M6020_Motor;

//上层数据Exp与PID参数,其中底层回传与发送为rm_motor_groupTypeDef CAN_3508;
//M3508_ControlTypeDef M3508_Motor;(在主文件定义）

//kp ki kd rate(中断频率)
void motor_pid_init(pid_controller* hpid, float kp, float ki, float kd, int32_t rate)
{
	hpid->kp = kp;
	hpid->ki = ki;
	hpid->kd = kd;
	hpid->rate = rate;
}

void motor_position_cal(DJI_Motor_ControlTypeDef* motor,uint8_t id)
{
	motor->motor_3508_position[id].now_position = motor->RM_CAN.motor_data[id].angle;
	float delta = motor->motor_3508_position[id].now_position - motor->motor_3508_position[id].last_position;
	if (delta >= 4095)
	{
		delta -= 8191;
	}
	else if (delta <= -4095)
	{
		delta += 8191;
	}
	motor->motor_3508_position[id].this_position += delta;
	motor->motor_3508_position[id].last_position = motor->motor_3508_position[id].now_position;
}

void motor_pid_data_init(DJI_Motor_ControlTypeDef* motor, uint8_t motor_type, uint8_t control_type, uint8_t MOTOR_GROUP)
{
	if (MOTOR_GROUP == LOW)
	{
		if (control_type == Nom_Ctrl)
		{
			if (motor_type == 8 || motor_type == 9)
			{
				motor_pid_init(motor->motor_s_pid, 3, 0, 0, PID_CAL_RATE);
				motor_pid_init(motor->motor_s_pid + 1, 3, 0, 0, PID_CAL_RATE);
				motor_pid_init(motor->motor_s_pid + 2, 3, 0, 0, PID_CAL_RATE);
				motor_pid_init(motor->motor_s_pid + 3, 3, 0, 0, PID_CAL_RATE);
				motor_pid_init(motor->motor_p_pid, 0.22, 0, 0, PID_CAL_RATE);
				motor_pid_init(motor->motor_p_pid + 1, 0.22, 0, 0, PID_CAL_RATE);
				motor_pid_init(motor->motor_p_pid + 2, 0.22, 0, 0, PID_CAL_RATE);
				motor_pid_init(motor->motor_p_pid + 3, 0.22, 0, 0, PID_CAL_RATE);
			}
			else if (motor_type == 10)
			{
				motor_pid_init(motor->motor_s_pid, 8, 0, 0, PID_CAL_RATE);
				motor_pid_init(motor->motor_s_pid + 1, 3, 0, 0, PID_CAL_RATE);
				motor_pid_init(motor->motor_s_pid + 2, 3, 0, 0, PID_CAL_RATE);
				motor_pid_init(motor->motor_s_pid + 3, 3, 0, 0, PID_CAL_RATE);
				motor_pid_init(motor->motor_p_pid, 4, 0, 0.2, PID_CAL_RATE);
				motor_pid_init(motor->motor_p_pid + 1, 5, 0, 0.2, PID_CAL_RATE);
				motor_pid_init(motor->motor_p_pid + 2, 5, 0, 0.2, PID_CAL_RATE);
				motor_pid_init(motor->motor_p_pid + 3, 5, 0, 0.2, PID_CAL_RATE);
			}
		}
		if (control_type == MIT_Ctrl)
		{
			motor_pid_init(motor->motor_s_pid, 0, 0, 0.2, PID_CAL_RATE);
			motor_pid_init(motor->motor_s_pid + 1, 0, 0, 0.2, PID_CAL_RATE);
			motor_pid_init(motor->motor_s_pid + 2, 0, 0, 0.2, PID_CAL_RATE);
			motor_pid_init(motor->motor_s_pid + 3, 0, 0, 0.2, PID_CAL_RATE);
			motor_pid_init(motor->motor_p_pid, 4, 0, 0, PID_CAL_RATE);
			motor_pid_init(motor->motor_p_pid + 1, 2, 0, 0, PID_CAL_RATE);
			motor_pid_init(motor->motor_p_pid + 2, 2, 0, 0, PID_CAL_RATE);
			motor_pid_init(motor->motor_p_pid + 3, 2, 0, 0, PID_CAL_RATE);
		}
	}
	else if (MOTOR_GROUP == HIGH)
	{
		if (control_type == Nom_Ctrl)
		{
			if (motor_type == 8 || motor_type == 9)
			{
				motor_pid_init(motor->motor_s_pid, 3, 0, 0, PID_CAL_RATE);
				motor_pid_init(motor->motor_s_pid + 1, 3, 0, 0, PID_CAL_RATE);
				motor_pid_init(motor->motor_s_pid + 2, 3, 0, 0, PID_CAL_RATE);
				motor_pid_init(motor->motor_s_pid + 3, 3, 0, 0, PID_CAL_RATE);
				motor_pid_init(motor->motor_p_pid, 0.22, 0, 0, PID_CAL_RATE);
				motor_pid_init(motor->motor_p_pid + 1, 0.22, 0, 0, PID_CAL_RATE);
				motor_pid_init(motor->motor_p_pid + 2, 0.22, 0, 0, PID_CAL_RATE);
				motor_pid_init(motor->motor_p_pid + 3, 0.22, 0, 0, PID_CAL_RATE);
			}
			else if (motor_type == 10)
			{
				motor_pid_init(motor->motor_s_pid, 3, 0, 0, PID_CAL_RATE);
				motor_pid_init(motor->motor_s_pid + 1, 3, 0, 0, PID_CAL_RATE);
				motor_pid_init(motor->motor_s_pid + 2, 3, 0, 0, PID_CAL_RATE);
				motor_pid_init(motor->motor_s_pid + 3, 3, 0, 0, PID_CAL_RATE);
				motor_pid_init(motor->motor_p_pid, 4, 0, 0.2, PID_CAL_RATE);
				motor_pid_init(motor->motor_p_pid + 1, 5, 0, 0.2, PID_CAL_RATE);
				motor_pid_init(motor->motor_p_pid + 2, 5, 0, 0.2, PID_CAL_RATE);
				motor_pid_init(motor->motor_p_pid + 3, 5, 0, 0.2, PID_CAL_RATE);
			}
		}
		if (control_type == MIT_Ctrl)
		{
			motor_pid_init(motor->motor_s_pid, 0, 0, 0.2, PID_CAL_RATE);
			motor_pid_init(motor->motor_s_pid + 1, 0, 0, 0.2, PID_CAL_RATE);
			motor_pid_init(motor->motor_s_pid + 2, 0, 0, 0.2, PID_CAL_RATE);
			motor_pid_init(motor->motor_s_pid + 3, 0, 0, 0.2, PID_CAL_RATE);
			motor_pid_init(motor->motor_p_pid, 4, 0, 0, PID_CAL_RATE);
			motor_pid_init(motor->motor_p_pid + 1, 2, 0, 0, PID_CAL_RATE);
			motor_pid_init(motor->motor_p_pid + 2, 2, 0, 0, PID_CAL_RATE);
			motor_pid_init(motor->motor_p_pid + 3, 2, 0, 0, PID_CAL_RATE);
		}
	}
}

/**
  *****************************************************************************
  *@attention 注意初始化方式
  *motor_ctrl_init,motor_MIT_ctrl_init只能二选一
  *
  ******************************************************************************
  *@motorctrlinit 控制方式是单独针对速度位置力矩分析或者进行元素叠加的控制
  *@motorMITctrlinit 控制方式是只用控制torque+KP*position+KD*speed,本质为力矩控制
  *
  ******************************************************************************
*/

//2006,3508初始化(0x200)  6020初始化(0x1fe) 
void motor_ctrl_init(DJI_Motor_ControlTypeDef* motor, uint8_t motor_type, CAN_HandleTypeDef* hcan, uint8_t MOTOR_GROUP)
{
	HAL_Delay(2000);
	if (MOTOR_GROUP == LOW)
	{
		if (motor_type == 8 || motor_type == 9)
		{
			rm_motor_init(&(motor->RM_CAN), 0x200, hcan);
		}
		else if (motor_type == 10)
		{
			rm_motor_init(&(motor->RM_CAN), 0x1ff, hcan);
		}
	}
	else if (MOTOR_GROUP == HIGH)
	{
		if (motor_type == 8 || motor_type == 9)
		{
			rm_motor_init(&(motor->RM_CAN), 0x1ff, hcan);
		}
		else if (motor_type == 10)
		{
			rm_motor_init(&(motor->RM_CAN), 0x2ff, hcan);
		}
	}
	rm_motor_cmd_max_config(&(motor->RM_CAN), PID_MAX, PID_MAX, PID_MAX, PID_MAX);
	for (int i = 0; i < 4; i++)
	{
		pid_controller_create(motor->motor_s_pid + i, 3, 0, 0, PID_CAL_RATE, 16384);
		pid_controller_create(motor->motor_p_pid + i, 0.215, 0, 0, PID_CAL_RATE, 16384);
	}
	motor_pid_data_init(motor, motor_type, Nom_Ctrl, MOTOR_GROUP);
	for (int i = 0; i < 4; i++)
	{
		motor->motor_3508_position[i].now_position = motor->RM_CAN.motor_data[i].angle;
		motor->motor_3508_position[i].last_position = motor->RM_CAN.motor_data[i].angle;
		motor->motor_3508_position[i].this_position = motor->RM_CAN.motor_data[i].angle;
		motor->motor_3508_position[i].last_target = 0;
		motor->motor_3508_position[i].stay_target = 0;
	}
	rm_motor_cmd_refresh(&(motor->RM_CAN), 0, 0, 0, 0);
	rm_motor_send_msg(&(motor->RM_CAN));
}

//MIT 只用控制torque+KP*position+KD*speed
void motor_MIT_ctrl_init(DJI_Motor_ControlTypeDef* motor, uint8_t motor_type, CAN_HandleTypeDef* hcan, uint8_t MOTOR_GROUP)
{
	HAL_Delay(2000);
	if (MOTOR_GROUP == LOW)
	{
		if (motor_type == 8 || motor_type == 9)
		{
			rm_motor_init(&(motor->RM_CAN), 0x200, hcan);
		}
		else if (motor_type == 10)
		{
			rm_motor_init(&(motor->RM_CAN), 0x1ff, hcan);
		}
	}
	else if (MOTOR_GROUP == HIGH)
	{
		if (motor_type == 8 || motor_type == 9)
		{
			rm_motor_init(&(motor->RM_CAN), 0x1ff, hcan);
		}
		else if (motor_type == 10)
		{
			rm_motor_init(&(motor->RM_CAN), 0x2ff, hcan);
		}
	}
	rm_motor_cmd_max_config(&(motor->RM_CAN), PID_MAX, PID_MAX, PID_MAX, PID_MAX);
	for (int i = 0; i < 4; i++)
	{
		pid_controller_create(motor->motor_s_pid + i, 0, 0, 0.0, PID_CAL_RATE, 16384);
		pid_controller_create(motor->motor_p_pid + i, 0.22, 0, 0, PID_CAL_RATE, 16384);
	}
	motor_pid_data_init(motor, motor_type, MIT_Ctrl, MOTOR_GROUP);
	for (int i = 0; i < 4; i++)
	{
		motor->motor_3508_position[i].now_position = motor->RM_CAN.motor_data[i].angle;
		motor->motor_3508_position[i].last_position = motor->RM_CAN.motor_data[i].angle;
		motor->motor_3508_position[i].this_position = motor->RM_CAN.motor_data[i].angle;
		motor->motor_3508_position[i].last_target = 0;
		motor->motor_3508_position[i].stay_target = 0;
	}
	rm_motor_cmd_refresh(&(motor->RM_CAN), 0, 0, 0, 0);
	rm_motor_send_msg(&(motor->RM_CAN));
}

//id = 0 1 2 3 => 1 2 3 4
void motor_3508_speed_pid(DJI_Motor_ControlTypeDef* motor, uint16_t id, int16_t target_speed)
{
	float error = target_speed - motor->RM_CAN.motor_data[id].speed;
	pid_error_input(motor->motor_s_pid + id, error);
	motor->cmd_buffer[id] = pid_output(motor->motor_s_pid + id);

	//M2006无温度回传 3508 6020 可用
	if (motor->RM_CAN.motor_data[id].temperature >= 50)
	{
		motor->cmd_buffer[id] = 0;
	}
	rm_motor_cmd_refresh(&(motor->RM_CAN), motor->cmd_buffer[0], motor->cmd_buffer[1], motor->cmd_buffer[2], motor->cmd_buffer[3]);
	rm_motor_send_msg(&(motor->RM_CAN));
}

//target_position 0-8191 => 0-360  到指定角度
void motor_3508_position_pid(DJI_Motor_ControlTypeDef* motor, uint16_t id, int16_t target_position)
{
	if (target_position >= 8191)
	{
		target_position = 8191;
	}
	else if (target_position <= 0)
	{
		target_position = 0;
	}

	float error = target_position - motor->RM_CAN.motor_data[id].angle;
	if (error >= 4095)
	{
		error -= 8191;
	}
	else if (error <= -4095)
	{
		error += 8191;
	}

	pid_error_input(motor->motor_p_pid + id, error);
	motor->cmd_buffer[id] = pid_output(motor->motor_p_pid + id);

	//M2006无温度回传 3508 6020 可用
	if (motor->RM_CAN.motor_data[id].temperature >= 50)
	{
		motor->cmd_buffer[id] = 0;
	}
	rm_motor_cmd_refresh(&(motor->RM_CAN), motor->cmd_buffer[0], motor->cmd_buffer[1], motor->cmd_buffer[2], motor->cmd_buffer[3]);
	rm_motor_send_msg(&(motor->RM_CAN));
}

//恒定速度走位置
void  motor_3508_speed_position_pid(DJI_Motor_ControlTypeDef* motor, uint16_t id, int16_t target_speed, int target_position)
{
	motor_position_cal(motor,id);
	pid_error_input(motor->motor_p_pid + id, target_position - motor->motor_3508_position[id].this_position);
	float speed_out = pid_output(motor->motor_p_pid + id);

	if (speed_out >= target_speed)
	{
		speed_out = target_speed;
	}
	else if (speed_out <= -target_speed)
	{
		speed_out = -target_speed;
	}

	pid_error_input(motor->motor_s_pid + id, speed_out - motor->RM_CAN.motor_data[id].speed);
	motor->cmd_buffer[id] = pid_output(motor->motor_s_pid + id);

	if (motor->RM_CAN.motor_data[id].temperature >= 50)
	{
		motor->cmd_buffer[id] = 0;
	}
	rm_motor_cmd_refresh(&(motor->RM_CAN), motor->cmd_buffer[0], motor->cmd_buffer[1], motor->cmd_buffer[2], motor->cmd_buffer[3]);
	rm_motor_send_msg(&(motor->RM_CAN));
}

//力矩计算
void motor_3508_torque_cal(DJI_Motor_ControlTypeDef* motor, uint16_t id, float torque, uint8_t motor_type)
{
	switch (motor_type)
	{
	case DJI_M2006:
		motor->cmd_buffer[id] += (torque / 1.0) * (10000.0 / 10.0);
		break;
	case DJI_M3508:
		motor->cmd_buffer[id] += (torque / 3.0) * (16384.0 / 20.0);
		break;
	case DJI_M6020:
		motor->cmd_buffer[id] += (torque / 1.2) * (25000.0 / 24.0);
		break;
	default:
		break;
	}
}

//id = 0 1 2 3   此处speed=rpm每分钟转数 
void motor_3508_speed_ctrl(DJI_Motor_ControlTypeDef* motor, uint16_t id, int16_t speed)
{
	motor_3508_speed_pid(motor, id, speed);
}

//0-8191 -> 0-360  
void motor_3508_position_ctrl(DJI_Motor_ControlTypeDef* motor, uint16_t id, int16_t position)
{
	motor_3508_position_pid(motor, id, position);
}

//position_speed推荐<540
void motor_3508_speed_position_ctrl(DJI_Motor_ControlTypeDef* motor, uint16_t id, int16_t speed, int position)
{
	motor_3508_speed_position_pid(motor, id, speed, position);
}

//力矩模式Nm
void motor_3508_torque_ctrl(DJI_Motor_ControlTypeDef* motor, uint16_t id, float torque, uint8_t motor_type)
{
	switch (motor_type)
	{
	case DJI_M2006:
		motor->cmd_buffer[id] = (torque / 1.0) * (10000.0 / 10.0);
		break;
	case DJI_M3508:
		motor->cmd_buffer[id] = (torque / 3.0) * (16384.0 / 20.0);
		break;
	case DJI_M6020:
		motor->cmd_buffer[id] = (torque / 1.2) * (25000.0 / 24.0);
		break;
	default:
		break;
	}
}

void motor_3508_speed_torque_ctrl(DJI_Motor_ControlTypeDef* motor, uint16_t id, int16_t speed, float torque, uint8_t motor_type)
{
	motor_3508_speed_pid(motor, id, speed);
	motor_3508_torque_cal(motor, id, torque, motor_type);
}

void motor_3508_position_torque_ctrl(DJI_Motor_ControlTypeDef* motor, uint16_t id, int16_t position, float torque, uint8_t motor_type)
{
	motor_3508_position_pid(motor, id, position);
	motor_3508_torque_cal(motor, id, torque, motor_type);
}

void motor_3508_speed_position_torque_ctrl(DJI_Motor_ControlTypeDef* motor, uint16_t id, int16_t speed, int position, float torque, uint8_t motor_type)
{
	motor_3508_speed_position_pid(motor, id, speed, position);
	motor_3508_torque_cal(motor, id, torque, motor_type);
}

void motor_3508_zero_torque_ctrl(DJI_Motor_ControlTypeDef* motor, uint16_t id)
{
	motor->cmd_buffer[id] = 0;
	rm_motor_cmd_refresh(&(motor->RM_CAN), motor->cmd_buffer[id], motor->cmd_buffer[1], motor->cmd_buffer[2], motor->cmd_buffer[3]);
}

void motor_3508_ctrl(DJI_Motor_ControlTypeDef* motor, uint16_t id, int16_t target_speed, int target_position, float torque, uint8_t mode, uint8_t motor_type)
{
	switch (mode)
	{
	case 0:
		motor_3508_speed_ctrl(motor, id, target_speed);
		break;
	case 1:
		motor_3508_position_ctrl(motor, id, target_position);
		break;
	case 2:
		motor_3508_speed_position_ctrl(motor, id, target_speed, target_position);
		break;
	case 3:
		motor_3508_torque_ctrl(motor, id, torque, motor_type);
		break;
	case 4:
		motor_3508_speed_torque_ctrl(motor, id, target_speed, torque, motor_type);
		break;
	case 5:
		motor_3508_position_torque_ctrl(motor, id, target_position, torque, motor_type);
		break;
	case 6:
		motor_3508_speed_position_torque_ctrl(motor, id, target_speed, target_position, torque, motor_type);
		break;
	case 7:
		motor_3508_zero_torque_ctrl(motor, id);
		break;
	default:
		break;
	}
	rm_motor_cmd_refresh(&(motor->RM_CAN), motor->cmd_buffer[0], motor->cmd_buffer[1], motor->cmd_buffer[2], motor->cmd_buffer[3]);
	rm_motor_send_msg(&(motor->RM_CAN));
}

void speed_ctrl(DJI_Motor_ControlTypeDef* motor, int16_t speed_1, int16_t speed_2, int16_t speed_3, int16_t speed_4, uint8_t motor_type)
{
	float speed[4] = { speed_1,speed_2,speed_3,speed_4 };
	for (int i = 0; i < 4; i++)
	{
		motor->M3508[i].exp_speed = speed[i];
	}
	for (int i = 0; i < 4; i++)
	{
		motor_3508_speed_ctrl(motor, i, motor->M3508[i].exp_speed);
	}
	rm_motor_cmd_refresh(&(motor->RM_CAN), motor->cmd_buffer[0], motor->cmd_buffer[1], motor->cmd_buffer[2], motor->cmd_buffer[3]);
	rm_motor_send_msg(&(motor->RM_CAN));
}

void position_ctrl(DJI_Motor_ControlTypeDef* motor, int16_t position_1, int16_t position_2, int16_t position_3, int16_t position_4, uint8_t motor_type)
{
	float position[4] = { position_1,position_2,position_3,position_4 };
	for (int i = 0; i < 4; i++)
	{
		motor->M3508[i].exp_position = position[i];
	}
	for (int i = 0; i < 4; i++)
	{
		motor_3508_position_ctrl(motor, i, motor->M3508[i].exp_position);
	}
	rm_motor_cmd_refresh(&(motor->RM_CAN), motor->cmd_buffer[0], motor->cmd_buffer[1], motor->cmd_buffer[2], motor->cmd_buffer[3]);
	rm_motor_send_msg(&(motor->RM_CAN));
}

void torque_ctrl(DJI_Motor_ControlTypeDef* motor, float torque_1, float torque_2, float torque_3, float torque_4, uint8_t motor_type)
{
	float torque[4] = { torque_1,torque_2,torque_3,torque_4 };
	for (int i = 0; i < 4; i++)
	{
		motor->M3508[i].exp_torque = torque[i];
	}
	for (int i = 0; i < 4; i++)
	{
		motor_3508_torque_ctrl(motor, i, motor->M3508[i].exp_torque, motor_type);
	}
	rm_motor_cmd_refresh(&(motor->RM_CAN), motor->cmd_buffer[0], motor->cmd_buffer[1], motor->cmd_buffer[2], motor->cmd_buffer[3]);
	rm_motor_send_msg(&(motor->RM_CAN));
}

void speed_torque_ctrl(DJI_Motor_ControlTypeDef* motor, int16_t speed_1, int16_t speed_2, int16_t speed_3, int16_t speed_4, float torque_1, float torque_2, float torque_3, float torque_4, uint8_t motor_type)
{
	float speed[4] = { speed_1,speed_2,speed_3,speed_4 };
	float torque[4] = { torque_1,torque_2,torque_3,torque_4 };
	for (int i = 0; i < 4; i++)
	{
		motor->M3508[i].exp_speed = speed[i];
		motor->M3508[i].exp_torque = torque[i];
	}
	for (int i = 0; i < 4; i++)
	{
		motor_3508_speed_torque_ctrl(motor, i, motor->M3508[i].exp_speed, motor->M3508[i].exp_torque, motor_type);
	}
	rm_motor_cmd_refresh(&(motor->RM_CAN), motor->cmd_buffer[0], motor->cmd_buffer[1], motor->cmd_buffer[2], motor->cmd_buffer[3]);
	rm_motor_send_msg(&(motor->RM_CAN));
}

void position_torque_ctrl(DJI_Motor_ControlTypeDef* motor, int16_t position_1, int16_t position_2, int16_t position_3, int16_t position_4, float torque_1, float torque_2, float torque_3, float torque_4, uint8_t motor_type)
{
	float position[4] = { position_1,position_2,position_3,position_4 };
	float torque[4] = { torque_1,torque_2,torque_3,torque_4 };
	for (int i = 0; i < 4; i++)
	{
		motor->M3508[i].exp_position = position[i];
		motor->M3508[i].exp_torque = torque[i];
	}
	for (int i = 0; i < 4; i++)
	{
		motor_3508_position_torque_ctrl(motor, i, motor->M3508[i].exp_position, motor->M3508[i].exp_torque, motor_type);
	}
	rm_motor_cmd_refresh(&(motor->RM_CAN), motor->cmd_buffer[0], motor->cmd_buffer[1], motor->cmd_buffer[2], motor->cmd_buffer[3]);
	rm_motor_send_msg(&(motor->RM_CAN));
}

void zero_torque_ctrl(DJI_Motor_ControlTypeDef* motor)
{
	motor_position_cal(motor,0);
	motor_position_cal(motor,1);
	motor_position_cal(motor,2);
	motor_position_cal(motor,3);
	rm_motor_cmd_refresh(&(motor->RM_CAN), 0, 0, 0, 0);
	rm_motor_send_msg(&(motor->RM_CAN));
}

//MIT_ctrl时,只用控制torque+KP*position+KD*speed(position的微分,不是自己给速度)
void MIT_ctrl(DJI_Motor_ControlTypeDef* motor, uint8_t id, int16_t position, float torque, uint8_t motor_type)
{
	motor_3508_position_pid(motor, id, position);
	motor_3508_torque_cal(motor, id, torque, motor_type);

	float error = (position - motor->motor_3508_position[id].last_target) / motor->motor_s_pid[id].rate;
	motor->motor_3508_position[id].last_target = position;

	pid_error_input(motor->motor_s_pid + id, error);
	motor->cmd_buffer[id] += pid_output(motor->motor_s_pid + id);
}
