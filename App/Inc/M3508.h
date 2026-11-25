#ifndef __M3508_H__
#define __M3508_H__

#include "main.h"
#include "rm_motor.h"
#include "pid.h"

typedef struct
{
	int16_t exp_speed;
	uint16_t exp_position;
	float exp_torque;
}ExpTypeDef;

typedef struct
{
    int now_position; // 当前角度
	int last_position; // 上一次的位置
	int this_position;//走过的位置
	int last_target;//上一次的目标（防止反复执行）
	int stay_target;//维持目标位置
}motor_positionTypeDef;

typedef struct
{
	rm_motor_groupTypeDef RM_CAN;
	float cmd_buffer[4];
	ExpTypeDef M3508[4];
	pid_controller motor_s_pid[4];
	pid_controller motor_p_pid[4];
	motor_positionTypeDef motor_3508_position[4];
}DJI_Motor_ControlTypeDef;

extern DJI_Motor_ControlTypeDef M2006_Motor;
extern DJI_Motor_ControlTypeDef M3508_Motor;
extern DJI_Motor_ControlTypeDef M6020_Motor;

#define Speed_Mode 0
#define Position_Mode 1
#define Speed_Position_Mode 2
#define Torque_Mode 3
#define Speed_Torque_Mode 4
#define Position_Torque_Mode 5
#define Speed_Position_Torque_Mode 6
#define Zero_Torque_Mode 7

#define DJI_M2006 8
#define DJI_M3508 9
#define DJI_M6020 10

#define Nom_Ctrl 11
#define MIT_Ctrl 12

#define LOW 13
#define HIGH 14

#define PID_CAL_RATE 1000
#define PID_MAX 8000

void motor_position_cal(DJI_Motor_ControlTypeDef* motor,uint8_t id);
void motor_pid_init(pid_controller* hpid, float kp, float ki, float kd, int32_t rate);
void motor_pid_data_init(DJI_Motor_ControlTypeDef* motor, uint8_t motor_type, uint8_t control_type,uint8_t MOTOR_GROUP);
void motor_ctrl_init(DJI_Motor_ControlTypeDef* motor, uint8_t motor_type,CAN_HandleTypeDef* hcan,uint8_t MOTOR_GROUP);
void motor_MIT_ctrl_init(DJI_Motor_ControlTypeDef* motor, uint8_t motor_type,CAN_HandleTypeDef* hcan,uint8_t MOTOR_GROUP);
void motor_3508_speed_pid(DJI_Motor_ControlTypeDef* motor, uint16_t id, int16_t target_speed);
void motor_3508_position_pid(DJI_Motor_ControlTypeDef* motor, uint16_t id, int16_t target_postion);
void motor_3508_speed_position_pid(DJI_Motor_ControlTypeDef* motor, uint16_t id, int16_t speed,int target_position);

//单个电机单个模式，若想使用请加rm_motor_send_msg(&(motor->RM_CAN));
void motor_3508_speed_ctrl(DJI_Motor_ControlTypeDef* motor, uint16_t id, int16_t speed);
void motor_3508_position_ctrl(DJI_Motor_ControlTypeDef* motor, uint16_t id, int16_t position);
void motor_3508_speed_position_ctrl(DJI_Motor_ControlTypeDef* motor, uint16_t id, int16_t speed,int position);
void motor_3508_torque_ctrl(DJI_Motor_ControlTypeDef* motor, uint16_t id, float force, uint8_t motor_type);
void motor_3508_speed_torque_ctrl(DJI_Motor_ControlTypeDef* motor, uint16_t id, int16_t speed, float force, uint8_t motor_type);
void motor_3508_position_torque_ctrl(DJI_Motor_ControlTypeDef* motor, uint16_t id, int16_t speed, float force, uint8_t motor_type);
void motor_3508_speed_position_torque_ctrl(DJI_Motor_ControlTypeDef* motor, uint16_t id, int16_t speed, int position, float force, uint8_t motor_type);
void motor_3508_zero_torque_ctrl(DJI_Motor_ControlTypeDef* motor, uint16_t id);

//一个函数控单个电机多种模式
void motor_3508_ctrl(DJI_Motor_ControlTypeDef* motor, uint16_t id, int16_t target_speed, int target_position, float torque, uint8_t mode, uint8_t motor_type);

//一个函数一次性控4个电机单个模式
void speed_ctrl(DJI_Motor_ControlTypeDef* motor, int16_t speed_1, int16_t speed_2, int16_t speed_3, int16_t speed_4, uint8_t motor_type);
void position_ctrl(DJI_Motor_ControlTypeDef* motor, int16_t position_1, int16_t position_2, int16_t position_3, int16_t position_4, uint8_t motor_type);
void torque_ctrl(DJI_Motor_ControlTypeDef* motor, float torque_1, float torque_2, float torque_3, float torque_4, uint8_t motor_type);
void speed_torque_ctrl(DJI_Motor_ControlTypeDef* motor, int16_t speed_1, int16_t speed_2, int16_t speed_3, int16_t speed_4, float torque_1, float torque_2, float torque_3, float torque_4, uint8_t motor_type);
void position_torque_ctrl(DJI_Motor_ControlTypeDef* motor, int16_t position_1, int16_t position_2, int16_t position_3, int16_t position_4, float torque_1, float torque_2, float torque_3, float torque_4, uint8_t motor_type);
void zero_torque_ctrl(DJI_Motor_ControlTypeDef* motor);

//MIT控制,一个函数控单个电机多种模式(推荐)
void MIT_ctrl(DJI_Motor_ControlTypeDef* motor, uint8_t id, int16_t position, float torque, uint8_t motor_type);
//使用MIT_ctrl请加上:
//rm_motor_cmd_refresh(&(motor->RM_CAN), motor->cmd_buffer[0], motor->cmd_buffer[1], motor->cmd_buffer[2], motor->cmd_buffer[3]);
//rm_motor_send_msg(&(motor->RM_CAN));


#endif
