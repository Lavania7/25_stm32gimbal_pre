#ifndef __GIMBAL_H_
#define __GIMBAL_H_

#include "main.h"
#include "M3508.h"
#include "receive.h"

typedef enum __find_state
{
    None,       // 上电状态
    Finding,    // 寻找靶心中
    Locked,     // 靶心位置已被K230捕获
}find_state;

typedef struct __mpu_dataTypeDef
{
    short gyrox;
    short gyroy;
    short gyroz;

    short ax;
    short ay;
    short az;

    float pitch;
    float yaw;
    float roll;

}mpu_dataTypeDef;

typedef struct __gimbal_dataTypeDef // 云台本地数据
{
    mpu_dataTypeDef mpu_data;

    uint16_t my_x;  // 云台激光笔x
    uint16_t my_y;  // 云台激光笔y

    uint16_t target_x;   // 目标点x
    uint16_t target_y;   // 目标点y

    float gimbal_error[2];  // 云台传给电机的位置误差
    pid_controller gimbal_pid[2];  // 云台pid

    // circle_dataTypeDef circle;    默认逆时针记录所有的循迹点坐标

    find_state fstate;   // 云台工作模式

}gimbal_dataTypeDef;

void gimbal_init(gimbal_dataTypeDef* gdata, DJI_Motor_ControlTypeDef* motor);
void gimbal_find_target(gimbal_dataTypeDef* gdata);
void gimbal_move(gimbal_dataTypeDef* gdata, DJI_Motor_ControlTypeDef* motor);
void gimbal_get_Kdata(gimbal_dataTypeDef* gdata, datapack_TypeDef* kdata);
void gimbal_get_mpudata(gimbal_dataTypeDef* gdata);

extern gimbal_dataTypeDef gimbal_data;

#endif
