#include "gimbal.h"
#include "mpu6050.h"
#include "inv_mpu6050.h"
#include "inv_dmp_mpu6050.h"
#include "pid.h"
#include "can.h"

#define gx_Kp    1
#define gx_Kd    0
#define gy_Kp    1
#define gy_Kd    0


gimbal_dataTypeDef gimbal_data = {0};

void gimbal_init(gimbal_dataTypeDef* gdata, DJI_Motor_ControlTypeDef* motor)
{
    motor_ctrl_init(motor, 0x1fe, &hcan1, HIGH);

    gdata->my_x = 355;
    gdata->my_y = 164;

    pid_controller_create(gdata->gimbal_pid+0, gx_Kp, 0, gx_Kd, 100, 65535);
    pid_controller_create(gdata->gimbal_pid+1, gy_Kp, 0, gy_Kd, 100, 65535);
}

// 逆时针旋转云台，扫描靶心位置
void gimbal_find_target(gimbal_dataTypeDef* gdata)
{
    if (gdata->fstate == Finding)
    {
        /* code */
    }
    else if (gdata->fstate == Locked)
    {
        /* code */
    }
    
}

// 云台载MCU计算传给云台电机的误差
void gimbal_pid_cal(gimbal_dataTypeDef* gdata)
{
    gdata->gimbal_error[0] = (float)(gdata->target_x - gdata->my_x);
    gdata->gimbal_error[1] = (float)(gdata->target_y - gdata->my_y);

    pid_error_input(gdata->gimbal_pid+0, gdata->gimbal_error[0]);
    pid_error_input(gdata->gimbal_pid+1, gdata->gimbal_error[1]);
}

// 云台控制：gimbal的坐标传给motor，motor调自己的pid函数完成运动
void gimbal_move(gimbal_dataTypeDef* gdata, DJI_Motor_ControlTypeDef* motor)
{
    gimbal_pid_cal(gdata);

    motor->M3508[0].exp_speed = pid_output(gdata->gimbal_pid+0);
    motor->M3508[1].exp_speed = pid_output(gdata->gimbal_pid+0);

    motor_3508_speed_ctrl(motor, 0, motor->M3508[0].exp_speed);
    motor_3508_speed_ctrl(motor, 1, motor->M3508[1].exp_speed);
}

// 收包回调：K230的数据提取到gimbal的坐标里
void gimbal_get_Kdata(gimbal_dataTypeDef* gdata, datapack_TypeDef* kdata)
{
    gdata->target_x |= kdata->this_pack[1] << 8;
    gdata->target_x |= kdata->this_pack[2] << 0;

    gdata->target_y |= kdata->this_pack[3] << 8;
    gdata->target_y |= kdata->this_pack[4] << 0;
}

void gimbal_get_mpudata(gimbal_dataTypeDef* gdata)
{
    mpu_dmp_get_data(&gdata->mpu_data.pitch, &gdata->mpu_data.roll, &gdata->mpu_data.yaw);
    MPU_Get_Accelerometer(&gdata->mpu_data.ax, &gdata->mpu_data.ay, &gdata->mpu_data.az);
    MPU_Get_Gyroscope(&gdata->mpu_data.gyrox, &gdata->mpu_data.gyroy, &gdata->mpu_data.gyroz);
}

