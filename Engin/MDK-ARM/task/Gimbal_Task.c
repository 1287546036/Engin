 /**
  ****************************(C)******** ****************************
  * @file       gimbal_task.c/h
  * @brief      完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-PI,PI）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用外接的陀螺仪（JY901S）解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有，停止状态等。。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM                1. 完成 
	*  V2.0.0     2021/12/29     RM、ljl            1. 完成 
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) ************************************
  */

#include "Gimbal_Task.h"
#include "main.h"
#include "remote_control.h"
#include "motor.h"
#include "pid.h"

extern motor_measure_t motor_gimbal[2];

pids gimbal_motor_pid;

#define rc_to_angle  100/660.0f
fp32 gimbal_speed[2];
int16_t gimbal_current[4];

void gimbal_task(void)
{
//
pidINIT(&gimbal_motor_pid,PID_POSITION,3,3,0,600,200);
fp32 set_angle=rc_ctrl.rc.ch[1]*8191/660*0.01;


gimbal_speed[0]=PID_calc(&gimbal_motor_pid,motor_gimbal[0].real_angle,motor_gimbal[0].real_angle+set_angle);
//current[0]=PID_calc(&motor_pid,motor_chassis[0].real_angle,motor_chassis[0].real_angle+set_angle)+PID_calc(&motor_pid,motor_chassis[0].speed_rpm,0);
gimbal_current[0]=PID_calc(&gimbal_motor_pid,motor_gimbal[0].speed_rpm,gimbal_speed[0]);

CAN_cmd_chassis(gimbal_current);
gimbal_current[0]=PID_calc(&gimbal_motor_pid,motor_gimbal[0].speed_rpm,-gimbal_speed[0]/10);
CAN_cmd_chassis(gimbal_current);


pidINIT(&gimbal_motor_pid,PID_POSITION,3,3,0,900,400);

gimbal_speed[1]=PID_calc(&gimbal_motor_pid,motor_gimbal[1].real_angle,motor_gimbal[1].real_angle+set_angle);
//current[0]=PID_calc(&motor_pid,motor_chassis[0].real_angle,motor_chassis[0].real_angle+set_angle)+PID_calc(&motor_pid,motor_chassis[0].speed_rpm,0);
gimbal_current[1]=PID_calc(&gimbal_motor_pid,motor_gimbal[1].speed_rpm,gimbal_speed[1]);
CAN_cmd_chassis(gimbal_current);
gimbal_current[1]=PID_calc(&gimbal_motor_pid,motor_gimbal[1].speed_rpm,-gimbal_speed[1]/10);
CAN_cmd_chassis(gimbal_current);

}

