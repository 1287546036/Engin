/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       gimbal_behaviour.c/h
  * @brief      完成云台行为任务。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H
#include "main.h"

#include "Gimbal_Task.h"

typedef enum
{
  GIMBAL_ZERO_FORCE = 0, //云台无力
  GIMBAL_INIT,           //云台初始化
  GIMBAL_ABSOLUTE_ANGLE, //云台陀螺仪绝对角度控制
  GIMBAL_RELATIVE_ANGLE, //云台电机编码值相对角度控制
} gimbal_behaviour_e;
typedef enum
{
  HAND_shoot,
	AUTO_shoot,
} shoot_behaviour_e;


extern void gimbal_behaviour_mode_set(Gimbal_Control_t *gimbal_mode_set);
extern void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_Control_t *gimbal_control_set);
extern bool_t gimbal_cmd_to_chassis_stop(void);
extern bool_t gimbal_cmd_to_shoot_stop(void);
#endif




