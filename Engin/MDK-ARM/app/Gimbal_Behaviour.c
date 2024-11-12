/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       gimbal_behaviour.c/h
  * @brief      完成云台行为任务。//记得写限位
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

#include "Gimbal_Behaviour.h"
//#include "arm_math.h"
//#include "vision_task.h"

//#include "Detect_Task.h"

//#include "math_lib.h"



#define int_abs(x) ((x) > 0 ? (x) : (-x))
/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定是发送1024过来，
  * @author         RM
  * @param[in]      输入的遥控器值
  * @param[in]      输出的死区处理后遥控器值
  * @param[in]      死区值
  * @retval         返回空
  */
#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }


/**
  * @brief          云台行为状态机设置，因为在cali等模式下使用了return，故而再用了一个函数
  * @author         RM
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_behavour_set(Gimbal_Control_t *gimbal_mode_set);

/**
  * @brief          云台无力控制，在这个模式下发送的yaw，pitch 是电机控制原始值，云台电机发送can零控制量，
		                使得云台无力。 当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
  *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
  * @author         RM
  * @param[in]      参数为输出，发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      参数为输出，发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);
		
/**
  * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);


/**
  * @brief          云台编码值控制，电机是相对角度控制，
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);


//云台行为状态机
 gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;
 shoot_behaviour_e shoot_behaviour =HAND_shoot;

/**
  * @brief          云台行为状态机以及电机状态机设置
  * @author         RM
  * @param[in]      云台数据指针
  * @retval         返回空
  */
void gimbal_behaviour_mode_set(Gimbal_Control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //云台行为状态机设置，根据遥控器键盘值或其他条件改变gimbal_behaviour这个变量（改变行为模式）
    gimbal_behavour_set(gimbal_mode_set);

    //根据云台行为状态机设置电机状态机
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)//云台无力->电机原生状态
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_INIT)//云台初始化->电机编码值角度控制
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }  
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)//云台电机编码值相对角度控制->电机编码值角度控制
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
		if(shoot_behaviour==AUTO_shoot)
		{
			gimbal_mode_set->gimbal_yaw_motor.AUTO_flag=1;
			gimbal_mode_set->gimbal_pitch_motor.AUTO_flag=1;
		}
		else if(shoot_behaviour==HAND_shoot)
		{
			gimbal_mode_set->gimbal_yaw_motor.AUTO_flag=0;
			gimbal_mode_set->gimbal_pitch_motor.AUTO_flag=0;
		}			
}

/**
  * @brief          云台行为控制，根据不同行为采用不同控制函数
  * @author         RM
  * @param[in]      设置的yaw角度增加值，单位 rad
  * @param[in]      设置的pitch角度增加值，单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_Control_t *gimbal_control_set)
{

    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static fp32 rc_add_yaw, rc_add_pit;
    static int16_t yaw_channel = 0, pitch_channel = 0;

    //将遥控器的数据处理死区（将得到的位于[-10，10]的通道值变为0） int16_t yaw_channel,pitch_channel
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YawChannel], yaw_channel, RC_deadband);
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PitchChannel], pitch_channel, RC_deadband);
		
    //将遥控器和鼠标的控制值转化为对pitch yaw 的角度控制值，遥控器值和鼠标值可以叠加
//    rc_add_yaw = yaw_channel * Yaw_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * Yaw_Mouse_Sen-rx_package.RX_data.zset*0.1;
		rc_add_yaw = yaw_channel * Yaw_RC_SEN - gimbal_control_set->referee_remote_control_t->mouse_x * Yaw_Mouse_Sen;
    rc_add_pit =-(pitch_channel * Pitch_RC_SEN + gimbal_control_set->referee_remote_control_t->mouse_y * Pitch_Mouse_Sen);

    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_zero_force_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);//设置为零
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_init_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);//改变了前两个参数的值
    }  
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_absolute_angle_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);//这里有按键控制180度掉头和45度左右转向，改变了第一个参数的值
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_relative_angle_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);//无操作
    }
   
    //将控制增加量赋值
    *add_yaw = rc_add_yaw;
    *add_pitch = rc_add_pit;
}

/**
  * @brief          云台在某些行为下，需要底盘不动
  * @author         RM
  * @param[in]      void
  * @retval         返回空
  */

bool_t gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;
/**
  * @brief          云台行为状态机设置，因为在cali等模式下使用了return，故而再用了一个函数
  * @author         RM
  * @param[in]      云台数据指针
  * @retval         返回空
  */
 int16_t count = 0;
int16_t flag;
static void gimbal_behavour_set(Gimbal_Control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
  
    //初始化模式判断是否到达中值位置
    if (gimbal_behaviour == GIMBAL_INIT)
    {
        static uint16_t init_time = 0;
        static uint16_t init_stop_time = 0;
        init_time++;
        //到达中值 计时
        if ((fabs(gimbal_mode_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
             fabs(gimbal_mode_set->gimbal_pitch_motor.relative_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR))
        {
            //到达初始化位置会进入这里
            if (init_stop_time < GIMBAL_INIT_STOP_TIME)
            {
                init_stop_time++;
            }
        }
        else
        {
            //没有到达初始化位置进入这里，时间计时
            if (init_time < GIMBAL_INIT_TIME)
            {
                init_time++;
            }
        }

        //超过初始化最大时间，或者已经稳定到中值一段时间，退出初始化状态开关打下档，或者掉线
//        if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME &&
//            !switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]) && !toe_is_error(DBUSTOE))
        if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME && !switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]) )
				{
            return;//在这跳出去就是为了让其初始化未完成一直是init模式
        }
        else
        {
            init_stop_time = 0;
            init_time = 0;
        }
    }

    //开关控制 云台状态
    if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]))
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }
    else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]))
    {
        gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;
    }
    else if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]))
    {
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
    }
				if(gimbal_mode_set->referee_remote_control_t->right_button_down)
		{   
	 shoot_behaviour=AUTO_shoot;
	 }
   else
	 {
	 shoot_behaviour=HAND_shoot;
   }

//    if( toe_is_error(DBUSTOE))
//    {

//        gimbal_behaviour = GIMBAL_ZERO_FORCE;
//    }

    //判断进入init状态机
    {
//         gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;
        if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE && gimbal_behaviour != GIMBAL_ZERO_FORCE)
        {
            gimbal_behaviour = GIMBAL_INIT;
        }
        last_gimbal_behaviour = gimbal_behaviour;
    }

     
    


}

/**
  * @brief          云台无力控制，在这个模式下发送的yaw，pitch 是电机控制原始值，云台电机发送can零控制量，使得云台无力
  * @author         RM
  * @param[in]      参数为输出，发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      参数为输出，发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}
/**
  * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    //初始化状态控制量计算
    if (fabs(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) > GIMBAL_INIT_ANGLE_ERROR)
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
    }
}


	


/**
  * @brief          云台编码值控制，电机是相对角度控制，
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
	
    
}




