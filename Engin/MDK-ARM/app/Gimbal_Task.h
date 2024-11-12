/**
  ****************************(C)******** ****************************
  * @file       gimbal_task.c/h
  * @brief      完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-PI,PI）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用外接的陀螺仪（WT901）解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有，停止状态等。。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成 
	*  V2.0.0     2021/12/29     RM、ljl          1. 完成 
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) ************************************
  */

#ifndef GIMBALTASK_H
#define GIMBALTASK_H
#include "main.h"
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
//#include "vision_task.h"
//#include "vision_uart.h"
//#include "math_lib.h"
//#include "referee.h"

extern void gimbal_task(void);


///***************pitch绝对角度控制（陀螺仪)***************/
////pitch 速度环 PID参数以及 PID最大输出，积分输出
//#define PITCH_SPEED_PID_KP 4100.0f
//#define PITCH_SPEED_PID_KI 0.0f
//#define PITCH_SPEED_PID_KD 500.0f
//#define PITCH_SPEED_PID_MAX_OUT 30000.0f
//#define PITCH_SPEED_PID_MAX_IOUT 5000.0f

////pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
//#define PITCH_GYRO_ABSOLUTE_PID_KP 18.0f
//#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
//#define PITCH_GYRO_ABSOLUTE_PID_KD 0.0f
//#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
//#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 3.0f

///***************pitch相对角度控制(电机编码器)***************/
////pitch 速度环 PID参数
//#define PITCH_SPEED_RELATIVE_PID_KP 3200.0f
//#define PITCH_SPEED_RELATIVE_PID_KI 0.0f
//#define PITCH_SPEED_RELATIVE_PID_KD 3000.0f
//#define PITCH_SPEED_RELATIVE_PID_MAX_OUT 30000.0f
//#define PITCH_SPEED_RELATIVE_PID_MAX_IOUT 5000.0f
////pitch 速度环 自瞄小角度PID参数
//#define AUTO_PITCH_SPEED_RELATIVE_PID_KP 3200.0f
//#define AUTO_PITCH_SPEED_RELATIVE_PID_KI 0.0f
//#define AUTO_PITCH_SPEED_RELATIVE_PID_KD 3000.0f

////pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
//#define PITCH_ENCODE_RELATIVE_PID_KP 30.0f
//#define PITCH_ENCODE_RELATIVE_PID_KI 0.0f
//#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f
//#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
//#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 3.0f
////pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
//#define AUTO_PITCH_ENCODE_RELATIVE_PID_KP 30.0f
//#define AUTO_PITCH_ENCODE_RELATIVE_PID_KI 0.0f
//#define AUTO_PITCH_ENCODE_RELATIVE_PID_KD 0.0f

///***************yaw相对角度控制(电机编码器)***************/
////yaw 速度环 PID参数以及 PID最大输出，积分输出
//#define YAW_SPEED_RELATIVE_PID_KP 6000.0f
//#define YAW_SPEED_RELATIVE_PID_KI 0.0f
//#define YAW_SPEED_RELATIVE_PID_KD 2000.0f
//#define YAW_SPEED_RELATIVE_PID_MAX_OUT 30000.0f
//#define YAW_SPEED_RELATIVE_PID_MAX_IOUT 5000.0f
////自瞄小角度yaw 速度环 PID参数以及 PID最大输出，积分输出
//#define AUTO_YAW_SPEED_RELATIVE_PID_KP 2575.0f
//#define AUTO_YAW_SPEED_RELATIVE_PID_KI 0.0f
//#define AUTO_YAW_SPEED_RELATIVE_PID_KD 1300.0f

////yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
//#define YAW_ENCODE_RELATIVE_PID_KP 20.0f
//#define YAW_ENCODE_RELATIVE_PID_KI 0.0f
//#define YAW_ENCODE_RELATIVE_PID_KD 0.0f
//#define YAW_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
//#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT 3.0f

////任务初始化 空闲一段时间
//#define GIMBAL_TASK_INIT_TIME 3000
////云台控制周期
//#define GIMBAL_CONTROL_TIME 1

////yaw,pitch控制通道以及状态开关通道
//#define YawChannel 2
//#define PitchChannel 3

////状态开关通道
//#define ModeChannel 0


//#define TurnLeftKeyBoard KEY_PRESSED_OFFSET_Q //头左转45度按键
//#define TurnBackKeyBoard KEY_PRESSED_OFFSET_X //掉头180度按键
//#define TurnRightKeyBoard KEY_PRESSED_OFFSET_E //头右转45度按键
////掉头、转45度角云台速度
//#define TurnSpeed 0.04f


////遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
//#define RC_deadband 10

////yaw，pitch角度与遥控器输入比例
//#define Yaw_RC_SEN -0.000005f
//#define Pitch_RC_SEN -0.000006f //0.005


////yaw,pitch角度和鼠标输入的比例
////yaw,pitch角度和鼠标输入的比例
//#define Yaw_Mouse_Sen 0.000026f
//#define Pitch_Mouse_Sen 0.000026f




////电机码盘值最大以及中值
//#define Half_ecd_range 4096
//#define ecd_range 8191

////云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
//#define GIMBAL_INIT_ANGLE_ERROR 0.025f
//#define GIMBAL_INIT_STOP_TIME 100
//#define GIMBAL_INIT_TIME 6000

////云台初始化回中值的速度以及控制到的角度
//#define GIMBAL_INIT_PITCH_SPEED 0.004f
//#define GIMBAL_INIT_YAW_SPEED   0.005f
//#define INIT_YAW_SET 0.0f
//#define INIT_PITCH_SET 0.0f

////判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
//#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
//#define GIMBAL_MOTIONLESS_TIME_MAX 3000

////电机（6020）编码值转化成角度值
//#ifndef Motor_Ecd_to_Rad
//#define Motor_Ecd_to_Rad 0.000766990394f //      2*  PI  /8192
//#endif


////下面的参数是需要人为结合实际情况（电机的安装位置）手动进行校准（就是修改下面的值）
////yaw轴电机相应的参数
//#define Yaw_Offset_Ecd  (5303)//实际的云台电机处于中值时反馈的编码值，这中值会随着电机的安装而发生改变，所以需要根据实际情况人为校准修改
////yaw轴不需要限幅，因此不需要确定yaw轴的最大最小值

////pitch轴电机相应的参数
//#define Pitch_Offset_Ecd  (7258)
////相对角度
//#define Pitch_Max_Relative_Angle (1.6288f)//1.3276f

//#define Pitch_Min_Relative_Angle (-0.431711f)//-0.4410f

////绝对角度
//#define Pitch_Max_Absolute_Angle (0.7088f)//1.3276f

//#define Pitch_Min_Absolute_Angle (-0.2072f)//-0.4522f

//typedef enum
//{
//    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
//    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
//    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
//} gimbal_motor_mode_e;

//typedef struct
//{
//    fp32 kp;
//    fp32 ki;
//    fp32 kd;//PID三参数

//    fp32 set;//
//    fp32 get;//
//    fp32 err;//

//    fp32 max_out;//最大输出
//    fp32 max_iout;//最大积分输出 用于初始化

//    fp32 Pout;//比例输出
//    fp32 Iout;//积分输出
//    fp32 Dout;//微分输出

//    fp32 out;//最终输出
//} Gimbal_PID_t;//角度PID

//typedef struct
//{
//    const motor_measure_t *gimbal_motor_measure;//云台电机的原始数据指针 初始化
//    
//	  Gimbal_PID_t gimbal_motor_absolute_angle_pid;//陀螺仪绝对角度PID
//	
//    Gimbal_PID_t gimbal_motor_relative_angle_pid;//编码器相对角度PID
//		
//	  Gimbal_PID_t AUTO_gimbal_motor_relative_angle_pid;//编码器相对角度PID
//    
//	  pids gimbal_motor_gyro_pid;//速度（角速度）环PID（绝对）
//    
//	  pids gimbal_motor_gyro_relative_pid;//速度（角速度）环PID（相对）
//	
//	  pids gimbal_motor_AUTO_gyro_relative_pid;//速度（角速度）环PID（相对）
//	
//	  gimbal_motor_mode_e gimbal_motor_mode;
//    gimbal_motor_mode_e last_gimbal_motor_mode;//云台电机的控制模式
//    
//	  uint16_t offset_ecd;//编码器中值，需要人为手动校准
//	
//	  uint16_t  AUTO_flag;
//	
//    fp32 max_relative_angle; //rad，需要人为手动校准
//    fp32 min_relative_angle; //rad，需要人为手动校准
//	
//	  fp32 max_absolute_angle; //rad，需要人为手动校准
//    fp32 min_absolute_angle; //rad，需要人为手动校准

//	
//    fp32 relative_angle;     //单位是rad
//    fp32 relative_angle_set; //rad
// 	
//	  fp32 absolute_angle;     //rad
//    fp32 absolute_angle_set; //rad
//    
//		fp32 motor_gyro;         //rad/s，角速度
//    fp32 motor_gyro_set;
//    
//		fp32 motor_speed;
//    
//		fp32 raw_cmd_current;//在GIMBAL_Set_Contorl（）函数中赋值，然后在 GIMBAL_Control_loop（）中赋给given_current
//    
//		fp32 current_set;//经过串级pid算出来的值，赋给given_current
//    int16_t given_current;//电流值期望

//   fp32 data;
//} Gimbal_Motor_t;


//typedef struct
//{
////	  const ext_referee_remote_control_t *referee_remote_control_t;
//    const RC_ctrl_t *gimbal_rc_ctrl;//获取遥控器数据
//    const fp32 *gimbal_IMU_angle_point;//获取欧拉角指针
//    const fp32 *gimbal_IMU_gyro_point;//获取陀螺仪指针
//    Gimbal_Motor_t gimbal_yaw_motor;//获取yaw电机数据
//    Gimbal_Motor_t gimbal_pitch_motor;//获取pitch电机数据
////	  VisionConnect *vision1;

//} Gimbal_Control_t;

//extern Gimbal_Control_t gimbal_control;
//extern const Gimbal_Motor_t *get_yaw_motor_point(void);
//extern const Gimbal_Motor_t *get_pitch_motor_point(void);


#endif



