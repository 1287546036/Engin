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
//#include "vision_task.h"
//#include "vision_uart.h"
//#include "Shoot_Task.h"

#include "main.h"

//#include "arm_math.h"
//#include "Gimbal_Behaviour.h"
//#include "math_lib.h"
//#include "BMI_task.h"
#include "remote_control.h"
#include "motor.h"
//#include "Detect_Task.h"
#include "pid.h"

//#include "cmsis_os.h"
//#include "task.h"

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








//	while(1)
////	{	
//		pidINIT(&gimbal_motor_pid,PID_POSITION,2,0.1,0,80,10);
//		fp32 yaw   = rc_ctrl.rc.ch[2]*rc_to_angle;
//		fp32 pitch = rc_ctrl.rc.ch[3]*rc_to_angle;
//		int16_t gimbal_current[2];
//		gimbal_current[0]=PID_calc(&gimbal_motor_pid,motor_gimbal[0].real_angle,motor_gimbal[0].real_angle+yaw);
//		gimbal_current[1]=PID_calc(&gimbal_motor_pid,motor_gimbal[1].real_angle,motor_gimbal[1].real_angle+pitch);
//		
//		CAN_cmd_gimbal(gimbal_current);

////		vTaskDelay(2);//已经注释掉了发送的delay,发送500hz,接收1000hz
////	}


////电机编码值规整 0—8191
//#define ECD_Format(ecd)         \
//    {                           \
//        if ((ecd) > ecd_range)  \
//				{                       \
//            (ecd) -= ecd_range; \
//				}                       \
//        else if ((ecd) < 0)     \
//				{                       \
//            (ecd) += ecd_range; \
//				}                       \
//    }

//#define gimbal_total_pid_clear(gimbal_clear)                                                   \
//    {                                                                                          \
//        Gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
//        Gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
//        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
//        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_relative_pid);			     \
//                                                                                               \
//        Gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
//        Gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
//        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
//        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_relative_pid);		     \
//    }//清除所有PID


////云台控制所有相关数据
//		
// Gimbal_Control_t gimbal_control;//全局变量

////发送的can 指令
// int16_t Yaw_Can_Set_Current = 0, Pitch_Can_Set_Current = 0;

////云台初始化
//static void GIMBAL_Init(Gimbal_Control_t *gimbal_init);
//		
////云台pid清零
//static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear);
//		
////云台状态设置
//static void GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode);
//		
////云台数据更新
//static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update);

////云台状态切换保存数据，例如从陀螺仪状态切换到编码器状态保存目标值
//static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change);

////计算云台电机相对中值的相对角度
//fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

////设置云台控制量
//static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control);

////云台控制pid计算
//static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop);

////陀螺仪绝对角度控制
//static void gimbal_motor_absolute_angle_control(Gimbal_Motor_t *gimbal_motor);

////编码器相对角度控制
//static void gimbal_motor_relative_angle_control(Gimbal_Motor_t *gimbal_motor);

//static void gimbal_motor_raw_angle_control(Gimbal_Motor_t *gimbal_motor);

////在陀螺仪角度控制下，对控制的目标值进限制以防超最大相对角度
//static void GIMBAL_absolute_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add);

////在编码器角度控制下，对控制的目标值进限制以防超最大相对角度
//static void GIMBAL_relative_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add);

////云台电机PID初始化
//static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);      

////云台电机PID计算
//static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);



//void GIMBAL_task(void)
//{
//    //云台初始化	
//    GIMBAL_Init(&gimbal_control);
//	  vTaskDelay(GIMBAL_TASK_INIT_TIME );//将任务挂起3s，等待电机传回来的数据
//	
//    while (1)
//    {
//       GIMBAL_Set_Mode(&gimbal_control);                    //设置云台控制模式,云台模式的变化可能导致云台电机的控制方式的变化
//       GIMBAL_Mode_Change_Control_Transit(&gimbal_control); //控制模式切换 控制数据过渡，根据云台电机的控制方式的变化保存数据
//       			
//			 GIMBAL_Feedback_Update(&gimbal_control);             //云台数据反馈
//			 GIMBAL_Set_Contorl(&gimbal_control);                 //根据云台控制模式，先根据通道值获取期望的角度，再根据电机控制模式，对其进一步处理
//       GIMBAL_Control_loop(&gimbal_control);                //云台控制PID计算（根据电机控制模式，计算发给电流值）
//      
//       Yaw_Can_Set_Current = gimbal_control.gimbal_yaw_motor.given_current;			

//       Pitch_Can_Set_Current =gimbal_control.gimbal_pitch_motor.given_current;
//			
//       CAN_cmd_gimbal(Pitch_Can_Set_Current,0, 0, 0);
//			 CAN_cmd_pitch(0,Yaw_Can_Set_Current,0,0);
//      vTaskDelay(GIMBAL_CONTROL_TIME);//云台控制周期 1
//    }
//}




//const Gimbal_Motor_t *get_yaw_motor_point(void)
//{
//    return &gimbal_control.gimbal_yaw_motor;
//}

//const Gimbal_Motor_t *get_pitch_motor_point(void)
//{
//    return &gimbal_control.gimbal_pitch_motor;
//}


////初始化pid 数据指针
//static void GIMBAL_Init(Gimbal_Control_t *gimbal_init)
//{
//    //速度环PID(绝对)
//    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
//    static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
//		
//		//速度环PID(相对)
//		static const fp32 Yaw_speed_relative_pid[3] = {YAW_SPEED_RELATIVE_PID_KP, YAW_SPEED_RELATIVE_PID_KI,YAW_SPEED_RELATIVE_PID_KD };
//		
//		static const fp32 Pitch_speed_relative_pid[3] = {PITCH_SPEED_RELATIVE_PID_KP , PITCH_SPEED_RELATIVE_PID_KI,PITCH_SPEED_RELATIVE_PID_KD};
//	
//    //遥控器数据指针获取
//    gimbal_init->gimbal_rc_ctrl = get_remote_control_point();//获取遥控器数据
//		  
//		
//		//初始化yaw轴电机中值  		
//		gimbal_init->gimbal_yaw_motor.offset_ecd=Yaw_Offset_Ecd;
//		 //初始化电机模式  初始化为原生状态机
//    gimbal_init->gimbal_yaw_motor.gimbal_motor_mode = gimbal_init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
//		//电机数据指针获取 返回电机变量地址，通过指针方式获取原始数据
//    gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();		
//		//yaw轴不需要限幅，因此不需要确定yaw轴的最大最小值
//		//初始化pitch轴电机中值 
//		gimbal_init->gimbal_pitch_motor.offset_ecd=Pitch_Offset_Ecd;
//		//初始化电机模式  初始化为原生状态机
//		gimbal_init->gimbal_pitch_motor.gimbal_motor_mode = gimbal_init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
//		//电机数据指针获取 返回电机变量地址，通过指针方式获取原始数据
//		gimbal_init->gimbal_pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
//		
//		gimbal_init->gimbal_pitch_motor.max_absolute_angle=Pitch_Max_Absolute_Angle;
//		gimbal_init->gimbal_pitch_motor.min_absolute_angle=Pitch_Min_Absolute_Angle;
//		gimbal_init->gimbal_pitch_motor.max_relative_angle=Pitch_Max_Relative_Angle;
//		gimbal_init->gimbal_pitch_motor.min_relative_angle=Pitch_Min_Relative_Angle;
//		gimbal_init->gimbal_pitch_motor.max_relative_angle=Pitch_Max_Relative_Angle;
//		
///***************云台电机采用串级pid，第一环为角度环，第二环为速度环，即第一环的输出当作期望的速度（以下）***************************/   
//		
//		/*************初始化yaw轴电机pid**********/
//		//角度环 yaw轴电机	陀螺仪
//    GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, 
//		                                                 YAW_GYRO_ABSOLUTE_PID_MAX_OUT, 
//		                                                YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, 
//		                                                      YAW_GYRO_ABSOLUTE_PID_KP,
//		                                                      YAW_GYRO_ABSOLUTE_PID_KI, 
//														                              YAW_GYRO_ABSOLUTE_PID_KD);
//	  //角度环 yaw轴电机	 编码器																								
//    GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid,
//		                                               YAW_ENCODE_RELATIVE_PID_MAX_OUT, 
//		                                              YAW_ENCODE_RELATIVE_PID_MAX_IOUT, 
//		                                                    YAW_ENCODE_RELATIVE_PID_KP, 
//		                                                    YAW_ENCODE_RELATIVE_PID_KI, 
//														                            YAW_ENCODE_RELATIVE_PID_KD);
//																																		
//		//角度环 yaw轴电机 自瞄用																										
//	/***************************************************************************************************/																											
//		GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.AUTO_gimbal_motor_relative_angle_pid,
//		                                               YAW_ENCODE_RELATIVE_PID_MAX_OUT, 
//		                                              YAW_ENCODE_RELATIVE_PID_MAX_IOUT, 
//		                                                    AUTO_YAW_ENCODE_RELATIVE_PID_KP, 
//		                                                    AUTO_YAW_ENCODE_RELATIVE_PID_KI, 
//														                            AUTO_YAW_ENCODE_RELATIVE_PID_KD);
//	/**************************************************************************************************/																											
//    //速度环 yaw轴电机（绝对）
//	  PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_gyro_pid, 
//	                                                   PID_POSITION,
//																									  Yaw_speed_pid, 
//																			      YAW_SPEED_PID_MAX_OUT, 
//																				   YAW_SPEED_PID_MAX_IOUT);
//    
//		//速度环 yaw轴电机（相对）
//	  PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_gyro_relative_pid, 
//	                                                   PID_POSITION,
//																									  Yaw_speed_relative_pid, 
//																			      YAW_SPEED_RELATIVE_PID_MAX_OUT, 
//																				  YAW_SPEED_RELATIVE_PID_MAX_IOUT);
//		
//		/*************初始化pitch轴/电机pid**********/

//		//角度环 pitch轴电机	编码器																									
//    GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid,
//	  	                                             PITCH_ENCODE_RELATIVE_PID_MAX_OUT,
//			                                            PITCH_ENCODE_RELATIVE_PID_MAX_IOUT,
//		                                                    PITCH_ENCODE_RELATIVE_PID_KP, 
//	                                                    	PITCH_ENCODE_RELATIVE_PID_KI,
//		                                                    PITCH_ENCODE_RELATIVE_PID_KD);
//			//角度环 pitch轴电机	编码器		自瞄用																						
//    GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.AUTO_gimbal_motor_relative_angle_pid,
//	  	                                             PITCH_ENCODE_RELATIVE_PID_MAX_OUT,
//			                                            PITCH_ENCODE_RELATIVE_PID_MAX_IOUT,
//		                                                    AUTO_PITCH_ENCODE_RELATIVE_PID_KP, 
//	                                                    	AUTO_PITCH_ENCODE_RELATIVE_PID_KI,
//		                                                    AUTO_PITCH_ENCODE_RELATIVE_PID_KD);																										
//		//速度环	pitch轴电机	（绝对）																						
//    PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_gyro_pid,
//		                                                   PID_POSITION, 
//																			              Pitch_speed_pid,
//		                                        PITCH_SPEED_PID_MAX_OUT,
//		                                       PITCH_SPEED_PID_MAX_IOUT);
//   //速度环	pitch轴电机	（相对）
//    PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_gyro_relative_pid, 
//	                                                   PID_POSITION,
//																									  Pitch_speed_relative_pid, 
//																			      PITCH_SPEED_RELATIVE_PID_MAX_OUT, 
//																				  PITCH_SPEED_RELATIVE_PID_MAX_IOUT);
//     //清除所有PID
//    gimbal_total_pid_clear(gimbal_init);
//		
///****************云台电机采用串级pid，第一环为角度环，第二环为速度环，即第一环输出当作期望的速度（以上）***************************/ 



//    //赋值了绝对角度absolute_angle，相对角度relative_angle，角速度motor_gyro
//		
//    GIMBAL_Feedback_Update(gimbal_init);//根据INS_Task传回来的数据对云台电机结构体的变量进行赋值

//    gimbal_init->gimbal_yaw_motor.absolute_angle_set = gimbal_init->gimbal_yaw_motor.absolute_angle;
//    gimbal_init->gimbal_yaw_motor.relative_angle_set = gimbal_init->gimbal_yaw_motor.relative_angle;
//    gimbal_init->gimbal_yaw_motor.motor_gyro_set = gimbal_init->gimbal_yaw_motor.motor_gyro;

//    gimbal_init->gimbal_pitch_motor.absolute_angle_set = gimbal_init->gimbal_pitch_motor.absolute_angle;
//    gimbal_init->gimbal_pitch_motor.relative_angle_set = gimbal_init->gimbal_pitch_motor.relative_angle;
//    gimbal_init->gimbal_pitch_motor.motor_gyro_set = gimbal_init->gimbal_pitch_motor.motor_gyro;
//   
//		gimbal_init->gimbal_yaw_motor.AUTO_flag=0;
//		gimbal_init->gimbal_pitch_motor.AUTO_flag=0;
//}

///*                                                 云台模式控制                                         */
///*
//云台行为状态机
//GIMBAL_ZERO_FORCE = 0, //云台无力
//GIMBAL_ABSOLUTE_ANGLE, //云台陀螺仪绝对角度控制
//GIMBAL_RELATIVE_ANGLE, //云台电机编码值相对角度控制
//电机状态机
//GIMBAL_MOTOR_RAW = 0, //电机原始值控制
//GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
//GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
//*/
//static void GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode)//云台行为状态机以及电机状态机设置
//{
//    if (gimbal_set_mode == NULL)
//    {
//        return;
//    }
//    gimbal_behaviour_mode_set(gimbal_set_mode);//调用gimbal_behaviour_mode_set 
//}


//static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update)//云台数据更新，在云台初始化中调用
//{
//    if (gimbal_feedback_update == NULL)
//    {
//        return;
//    }


////计算相对角度
//fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
//{  
//    int32_t relative_ecd = (ecd - offset_ecd);
//    if (relative_ecd > Half_ecd_range)//Half_ecd_range 4096 电机码盘值中间值
//    {
//        relative_ecd -= ecd_range;//ecd_range 8191 电机码盘值最大值
//    }
//    else if (relative_ecd < -Half_ecd_range)
//    {
//        relative_ecd += ecd_range;
//    }
//  
//    return relative_ecd * Motor_Ecd_to_Rad;//2*PI/8192 电机编码值转化成角度值
//     
//}

////云台状态切换保存，用于状态切换过渡
////根据云台电机的控制方式的变化保存数据
//static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change)
//{
//    if (gimbal_mode_change == NULL)
//    {
//        return;
//    }
//    //yaw电机状态机切换保存数据
//    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
//    {
//        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
//    }
//    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
//    {
//        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
//    }
//    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
//    {
//        gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
//    }
//    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

//    //pitch电机状态机切换保存数据
//    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
//    {
//        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
//    }
//    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
//    {
//        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
//    }
//    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
//    {
//        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
//    }

//    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
//}

///**
//  * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".         
//  * @param[out]     gimbal_set_control: "gimbal_control" valiable point
//  * @retval         none
//  */
///**
//  * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
//  * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
//  * @retval         none
//  */
////云台控制量设置


////云台控制量设置
//fp32 add_yaw_angle = 0.0f;//增加的期望角度
//fp32 add_pitch_angle = 0.0f;//增加的期望角度
//fp32  yaw_angle_set=0.0f;
//fp32 a=0;

//static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control)
//{
//    if (gimbal_set_control == NULL)
//    {
//        return;
//    }  
//      gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, gimbal_set_control);		
//		//yaw电机模式控制
//    if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
//    {
//        //raw模式下，直接发送控制值
//        gimbal_set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
//    }
//    else if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
//    {			 
//			
//			 if(gimbal_set_control->gimbal_yaw_motor.AUTO_flag==1)
//			 {//自瞄
//				 if(vision_Package.RX_data.yaw_angle_set1>180)
//				 {
//				 vision_Package.RX_data.yaw_angle_set1-=360;
//					 HAL_Delay(1);
//				 }
//				 yaw_angle_set =gimbal_set_control->gimbal_yaw_motor.absolute_angle_set;
//				  gimbal_set_control->gimbal_yaw_motor.absolute_angle_set=rad_format(yaw_angle_set+add_yaw_angle+((vision_Package.RX_data.yaw_angle_set1+1)*PI/180-gimbal_set_control->gimbal_yaw_motor.absolute_angle)*gimbal_set_control->gimbal_yaw_motor.data);		 
//  	 		 }
//         yaw_angle_set = gimbal_set_control->gimbal_yaw_motor.absolute_angle_set;
//					gimbal_set_control->gimbal_yaw_motor.absolute_angle_set = rad_format( yaw_angle_set + add_yaw_angle);
//			 //rad_format()这个函数作用是将期望的角度限制在（-PI，PI）中，因为反馈的实际角度是在（-PI,PI）里
//    }
//    else if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
//    {
//			  yaw_angle_set = gimbal_set_control->gimbal_yaw_motor.relative_angle_set;
//        gimbal_set_control->gimbal_yaw_motor.relative_angle_set = rad_format(yaw_angle_set + add_yaw_angle);		 
//		}
///*---------------------------------------------------------------------------------------------------*/
///*---------------------------------------------------------------------------------------------------*/
///*---------------------------------------------------------------------------------------------------*/
///*---------------------------------------------------------------------------------------------------*/
///*---------------------------------------------------------------------------------------------------*/		
//    //pitch电机模式控制
//    if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
//    {
//        //raw模式下，直接发送控制值
//        gimbal_set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
//    }
//    else if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
//    {
//        //gyro模式下，陀螺仪角度控制
//			  //pitch轴角度需要限幅
//        GIMBAL_absolute_angle_limit(&gimbal_set_control->gimbal_pitch_motor, add_pitch_angle);
//					if(gimbal_set_control->gimbal_pitch_motor.AUTO_flag==1)
//			   {		   
//						
//           GIMBAL_absolute_angle_limit(&gimbal_set_control->gimbal_pitch_motor, add_pitch_angle+((vision_Package.RX_data.pitch_angle_set1+0.6)/57.3-gimbal_set_control->gimbal_pitch_motor.absolute_angle)*(gimbal_set_control->gimbal_pitch_motor.data));	
//			   }	
//    }
//    else if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
//    {
//        //enconde模式下，电机编码角度控制
//			  //pitch轴角度需要限幅		 
//	
//          GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_pitch_motor, add_pitch_angle);
//			
//    }
//}

///**
//  * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control. 
//  * @param[out]     gimbal_motor: yaw motor or pitch motor
//  * @retval         none
//  */
///**
//  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
//  * @param[out]     gimbal_motor:yaw电机或者pitch电机
//  * @retval         none
//  */
//static void GIMBAL_absolute_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add)
//{
//    if (gimbal_motor == NULL)
//    {
//        return;
//    }
//    gimbal_motor->absolute_angle_set += add;
//    //是否超过最大 最小值
//    if (gimbal_motor->absolute_angle_set > gimbal_motor->max_absolute_angle)
//    {
//        gimbal_motor->absolute_angle_set = gimbal_motor->max_absolute_angle;
//    }
//    else if (gimbal_motor->absolute_angle_set < gimbal_motor->min_absolute_angle)
//    {
//        gimbal_motor->absolute_angle_set = gimbal_motor->min_absolute_angle;
//    }
//}

///**
//  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control. 
//  * @param[out]     gimbal_motor: yaw motor or pitch motor
//  * @retval         none
//  */
///**
//  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
//  * @param[out]     gimbal_motor:yaw电机或者pitch电机
//  * @retval         none
//  */
//static void GIMBAL_relative_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add)
//{
//    if (gimbal_motor == NULL)
//    {
//        return;
//    }
//    gimbal_motor->relative_angle_set = add+gimbal_motor->relative_angle_set;
//    //是否超过最大 最小值
//    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
//    {
//        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
//    }
//    else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
//    {
//        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
//    }
//}

///**
//  * @brief          control loop, according to control set-point, calculate motor current, 
//  *                 motor current will be sent to motor
//  * @param[out]     gimbal_control_loop: "gimbal_control" valiable point
//  * @retval         none
//  */
///**
//  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
//  * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
//  * @retval         none
//  */
////云台控制状态使用不同控制pid
//static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop)
//{
//    if (gimbal_control_loop == NULL)
//    {
//        return;
//    }
//    //yaw不同模式对于不同的控制函数
//    if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
//    {
//        //raw控制
//        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
//    }
//    else if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
//    {
//        //gyro角度控制
//        gimbal_motor_absolute_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
//    }
//    else if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
//    {
//        gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
//    }
//    //pitch不同模式对于不同的控制函数
//    if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
//    {
//        //raw控制
//        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
//    }
//    else if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
//    {
//        //gyro角度控制
//        gimbal_motor_absolute_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
//    }
//    else if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
//    {
//        //enconde角度控制
//        gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
//    }
//}

///**
//  * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control. 
//  * @param[out]     gimbal_motor: yaw motor or pitch motor
//  * @retval         none
//  */
///**
//  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
//  * @param[out]     gimbal_motor:yaw电机或者pitch电机
//  * @retval         none
//  */
//static void gimbal_motor_absolute_angle_control(Gimbal_Motor_t *gimbal_motor)
//{
//    if (gimbal_motor == NULL)
//    {
//        return;
//    }
//    //角度环，速度环串级pid调试
//    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, 
//		                                                                 gimbal_motor->absolute_angle,
//		                                                             gimbal_motor->absolute_angle_set, 
//	                                                                      	gimbal_motor->motor_gyro);
//		
//    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
//    //控制值赋值
//    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
//}


///**
//  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control. 
//  * @param[out]     gimbal_motor: yaw motor or pitch motor
//  * @retval         none
//  */
///**
//  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
//  * @param[out]     gimbal_motor:yaw电机或者pitch电机
//  * @retval         none
//  */
//static void gimbal_motor_relative_angle_control(Gimbal_Motor_t *gimbal_motor)
//{
//    if (gimbal_motor == NULL)
//    {
//        return;
//    }

//    //角度环，速度环串级pid调试，relative_angle和absolute_angle和motor_gyro在GIMBAL_Feedback_Update（）更新
//    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_relative_angle_pid, 
//		                                                                 gimbal_motor->relative_angle, 
//		                                                             gimbal_motor->relative_angle_set, 
//		                                                                     gimbal_motor->motor_gyro);
//		
//		                                                                   
//    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_relative_pid, 
//		                                      gimbal_motor->motor_gyro, 
//		                                      gimbal_motor->motor_gyro_set);
//    //控制值赋值
//    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
//}
///**
//  * @brief          gimbal control mode :GIMBAL_MOTOR_RAW, current  is sent to CAN bus. 
//  * @param[out]     gimbal_motor: yaw motor or pitch motor
//  * @retval         none
//  */
///**
//  * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
//  * @param[out]     gimbal_motor:yaw电机或者pitch电机
//  * @retval         none
//  */
//static void gimbal_motor_raw_angle_control(Gimbal_Motor_t *gimbal_motor)
//{
//    if (gimbal_motor == NULL)
//    {
//        return;
//    }
//    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;//在GIMBAL_Set_Contorl（）中，直接 gimbal_set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
//    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
//}



//static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
//{
//    if (pid == NULL)
//    {
//        return;
//    }
//    pid->kp = kp;
//    pid->ki = ki;
//    pid->kd = kd;

//    pid->err = 0.0f;
//    pid->get = 0.0f;

//    pid->max_iout = max_iout;
//    pid->max_out = maxout;
//}
//static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
//{
//    fp32 err;
//    if (pid == NULL)
//    {
//        return 0.0f;
//    }
//    pid->get = get;
//    pid->set = set;

//    err = set - get;
//    pid->err = rad_format(err);
//    pid->Pout = pid->kp * pid->err;
//    pid->Iout += pid->ki * pid->err;
//    pid->Dout = pid->kd * error_delta;
//    abs_limit(&pid->Iout, pid->max_iout);
//    pid->out = pid->Pout + pid->Iout + pid->Dout;
//    abs_limit(&pid->out, pid->max_out);
//    return pid->out;
//}

////pid数据清理
//static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear)
//{
//    if (gimbal_pid_clear == NULL)
//    {
//        return;
//    }
//    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
//    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
//}
