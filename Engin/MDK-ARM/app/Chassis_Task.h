#include "can.h"
#include "main.h"
#include "struct_typedef.h"
#include "pid.h"
#include "motor.h"
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

typedef struct
{
	short set_speedx;
	short set_speedy;
	short set_speedw;
	fp32 set_speed[4];
}chassis_speed_t;

static chassis_speed_t chassis_speed;
extern void chassis_task(void);
void chassis_mode_execute(chassis_speed_t *chassis_speed);
void Chassis_SolutionForward(chassis_speed_t *chassis_speed);
void chassis_pid(int16_t chassis_motor_current[],chassis_speed_t *chassis_speed);


 //任务开始空闲一段时间
 #define CHASSIS_TASK_INIT_TIME 1000 




 #define CHASSIS_ACCEL_X_NUM 0.1666666667f//??
 #define CHASSIS_ACCEL_Y_NUM 0.3333333333f

 #define CHASSIS_RC_DEADLINE 10         

 #define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
 #define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
 #define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

 #define MOTOR_DISTANCE_TO_CENTER 0.2f//??

 //底盘任务控制间隔 2ms
 #define CHASSIS_CONTROL_TIME_MS 2
// //底盘任务控制间隔 0.002s
// #define CHASSIS_CONTROL_TIME 0.002
// //底盘任务控制频率，注意这里的频率等于1/CHASSIS_CONTROL_TIME_MS(2ms)
// #define CHASSIS_CONTROL_FREQUENCE 500.0f
 //底盘3508最大can发送电流值
 #define MAX_MOTOR_CAN_CURRENT 16000.0f

 //底盘前后左右控制按键
 #define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
 #define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
 #define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
 #define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D
// //小陀螺按键
// #define REVOLVE_START_KEY  KEY_PRESSED_OFFSET_SHIFT
// #define REVOLVE_STOP_KEY    KEY_PRESSED_OFFSET_CTRL

 //3508电机rmp 变化成 旋转速度的比例，转换后单位是(m/s),(我自己算的是(2PI/(60秒*(3591/187)(这是减速比，约为19：1)))*(0.1525m/2(麦轮半径))=0.000415...),我的PI取了3.14
 //这是直径为0.1525m的麦轮的线速度比例
 #define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
 #define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

 //底盘电机最速度
 #define MAX_WHEEL_SPEED 4.0f
 //底盘运动过程最大前进速度
 #define NORMAL_MAX_CHASSIS_SPEED_X 3.0f
 //底盘运动过程最大平移速度
 #define NORMAL_MAX_CHASSIS_SPEED_Y 2.9f
 //底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿//???????????????
 #define CHASSIS_WZ_SET_SCALE 0.0f


 //底盘电机速度环PID
 #define M3505_MOTOR_SPEED_PID_KP 20000.0f
 #define M3505_MOTOR_SPEED_PID_KI 3000.0f                                                                                                                              
 #define M3505_MOTOR_SPEED_PID_KD 0.0f
 #define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT//16000
 #define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
 // //底盘旋转跟随PID
 // #define CHASSIS_FOLLOW_GIMBAL_PID_KP 12.0f
 // #define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
 // #define CHASSIS_FOLLOW_GIMBAL_PID_KD 15.0f
 // #define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
 // #define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

// #define power_buffer_PID_KP  5.0f
// #define power_buffer_PID_KI  0.0f
// #define power_buffer_PID_KD  0.0f
// #define power_buffer_PID_MAX_OUT 15.0f
// #define power_buffer_PID_MAX_IOUT 0.2f

 typedef enum
 {
   CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW=0,//chassis will follow yaw gimbal motor relative angle.底盘会跟随云台相对角度
   CHASSIS_VECTOR_NO_FOLLOW_YAW=1, //chassis will have rotation speed control. 底盘有旋转速度控制
   CHASSIS_VECTOR_RAW=2, //control-current will be sent to CAN bus derectly.

 } chassis_mode_e;

 // typedef enum
 // {
 // 	EXPECTED_FORWARD=1,
 // 	EXPECTED_LEFT=2,
 //   EXPECTED_BACKWARD=3,
 // 	EXPECTED_RIGHT=4,
	
 // }expected_coordinate_system;//期望的底盘坐标系，不同坐标系下，同一位置云台与底盘的相对角度会不同

 typedef struct
 {
   const motor_measure_t *chassis_motor_measure;//电机反馈的原始数据
   fp32 accel;//加速度
   fp32 speed;//电机实际速度 转速乘比例
   fp32 speed_set;//电机期望速度 底盘速度解算得来
   int16_t give_current;//电机电流期望 PID输出得来
 } Chassis_Motor_t;


 typedef struct
 {
//   const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针
//  
// 	const Gimbal_Motor_t *chassis_yaw_motor;   //底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角
// 	
//  
// 	chassis_mode_e chassis_mode;               //底盘控制状态机
//  chassis_mode_e last_chassis_mode;          //底盘上次控制状态机
//  
// 	Chassis_Motor_t motor_chassis[4];          //底盘电机数据
//  
// 	PidTypeDef motor_speed_pid[4];             //底盘电机速度pid
//   PidTypeDef chassis_angle_pid;              //底盘跟随角度pid
//   PidTypeDef buffer_pid;
//	
//   first_order_filter_type_t chassis_cmd_slow_set_vx; //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
//   first_order_filter_type_t chassis_cmd_slow_set_vy;//低通，use first order filter to slow set-point.使用一阶低通滤波减缓设定值


   fp32 vx;                         //底盘实际速度 前进方向 前为正，单位 m/s
   fp32 vy;                         //底盘实际速度 左右方向 左为正  单位 m/s
   fp32 wz;                         //底盘实际旋转角速度，逆时针为正 单位 rad/s
  
 	fp32 vx_set;                     //底盘设定速度 前进方向 前为正，单位 m/s
   fp32 vy_set;                     //底盘设定速度 左右方向 左为正，单位 m/s
   fp32 wz_set;                     //底盘设定旋转角速度，逆时针为正 单位 rad/s
  
 	fp32 chassis_relative_angle;     //底盘与云台的相对角度，单位 rad/s
   fp32 chassis_relative_angle_set; //设置底盘相对云台控制角度
   fp32 chassis_yaw_set;

   fp32 vx_max_speed;  //前进方向最大速度 单位m/s
   fp32 vx_min_speed;  //前进方向最小速度 单位m/s
   fp32 vy_max_speed;  //左右方向最大速度 单位m/s
   fp32 vy_min_speed;  //左右方向最小速度 单位m/s
 } chassis_move_t;


#define chassis_deadline_limit(input, output, dealine)        \
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







#endif
