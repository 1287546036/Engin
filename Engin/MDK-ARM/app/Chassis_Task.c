#include "Chassis_Task.h"

#include "remote_control.h"
//#include "cmsis_os.h"
//#include "task.h"


int16_t chassis_motor_current[4];
extern motor_measure_t motor_chassis[4];
pids chassis_motor_pid;
#define speed_transform 10000/660.0

void chassis_mode_execute(chassis_speed_t *chassis_speed)//app\Chassis_Task.c(12): warning:  #550-D: parameter "speedx"  was set but never used????????
{
//	if(control_mode.mode == chassis_enable){//使能
		if(rc_ctrl.rc.s[0] == 2){
		chassis_deadline_limit(-rc_ctrl.rc.ch[1]*speed_transform,chassis_speed->set_speedx,CHASSIS_RC_DEADLINE);//负值,通道值指向下方
		chassis_deadline_limit( rc_ctrl.rc.ch[0]*speed_transform,chassis_speed->set_speedy,CHASSIS_RC_DEADLINE);
		chassis_deadline_limit( rc_ctrl.rc.ch[2]*speed_transform/2,chassis_speed->set_speedw,CHASSIS_RC_DEADLINE);//可以转半个值,云台转一个值,其余模式只有云台转
		}
	else// 其余速度固定为零,有力停止
		{
		chassis_speed->set_speedx = 0;
		chassis_speed->set_speedy = 0;
		chassis_speed->set_speedw = 0;
		}
}
/*
						Chassis_SolutionForward                       

							1 //     \\ 0
							 //       \\  

							 \\       //
							2 \\     // 3

*/
void Chassis_SolutionForward(chassis_speed_t *chassis_speed)
{
// static float rotate_ratio_f = ((WHEELBASE+WHEELTRACK)/2.0f - GIMBAL_OFFSET)/RADIAN_COEF;
// static float rotate_ratio_b = ((WHEELBASE+WHEELTRACK)/2.0f + GIMBAL_OFFSET)/RADIAN_COEF;
// static float wheel_rpm_ratio = 60.0f/(PERIMETER*CHASSIS_DECELE_RATIO);
 static float rotate_ratio = (0.36846+0.452)/2.0f/57.3;
 static float wheel_rpm_ratio = 60.0f/(0.64*1/19);
	fp32 vx = (fp32)chassis_speed->set_speedx;
	fp32 vy = (fp32)chassis_speed->set_speedy;
	fp32 vw = (fp32)chassis_speed->set_speedw;
 chassis_speed->set_speed[0] = (+vx - vy + vw * rotate_ratio) * wheel_rpm_ratio;
 chassis_speed->set_speed[1] = (+vx + vy + vw * rotate_ratio) * wheel_rpm_ratio;
 chassis_speed->set_speed[2] = (-vx + vy + vw * rotate_ratio) * wheel_rpm_ratio;
 chassis_speed->set_speed[3] = (-vx - vy + vw * rotate_ratio) * wheel_rpm_ratio;
}

 
void chassis_pid(int16_t chassis_motor_current[],chassis_speed_t *chassis_speed)
{			pidINIT(&chassis_motor_pid,PID_POSITION,3,0.1,0,160000,1000);
	chassis_motor_current[0] = PID_calc(&chassis_motor_pid,motor_chassis[0].speed_rpm,chassis_speed->set_speed[0]);
	chassis_motor_current[1] = PID_calc(&chassis_motor_pid,motor_chassis[1].speed_rpm,chassis_speed->set_speed[1]);
	chassis_motor_current[2] = PID_calc(&chassis_motor_pid,motor_chassis[2].speed_rpm,chassis_speed->set_speed[2]);
	chassis_motor_current[3] = PID_calc(&chassis_motor_pid,motor_chassis[3].speed_rpm,chassis_speed->set_speed[3]);
}

void chassis_task(void)
{
//////	while(1)
//////	{	
////if(control_mode.mode !=all_disability)
////	{
	chassis_mode_execute(&chassis_speed);
	Chassis_SolutionForward(&chassis_speed);//
	chassis_pid(chassis_motor_current,&chassis_speed);
	CAN_cmd_chassis(chassis_motor_current);
////	}

//////		vTaskDelay(CHASSIS_CONTROL_TIME_MS);//已经注释掉了发送的delay,发送500hz,接收1000hz
//////	}
}

