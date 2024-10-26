#include "Chassis_Task.h"

#include "remote_control.h"



//////////////////////////////////////////////////////////////////////////////

//////                   Chassis_SolutionForward                       ///////

//////////////////////////////////////////////////////////////////////////////
/*
//        1 //     \\ 0
//         //       \\  
//
//         \\       //
//        2 \\     // 3
//
*/

void Chassis_SolutionForward(fp32 wheel_rpm[],short x,short y,short w)
{
// static float rotate_ratio_f = ((WHEELBASE+WHEELTRACK)/2.0f - GIMBAL_OFFSET)/RADIAN_COEF;
// static float rotate_ratio_b = ((WHEELBASE+WHEELTRACK)/2.0f + GIMBAL_OFFSET)/RADIAN_COEF;
// static float wheel_rpm_ratio = 60.0f/(PERIMETER*CHASSIS_DECELE_RATIO);
 static float rotate_ratio = 1;//(0.36846+0.452)/2.0f/57.3;
 static float wheel_rpm_ratio =1;// 60.0f/(0.64*1/19);
	fp32 vx = (fp32)x;
	fp32 vy = (fp32)y;
	fp32 vw = (fp32)w;
 wheel_rpm[0] = (+vx - vy + vw * rotate_ratio) * wheel_rpm_ratio;
 wheel_rpm[1] = (+vx + vy + vw * rotate_ratio) * wheel_rpm_ratio;
 wheel_rpm[2] = (-vx + vy + vw * rotate_ratio) * wheel_rpm_ratio;
 wheel_rpm[3] = (-vx - vy + vw * rotate_ratio) * wheel_rpm_ratio;

}

#define unit_speed 10000/660.0 

/**
底盘状态:使能,失能,有力停止
*/
void chassis_behaviour_mode(int16_t chassis_motor_current[],fp32 set_speed[])
{

typedef enum
{
     switch_right_up=1,
    switch_right_down=2,
   switch_right_mid=3
}switch_right;
	
	
	
	switch(rc_ctrl.rc.s[0])
    {case switch_right_up:
		{
            chassis_motor_current[0] = PID_calc(&motor_pid,motor_chassis[0].speed_rpm,set_speed[0]);
            chassis_motor_current[1] = PID_calc(&motor_pid,motor_chassis[1].speed_rpm,set_speed[1]);
            chassis_motor_current[2] = PID_calc(&motor_pid,motor_chassis[2].speed_rpm,set_speed[2]);
            chassis_motor_current[3] = PID_calc(&motor_pid,motor_chassis[3].speed_rpm,set_speed[3]);
			break;
		}
	case switch_right_down:
        {
            chassis_motor_current[0] = PID_calc(&motor_pid,motor_chassis[0].real_angle,motor_chassis[0].real_angle);//如何保持停止??
            chassis_motor_current[1] = PID_calc(&motor_pid,motor_chassis[1].real_angle,motor_chassis[1].real_angle);
            chassis_motor_current[2] = PID_calc(&motor_pid,motor_chassis[2].real_angle,motor_chassis[2].real_angle);
            chassis_motor_current[3] = PID_calc(&motor_pid,motor_chassis[3].real_angle,motor_chassis[3].real_angle);
			break;
        }
    case switch_right_mid:
        {
			chassis_motor_current=NULL;
			break;
		}
	default:
        {
            break;
        }
	}
}

void chassis_task(void)
{
	while(1)
	{
		//加入底盘有力停止,因为角度锁死不被干扰必须陀螺仪
		pidINIT(&motor_pid,PID_POSITION,2,0.1,0,8000,100);
		
		short set_speedx =-rc_ctrl.rc.ch[1];
		short set_speedy = rc_ctrl.rc.ch[0];
		short set_speedw = 0;//rc_ctrl->rc.ch[2];
		fp32 set_speed[4];
		int16_t chassis_motor_current[4];


	    Chassis_SolutionForward(set_speed,set_speedx,set_speedy,set_speedw);
		chassis_behaviour_mode(chassis_motor_current,set_speed);
		CAN_cmd_chassis(chassis_motor_current);

 	
	}
}