#include "Lifting_Task.h"
 
 
void Lifting_Behaviour_mode(fp32 set_lifting_angle,fp32 set_protract_angle)
{
	//控制中+有力停止,全部收回//始终保持锁死状态
	//防堵转,同步,角度锁死,收回位置,
	#define angle_ratio 0.01*8191/660.0	

//	pidINIT(motor_dual_pid,PID_DELTA,2,0.1,0,10000,1000);
	
 	set_lifting_angle = rc_ctrl.rc.ch[2]*angle_ratio*0.01;////左摇杆横向
	set_protract_angle = rc_ctrl.rc.ch[2]*angle_ratio*0.01;////左摇杆横向
	typedef enum
	{
	   switch_left_up=1,
	   switch_left_down=2,
	   switch_left_mid=3
	}switch_right;
	switch(rc_ctrl.rc.s[1])
    {case switch_left_up:
		{//锁死
          set_lifting_angle=NULL;
		  set_protract_angle=NULL;
			break;
		}
//	case switch_left_down:
//        {//气泵
//           
//			break;
//        }
    case switch_left_mid:
        {//控制
		    set_lifting_angle=rc_ctrl.rc.ch[2];
			set_protract_angle=rc_ctrl.rc.ch[3];
			break;
		}
	default:
        {
            break;
        }
	}
	
}


fp32 pid_angle_calc(pids *pid, fp32 ref, fp32 set)
{
	pidINIT(pid,PID_DELTA,3,0,0,4000,100);
	PID_calc(pid, ref, set);
	return pid->out;
}
fp32 pid_speed_calc(pids *pid, fp32 ref, fp32 set)
{
	pidINIT(pid,PID_DELTA,3,0,0,10000,100);
	PID_calc(pid, ref, set);
	return pid->out;
}
fp32 pid_current_calc(pids *pid, fp32 ref, fp32 set)
{
	pidINIT(pid,PID_DELTA,3,0,0,16000,1000);
	PID_calc(pid, ref, set);
	return pid->out;
}

void Lifting_dual_motor_pid(fp32 set_lifting_angle,fp32 set_protract_angle,pids *motor_dual_pid,int16_t pid_dual_out[],motor_measure_t *motor1_dual,motor_measure_t *motor2_dual)
{
	fp32 deviation[4];
	fp32 set_angle;
	
	fp32 set_speed;
	fp32 tar_angle[2];
	fp32 tar_speed[2];
	fp32 tar_current[2];
	fp32 tar[2];

  

	tar_angle[0] = motor1_dual->real_angle +set_angle;
	tar_angle[1] = motor1_dual->real_angle +set_angle;
	
	deviation[0]=motor1_dual->real_angle - motor2_dual->real_angle;//写好锁死
	deviation[1]=deviation[0];
	deviation[2]=deviation[0]-deviation[1];

	tar_speed[0]= pid_angle_calc(motor_dual_pid,motor1_dual->real_angle,tar_angle[0] - 0.5f*deviation[2]);//增量式,得到值应该不同???
	tar_speed[1]= pid_angle_calc(motor_dual_pid,motor1_dual->real_angle,tar_angle[1] + 0.5f*deviation[2]);//增量式,得到值应该不同???

	tar_current[0] = pid_speed_calc(motor_dual_pid,motor1_dual->speed_rpm,tar_speed[0]);
    tar_current[1] = pid_speed_calc(motor_dual_pid,motor2_dual->speed_rpm,tar_speed[1]);
	
	//tar_current[0] += pid_current_calc(motor_dual_pid,motor1_dual->real_current,tar_current[0]);
    //tar_current[1] += pid_current_calc(motor_dual_pid,motor2_dual->real_current,tar_current[1]);

	pid_dual_out[0] =(uint16_t)tar_current[0];
	pid_dual_out[1] =(uint16_t)tar_current[1];

}


void lifitng_task(void)
{
	while(1)
	{
///////////////////////lifting  debug///////////////////////////////////////////////
//set_speed[4] =500;
//set_speed[5] =500;
//pid_lifting[0] = PID_calc(&motor_pid,motor_lifting[0].real_current,set_speed[4]);
//pid_lifting[1] = PID_calc(&motor_pid,motor_lifting[1].real_current,set_speed[5]);
		
   fp32 set_lifting_angle,set_protract_angle;
		
   Lifting_Behaviour_mode(set_lifting_angle,set_protract_angle);
   Lifting_dual_motor_pid(set_lifting_angle,set_protract_angle,&motor_pid,pid_lifting,&motor_lifting[1],&motor_lifting[2]);
   CAN_cmd_lifting(pid_lifting);
	}
}
