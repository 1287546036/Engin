#include "Lifting_Task.h"
 

int16_t pid_lifting[4];
pids lifting_motor_pid;
fp32 rc_lifting_angle,rc_protract_angle;
fp32 set_lifting_angle,set_protract_angle;
fp32 real_angle_keep;


	//控制中+有力停止,全部收回//始终保持锁死状态//双电机检测,防止单个转动卡死
	//防堵转,同步,角度锁死,收回位置,//需要位置判断,不能直接掉下来
    //如果一个掉了,另一个作何反应?




fp32 pid_angle_calc(pids *pid, fp32 ref, fp32 set)//后将pidData放于一个文件
{
	pidINIT(pid,PID_DELTA ,lifting_angle_KP     
						  ,lifting_angle_KI     
						  ,lifting_angle_KD     
						  ,lifting_angle_MAXout 
						  ,lifting_angle_MAXIout);//长度共19cm,转动距离是一半,转轴直径4.1cm,姑且5秒一圈,1:51,每秒最大10转,10*8000
	PID_calc(pid, ref, set);
	return pid->out;
}
fp32 pid_speed_calc(pids *pid, fp32 ref, fp32 set)
{
	pidINIT(pid,PID_DELTA,lifting_speed_KP     
						 ,lifting_speed_KI     
						 ,lifting_speed_KD     
						 ,lifting_speed_MAXout 
						 ,lifting_speed_MAXIout);
	PID_calc(pid, ref, set);
	return pid->out;
}
fp32 pid_deviation_angle_calc(pids *pid, fp32 ref, fp32 set)
{
	pidINIT(pid,PID_DELTA,0,0,1,1000,0);
	PID_calc(pid, ref, set);
	return pid->out;
}
fp32 pid_deviation_speed_calc(pids *pid, fp32 ref, fp32 set)
{
	pidINIT(pid,PID_DELTA,0,0,1,100,0);
	PID_calc(pid, ref, set);
	return pid->out;
}

void Lifting_dual_motor_pid(fp32 set_angle,pids *motor_dual_pid,int16_t pid_dual_out[],motor_measure_t *motor1_dual,motor_measure_t *motor2_dual)
{

	fp32 tar_angle[2];
	fp32 tar_speed[2];
	fp32 tar_current[2];

	fp32 bloking_time1;
	fp32 bloking_time2;
	
	fp32 kp=0.3f;

	set_angle=100;
	
	if(set_angle==0)
	{
		tar_speed[0]=0;
		tar_speed[1]=0;
	}
	else
	{
		tar_angle[0] = motor1_dual->real_angle +set_angle;
		tar_angle[1] = motor2_dual->real_angle +set_angle;
		
//		deviation_a[1]=deviation_a[0];	
//		deviation_a[0]=motor1_dual->real_angle - motor2_dual->real_angle;//写好锁死		
//		deviation_a[2]=deviation_a[0]-deviation_a[1];
//		if(deviation_a[2]>8000)
//			k++;
//		if(deviation_a[2]<-8000)
//			k--;
//		deviation_a[2]+=k*8000;
//		
		    fp32 temp;
    if (motor1_dual->real_angle > motor2_dual->real_angle) {  // 得到编码值
      temp = motor1_dual->real_angle - motor2_dual->real_angle;
    }
    else {
      temp = motor1_dual->real_angle - motor2_dual->real_angle + 8191;
    }
    // 过零处理
    if (0 - temp > 4096) {
      temp = temp + (4096 * 2);
    }
    else if (0 - temp <= -4096) {
      temp = temp - (4096 * 2);
    }

		
		
		
		
		tar_speed[0]= pid_angle_calc(motor_dual_pid,motor1_dual->real_angle,tar_angle[0] - kp*temp);//deviation_a[2]);
		tar_speed[1]= pid_angle_calc(motor_dual_pid,motor2_dual->real_angle,tar_angle[1] + kp*temp);//deviation_a[2]);
		
//		tar_speed[0]= pid_angle_calc(motor_dual_pid,motor1_dual->real_angle, - 1.0f*deviation_a[2]);
//		tar_speed[1]= pid_angle_calc(motor_dual_pid,motor1_dual->real_angle,   1.0f*deviation_a[2]);
//		
//		deviation_s[1]=deviation_s[0];	
//		deviation_s[0]=motor1_dual->speed_rpm - motor2_dual->speed_rpm;//写好锁死		
//		deviation_s[2]=deviation_s[0]-deviation_s[1];
//		
		
		/*      防堵转       */
		if(motor1_dual->speed_rpm < tar_speed[0]/2)
			bloking_time1++;
		else
			bloking_time1-=0.01f;
		if(bloking_time1>100)
		{
			tar_speed[0]=-tar_speed[0];
		}
		
		if(motor2_dual->speed_rpm< tar_speed[1]/2)//////////////////////////////////////////////////////////////
			bloking_time2++;
		else
			bloking_time2-=0.01f;
		if(bloking_time2>100)
		{
			tar_speed[1]=-tar_speed[1];
		}

		
	}

	
	tar_current[0] = pid_speed_calc(motor_dual_pid,motor1_dual->speed_rpm,tar_speed[0]);
    tar_current[1] = pid_speed_calc(motor_dual_pid,motor2_dual->speed_rpm,tar_speed[1]);

//	tar_current[0] = pid_speed_calc(motor_dual_pid,motor1_dual->speed_rpm,- 1.0f*deviation_s[2]);
//    tar_current[1] = pid_speed_calc(motor_dual_pid,motor2_dual->speed_rpm,  1.0f*deviation_s[2]);
//	
	
	pid_dual_out[0] =(uint16_t)tar_current[0];
	pid_dual_out[1] =(uint16_t)tar_current[1];

}
	void lifting_mode_execute(fp32 rc_lifting,fp32 rc_protract)
{
	switch(control_mode.mode)
		{
			case lifting_enable:
			{//使能
			rc_lifting  = -rc_ctrl.rc.ch[0]*0.1;//负值,通道值指向下方
			rc_protract =  rc_ctrl.rc.ch[1]*0.1;
				break;
			}
			default:
			{//其余角度环有力停止
				rc_lifting = 0;
				rc_protract = 0;
				break;
			}
		}
}
void lifitng_task(void)
{
	
		#define angle_ratio 0.01*8191/660.0	
		lifting_mode_execute(rc_lifting_angle,rc_protract_angle);
		rc_deadline_limit(rc_lifting_angle,set_lifting_angle,10);
		rc_deadline_limit(rc_protract_angle,set_protract_angle,10);


		Lifting_dual_motor_pid(set_lifting_angle,&lifting_motor_pid,pid_lifting,&motor_lifting[0],&motor_lifting[1]);
//		Lifting_dual_motor_pid(set_protract_angle,&lifting_motor_pid,pid_lifting,&motor_lifting[2],&motor_lifting[3]);
		
		CAN_cmd_lifting(pid_lifting);
}
