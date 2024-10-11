#include "lifting.h"
 
fp32 pid_dif1_calc(pids *pid, fp32 ref, fp32 set)
{
	pidINIT(pid,PID_POSITION,2,0.1,0,10000,1000);
	PID_calc(pid, ref, set);
	return pid->out;
}
fp32 pid_dif2_calc(pids *pid, fp32 ref, fp32 set)
{
	pidINIT(pid,PID_POSITION,5,1,0,10000,5000);
	PID_calc(pid, ref, set);
	return pid->out;
}
void Lifting_dual_motor_pid(pids *motor_dual_pid,int16_t pid_dual_out[],motor_measure_t *motor1_dual,motor_measure_t *motor2_dual)
{
	fp32 deviation[2];
	fp32 set_angle;
	fp32 tar_angle[2];
	fp32 tar_speed[2];
	fp32 tar_current[2];
	fp32 tar[2];

  #define angle_ratio 0.01*8191/660.0	

//	pidINIT(motor_dual_pid,PID_DELTA,2,0.1,0,10000,1000);
	
 	set_angle = rc_ctrl.rc.ch[2]*angle_ratio;////左摇杆横向

	tar_angle[0] = motor1_dual->real_angle +set_angle;
	tar_angle[1] = motor2_dual->real_angle +set_angle;

	tar_speed[0]= pid_dif1_calc(motor_dual_pid,motor1_dual->real_angle,tar_angle[0]);//增量式,得到值应该不同???
	tar_speed[1]= pid_dif2_calc(motor_dual_pid,motor2_dual->real_angle,tar_angle[1]);
	deviation[0] = 0.25f*(tar_speed[0] - tar_speed[1]);
	
	tar_current[0] = pid_dif1_calc(motor_dual_pid,motor1_dual->speed_rpm,tar_speed[0])-deviation[0];
  tar_current[1] = pid_dif2_calc(motor_dual_pid,motor2_dual->speed_rpm,tar_speed[1])-deviation[0];////deffirent
	//deviation[1] = 0.25f*(tar_current[0] - tar_current[1]);
	
	tar[0] = pid_dif1_calc(motor_dual_pid,motor1_dual->real_current,tar_current[0]);//-deviation[1];
  tar[1] = pid_dif2_calc(motor_dual_pid,motor2_dual->real_current,tar_current[1]);//+deviation[1];//
	
	pid_dual_out[0] =(uint16_t)tar[0]	;
	pid_dual_out[1] =(uint16_t)tar[1]	;
}
