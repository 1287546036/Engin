#include "Lifting_Task.h"
 
// #include "cmsis_os.h"
//#include "task.h"

extern void lifitng_task(void);

fp32 rc_lifting_angle,rc_protract_angle;
fp32 set_lifting_angle,set_protract_angle;
fp32 real_angle_keep;


int16_t pid_lifting[4];
extern motor_measure_t motor_lifting[4];

 pids lifting_motor_pid;


	//控制中+有力停止,全部收回//始终保持锁死状态//双电机检测,防止单个转动卡死
	//防堵转,同步,角度锁死,收回位置,//需要位置判断,不能直接掉下来
    //如果一个掉了,另一个作何反应?

/*debug*/



//// 定义电机参数  
//#define JA 1.0  // 电机A的转动惯量  
//#define JB 1.0  // 电机B的转动惯量  
//#define KA 1.0  // 电机A的控制增益  
//#define KB 1.0  // 电机B的控制增益  
//  
//// PID控制器参数  
//#define KP 1.0  // 比例系数  
//#define KI 0.1  // 积分系数  
//#define KD 0.01 // 微分系数  
//  
//// 模拟时间步长  
//#define TIME_STEP 0.01  
//  
//// 电机状态结构体  
//typedef struct {  
//    double theta;    // 旋转角度  real_angle
//    double omega;    // 角速度  
// //   double torque;   // 干扰力矩  
//    double control;  // 控制输入  
//    double error;    // 误差  
//    double error_int; // 误差积分  
//    double prev_error; // 上一次误差  
//} Motor;  
//  
//// PID控制器函数  
//double pid_control(double setpoint, double measured, Motor *motor, double dt) {  
//    motor->error = setpoint - measured;  
//    motor->error_int += motor->error * dt;  
//    double derivative = (motor->error - motor->prev_error) / dt;  
//    motor->prev_error = motor->error;  
//    return KP * motor->error + KI * motor->error_int + KD * derivative;  
//}  
//  
//// 双电机同步控制函数  
//void dual_motor_control(motor_measure_t *motor1_dual,motor_measure_t *motor2_dual,Motor *motorA, Motor *motorB, double target_angle, double dt) {  

//    // 计算耦合误差  
//    double eA = target_angle - motorA->theta - KA * (motorA->theta - motorB->theta);  
//    double eB = target_angle - motorB->theta + KB * (motorA->theta - motorB->theta);  
//  
//    // 应用PID控制  
//    motorA->control = pid_control(eA, motorA->theta, motorA, dt);  
//    motorB->control = pid_control(eB, motorB->theta, motorB, dt);  
//  
////    // 更新电机状态（这里简化了物理模型，仅用于演示） //?? 
////    motorA->omega += (KA * motorA->control - motorA->torque) / JA * dt;  
////    motorB->omega += (KB * motorB->control - motorB->torque) / JB * dt;  
////    motorA->theta += motorA->omega * dt;  
////    motorB->theta += motorB->omega * dt;  
//	
//	//写一个角度的累加计算,相当于模型,将计算值作为目标值,
//}  




fp32 pid_angle_calc(pids *pid, fp32 ref, fp32 set)//后将pidData放于一个文件
{
	pidINIT(pid,PID_DELTA,4,0.1,0.01,3500,500);//长度共19cm,转动距离是一半,转轴直径4.1cm,姑且5秒一圈,1:51,每秒最大10转,10*8000
	PID_calc(pid, ref, set);
	return pid->out;
}
fp32 pid_speed_calc(pids *pid, fp32 ref, fp32 set)
{
	pidINIT(pid,PID_DELTA,3,0.1,0,16000,1000);
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
//fp32 pid_current_calc(pids *pid, fp32 ref, fp32 set)
//{
//	pidINIT(pid,PID_DELTA,3,0,0,16000,1000);
//	PID_calc(pid, ref, set);
//	return pid->out;
//}

void Lifting_dual_motor_pid(fp32 set_angle,pids *motor_dual_pid,int16_t pid_dual_out[],motor_measure_t *motor1_dual,motor_measure_t *motor2_dual)
{
	fp32 deviation_a[3];//????
	fp32 deviation_s[3];//????
	
	fp32 tar_angle[2];
	fp32 tar_speed[2];
	fp32 tar_current[2];

	fp32 bloking_time1;
	fp32 bloking_time2;
	
	fp32 kp=0.3f;
	uint8_t k=0;//圈数
	
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
//	while(1)
//	{
//		
		#define angle_ratio 0.01*8191/660.0	
		lifting_mode_execute(rc_lifting_angle,rc_protract_angle);
		rc_deadline_limit(rc_lifting_angle,set_lifting_angle,10);
		rc_deadline_limit(rc_protract_angle,set_protract_angle,10);


		Lifting_dual_motor_pid(set_lifting_angle,&lifting_motor_pid,pid_lifting,&motor_lifting[0],&motor_lifting[1]);
//		Lifting_dual_motor_pid(set_protract_angle,&lifting_motor_pid,pid_lifting,&motor_lifting[2],&motor_lifting[3]);
		
		CAN_cmd_lifting(pid_lifting);
//		vTaskDelay(2);
//	}
}



  /*
int main() {  
    // 初始化电机状态  
    Motor motorA = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  
    Motor motorB = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  
  
    // 目标角度  
    double target_angle = 1.0;  
  
    // 模拟时间  
    double time = 0.0;  
    while (time < 10.0) { // 模拟10秒  
        dual_motor_control(&motorA, &motorB, target_angle, TIME_STEP);  
        time += TIME_STEP;  
  
        // 打印当前状态（可选）  
        printf("Time: %.2f, MotorA Angle: %.2f, MotorB Angle: %.2f\n", time, motorA.theta, motorB.theta);  
    }  
  
    return 0;  
}

*/
/*debug*/

//extern motor_measure_t motor_chassis[4];
//pids motor_pid;

//int16_t current[4];
//int16_t speed[4];

//int16_t com_tar_angle[1];	
//void angle_debug(void)
//{
////pidINIT(&motor_pid,PID_POSITION,3,0.1,0,1000,100);
//fp32 set_angle=rc_ctrl.rc.ch[1]*angle_ratio*0.1;

//com_tar_angle[0] += set_angle;
//if(com_tar_angle[0]>8191)
//	com_tar_angle[0]-=8191;
//speed[0]=pid_angle_calc(&motor_pid,motor_chassis[0].real_angle,com_tar_angle[0]);
////current[0]=PID_calc(&motor_pid,motor_chassis[0].real_angle,motor_chassis[0].real_angle+set_angle)+PID_calc(&motor_pid,motor_chassis[0].speed_rpm,0);
//current[0]=pid_speed_calc(&motor_pid,motor_chassis[0].speed_rpm,speed[0]);

//CAN_cmd_chassis(current);
//			

//}
