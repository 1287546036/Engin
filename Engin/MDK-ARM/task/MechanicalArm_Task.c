#include "MechanicalArm_Task.h"
#include "main.h"
#include "remote_control.h"



void mechanicalarm_init(void)
{
	read_motor_ctrl_fbdata(Motor1.id);//init
	read_motor_ctrl_fbdata(Motor2.id);
	read_motor_ctrl_fbdata(Motor3.id);
	read_motor_ctrl_fbdata(Motor4.id);

	  DM_Motor_Enable(&hcan1,&Motor1); 
      DM_Motor_Enable(&hcan1,&Motor2); 
      DM_Motor_Enable(&hcan1,&Motor3); 
      DM_Motor_Enable(&hcan1,&Motor4); 
      
	dm_motor_init();
	
}

void mechanicalarm_task(float encoder_angle_set[])
{
	
	/*debug_circle*/
//		Motor1.ctrl.last_pos = Motor1.ctrl.pos_set;
//		Motor1.ctrl.pos_set +=0.001f;
//	if(Motor1.ctrl.pos_set >= 12.5f )
//		{
//			DM_disable_motor_mode(&Motor,Motor1.id,MIT_MODE);
//			Motor1.ctrl.pos_set = Motor1.ctrl.pos_set - Motor1.ctrl.last_pos;
//			save_pos_zero(&Motor,Motor1.id,MIT_MODE);

//			DM_Motor_Enable(&hcan1,&Motor1); 
//		}
	
//		
		if((Motor1.ctrl.pos_set-Motor1.ctrl.last_pos)>3||(Motor1.ctrl.pos_set-Motor1.ctrl.last_pos)<-3)
		{Motor1.ctrl.pos_set = Motor1.ctrl.last_pos;}
		
		Motor1.ctrl.last_pos = Motor1.ctrl.pos_set;
		Motor1.ctrl.pos_set = encoder_angle_set[2];
		//	if(Motor1.ctrl.pos_set >= 12.5f )// 编码器不会大于,应该改为过零点处理

//	Motor1.ctrl.vel_set=1.0f;
		
		DM_Motor_Enable(&hcan1,&Motor1); 	
		DM_Motor_Ctrl_Send(&Motor,&Motor1);

		/*-12.5 -> 0 -> 12.5 -> -12.5|*/
//   read_all_motor_data(&Motor1);
		
}

void encoder_to_damiao(uint8_t rxBuffer[],float encoder_angle_set[])
{
			encoder_angle_set[0] = ((float)rxBuffer[4] *100+(float)rxBuffer[5] +(float)rxBuffer[6] /100.0f)/360.0f*12.5f;
			encoder_angle_set[1] = ((float)rxBuffer[8] *100+(float)rxBuffer[9] +(float)rxBuffer[10]/100.0f)/360.0f*12.5f;
			encoder_angle_set[2] = ((float)rxBuffer[12]*100+(float)rxBuffer[13]+(float)rxBuffer[14]/100.0f)/360.0f*12.5f;
			encoder_angle_set[3] = ((float)rxBuffer[16]*100+(float)rxBuffer[17]+(float)rxBuffer[18]/100.0f)/360.0f*12.5f;
	}
 /*  发送帧如下,可改动
	  custom_robot_data.data[0] = 0xff;
      custom_robot_data.data[1] = 0x00;
      custom_robot_data.data[2] = 0xee;

      custom_robot_data.data[3] = 0xaa;
      custom_robot_data.data[4] = encoder_A_back.angle / 100;
      custom_robot_data.data[5] = encoder_A_back.angle - custom_robot_data.data[4] * 100;
      custom_robot_data.data[6] = (encoder_A_back.angle - custom_robot_data.data[5] - custom_robot_data.data[4] * 100) * 100;

      custom_robot_data.data[7] = 0xbb;
      custom_robot_data.data[8] = encoder_B_back.angle / 100;
      custom_robot_data.data[9] = encoder_B_back.angle - custom_robot_data.data[8] * 100;
      custom_robot_data.data[10] = (encoder_B_back.angle - custom_robot_data.data[9] - custom_robot_data.data[8] * 100) * 100;

      custom_robot_data.data[11] = 0xcc;
      custom_robot_data.data[12] = encoder_C_back.angle / 100;
      custom_robot_data.data[13] = encoder_C_back.angle - custom_robot_data.data[12] * 100;
      custom_robot_data.data[14] = (encoder_C_back.angle - custom_robot_data.data[13] - custom_robot_data.data[12] * 100) * 100;

      custom_robot_data.data[15] = 0xdd;
      custom_robot_data.data[16] = encoder_D_back.angle / 100;
      custom_robot_data.data[17] = encoder_D_back.angle - custom_robot_data.data[16] * 100;
      custom_robot_data.data[18] = (encoder_D_back.angle - custom_robot_data.data[17] - custom_robot_data.data[16] * 100) * 100;

      custom_robot_data.data[19] = 0xab;
      custom_robot_data.data[20] = 0xff;
      custom_robot_data.data[21] = 0xcd;
*/




