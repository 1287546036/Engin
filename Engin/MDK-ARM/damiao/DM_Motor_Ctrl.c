#include "DM_Motor.h"
#include "DM_Motor_Ctrl.h"
#include "string.h"
#include "stdbool.h"
#include "pid_data.h"

DM_Motor_t Motor1;
DM_Motor_t Motor2;
DM_Motor_t Motor3;
DM_Motor_t Motor4;


/**
************************************************************************
* @brief:      	dm4310_motor_init: DM4310电机初始化函数
* @param:      	void
* @retval:     	void
* @details:    	初始化1个DM4310型号的电机，设置默认参数和控制模式。
*               设置ID、控制模式和命令模式等信息。
************************************************************************
**/
void dm_motor_init(void)
{
	// 设置Motor1的电机信息
	Motor1.id = 0x01;
	Motor1.mst_id = 0x11;
	Motor1.ref.read_flag = 1;
	Motor1.mode = mit_mode;
	Motor1.ctrl.vel_set = Motor1_vel_set;
	Motor1.ctrl.last_pos= Motor1.DM_MotorMeasure.POS;
	Motor1.ctrl.pos_set = Motor1.DM_MotorMeasure.POS;
	Motor1.ctrl.cur_set = Motor1_cur_set ;
	Motor1.ctrl.tor_set = Motor1_tor_set ;
	Motor1.ctrl.kp_set  = Motor1_kp_set   ;
	Motor1.ctrl.kd_set  = Motor1_kd_set   ; 
	Motor1.ref.PMAX 	= Motor1_PMAX ;
	Motor1.ref.VMAX 	= Motor1_VMAX ;
	Motor1.ref.TMAX 	= Motor1_TMAX ;
	// 设置Motor2的电机信息
	Motor2.id = 0x02;
	Motor2.mst_id = 0x12;
	Motor2.ref.read_flag = 1;
	Motor2.mode = mit_mode;
	Motor2.ctrl.vel_set = Motor2_vel_set;
	Motor2.ctrl.last_pos= Motor2.DM_MotorMeasure.POS;
	Motor2.ctrl.pos_set = Motor2.DM_MotorMeasure.POS;
	Motor2.ctrl.cur_set = Motor2_cur_set ;
	Motor2.ctrl.tor_set = Motor2_tor_set ;
	Motor2.ctrl.kp_set  = Motor2_kp_set   ;
	Motor2.ctrl.kd_set  = Motor2_kd_set   ; 
	Motor2.ref.PMAX 	= Motor2_PMAX ;
	Motor2.ref.VMAX 	= Motor2_VMAX ;
	Motor2.ref.TMAX 	= Motor2_TMAX ;
	// 设置Motor3的电机信息
	Motor3.id = 0x03;
	Motor3.mst_id = 0x13;
	Motor3.ref.read_flag = 1;
	Motor3.mode = mit_mode;
	Motor3.ctrl.vel_set = Motor3_vel_set;
	Motor3.ctrl.last_pos= Motor3.DM_MotorMeasure.POS;
	Motor3.ctrl.pos_set = Motor3.DM_MotorMeasure.POS;
	Motor3.ctrl.cur_set = Motor3_cur_set ;
	Motor3.ctrl.tor_set = Motor3_tor_set ;
	Motor3.ctrl.kp_set  = Motor3_kp_set   ;
	Motor3.ctrl.kd_set  = Motor3_kd_set   ; 
	Motor3.ref.PMAX 	= Motor3_PMAX ;
	Motor3.ref.VMAX 	= Motor3_VMAX ;
	Motor3.ref.TMAX 	= Motor3_TMAX ;
	// 设置Motor4的电机信息
	Motor4.id = 0x04;
	Motor4.mst_id = 0x14;
	Motor4.ref.read_flag = 1;
	Motor4.mode = mit_mode;
	Motor4.ctrl.vel_set = Motor4_vel_set;
	Motor4.ctrl.last_pos= Motor4.DM_MotorMeasure.POS;
	Motor4.ctrl.pos_set = Motor4.DM_MotorMeasure.POS;
	Motor4.ctrl.cur_set = Motor4_cur_set ;
	Motor4.ctrl.tor_set = Motor4_tor_set ;
	Motor4.ctrl.kp_set  = Motor4_kp_set   ;
	Motor4.ctrl.kd_set  = Motor4_kd_set   ; 
	Motor4.ref.PMAX 	= Motor4_PMAX ;
	Motor4.ref.VMAX 	= Motor4_VMAX ;
	Motor4.ref.TMAX 	= Motor4_TMAX ;
}
/**
************************************************************************
* @brief:      	read_all_motor_data: 读取电机的所有寄存器的数据信息
* @param:      	motor_t：电机参数结构体
* @retval:     	void
* @details:    	逐次发送读取命令
************************************************************************
**/
void read_all_motor_data(DM_Motor_t *motor)
{
    switch (motor->ref.read_flag)
    {
		case 1:	 read_motor_data(motor->id, 0);  break; // UV_Value
        case 2:	 read_motor_data(motor->id, 1);  break; // KT_Value
		case 3:  read_motor_data(motor->id, 2);  break; // OT_Value
        case 4:  read_motor_data(motor->id, 3);  break; // OC_Value
		case 5:	 read_motor_data(motor->id, 4);  break; // ACC
        case 6:	 read_motor_data(motor->id, 5);  break; // DEC
		case 7:  read_motor_data(motor->id, 6);  break; // MAX_SPD
        case 8:  read_motor_data(motor->id, 7);  break;// MSC_ID 
		case 9:  read_motor_data(motor->id, 8);  break;// ESC_ID
        case 10: read_motor_data(motor->id, 9);  break;// TIMEOUT 
		case 11: read_motor_data(motor->id, 10); break;// CTRL_MODE 
        case 12: read_motor_data(motor->id, 11); break;// Damp 
		case 13: read_motor_data(motor->id, 12); break;// Inertia
        case 14: read_motor_data(motor->id, 13); break;// Rsv1 
		case 15: read_motor_data(motor->id, 14); break;// sw_ver 
        case 16: read_motor_data(motor->id, 15); break;// Rsv2 
		case 17: read_motor_data(motor->id, 16); break;// NPP 
        case 18: read_motor_data(motor->id, 17); break;// Rs 
		case 19: read_motor_data(motor->id, 18); break;// Ls 
        case 20: read_motor_data(motor->id, 19); break;// Flux 
		case 21: read_motor_data(motor->id, 20); break;// Gr 
        case 22: read_motor_data(motor->id, 21); break;// PMAX 
		case 23: read_motor_data(motor->id, 22); break;// VMAX 
        case 24: read_motor_data(motor->id, 23); break;// TMAX 
		case 25: read_motor_data(motor->id, 24); break;// I_BW 
        case 26: read_motor_data(motor->id, 25); break;// KP_ASR 
		case 27: read_motor_data(motor->id, 26); break;// KI_ASR 
        case 28: read_motor_data(motor->id, 27); break;// KP_APR 
		case 29: read_motor_data(motor->id, 28); break;// KI_APR 
		case 30: read_motor_data(motor->id, 29); break;// OV_Value 
        case 31: read_motor_data(motor->id, 30); break;// GREF 
		case 32: read_motor_data(motor->id, 31); break;// Deta 
        case 33: read_motor_data(motor->id, 32); break;// V_BW 
		case 34: read_motor_data(motor->id, 33); break;// IQ_c1 
        case 35: read_motor_data(motor->id, 34); break;// VL_c1 
		case 36: read_motor_data(motor->id, 35); break;// can_br 
        case 37: read_motor_data(motor->id, 36); break;// sub_ver 
		case 38: read_motor_data(motor->id, 50); break;// u_off 
        case 39: read_motor_data(motor->id, 51); break;// v_off 
		case 40: read_motor_data(motor->id, 52); break;// k1 
        case 41: read_motor_data(motor->id, 53); break;// k2 
		case 42: read_motor_data(motor->id, 54); break;// m_off 
		case 43: read_motor_data(motor->id, 55); break;// dir 
		case 44: read_motor_data(motor->id, 80); break;// pm 
		case 45: read_motor_data(motor->id, 81); break;// xout 
    }
}
/**
************************************************************************
* @brief:      	receive_motor_data: 接收电机返回的数据信息
* @param:      	motor_t：电机参数结构体
* @param:      	data：接收的数据
* @retval:     	void
* @details:    	逐次接收电机回传的参数信息
************************************************************************
**/
void receive_motor_data(DM_Motor_t *motor, uint8_t *data)
{
	if(motor->ref.read_flag == 0)
		return ;
	
	float_type_u y;
	
	if(data[2] == 0x33)
	{
		y.b_val[0] = data[4];
		y.b_val[1] = data[5];
		y.b_val[2] = data[6];
		y.b_val[3] = data[7];
		
		switch(data[3])
		{
			case  0: motor->ref.UV_Value = y.f_val; motor->ref.read_flag =  2; break;
			case  1: motor->ref.KT_Value = y.f_val; motor->ref.read_flag =  3; break;
			case  2: motor->ref.OT_Value = y.f_val; motor->ref.read_flag =  4; break;
			case  3: motor->ref.OC_Value = y.f_val; motor->ref.read_flag =  5; break;
			case  4: motor->ref.ACC 	 = y.f_val; motor->ref.read_flag =  6; break;
			case  5: motor->ref.DEC 	 = y.f_val; motor->ref.read_flag =  7; break;
			case  6: motor->ref.MAX_SPD  = y.f_val; motor->ref.read_flag =  8; break;
			case  7: motor->ref.MST_ID   = y.u_val; motor->ref.read_flag =  9; break;
			case  8: motor->ref.ESC_ID   = y.u_val; motor->ref.read_flag = 10; break;
			case  9: motor->ref.TIMEOUT  = y.u_val; motor->ref.read_flag = 11; break;
			case 10: motor->ref.cmode    = y.u_val; motor->ref.read_flag = 12; break;
			case 11: motor->ref.Damp 	 = y.f_val; motor->ref.read_flag = 13; break;
			case 12: motor->ref.Inertia  = y.f_val; motor->ref.read_flag = 14; break;
			case 13: motor->ref.hw_ver   = y.u_val; motor->ref.read_flag = 15; break;
			case 14: motor->ref.sw_ver   = y.u_val; motor->ref.read_flag = 16; break;
			case 15: motor->ref.SN 	  	 = y.u_val; motor->ref.read_flag = 17; break;
			case 16: motor->ref.NPP 	 = y.u_val; motor->ref.read_flag = 18; break;
			case 17: motor->ref.Rs 	  	 = y.f_val; motor->ref.read_flag = 19; break;
			case 18: motor->ref.Ls 	  	 = y.f_val; motor->ref.read_flag = 20; break;
			case 19: motor->ref.Flux 	 = y.f_val; motor->ref.read_flag = 21; break;
			case 20: motor->ref.Gr 	  	 = y.f_val; motor->ref.read_flag = 22; break;
			case 21: motor->ref.PMAX 	 = y.f_val; motor->ref.read_flag = 23; break;
			case 22: motor->ref.VMAX 	 = y.f_val; motor->ref.read_flag = 24; break;
			case 23: motor->ref.TMAX 	 = y.f_val; motor->ref.read_flag = 25; break;
			case 24: motor->ref.I_BW 	 = y.f_val; motor->ref.read_flag = 26; break;
			case 25: motor->ref.KP_ASR   = y.f_val; motor->ref.read_flag = 27; break;
			case 26: motor->ref.KI_ASR   = y.f_val; motor->ref.read_flag = 28; break;
			case 27: motor->ref.KP_APR   = y.f_val; motor->ref.read_flag = 29; break;
			case 28: motor->ref.KI_APR   = y.f_val; motor->ref.read_flag = 30; break;
			case 29: motor->ref.OV_Value = y.f_val; motor->ref.read_flag = 31; break;
			case 30: motor->ref.GREF 	 = y.f_val; motor->ref.read_flag = 32; break;
			case 31: motor->ref.Deta 	 = y.f_val; motor->ref.read_flag = 33; break;
			case 32: motor->ref.V_BW 	 = y.f_val; motor->ref.read_flag = 34; break;
			case 33: motor->ref.IQ_cl 	 = y.f_val; motor->ref.read_flag = 35; break;
			case 34: motor->ref.VL_cl 	 = y.f_val; motor->ref.read_flag = 36; break;
			case 35: motor->ref.can_br   = y.u_val; motor->ref.read_flag = 37; break;
			case 36: motor->ref.sub_ver  = y.u_val; motor->ref.read_flag = 38; break;
			case 50: motor->ref.u_off 	 = y.f_val; motor->ref.read_flag = 39; break;
			case 51: motor->ref.v_off 	 = y.f_val; motor->ref.read_flag = 40; break;
			case 52: motor->ref.k1 		 = y.f_val; motor->ref.read_flag = 41; break;
			case 53: motor->ref.k2		 = y.f_val; motor->ref.read_flag = 42; break;
			case 54: motor->ref.m_off 	 = y.f_val; motor->ref.read_flag = 43; break;
			case 55: motor->ref.dir 	 = y.f_val; motor->ref.read_flag = 44; break;
			case 80: motor->ref.p_m 	 = y.f_val; motor->ref.read_flag = 45; break;
			case 81: motor->ref.x_out 	 = y.f_val; motor->ref.read_flag = 0 ; break;
		}
	}
}

/**
************************************************************************
* @brief:      	fdcan1_rx_callback: CAN1接收回调函数
* @param:      	void
* @retval:     	void
* @details:    	处理CAN1接收中断回调，根据接收到的ID和数据，执行相应的处理。
*               当接收到ID为0时，调用dm4310_fbdata函数更新Motor的反馈数据。
************************************************************************
**/
void can1_rx_callback(void)
{
	uint16_t rec_id;
	uint8_t rx_data[8] = {0};
	canx_receive(&hcan1, &rec_id, rx_data);
	switch (rec_id)
	{
 		case 0x11: Get_DM_MotorInfo(&Motor1.DM_MotorMeasure, rx_data); receive_motor_data(&Motor1, rx_data); break;
		case 0x12: Get_DM_MotorInfo(&Motor2.DM_MotorMeasure, rx_data); receive_motor_data(&Motor2, rx_data); break;
		case 0x13: Get_DM_MotorInfo(&Motor3.DM_MotorMeasure, rx_data); receive_motor_data(&Motor3, rx_data); break;
		case 0x14: Get_DM_MotorInfo(&Motor4.DM_MotorMeasure, rx_data); receive_motor_data(&Motor4, rx_data); break;
	}
}


