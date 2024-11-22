#include "DM_Motor.h"



void DM_Motor_Enable(CAN_HandleTypeDef* hcan, DM_Motor_t *motor)
{
	switch(motor->mode)
	{
		case mit_mode:
			DM_enable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
		case pos_mode:
			DM_enable_motor_mode(hcan, motor->id, POS_MODE);
			break;
		case spd_mode:
			DM_enable_motor_mode(hcan, motor->id, SPD_MODE);
			break;
		case psi_mode:
			DM_enable_motor_mode(hcan, motor->id, PSI_MODE);
			break;
	}	
}

/**
************************************************************************
* @brief:      	enable_motor_mode: 启用电机模式函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要开启的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送启用特定模式的命令
************************************************************************
**/
void DM_enable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	
	canx_send_data(hcan, id, data, 8);
	Delay_us(300);
}

/**
************************************************************************
* @brief:      	disable_motor_mode: 禁用电机模式函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要禁用的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送禁用特定模式的命令
************************************************************************
**/
void DM_disable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
	
	canx_send_data(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	save_pos_zero: 保存位置零点函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要保存位置零点的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送保存位置零点的命令
************************************************************************
**/
void save_pos_zero(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFE;
	
	canx_send_data(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	clear_err: 清除电机错误函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要清除错误的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送清除错误的命令。
************************************************************************
**/
void clear_err(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFB;
	
	canx_send_data(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	dm4310_ctrl_send: 发送DM4310电机控制命令函数
* @param[in]:   hcan:    指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	根据电机控制模式发送相应的命令到DM4310电机
*               支持的控制模式包括位置模式、位置速度控制模式和速度控制模式
************************************************************************
**/
void DM_Motor_Ctrl_Send(CAN_HandleTypeDef* hcan, DM_Motor_t *motor)
{
	switch(motor->mode)
	{
		case mit_mode:
			DM_MIT_Ctrl(hcan, motor, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set);
			break;
		case pos_mode:
			DM_Pos_Ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
			break;
		case spd_mode:
			DM_Spd_Ctrl(hcan, motor->id, motor->ctrl.vel_set);
			break;
		case psi_mode:
			DM_Psi_Ctrl(hcan, motor->id,motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.cur_set);
			break;
	}	
}

/**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的电机控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
void DM_MIT_Ctrl(CAN_HandleTypeDef *hcan, DM_Motor_t *motor, float pos, float vel, float KP, float KD, float torq)
{
    uint16_t pos_tem, vel_tem, kp_tem, kd_tem, tor_tem;
    pos_tem = float_to_uint(pos, Pos_MIN, Pos_MAX, 16);
    vel_tem = float_to_uint(vel, Vel_MIN, Vel_MAX, 12);
    kp_tem = float_to_uint(KP, DM_KP_MIN, DM_KP_MAX, 12);
    kd_tem = float_to_uint(KD, DM_KD_MIN, DM_KD_MAX, 12);
    tor_tem = float_to_uint(torq, Torq_MIN, Torq_MAX, 12);

    uint8_t Tx_Data[8] = {0};
    Tx_Data[0] = (pos_tem >> 8);
    Tx_Data[1] = pos_tem;
    Tx_Data[2] = (vel_tem >> 4);
    Tx_Data[3] = (vel_tem & 0x0F << 4) | (kp_tem >> 8);
    Tx_Data[4] = kp_tem;
    Tx_Data[5] = (kd_tem >> 4);
    Tx_Data[6] = ((kd_tem & 0x0F) << 4) | (tor_tem >> 8);
    Tx_Data[7] = tor_tem;

    canx_send_data(hcan, motor -> id, Tx_Data, 8);
	Delay_us(200);// mit延时整整200ms??
	//HAL_Delay(20);
}

/**
************************************************************************
* @brief:      	pos_speed_ctrl: 位置速度控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   vel:			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
void DM_Pos_Ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	id = motor_id + POS_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	speed_ctrl: 速度控制函数
* @param[in]:   hcan: 		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   vel: 			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送速度控制命令
************************************************************************
**/
void DM_Spd_Ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];
	
	id = motor_id + SPD_MODE;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 4);
}

/**
************************************************************************
* @brief:      	pos_speed_ctrl: 混控模式
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   i:				电流给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
void DM_Psi_Ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel, float cur)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf, *ibuf;
	uint8_t data[8];
	
	uint16_t u16_vel = vel*100;
	uint16_t u16_cur  = cur*10000;
	
	id = motor_id + PSI_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&u16_vel;
	ibuf=(uint8_t*)&u16_cur;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	
	data[6] = *ibuf;
	data[7] = *(ibuf+1);
	
	canx_send_data(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	read_motor_data: 发送读取寄存器命令
* @param[in]:   id:    电机can id
* @param[in]:   rid:   寄存器地址
* @retval:     	void
* @details:    	读取电机参数
************************************************************************
**/
void read_motor_data(uint16_t id, uint8_t rid) 
{
	uint8_t can_id_l = id & 0x0F;
	uint8_t can_id_h = (id >> 4) & 0x0F;
	
	uint8_t data[4] = {can_id_l, can_id_h, 0x33, rid};
	canx_send_data(&hcan1, 0x7FF, data, 4);
}

/**
************************************************************************
* @brief:      	read_motor_ctrl_fbdata: 发送读取电机反馈数据的命令
* @param[in]:   id:    电机can id
* @retval:     	void
* @details:    	读取电机控制反馈的数据
************************************************************************
**/
void read_motor_ctrl_fbdata(uint16_t id) 
{
	uint8_t can_id_l = id & 0x0F;
	uint8_t can_id_h = (id >> 4) & 0x0F;
	
	uint8_t data[4] = {can_id_l, can_id_h, 0xCC, 0x00};
	canx_send_data(&hcan1, 0x7FF, data, 4);
}
/**
************************************************************************
* @brief:      	write_motor_data: 发送写寄存器命令
* @param[in]:   id:    电机can id
* @param[in]:   rid:   寄存器地址
* @param[in]:   d0-d3: 写入的数据
* @retval:     	void
* @details:    	向寄存器写入数据
************************************************************************
**/
void write_motor_data(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
	uint8_t can_id_l = id & 0x0F;
	uint8_t can_id_h = (id >> 4) & 0x0F;
	
	uint8_t data[8] = {can_id_l, can_id_h, 0x55, rid, d0, d1, d2, d3};
	canx_send_data(&hcan1, 0x7FF, data, 8);
}
/**
************************************************************************
* @brief:      	save_motor_data: 发送保存命令
* @param[in]:   id:    电机can id
* @param[in]:   rid:   寄存器地址
* @retval:     	void
* @details:    	保存写入的电机参数
************************************************************************
**/
void save_motor_data(uint16_t id, uint8_t rid) 
{
	uint8_t can_id_l = id & 0x0F;
	uint8_t can_id_h = (id >> 4) & 0x0F;
	
	uint8_t data[4] = {can_id_l, can_id_h, 0xAA, 0x01};
	canx_send_data(&hcan1, 0x7FF, data, 4);
}
	  uint16_t p_int,v_int,t_int;

void Get_DM_MotorInfo(DM_MotorMeasure_t* dm_motor_t, uint8_t data[8])
{
	uint16_t p_int,v_int,t_int;
  p_int = (data[1] << 8) | data[2];
  v_int = (data[3] << 4) | (data[4] >> 4);
  t_int = ((data[4]&0x0F) << 8) | data[5];

  Motor1.DM_MotorMeasure.ID   = data[0] & 0x0F;
  Motor1.DM_MotorMeasure.ERR  = (data[0] & 0xF0) >> 4;
  Motor1.DM_MotorMeasure.POS  = uint_to_float(p_int,Pos_MIN,Pos_MAX,16);
  Motor1.DM_MotorMeasure.VEL  = uint_to_float(v_int,Vel_MIN,Vel_MAX,12);
  Motor1.DM_MotorMeasure.Torq = uint_to_float(t_int,Torq_MIN,Torq_MAX,12);
  Motor1.DM_MotorMeasure.T_MOS = data[6];
  Motor1.DM_MotorMeasure.T_Motor = data[7];

}

