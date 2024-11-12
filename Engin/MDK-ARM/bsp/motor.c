#include "motor.h"

/*******************************************************************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  *             注意:底盘4个         下板can10
                    抬升前伸4个      下板can11
                    横向1个          上板can11
                    小云台2个        上板can11
                    机械臂4个        上板can10
                    双板通信            can20
                    遥控接上板
                    气泵下板一个usart发给f1控gpio
                    
                    先全写一起
                    底盘4个          can10
                    抬升前伸4个      can11(降频)
                    横向1个          can11
                    小云台2个        can20
                    机械臂4个        can21
                    遥控DBUS(usart3)
                    气泵一个usart1发给f1控gpio
                    图传uart6
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ********************************************************************************/

//////////////////////////////////////////////////////////////////////////////

/////                             filter                               ///////

//////////////////////////////////////////////////////////////////////////////
void can_filter_init(void)
{
 
    CAN_FilterTypeDef can_filter_st;
 
    can_filter_st.FilterActivation = CAN_FILTER_ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
    can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
    can_filter_st.FilterIdHigh = 0x201<<5;
    can_filter_st.FilterIdLow = 0x202<<5;
    can_filter_st.FilterMaskIdHigh = 0x203<<5;
    can_filter_st.FilterMaskIdLow = 0x204<<5;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
 
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
    can_filter_st.FilterIdHigh = 0x205<<5;
    can_filter_st.FilterIdLow = 0x206<<5;
    can_filter_st.FilterMaskIdHigh = 0x207<<5;
    can_filter_st.FilterMaskIdLow = 0x208<<5;
    can_filter_st.FilterBank = 2;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);


    can_filter_st.SlaveStartFilterBank = 14;

    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
    can_filter_st.FilterIdHigh = 0x201<<5;
    can_filter_st.FilterIdLow = 0x202<<5;
    can_filter_st.FilterMaskIdHigh = 0x203<<5;//2006
    can_filter_st.FilterMaskIdLow = 0x1000<<5;
    can_filter_st.FilterBank = 16;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);

//    can_filter_st.FilterActivation = ENABLE;
//    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
//    can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
//    can_filter_st.FilterIdHigh = 0xdamaio<<5;
//    can_filter_st.FilterIdLow = 0x206<<5;
//    can_filter_st.FilterMaskIdHigh = 0x207<<5;
//    can_filter_st.FilterMaskIdLow = 0x208<<5;
//    can_filter_st.FilterBank = 18;
//    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
//    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);

}


//////////////////////////////////////////////////////////////////////////////

/////                               tx                              //////////

//////////////////////////////////////////////////////////////////////////////

uint8_t chassis_can_send_data[8]; 
CAN_TxHeaderTypeDef chassis_tx_message;
     uint32_t send_mail_box;
 
/////////////////////can1 fifo0-1234
void CAN_cmd_chassis(int16_t motor[])
{

    chassis_tx_message.StdId=CAN_FIRST_ALL_ID;//0X200
    chassis_tx_message.IDE=CAN_ID_STD;
    chassis_tx_message.RTR=CAN_RTR_DATA;
    chassis_tx_message.DLC=0x08;

    chassis_can_send_data[0]=motor[0]>>8;//ALL CHASSIS
    chassis_can_send_data[1]=motor[0];
    chassis_can_send_data[2]=motor[1]>>8;
    chassis_can_send_data[3]=motor[1];
    chassis_can_send_data[4]=motor[2]>>8;
    chassis_can_send_data[5]=motor[2];
    chassis_can_send_data[6]=motor[3]>>8;
    chassis_can_send_data[7]=motor[3];
    
    HAL_CAN_AddTxMessage(&hcan1,&chassis_tx_message,chassis_can_send_data,&send_mail_box);
	
	  HAL_Delay(1);
}
///////////////can1 fifo1-5678
	  uint8_t lifting_can_send_data[8]; 
    CAN_TxHeaderTypeDef lifting_tx_message;
void CAN_cmd_lifting(int16_t motor[])
{

    uint32_t send_mail_box;
	
    lifting_tx_message.StdId=CAN_SECOND_ALL_ID;//0x1ff
    lifting_tx_message.IDE=CAN_ID_STD;
    lifting_tx_message.RTR=CAN_RTR_DATA;
    lifting_tx_message.DLC=0x08;
	
    lifting_can_send_data[0]=motor[0]>>8;//LIFTING2
    lifting_can_send_data[1]=motor[0];
    lifting_can_send_data[2]=motor[1]>>8;
    lifting_can_send_data[3]=motor[1];
    lifting_can_send_data[4]=motor[2]>>8;//PROTRACT2
    lifting_can_send_data[5]=motor[2];
    lifting_can_send_data[6]=motor[3]>>8;
    lifting_can_send_data[7]=motor[3];
   
    HAL_CAN_AddTxMessage(&hcan1,&lifting_tx_message,lifting_can_send_data,&send_mail_box);
	
	  HAL_Delay(1);
}
///////////////can2 fifo0
	  uint8_t gimbal_can_send_data[8]; 
    CAN_TxHeaderTypeDef gimbal_tx_message;
void CAN_cmd_gimbal(int16_t motor[])
{

    uint32_t send_mail_box;
	
    gimbal_tx_message.StdId=CAN_FIRST_ALL_ID;//0x200
    gimbal_tx_message.IDE=CAN_ID_STD;
    gimbal_tx_message.RTR=CAN_RTR_DATA;
    gimbal_tx_message.DLC=0x08;
	
    gimbal_can_send_data[0]=motor[0]>>8;//pitch
    gimbal_can_send_data[1]=motor[0];
    gimbal_can_send_data[2]=motor[1]>>8;//yaw
    gimbal_can_send_data[3]=motor[1];
    gimbal_can_send_data[4]=motor[2]>>8;//2006
    gimbal_can_send_data[5]=motor[2];
    gimbal_can_send_data[6]=motor[3]>>8;//0
    gimbal_can_send_data[7]=motor[3];//0
   
    HAL_CAN_AddTxMessage(&hcan1,&gimbal_tx_message,gimbal_can_send_data,&send_mail_box);
	
	  HAL_Delay(1);
}
//void ctrl_motor(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel,
//float _KP, float _KD, float _torq)
//{
//    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
//    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
//    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
//    kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
//    kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
//    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

//    hcan->pTxMsg->StdId = id;
//    hcan->pTxMsg->IDE = CAN_ID_STD;
//    hcan->pTxMsg->RTR = CAN_RTR_DATA;
//    hcan->pTxMsg->DLC = 0x08;
//    hcan->pTxMsg->Data[0] = (pos_tmp >> 8);
//    hcan->pTxMsg->Data[1] = pos_tmp;
//    hcan->pTxMsg->Data[2] = (vel_tmp >> 4);
//    hcan->pTxMsg->Data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
//    hcan->pTxMsg->Data[4] = kp_tmp;
//    hcan->pTxMsg->Data[5] = (kd_tmp >> 4);
//    hcan->pTxMsg->Data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
//    hcan->pTxMsg->Data[7] = tor_tmp;

//    HAL_CAN_Transmit(hcan, 100);
// }
// ///////////////can2 fifo1-4310
// 	  uint8_t arm_can_send_data[8]; 
//     CAN_TxHeaderTypeDef arm_tx_message;
// void CAN_cmd_lifting(int16_t motor[])
// {

//     uint32_t send_mail_box;
	
//     arm_tx_message.StdId=
//     arm_tx_message.IDE=
//     arm_tx_message.RTR=
//     arm_tx_message.DLC=
// 	arm
//     arm_can_send_data[0]=motor[0]>>8;??
//     arm_can_send_data[1]=motor[0];
//     arm_can_send_data[2]=motor[1]>>8;
//     arm_can_send_data[3]=motor[1];
//     arm_can_send_data[4]=motor[2]>>8;
//     arm_can_send_data[5]=motor[2];
//     arm_can_send_data[6]=motor[3]>>8;
//     arm_can_send_data[7]=motor[3];
   
//     HAL_CAN_AddTxMessage(&hcan1,&lifting_tx_message,lifting_can_send_data,&send_mail_box);
	
// 	  HAL_Delay(1);
// }
//////////////////////////////////////////////////////////////////////////////

/////                             rx                                    //////

//////////////////////////////////////////////////////////////////////////////

#define get_motor_measure(ptr,data)\
{\
    (ptr)->angle_value=(uint16_t)((data)[0]<<8|(data)[1]);\
    (ptr)->speed_rpm=(uint16_t)((data)[2]<<8|(data)[3]);\
    (ptr)->real_current=(uint16_t)((data)[4]<<8|(data)[5]);\
    (ptr)->temperate=(data)[6];\
    (ptr)->real_angle=(ptr)->angle_value/8192.0f*360.0f;\
}
 
/////////////////////fifo0-1234
motor_measure_t motor_chassis[4]; 
uint8_t rx_data[8]; 

motor_measure_t motor_gimbal[4]; 


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    
    CAN_RxHeaderTypeDef rx_header;
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
  	switch(rx_header.StdId)
    {
        case motor1:
        case motor2:
        case motor3:
        case motor4:
        {
            static uint8_t n = 0;
            n = rx_header.StdId - motor1;
            get_motor_measure(&motor_chassis[n],rx_data);
            break;
        }
        default:
        {
            break;
        }
    }
		
		
		///////////////////////can2 fifo0


    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, rx_data);
  	switch(rx_header.StdId)
    {
        case motor1:
        case motor2:
        case motor3:
        {
            static uint8_t i = 0;
            i = rx_header.StdId - motor1;
            get_motor_measure(&motor_gimbal[i],rx_data);
            break;
        }
        default:
        {
            break;
        }
    }
		
		
		 
		/*	if(hcan->Instance==CAN2)
	{
	  switch (rx_header.StdId) 
    {       
		 //下面是英雄底盘的关键，获取底盘三个方向的期望速度 
			case CAN_GIMBAL_CONNECT_CHASSIS :
      {
				get_vx_vy_wz_set(&processed_send_data,rx_data);
				
			 //唤醒底盘任务,
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
         static BaseType_t xHigherPriorityTaskWoken;
         vTaskNotifyGiveFromISR( Chassis_Task_Local_Handler, &xHigherPriorityTaskWoken);
         portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
					i++;
        }			 
        break;
      }
     //上面是英雄底盘的关键，获取底盘三个方向的期望速度
			 
     default:
     {
        break;
     }
    }
	}*/

}


///////////////fifo1-5678
motor_measure_t motor_lifting[4]; 
uint8_t rx_lifting_data[8]; 

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx1_header;
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO1, &rx1_header, rx_lifting_data);
  	switch(rx1_header.StdId)
    {
		case motor5:
        case motor6:
        case motor7:
        case motor8:
        {
            static uint8_t g = 0; 
            g = rx1_header.StdId - motor5;
            get_motor_measure(&motor_lifting[g],rx_lifting_data);//////////////
            break;
        }
        default:
        {
            break;
        }
    }
}





//void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
//{
//if(HAL_GetTick() - FlashTimer>500){

//HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
//FlashTimer = HAL_GetTick();
//}

////ignore can1 or can2.
// if(_hcan->pRxMsg->StdId == 0)
// {
// p_int=(_hcan->pRxMsg->Data[1]<<8)|_hcan->pRxMsg->Data[2];
// v_int=(_hcan->pRxMsg->Data[3]<<4)|(_hcan->pRxMsg->Data[4]>>4);
// t_int=((_hcan->pRxMsg->Data[4]&0xF)<<8)|_hcan->pRxMsg->Data[5];
// position = uint_to_float(p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
// velocity = uint_to_float(v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
// torque = uint_to_float(t_int, T_MIN, T_MAX, 12); // (-18.0,18.0)
// }
// /*#### add enable can it again to solve can receive only one ID problem!!!
//#**/
// __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
// }


/*反馈帧 ID 由调试助手设置（Master ID），默认为 0，主要反馈电机的位置，
速度和扭矩信息，其帧格式定义为：
反馈报文 D[0] D[1] D[2] D[3] D[4] D[5] D[6] D[7]
MST_ID ID|ERR<<4 POS[15:8] POS[7:0] VEL[11:4] VEL[3:0]|T[11:8] T[7:0] T_MOS T_Rotor 其中：
ID 表示控制器的 ID，取 CAN_ID 的低 8 位
ERR 表示状态，对应状态类型为：
0——失能；
1——使能；
8——超压；
9——欠压；
A——过电流；
B——MOS 过温；
C——电机线圈过温；
D——通讯丢失；
E——过载；
POS 表示电机的位置信息
VEL 表示电机的速度信息
T 表示电机的扭矩信息
T_MOS 表示驱动上 MOS 的平均温度，单位℃
调试助手使用说明书（达妙驱动控制协议）V1.4
第 33 页 共 40 页
T_Rotor 表示电机内部线圈的平均温度，单位℃
位置、速度和扭矩采用线性映射的关系将浮点型数据转换成有符号的定点数
据，其中位置采用 16 位数据，速度和扭矩均使用 12 位，以速度为例说明映射关
系。
如电机当前速度为 25.0rad/s，设置的速度范围 VMAX=45rad/s，则发送的数
据为：VEL=25.0/(45-(-45))*2^12+2^11=3185=0xC71*/




//////////////////////////////////////////////////////////////////////////////

//////                            init                                ////////

//////////////////////////////////////////////////////////////////////////////
void can_start(void)
{
    can_filter_init();
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_RX_FIFO1_MSG_PENDING);
}



///*                           亮灯显示接收状态                             */
//void LED_RX_Status_display()
//{
//	//每个电机分开写if,怎么判断值齐不齐?//不需要判断,都有就不会进if
//	//缺那几个id的就连续闪几下,隔几秒再闪几下
//	//不同部分颜色分开,分先后
//	int i;
//	for(i=0;i++;i<=3)
//	{
//	if(motor_chassis[i] == NULL)
//		HAL_GPIO_WritePin(GPIOA,LED_B_GPIO_Port,GPIO_PIN_SET);
//	}
//}
