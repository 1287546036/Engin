#include "motor.h"
//////////////////////////////////////////////////////////////////////////////

/////                             filter                               ///////

//////////////////////////////////////////////////////////////////////////////
void can_filter_fifo0_init(void)
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
	//  can_filter_st.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

}

void can_filter_fifo1_init(void)
{
 
    CAN_FilterTypeDef can_filter_st;
 
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
}
//////////////////////////////////////////////////////////////////////////////

/////                               tx                              //////////

//////////////////////////////////////////////////////////////////////////////

uint8_t chassis_can_send_data[8]; 
CAN_TxHeaderTypeDef chassis_tx_message;
     uint32_t send_mail_box;
 
/////////////////////fifo0-1234//6020
void CAN_cmd_chassis(int16_t motor[])
{

    chassis_tx_message.StdId=CAN_CHASSIS_ALL_ID;//0x200
////// chassis_tx_message.StdId=CAN_AUXILIARY_ALL_ID;//0x1ff
    chassis_tx_message.IDE=CAN_ID_STD;
    chassis_tx_message.RTR=CAN_RTR_DATA;
    chassis_tx_message.DLC=0x08;
//////	chassis_tx_message.DLC=8;
//////	chassis_can_send_data[0]=(M1>>8)&0xff;////////////////6020
//////  chassis_can_send_data[1]=(M1)&0xff;

    chassis_can_send_data[0]=motor[0]>>8;
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
///////////////fifo1-5678
	  uint8_t lifting_can_send_data[8]; 
    CAN_TxHeaderTypeDef lifting_tx_message;
void CAN_cmd_lifting(int16_t motor[])
{

    uint32_t send_mail_box;
	
    lifting_tx_message.StdId=CAN_AUXILIARY_ALL_ID;//0x1ff
    lifting_tx_message.IDE=CAN_ID_STD;
    lifting_tx_message.RTR=CAN_RTR_DATA;
    lifting_tx_message.DLC=0x08;
	
    lifting_can_send_data[0]=motor[0]>>8;
    lifting_can_send_data[1]=motor[0];
    lifting_can_send_data[2]=motor[1]>>8;
    lifting_can_send_data[3]=motor[1];
    lifting_can_send_data[4]=motor[2]>>8;
    lifting_can_send_data[5]=motor[2];
    lifting_can_send_data[6]=motor[3]>>8;
    lifting_can_send_data[7]=motor[3];
   
    HAL_CAN_AddTxMessage(&hcan1,&lifting_tx_message,lifting_can_send_data,&send_mail_box);
	
	  HAL_Delay(1);
}
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
motor_measure_t motor_lifting[4]; 
uint8_t rx_lifting_data[8]; 


uint8_t rx_data[8]; 
motor_measure_t motor_chassis[4]; 


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
            static uint8_t i = 0;
            i = rx_header.StdId - motor1;
            get_motor_measure(&motor_chassis[i],rx_data);
            break;
        }
        default:
        {
            break;
        }
    }
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

//////////////////////////////////////////////////////////////////////////////

//////                            init                                ////////

//////////////////////////////////////////////////////////////////////////////
void can1_start(void)
{
    can_filter_fifo0_init();
    can_filter_fifo1_init();
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_RX_FIFO1_MSG_PENDING);
}

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