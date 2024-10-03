#include "lifting.h"

//////////////////////tx


void CAN_cmd_lifting(int16_t motor[])
{
	  uint8_t chassis_can_send_data[8]; 
    CAN_TxHeaderTypeDef chassis_tx_message;
    uint32_t send_mail_box;
	
    chassis_tx_message.StdId=CAN_AUXILIARY_ALL_ID;//0x1ff
    chassis_tx_message.IDE=CAN_ID_STD;
    chassis_tx_message.RTR=CAN_RTR_DATA;
    chassis_tx_message.DLC=0x08;
	
    chassis_can_send_data[0]=motor[5]>>8;
    chassis_can_send_data[1]=motor[5];
    chassis_can_send_data[2]=motor[6]>>8;
    chassis_can_send_data[3]=motor[6];
    chassis_can_send_data[4]=motor[7]>>8;
    chassis_can_send_data[5]=motor[7];
    chassis_can_send_data[6]=motor[8]>>8;
    chassis_can_send_data[7]=motor[8];
    
    HAL_CAN_AddTxMessage(&hcan1,&chassis_tx_message,chassis_can_send_data,&send_mail_box);
	
	  HAL_Delay(1);
}