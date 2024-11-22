#include "bsp_can.h"
#include "RM_Motor.h"
#include "DM_Motor_Ctrl.h"




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


    can_filter_st.SlaveStartFilterBank = CAN2_FILTER_BANK;

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




uint8_t canx_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{	
	CAN_TxHeaderTypeDef	tx_header;
	
	tx_header.StdId = id;
	tx_header.ExtId = 0;
	tx_header.IDE   = 0;
	tx_header.RTR   = 0;
	tx_header.DLC   = len;
  /*找到空的发送邮箱，把数据发送出去*/
	if(HAL_CAN_AddTxMessage(hcan, &tx_header, data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) {
		if(HAL_CAN_AddTxMessage(hcan, &tx_header, data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) {
			HAL_CAN_AddTxMessage(hcan, &tx_header, data, (uint32_t*)CAN_TX_MAILBOX2);
    }
  }
  return 0;
}

/**
************************************************************************
* @brief:      	canx_bsp_receive(CAN_HandleTypeDef *hcan, uint8_t *buf)
* @param:       hcan: CAN句柄
* @param[out]:  rec_id: 	接收到数据的CAN设备ID
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t canx_receive(CAN_HandleTypeDef *hcan, uint16_t *rec_id, uint8_t *buf)
{	
	CAN_RxHeaderTypeDef rx_header;
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, buf) == HAL_OK)
	{
		*rec_id = rx_header.StdId;
		return rx_header.DLC; //接收数据长度
	}
	else
		return 0;
}

/**
 * @description  : CAN错误中断回调函数，在这里重新打开接收中断
 * @param         {CAN_HandleTypeDef} *hcan
 * @return        {*}
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
  }
 if (hcan->Instance == CAN2)
 {
   HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);
 }
}

/**
************************************************************************
* @brief:      	can_start(void)
* @param:       void
* @details:    	初始化
************************************************************************
**/
void can_start(void)
{
    can_filter_init();
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_RX_FIFO1_MSG_PENDING);
}


