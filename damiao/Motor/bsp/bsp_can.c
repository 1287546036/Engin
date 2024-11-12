#include "bsp_can.h"
#include "RM_Motor.h"
#include "DM_Motor_Ctrl.h"
/**
 * @description  : CAN过滤器配置
 * @return       : 无
 */
void CAN_Filter_Init(void)
{
  CAN_FilterTypeDef CAN_FilterInitStructure;
  
  CAN_FilterInitStructure.FilterActivation     = CAN_FILTER_ENABLE;     //过滤器使能
  CAN_FilterInitStructure.FilterBank           = CAN1_FILTER_BANK;      //CAN1起始过滤器组编号
  CAN_FilterInitStructure.SlaveStartFilterBank = CAN2_FILTER_BANK;
  CAN_FilterInitStructure.FilterFIFOAssignment = CAN_RX_FIFO0;          //过滤器组关联FIFO0
  CAN_FilterInitStructure.FilterIdHigh         = 0x0000;
  CAN_FilterInitStructure.FilterIdLow          = 0x0000;
  CAN_FilterInitStructure.FilterMaskIdHigh     = 0x0000;
  CAN_FilterInitStructure.FilterMaskIdLow      = 0x0000;
  CAN_FilterInitStructure.FilterMode           = CAN_FILTERMODE_IDMASK; //掩码模式
  CAN_FilterInitStructure.FilterScale          = CAN_FILTERSCALE_32BIT; //32位
  
  HAL_CAN_ConfigFilter(&hcan1,&CAN_FilterInitStructure);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);     //开启FIFO0的接收中断
  
 CAN_FilterInitStructure.FilterFIFOAssignment = CAN_RX_FIFO1;
 CAN_FilterInitStructure.SlaveStartFilterBank = CAN2_FILTER_BANK;
 CAN_FilterInitStructure.FilterBank           = CAN2_FILTER_BANK;
 HAL_CAN_ConfigFilter(&hcan2,&CAN_FilterInitStructure);
 HAL_CAN_Start(&hcan2);
 HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);      //开启FIFO1的接收中断
}

/**
 * @description  : 接收FIFO0中断回调函数，处理CAN1接收到的报文
 * @param         {CAN_HandleTypeDef} *hcan
 * @return        {*}
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  uint8_t Rx_Data[8] = {0};
  CAN_RxHeaderTypeDef CAN_RxHeader;

  HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&CAN_RxHeader,Rx_Data);
  if(hcan -> Instance == CAN1)
  {
//		can1_rx_callback();
		switch(CAN_RxHeader.StdId)
		{
			case(DM_MST_ID1):
				Get_DM_MotorInfo(&Motor1.DM_MotorMeasure,Rx_Data);
        receive_motor_data(&Motor1, Rx_Data);
			  break;
			default:
				break;
		}
  }
}

/**
 * @description  : 接收FIFO1中断回调函数，处理CAN2接收到的报文
 * @param         {CAN_HandleTypeDef} *hcan
 * @return        {*}
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
 uint8_t Rx_Data[8] = {0};
 CAN_RxHeaderTypeDef CAN_RxHeader;

 HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&CAN_RxHeader,Rx_Data);
 if(hcan -> Instance == CAN2)
 {
   switch(CAN_RxHeader.StdId)
   {			
     default:
       break;
   }
 }
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
