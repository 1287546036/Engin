#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "main.h"
#include "can.h"

#define  CAN1_FILTER_BANK  0    //CAN1起始过滤器组编号
#define  CAN2_FILTER_BANK  14   //CAN2起始过滤器组编号

void CAN_Filter_Init(void);
void can_start(void);
uint8_t canx_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t canx_receive(CAN_HandleTypeDef *hcan, uint16_t *rec_id, uint8_t *buf);
#endif
