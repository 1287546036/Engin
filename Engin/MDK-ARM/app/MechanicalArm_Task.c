#include "MechanicalArm_Task.h"
#include "main.h"





///*
//4.5 使能
//上电自检完毕后需发送“使能”命令才可以进行控制。“使能”帧属于控制
//帧，帧 ID 如前所述，不同的是数据段，无论处于哪种模式，“使能”的数据定义
//是相同的，如下：
//D[0] D[1] D[2] D[3] D[4] D[5] D[6] D[7]
//0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFC
//4.6 失能
//失能为电机上电默认状态，此时电机三相端子电压波形相同，均为电源电压
//的 50%调制波。“失能”帧属于控制帧，帧 ID 如前所述，数据段定义如下：
//D[0] D[1] D[2] D[3] D[4] D[5] D[6] D[7]
//0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFD
//4.7 保存位置零点
//“保存位置零点”帧属于控制帧，帧 ID 如前所述，数据段定义如下：
//D[0] D[1] D[2] D[3] D[4] D[5] D[6] D[7]
//0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFE
//4.8 清除错误
//电机出现过热等错误时，发送“清除”命令可以清除错误。“清除”帧属于控
//制帧，帧 ID 如前所述，数据段定义如下：
//D[0] D[1] D[3] D[4] D[5] D[6] D[7]
//0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFB
//*/



extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;







//extern UART_HandleTypeDef huart6;

//extern DMA_HandleTypeDef hdma_usart6_rx;

void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver request
    //DMA   SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    //disable DMA
    //DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //memory buffer 1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //  hdma_usart6_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer 
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);
    //enable DMA 
    __HAL_DMA_ENABLE(&hdma_usart6_rx);

}



/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

	
/**
  * @brief          控制模式判断
  * @param[in]      rc_ctrl: 遥控器数据指针
  * @param[out]     control_mode:模式输出
  * @retval         none
  */
 void control_mode_jud(RC_ctrl_t *rc_ctrl,control_mode_t *control_mode);

//remote control data 
//遥控器控制变量
RC_ctrl_t rc_ctrl;




control_mode_t control_mode;
//receive data, 18 bytes one frame, but set 36 bytes 
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

//串口中断
void USART6_IRQHandler(void)
{
    if(huart6.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
    }
    else if(USART6->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart6);

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
    
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);

            }
        }
    }
}



/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

	
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}


/**
  * @brief          Control mode judgment
  * @param[in]      rc_ctrl: remote control data struct point
  * @param[out]     control_mode:model output
  * @retval         none
  */
/**
  * @brief          控制模式判断
  * @param[in]      rc_ctrl: 遥控器数据指针
  * @param[out]     control_mode:模式输出
  * @retval         none
  */
static void control_mode_jud(RC_ctrl_t *rc_ctrl,control_mode_t *control_mode)
{
	    if (rc_ctrl == NULL)
    {
        return;
    }
		typedef enum
	 {
		up=1,
		down=2,
		mid=3
	 }rc_switch;
int switch_right = rc_ctrl->rc.s[1];
int switch_left  = rc_ctrl->rc.s[0];

	if(switch_right ==up)
	{control_mode->mode = all_disability;
//		control_mode->right_ch = NULL;
//	  control_mode->left_ch = NULL;
		}
		else if(switch_right==mid)
      control_mode->mode = chassis_enable;
  else if(switch_right==down)
  {
    if(switch_left==up)
        control_mode->mode = lifting_enable;
    else if(switch_left==mid)
        control_mode->mode = mecharm_enable;
    else if(switch_left==down)
        control_mode->mode = arm_air_enable;
  }
}
/*
  右开关
   1.失能所有
	 2.底盘使能,移动,其他锁定
	 3.锁死底盘(有力停止),开始其他功能
  左开关
   1.抬升前伸移动,机械臂锁死,原底盘右遥杆切换抬升前伸
	 2.除机械臂全部锁死,切换自定义控制器
	 3.气泵打开,除机械臂全部锁死,切换自定义控制器
	 
	 左摇杆一直控制云台,右在抬升前伸时切换
  */
