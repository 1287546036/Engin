#ifndef __DM_MOTOR_H
#define __DM_MOTOR_H

#include "bsp_can.h"
#include "bsp_tim.h"
#include "controller.h"

#define Motor  hcan1

#define DM_MID_STD_ID            0x200
#define DM_MOTOR1_ID             0x01
#define DM_MST_ID1               0x11

#define Pos_MAX 12.5
#define Pos_MIN -12.5
#define Vel_MAX 30
#define Vel_MIN -30
#define Torq_MAX 10
#define Torq_MIN -10

#define DM_KP_MAX 500.0
#define DM_KP_MIN 0.0
#define DM_KD_MAX 5.0
#define DM_KD_MIN 0.0

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPD_MODE			0x200
#define PSI_MODE		  	0x300
        

typedef enum
{
	mit_mode = 1,
	pos_mode = 2,
	spd_mode = 3,
	psi_mode = 4
} mode_e;

// 电机参数
typedef struct
{
	uint8_t read_flag;
	uint8_t write_flag;
	uint8_t save_flag;
	
    float UV_Value;		// 低压保护值
    float KT_Value;		// 扭矩系数
    float OT_Value;		// 过温保护值
    float OC_Value;		// 过流保护值
    float ACC;			// 加速度
    float DEC;			// 减速度
    float MAX_SPD;		// 最大速度
    uint32_t MST_ID;	// 反馈ID
    uint32_t ESC_ID;	// 接收ID
    uint32_t TIMEOUT;	// 超时警报时间
    uint32_t cmode;		// 控制模式
    float  	 Damp;		// 电机粘滞系数
    float    Inertia;	// 电机转动惯量
    uint32_t hw_ver;	// 保留
    uint32_t sw_ver;	// 软件版本号
    uint32_t SN;		// 保留
    uint32_t NPP;		// 电机极对数
    float    Rs;		// 电阻
    float    Ls;		// 电感
    float    Flux;		// 磁链
    float    Gr;		// 齿轮减速比
    float    PMAX;		// 位置映射范围
    float    VMAX;		// 速度映射范围
    float    TMAX;		// 扭矩映射范围
    float    I_BW;		// 电流环控制带宽
    float    KP_ASR;	// 速度环Kp
    float    KI_ASR;	// 速度环Ki
    float    KP_APR;	// 位置环Kp
    float    KI_APR;	// 位置环Ki
    float    OV_Value;	// 过压保护值
    float    GREF;		// 齿轮力矩效率
    float    Deta;		// 速度环阻尼系数
    float 	 V_BW;		// 速度环滤波带宽
    float 	 IQ_cl;		// 电流环增强系数
    float    VL_cl;		// 速度环增强系数
    uint32_t can_br;	// CAN波特率代码
    uint32_t sub_ver;	// 子版本号
	float 	 u_off;		// u相偏置
	float	 v_off;		// v相偏置
	float	 k1;		// 补偿因子1
	float 	 k2;		// 补偿因子2
	float 	 m_off;		// 角度偏移
	float  	 dir;		// 方向
	float	 p_m;		// 电机位置
	float	 x_out;		// 输出轴位置
} esc_inf_t;

// 电机参数设置结构体
typedef struct
{   
		float zero_pos;
		float last_pos;
    float pos_set;
    float vel_set;
    float tor_set;
		float cur_set;
    float kp_set;
    float kd_set;
} motor_ctrl_t;

typedef struct
{

	uint16_t 	ID;
	uint16_t 	ERR;
	float 	POS;        //位置
	float    VEL;        //速度
	float    Torq;       //力矩
	float     T_MOS;      //驱动上MOS的平均温度
	float 	T_Motor;    //电机内部线圈的平均温度
} DM_MotorMeasure_t;

typedef struct
{
		uint16_t id;
		uint16_t mst_id;
    DM_MotorMeasure_t DM_MotorMeasure;
    mode_e mode;
    motor_ctrl_t ctrl;
    esc_inf_t ref;
    PID_t MotorCtrl_Angle;
    PID_t MotorCtrl_Speed;
} DM_Motor_t;



extern DM_Motor_t Motor1;

void DM_Motor_Enable(CAN_HandleTypeDef* hcan, DM_Motor_t *motor);
void DM_enable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);
void DM_disable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);

void DM_Motor_Ctrl_Send(CAN_HandleTypeDef* hcan, DM_Motor_t *motor);
void DM_MIT_Ctrl(CAN_HandleTypeDef *hcan, DM_Motor_t *motor, float pos, float vel, float KP, float KD, float torq);
void DM_Pos_Ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float pos, float vel);
void DM_Spd_Ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float vel);
void DM_Psi_Ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel, float cur);

void read_motor_data(uint16_t id, uint8_t rid);
void read_motor_ctrl_fbdata(uint16_t id);
void write_motor_data(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
void save_motor_data(uint16_t id, uint8_t rid);
void save_pos_zero(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);

void Get_DM_MotorInfo(DM_MotorMeasure_t* dm_motor_t, uint8_t data[8]);






#endif

