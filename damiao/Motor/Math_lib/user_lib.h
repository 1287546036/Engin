/**
 ******************************************************************************
 * @file	 user_lib.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2021/2/18
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _USER_LIB_H
#define _USER_LIB_H
#include "stdint.h"
#include "main.h"

//#include "cmsis_os.h"

enum
{
    CHASSIS_DEBUG = 1,
    GIMBAL_DEBUG,
    INS_DEBUG,
    RC_DEBUG,
    IMU_HEAT_DEBUG,
    SHOOT_DEBUG,
    AIMASSIST_DEBUG,
};

extern uint8_t GlobalDebugMode;

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

/* math relevant */
/* radian coefficient */
#ifndef RADIAN_COEF
#define RADIAN_COEF 57.295779513f
#endif

/* circumference ratio */
#ifndef PI
#define PI 3.14159265354f
#endif

#define VAL_LIMIT(val, min, max) \
    do                           \
    {                            \
        if ((val) <= (min))      \
        {                        \
            (val) = (min);       \
        }                        \
        else if ((val) >= (max)) \
        {                        \
            (val) = (max);       \
        }                        \
    } while (0)

#define ANGLE_LIMIT_360(val, angle)     \
    do                                  \
    {                                   \
        (val) = (angle) - (int)(angle); \
        (val) += (int)(angle) % 360;    \
    } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
    do                              \
    {                               \
        if ((val) > 180)            \
            (val) -= 360;           \
    } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

typedef struct
{
    float input;        //闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗
    float out;          //闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻敓锟�
    float min_value;    //闁跨喓娼鹃崙銈嗗闁跨喐鏋婚幏宄扮毈閸婏拷
    float max_value;    //闁跨喓娼鹃崙銈嗗闁跨喐鏋婚幏鐑芥晸鐞涙鎷�
    float frame_period; //閺冨爼鏁撻弬銈嗗闁跨喐鏋婚幏锟�
} ramp_function_source_t;

typedef __packed struct
{
    uint16_t Order;
    uint32_t Count;

    float *x;
    float *y;

    float k;
    float b;

    float StandardDeviation;

    float t[4];
} Ordinary_Least_Squares_t;

//闁煎浜滈悾鐐▕婢跺﹥褰ラ梺顐ゅ枎缁辨垿寮敓锟�
float Sqrt(float x);

//閺傛粓鏁撻弬銈嗗闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹峰嘲顫愰柨鐔告灮閹凤拷
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);
//閺傛粓鏁撻弬銈嗗闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗
float ramp_calc(ramp_function_source_t *ramp_source_type, float input);

//闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗
float abs_limit(float num, float Limit);
//闁跨喎褰ㄩ弬顓炲殩閹风兘鏁撻弬銈嗗娴ｏ拷
float sign(float value);
//闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗
float float_deadband(float Value, float minValue, float maxValue);
// int26闁跨喐鏋婚幏鐑芥晸閺傘倖瀚�
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//闁跨喓娼鹃崙銈嗗闁跨喐鏋婚幏鐑芥晸閺傘倖瀚�
float float_constrain(float Value, float minValue, float maxValue);
//闁跨喓娼鹃崙銈嗗闁跨喐鏋婚幏鐑芥晸閺傘倖瀚�
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//瀵邦亪鏁撻弬銈嗗闁跨喓娼鹃崙銈嗗闁跨喐鏋婚幏鐑芥晸閺傘倖瀚�
float loop_float_constrain(float Input, float minValue, float maxValue);
//闁跨喕顫楃拋瑙勫 闁跨喐鏋婚幏鐑芥晸閻偄鍤栭幏锟� 180 ~ -180
float theta_format(float Ang);

int float_rounding(float raw);

//闁跨喐鏋婚幏鐑芥晸妤楁椽娼婚幏宄扮础闁跨喐鏋婚幏铚傝礋-PI~PI
#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order);
void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS);
float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS);

float Saturate_Float(float in,float min,float max);
int Saturate_Int(int in,int min,int max);

float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x,float x_min, float x_max, int bits);

#endif
