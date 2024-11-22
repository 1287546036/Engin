#ifndef PID_DATA_H
#define PID_DATA_H


///*        _speed_pid              */
//#define   _speed_KP     
//#define   _speed_KI     
//#define   _speed_KD     
//#define   _speed_MAXout 
//#define   _speed_MAXIout

///*        _angle_pid              */
//#define   _angle_KP     
//#define   _angle_KI     
//#define   _angle_KD     
//#define   _angle_MAXout 
//#define   _angle_MAXIout



/*          chassis_speed_pid			*/
#define     chassis_speed_KP        3.0f
#define     chassis_speed_KI        0.1f
#define     chassis_speed_KD        0.0f
#define     chassis_speed_MAXout    10000.0f
#define     chassis_speed_MAXIout   1000.0f

/*			lifting_angle_pid	    	*/
#define   lifting_angle_KP        4.0f
#define   lifting_angle_KI        0.1f
#define   lifting_angle_KD        0.01f
#define   lifting_angle_MAXout    3500.0f
#define   lifting_angle_MAXIout   500.0f


/*			lifting_speed_pid	    	*/
#define   lifting_speed_KP        3.0f
#define   lifting_speed_KI        0.1f
#define   lifting_speed_KD        0.0f
#define   lifting_speed_MAXout    16000.0f
#define   lifting_speed_MAXIout   10000.0f

/*      MechanicalArm_damiao_set        */ 
#define 	Motor1_vel_set        0.0f;
#define 	Motor1_zero_pos       0.0f;
#define 	Motor1_cur_set        0.00f;
#define 	Motor1_tor_set        0.00f;
#define 	Motor1_kp_set         1.0f;
#define 	Motor1_kd_set         1.0f;
#define 	Motor1_PMAX           12.5f;// 位置映射范围
#define 	Motor1_VMAX           0.5f;// 速度映射范围
#define 	Motor1_TMAX           10.0f;// 扭矩映射范围
    
#define 	Motor2_vel_set        0.0f;
#define 	Motor2_zero_pos       0.0f;
#define 	Motor2_cur_set        0.00f;
#define 	Motor2_tor_set        0.00f;
#define 	Motor2_kp_set         1.0f;
#define 	Motor2_kd_set         1.0f;
#define 	Motor2_PMAX           12.5f;// 位置映射范围
#define 	Motor2_VMAX           0.5f;// 速度映射范围
#define 	Motor2_TMAX           10.0f;// 扭矩映射范围
    
#define 	Motor3_vel_set        0.0f;
#define 	Motor3_zero_pos       0.0f;
#define 	Motor3_cur_set        0.00f;
#define 	Motor3_tor_set        0.00f;
#define 	Motor3_kp_set         1.0f;
#define 	Motor3_kd_set         1.0f;
#define 	Motor3_PMAX           12.5f;// 位置映射范围
#define 	Motor3_VMAX           0.5f;// 速度映射范围
#define 	Motor3_TMAX           10.0f;// 扭矩映射范围
    
#define 	Motor4_vel_set        0.0f;
#define 	Motor4_zero_pos       0.0f;
#define 	Motor4_cur_set        0.00f;
#define 	Motor4_tor_set        0.00f;
#define 	Motor4_kp_set         1.0f;
#define 	Motor4_kd_set         1.0f;
#define 	Motor4_PMAX           12.5f;// 位置映射范围
#define 	Motor4_VMAX           0.5f;// 速度映射范围
#define 	Motor4_TMAX           10.0f;// 扭矩映射范围


#endif
