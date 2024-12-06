#ifndef __IMU_H
#define __IMU_H
#include "Allheaderfile.h"
#include "sys.h"

#define IMU_REC_LEN 50
typedef struct
{
   float roll;     
   float pitch;
   float yaw;
	 float init_roll;
	 float init_pitch;
	 float init_yaw;
}Euler_angle;

extern Detached_Params detached_params_imu;

extern Euler_angle imu_data;

//enum AngleTypes
//{
//	ARoll=0,
//	APitch=1,
//	AYaw=2,
//};

extern float yawwant;
extern float pitchwant;
extern float rollwant;
extern u8 IMU_Control_Flag;
extern float IMU_PID_Intensity;
extern Euler_angle Euler; //usart ÉùÃ÷
void zhijiaoyawchuli(void);
void xiejiaoyawchuli(void);
void IMU_Init(void);
//void IMU_Data_Process(u16 rx_len);
//void uart_rx_angle_init(u32 bound);
void AttitudeControl(float roll_set,float pitch_set,float yaw_set,Detached_Params *State_Detached_Params,int direction);
void standControl(float roll_set,float pitch_set,Detached_Params *State_Detached_Params);
//u8 IMU_WaitAngle(u8 angle_type[2], float takeoff_inclination, float imu_angle_half_range, u8 lock, u8 second_flag);

 

#endif
