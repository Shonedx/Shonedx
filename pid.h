#ifndef __PID_H
#define __PID_H
#include <sys.h>
#include "usart.h"
//#include "robocon.h"
#include "Allheaderfile.h"
#define PWM_MAX 15000
#define PWM_MIN -15000
#define TIAOSPEED 7000
#define QIANTIAOSPEED 8000
 
 enum PID_MODE //pid位置环和速度环的选择
{
    PID_POSITION = 0,
    PID_DELTA    = 1
};

typedef struct
{
    uint8_t mode;           // 控制模式：PID_POSITION 或 PID_DELTA

    float Kp;               // 比例增益
    float Ki;               // 积分增益
    float Kd;               // 微分增益

    float max_out;          // 最大输出
    float max_iout;         // 最大积分输出

    float set;              // 设定值
    float fdb;              // 反馈值

    float out;              // 总输出
    float Pout;             // 比例项输出
    float Iout;             // 积分项输出
    float Dout;             // 微分项输出
    float Dbuf[3];          // 微分项缓存
    float error[3];         // 误差缓存
} PidTypeDef;

typedef struct
{
    int angle;              // 当前角度
    int POS_ABS;            // 绝对角度
    int target_angle;       // 目标角度
	int angle_out;
    short corrent_output;   // 最终输出电流
	
	int speed;				//当前速度
	int target_speed;		//目标速度
	
	int last_ecd; 			//上一次机械角度
	int ecd;				//获得转子机械角度范围是0~8191，对应360度
} MOTOR_3508_MSG;

typedef struct
{
    MOTOR_3508_MSG ID1;				//电机相关结构体
    MOTOR_3508_MSG ID2;
    MOTOR_3508_MSG ID3;
    MOTOR_3508_MSG ID4;
    MOTOR_3508_MSG ID5;
    MOTOR_3508_MSG ID6;
    MOTOR_3508_MSG ID7;
    MOTOR_3508_MSG ID8;
} MOTOR_3508;

typedef struct
{
    PidTypeDef M3508_SPEED_ID1; //速度环pid
    PidTypeDef M3508_SPEED_ID2;
    PidTypeDef M3508_SPEED_ID3;
    PidTypeDef M3508_SPEED_ID4;
    PidTypeDef M3508_SPEED_ID5;
    PidTypeDef M3508_SPEED_ID6;
    PidTypeDef M3508_SPEED_ID7;
    PidTypeDef M3508_SPEED_ID8;

    PidTypeDef M3508_POS_ID1;	//角度有关
    PidTypeDef M3508_POS_ID2;
    PidTypeDef M3508_POS_ID3;
    PidTypeDef M3508_POS_ID4;
    PidTypeDef M3508_POS_ID5;
    PidTypeDef M3508_POS_ID6;
    PidTypeDef M3508_POS_ID7;
    PidTypeDef M3508_POS_ID8;

    PidTypeDef M3508_STAND_ID1;	//stand姿态位置环pid
    PidTypeDef M3508_STAND_ID2;
    PidTypeDef M3508_STAND_ID3;
    PidTypeDef M3508_STAND_ID4;
    PidTypeDef M3508_STAND_ID5;
    PidTypeDef M3508_STAND_ID6;
    PidTypeDef M3508_STAND_ID7;
    PidTypeDef M3508_STAND_ID8;
} PidMsg;						//相关pid结构体

typedef struct
{
	float motorangle1;
	float motorangle2;
	float motorangle3;
	float motorangle4;
	float motorangle5;
	float motorangle6;
	float motorangle7;
	float motorangle8;
}Dog_Motor_Angle;  			//狗腿电机角度

//这三个都在pid.c文件中声明了
extern PidMsg pidmsg;
extern MOTOR_3508 motor_3508;
extern Dog_Motor_Angle Leg_angle;

extern int Stand_ON_OFF; //站立状态出腿收腿标志量（在RC_Command.c中被使用）
extern int Crouch_ON_OFF;//蹲下状态出腿收腿标志量（在RC_Command.c中被使用）
/***********************************/
void ChangeTheGainOfPID_KP_KI_KD(float sp_kp, float sp_ki, float sp_kd, float pos_kp, float pos_ki, float pos_kd); //暂未用，下同
void LegPID_Change(u8 LegId, float sp_kp, float sp_ki, float sp_kd, float pos_kp, float pos_ki, float pos_kd);

//计算电机绝对角度（过零处理）
void Motor_Angle_Cal_1(float T);
void Motor_Angle_Cal_2(float T);
void Motor_Angle_Cal_3(float T);
void Motor_Angle_Cal_4(float T);
void Motor_Angle_Cal_5(float T);
void Motor_Angle_Cal_6(float T);
void Motor_Angle_Cal_7(float T);
void Motor_Angle_Cal_8(float T);



void PID_Init(PidMsg *pid); //pidmsg中的各个结构体成员初始化

float PID_Calc( PidTypeDef *pid,MOTOR_3508_MSG *motor_temp); // 速度环PID公式

float PID_Cal_STAND( PidTypeDef *PID ,float get);//起立姿态位置环

float PID_Cal_POSITION(PidTypeDef *PID, float get);//PID位置环

void Target_Pos_Setting(PidTypeDef *pid,float target); //在AllLeg_Set_angle（ges_cal.c中）被调用过

void SetPoint(PidTypeDef *pid,float target);//IMU模块调用

void PID_Setting(PidTypeDef *pid,float kp,float kd,float ki); //设置KP,KD,KI

void PID_Pos_Setting(PidTypeDef* pid,float kp,float kd,float ki);

void ChangeTheGainOfPID_KP_KI_KD(float sp_kp,float sp_ki,float sp_kd,float pos_kp,float pos_ki,float pos_kd);

void LegPID_Change(u8 LegId,float sp_kp,float sp_kd,float sp_ki,float pos_kp,float pos_kd,float pos_ki);


void feed_dog(void); //看门狗相关
	
	
#endif
