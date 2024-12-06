#ifndef __MAIN_PARAMS_H
#define __MAIN_PARAMS_H
#include "Allheaderfile.h"
#include "sys.h"

//注:该文件用于统一讲main中声明的变量放到头文件中用以给其他文件调用变量，方便管理和查看

typedef struct
{
	int forward;  	//前进
	int back;		//后退
	int turn_left;	//左转
	int turn_right;	//右转
}Left_Rocker;

typedef struct
{
	int forward;  	//前进
	int back;		//后退
	int turn_left;	//左转
	int turn_right;	//右转
}Right_Rocker;

extern 	int start;
extern 	int stand;
extern  int crouch;
extern  int bound;
extern  int stand_end;
extern  int bound_prepare;
extern  int bound_time_ctrl;
extern  int after_bound;

extern  double Leg_Length;
extern  double X_Offset;

extern  int normal_control;
extern  int slow_control;

extern  int reset;	

extern	Left_Rocker left_rocker_flag;
extern	Right_Rocker right_rocker_flag;

extern	int left_x,left_y,right_x,right_y; //RemoteControl中声明的




#endif
