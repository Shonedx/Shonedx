#ifndef __MAIN_PARAMS_H
#define __MAIN_PARAMS_H
#include "Allheaderfile.h"
#include "sys.h"

//ע:���ļ�����ͳһ��main�������ı����ŵ�ͷ�ļ������Ը������ļ����ñ������������Ͳ鿴

typedef struct
{
	int forward;  	//ǰ��
	int back;		//����
	int turn_left;	//��ת
	int turn_right;	//��ת
}Left_Rocker;

typedef struct
{
	int forward;  	//ǰ��
	int back;		//����
	int turn_left;	//��ת
	int turn_right;	//��ת
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

extern	int left_x,left_y,right_x,right_y; //RemoteControl��������




#endif
