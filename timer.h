#ifndef __TIMER_H
#define __TIMER_H

#include <sys.h>	 


void TIM4_Int_Init(u16 arr,u16 psc);
extern int feed,flag_back,flag_forward,flag_right,flag_left,flag_jump_init,flag_jump,flag_stand,reset,slow_forward;
extern int flag_situ_left,flag_situ_right;
extern int normal_control,slow_control,high_control;
extern int left_pingdong,right_pingdong;

void Normal_Ctrl(void);
void Slow_Ctrl(void);
void High_Ctrl(void);
#endif
