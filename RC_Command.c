/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "stm32f4xx.h"
#include "Allheaderfile.h"
#include "math.h"

extern int left_push_stick, right_push_stick; //左上，右上推杆


/*		LPS											RPS			*/
/*	上 LPS=2 		 						上RPS=1 // 奔跑		*/
/*上电前遥杆要拨到这里*/	
/*	中 LPS=3		 						中RPS=3 //拨到这是走	*/
/*直立状态*/
/*	下 LPS=1		 						下RPS=2 //拨到这是跳	*/
/*运动状态*/


void Controller (void)
{
	if(LPS==2)
	{
		M3508_ALL_ZERO_SET();
//start 标志量		
		start=0;
//stand 相关		
		stand=0;
		stand_end=0;
//bound 相关
		bound_time_ctrl=0;
//控制收腿出腿的有关变量，在pid.c中初始化		
		Stand_ON_OFF=1; 

	}
	else if(LPS==3)
	{
		start=0;
		stand=1;
		
	}
	else if(LPS==1)
	{
		stand=2;

	}
	
	if(RPS==1)
	{
		crouch=0;
		bound_prepare=0;
		normal_control=1; //正常控制标志量，置一说明打开了Normal_Action
		slow_control=0;
		Normal_Action();
	}
	else if(RPS==3)
	{
		after_bound=0;
		bound_time_ctrl=0;
		bound_prepare=0;
		
		crouch=1;
	}
	else if(RPS==2)
	{
		crouch=0;
		bound_prepare=1;
		normal_control=0;
		slow_control=1;

	}
}
void Normal_Action(void)
{
	//右摇杆有关操作，右摇杆下拉视为慢跑状态，其他操作为默认奔跑状态
	if(right_y <= -230 && RPS == 1) 
	{	
		right_rocker_flag.back=1;	//行走步态标志位置一
		right_rocker_flag.forward = 0;  // 默认步态标志位置零
	}
	else //遥控器右遥杆未下拉，此时即默认状态
	{
		right_rocker_flag.back=0;	//行走步态标志位置零
		right_rocker_flag.forward = 1;  //默认步态标志位置一
	}
/***********************************************/	
	//前进
	if (left_y >= 230 && abs(left_x) <= 330 && RPS == 1) 
	{	reset = 0;  //重置标志置零
		left_rocker_flag.back=0;	//后退置零
		left_rocker_flag.forward = 1;  // 前进
		left_rocker_flag.turn_left=left_rocker_flag.turn_right=0; 	//左右转置零
	}
	// 差速左转
	else if (left_x <= -330 &&left_y >= 230 && RPS == 1) 
	{
		reset = 0;  //重置标志置零
		left_rocker_flag.turn_left=1; 	//左转置一
		left_rocker_flag.back=0;	//后退置零
		left_rocker_flag.forward = 1;  // 前进置一
		left_rocker_flag.turn_right=0; 	//右转置零
	}
	// 差速右转
	else if (left_x >= 330 &&left_y >= 230 && RPS == 1) {
		reset = 0;  //重置标志置零
		left_rocker_flag.turn_right=1; 	//右转置一
		left_rocker_flag.back=0;	//后退置零
		left_rocker_flag.forward = 1;  // 前进置一
		left_rocker_flag.turn_left=0; 	//左转置零
	}
	// 原地左转
	else if (right_x <= -100 && RPS == 1 && abs(left_y) <= 330 && abs(left_x) <= 330 ) 
	{
		reset = 0;  //重置标志置零
		//左摇杆部分
		left_rocker_flag.turn_left=0; 	//左转置一
		left_rocker_flag.back=0;	//后退置零
		left_rocker_flag.forward = 0;  // 前进置一
		left_rocker_flag.turn_right=0; 	//右转置零
		//右遥杆部分
		right_rocker_flag.turn_left=1;
		right_rocker_flag.turn_right=0;
	}
	// 原地右转
	else if (right_x >= 100  && RPS == 1 && abs(left_y) <= 330 && abs(left_x) <= 330) 
	{
		reset = 0;  //重置标志置零
		//左摇杆部分
		left_rocker_flag.turn_right=0; 	//右转置一
		left_rocker_flag.back=0;	//后退置零
		left_rocker_flag.forward = 0;  // 前进置一
		left_rocker_flag.turn_left=0; 	//左转置零
		//右遥杆部分
		right_rocker_flag.turn_left=0;
		right_rocker_flag.turn_right=1;
	}
//	if
//		(
//			abs(right_x) <= 330 			 
//		) 
//	{
//		right_rocker_flag.turn_left=0;
//		right_rocker_flag.turn_right=0;
//	}

//// 后退
//else if (left_y <= -550) {
//	flag_back = 1;
//	right_pingdong = left_pingdong = flag_situ_right = flag_situ_left = reset = flag_forward = flag_left = flag_right = 0;
//	TIM_Cmd(TIM4, ENABLE);
//}
// 重置 
	else if 
		(
			abs(left_y) <= 330 
			&& abs(left_x) <= 330
			&&abs(right_x) <= 330 	
		) 
	{
		reset = 1;
		left_rocker_flag.back = left_rocker_flag.forward = left_rocker_flag.turn_left = left_rocker_flag.turn_right = 0;
		right_rocker_flag.turn_left=0;
		right_rocker_flag.turn_right=0;
		TIM_Cmd(TIM4, ENABLE);
	}
	

}
//跳
void Slow_Action(void)
{

}

   
	



