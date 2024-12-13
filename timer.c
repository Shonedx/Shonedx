

#include "Allheaderfile.h"
#include "ticks.h"

//LEGS_COMPENSATE legs_compensate;
//GaitParams params;
extern double trot_time; //取自ges_cal.c
extern double walk_time;
//Ft=168Mhz/4*时钟分频



void TIM4_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  ///使能TIM4时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = 65535; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=0;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //允许定时器4更新中断
	TIM_Cmd(TIM4,DISABLE); //使能定时器4
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; //定时器4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02; //抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x02; //子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

void TIM4_IRQHandler()
{  
		if(TIM_GetITStatus(TIM4, TIM_IT_Update) !=RESET)
		{
			if(normal_control==1)
			{
				Normal_Ctrl();
			}
			else if(slow_control==1)
			{
				Slow_Ctrl();
			}

		     
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
			
	}
}


void Normal_Ctrl(void)
{
	//Tf为占空比，x_target为x轴目标值，z_target为z轴目标，x0为x轴起始坐标，z0为z轴起始坐标，
	//xv0为摆动相抬腿前的腿的瞬间速度，zf为支撑相的z坐标，direction为相关方向的参数 ，change为改变步态状态相关参数
	if //正常跑
		(	
			left_rocker_flag.forward==1
			&&left_rocker_flag.turn_left==0
			&&left_rocker_flag.turn_right==0
			&&left_rocker_flag.back==0
			&&right_rocker_flag.forward==1
		)
	{					
		ChangeTheGainOfPID_KP_KI_KD(8.2,0.3,4.81,9.1,0.3,1.82);
		Dir_Set(1); //1是前进
		Gait(0.5,5.5,3.5,-0,0,1,0,0,0,Leg_Length,0); 
	}
	if //慢跑状态，右摇杆后拉即可
		(	
			left_rocker_flag.forward==1
			&&left_rocker_flag.turn_left==0
			&&left_rocker_flag.turn_right==0
			&&left_rocker_flag.back==0
			&&right_rocker_flag.back==1
		)
	{					
		ChangeTheGainOfPID_KP_KI_KD(8.7,0.3,4.81,9.1,0.3,2.82);
		Dir_Set(1); //1是前进
		Gait(0.5,2.5,1.5,-1,0,0,0,0,1,Leg_Length,1);
	}
	if //差速左转（奔跑慢跑都可）
		(	
				left_rocker_flag.forward==1
				&&left_rocker_flag.turn_left==1
				&&left_rocker_flag.turn_right==0
				&&left_rocker_flag.back==0
				&&right_rocker_flag.forward==1
			//奔跑转弯
		)
	{					
		ChangeTheGainOfPID_KP_KI_KD(8.2,0.3,4.81,9.1,0.3,1.82);
		Dir_Set(2); //2是差速左转
		Gait(0.5,4.5,3.5,-1,0,1,0,0,0,Leg_Length,0); 
	}
	else if  //差速左转 
		(	
				left_rocker_flag.forward==1
				&&left_rocker_flag.turn_left==1
				&&left_rocker_flag.turn_right==0
				&&left_rocker_flag.back==0
				&&right_rocker_flag.back==1		
		)//慢跑转弯
	{					
		ChangeTheGainOfPID_KP_KI_KD(8.2,0.3,4.81,9.1,0.3,1.82);
		Dir_Set(2); //2是差速左转
		Gait(0.5,2.5,1.5,-1,0,1,0,0,1,Leg_Length,1); 
	}
	if //差速右转 
		(	
				left_rocker_flag.forward==1
				&&left_rocker_flag.turn_right==1
				&&left_rocker_flag.turn_left==0
				&&left_rocker_flag.back==0
				&&right_rocker_flag.forward==1		
		)//奔跑转弯
	{					
		ChangeTheGainOfPID_KP_KI_KD(8.2,0.3,4.81,9.1,0.3,1.82);
		Dir_Set(3); //3是差速右转
		Gait(0.5,4.5,3.5,-1,0,1,0,0,0,Leg_Length,0); 
	}
	else if  //差速右转 
		(	
				left_rocker_flag.forward==1
				&&left_rocker_flag.turn_right==1
				&&left_rocker_flag.turn_left==0
				&&left_rocker_flag.back==0
				&&right_rocker_flag.back==1		
		)//慢跑转弯
	{					
		ChangeTheGainOfPID_KP_KI_KD(8.2,0.3,4.81,9.1,0.3,1.82);
		Dir_Set(3); //3是差速右转
		Gait(0.5,2.5,1.5,-1,0,1,0,0,1,Leg_Length,1); 
	}
	
/******************************************************/
	if //原地左转
		(	
				left_rocker_flag.forward==0
				&&left_rocker_flag.turn_left==0
				&&left_rocker_flag.turn_right==0
				&&left_rocker_flag.back==0
				&&right_rocker_flag.turn_left==1
				&&right_rocker_flag.turn_right==0
		)
	{					
		ChangeTheGainOfPID_KP_KI_KD(8.2,0.3,4.81,9.1,0.3,1.82);
		Dir_Set(22); //22是原地左转
		Gait(0.5,4.5,3.5,-1,0,1,0,0,0,Leg_Length,0); 
	}
	
	else if //原地右转 
		(	
				left_rocker_flag.forward==0
				&&left_rocker_flag.turn_right==0
				&&left_rocker_flag.turn_left==0
				&&left_rocker_flag.back==0
				&&right_rocker_flag.turn_right==1
				&&right_rocker_flag.turn_left==0
		)
	{					
		ChangeTheGainOfPID_KP_KI_KD(8.2,0.3,4.81,9.1,0.3,1.82);
		Dir_Set(33); //33是原地右转
		Gait(0.5,4.5,3.5,-1,0,1,0,0,0,Leg_Length,0); 
	}

//	else if(flag_forward&&flag_left==1)//只有前进状态下才允许向左差速转弯
//	{
//	legs_compensate.ALL.Forward_compensate_angle=180;
//	legs_compensate.ALL.Behind_compensate_angle=300;
//					
//	ChangeTheGainOfPID_KP_KI_KD(8.2,0.3,4.81,9.1,0.3,1.82);
//	Dir_Set(4);
//	Gait(0.5,15,7.5,-1,0,1,0,params,0,0); 
//	dir=0;
//	}
//	else if((flag_forward&&flag_right)==1)//只有前进状态下才允许向右差速转弯
//	{
//	legs_compensate.ALL.Forward_compensate_angle=180;
//	legs_compensate.ALL.Behind_compensate_angle=300;
//	
//	ChangeTheGainOfPID_KP_KI_KD(8.2,0.3,4.81,9.1,0.3,1.82);
//	Dir_Set(6);
//	Gait(0.5,15,7.5,-1,0,1,0,params,0,0); 
//	dir=0;
//	}
//				
//								
//	else if(flag_situ_left == 1)//原地转弯
//	{
//	legs_compensate.ALL.Forward_compensate_angle=0;//前腿旋转偏移量-10
//	legs_compensate.ALL.Behind_compensate_angle=200;//后退旋转偏移量-110
//	ChangeTheGainOfPID_KP_KI_KD(8.7,0.3,4.81,9.1,0.3,2.82);
//	Dir_Set(44);
//	Gait(0.5,7,5,-1,0,1,0,params,2,0);
//	dir=2;
//	}
//				
//	else if(flag_situ_right == 1)
//	{
//	legs_compensate.ALL.Forward_compensate_angle=0;
//	legs_compensate.ALL.Behind_compensate_angle=200;
//	ChangeTheGainOfPID_KP_KI_KD(8.7,0.3,4.81,9.1,0.3,2.82);
//	
//	Dir_Set(66);
//	Gait(0.5,7,5,-1,0,0,0,params,1,0);
//	dir=1;
//	}


//				
//	else if(reset == 1 )  //遥控器在中央时reset赋1 
//	{
//	legs_compensate.ALL.Forward_compensate_angle=0;
//	
//	legs_compensate.ID1.Behind_compensate_angle=0;//1
//	legs_compensate.ID3.Behind_compensate_angle=0;//右四修正
//	legs_compensate.ID1.Forward_compensate_angle=0;
//	legs_compensate.ID2.Forward_compensate_angle=0;
//	legs_compensate.ID3.Forward_compensate_angle=0;
//	legs_compensate.ID4.Forward_compensate_angle=0;
//	if((trot_time<=0.9999||trot_time>=1.0001)&&dir==0)//0.1801--1.1803
//	{
//	 ChangeTheGainOfPID_KP_KI_KD(8.7,0.3,4.81,9.1,0.3,2.82);
//	Gait(0.5,7,5,-1,0,0,0,0,1);
//	}
//	else if((trot_time<=0.9999||trot_time>=1.0001)&&dir==2)
//	{
//	Gait(0.5,0,5,-1,0,0,0,params,2,0);
//	}
//	else if((trot_time<=0.9999||trot_time>=1.0001)&&dir==1)
//	{
//	Gait(0.5,0,5,-1,0,0,0,params,1,0);
//	}
//	if(trot_time>0.9999&&trot_time<1.0001&&dir!=3)
//	{
//	Leg_angle.motorangle1=0-legs_compensate.ALL.Forward_compensate_angle; 	  
//	Leg_angle.motorangle2=0+legs_compensate.ALL.Forward_compensate_angle;	
//	Leg_angle.motorangle3=0-legs_compensate.ALL.Forward_compensate_angle;
//	Leg_angle.motorangle4=0+legs_compensate.ALL.Forward_compensate_angle;
//	Leg_angle.motorangle5=0-legs_compensate.ALL.Behind_compensate_angle;
//	Leg_angle.motorangle6=0+legs_compensate.ALL.Behind_compensate_angle;
//	Leg_angle.motorangle7=0-legs_compensate.ALL.Behind_compensate_angle;
//	Leg_angle.motorangle8=0+legs_compensate.ALL.Behind_compensate_angle;
//	trot_time=1;//初始化 让狗子位置 参数为刚站起的时候
//	dir=3;
//	}
//	}
	if(reset == 1 )  //遥控器在中央时reset赋1 
	{

		Stand(X_Offset,Leg_Length); //正常站立函数
		trot_time=1;//初始化 让狗子位置 参数为刚站起的时候
		walk_time=1;
	}
}



void Slow_Ctrl(void)
{
	if(left_y>300&&bound==0) 
		bound=1;
	if(after_bound==0)
		Stand(X_Offset,Leg_Length);
	else if(after_bound==1)
		Stand_After_Bound(X_Offset,Leg_Length);
}


