

#include "Allheaderfile.h"
#include "ticks.h"

//LEGS_COMPENSATE legs_compensate;
//GaitParams params;
extern double trot_time; //ȡ��ges_cal.c
extern double walk_time;
//Ft=168Mhz/4*ʱ�ӷ�Ƶ



void TIM4_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  ///ʹ��TIM4ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = 65535; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=0;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //����ʱ��4�����ж�
	TIM_Cmd(TIM4,DISABLE); //ʹ�ܶ�ʱ��4
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; //��ʱ��4�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02; //��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x02; //�����ȼ�2
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
	//TfΪռ�ձȣ�x_targetΪx��Ŀ��ֵ��z_targetΪz��Ŀ�꣬x0Ϊx����ʼ���꣬z0Ϊz����ʼ���꣬
	//xv0Ϊ�ڶ���̧��ǰ���ȵ�˲���ٶȣ�zfΪ֧�����z���꣬directionΪ��ط���Ĳ��� ��changeΪ�ı䲽̬״̬��ز���
	if //������
		(	
			left_rocker_flag.forward==1
			&&left_rocker_flag.turn_left==0
			&&left_rocker_flag.turn_right==0
			&&left_rocker_flag.back==0
			&&right_rocker_flag.forward==1
		)
	{					
		ChangeTheGainOfPID_KP_KI_KD(8.2,0.3,4.81,9.1,0.3,1.82);
		Dir_Set(1); //1��ǰ��
		Gait(0.5,5.5,3.5,-0,0,1,0,0,0,Leg_Length,0); 
	}
	if //����״̬����ҡ�˺�������
		(	
			left_rocker_flag.forward==1
			&&left_rocker_flag.turn_left==0
			&&left_rocker_flag.turn_right==0
			&&left_rocker_flag.back==0
			&&right_rocker_flag.back==1
		)
	{					
		ChangeTheGainOfPID_KP_KI_KD(8.7,0.3,4.81,9.1,0.3,2.82);
		Dir_Set(1); //1��ǰ��
		Gait(0.5,2.5,1.5,-1,0,0,0,0,1,Leg_Length,1);
	}
	if //������ת���������ܶ��ɣ�
		(	
				left_rocker_flag.forward==1
				&&left_rocker_flag.turn_left==1
				&&left_rocker_flag.turn_right==0
				&&left_rocker_flag.back==0
				&&right_rocker_flag.forward==1
			//����ת��
		)
	{					
		ChangeTheGainOfPID_KP_KI_KD(8.2,0.3,4.81,9.1,0.3,1.82);
		Dir_Set(2); //2�ǲ�����ת
		Gait(0.5,4.5,3.5,-1,0,1,0,0,0,Leg_Length,0); 
	}
	else if  //������ת 
		(	
				left_rocker_flag.forward==1
				&&left_rocker_flag.turn_left==1
				&&left_rocker_flag.turn_right==0
				&&left_rocker_flag.back==0
				&&right_rocker_flag.back==1		
		)//����ת��
	{					
		ChangeTheGainOfPID_KP_KI_KD(8.2,0.3,4.81,9.1,0.3,1.82);
		Dir_Set(2); //2�ǲ�����ת
		Gait(0.5,2.5,1.5,-1,0,1,0,0,1,Leg_Length,1); 
	}
	if //������ת 
		(	
				left_rocker_flag.forward==1
				&&left_rocker_flag.turn_right==1
				&&left_rocker_flag.turn_left==0
				&&left_rocker_flag.back==0
				&&right_rocker_flag.forward==1		
		)//����ת��
	{					
		ChangeTheGainOfPID_KP_KI_KD(8.2,0.3,4.81,9.1,0.3,1.82);
		Dir_Set(3); //3�ǲ�����ת
		Gait(0.5,4.5,3.5,-1,0,1,0,0,0,Leg_Length,0); 
	}
	else if  //������ת 
		(	
				left_rocker_flag.forward==1
				&&left_rocker_flag.turn_right==1
				&&left_rocker_flag.turn_left==0
				&&left_rocker_flag.back==0
				&&right_rocker_flag.back==1		
		)//����ת��
	{					
		ChangeTheGainOfPID_KP_KI_KD(8.2,0.3,4.81,9.1,0.3,1.82);
		Dir_Set(3); //3�ǲ�����ת
		Gait(0.5,2.5,1.5,-1,0,1,0,0,1,Leg_Length,1); 
	}
	
/******************************************************/
	if //ԭ����ת
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
		Dir_Set(22); //22��ԭ����ת
		Gait(0.5,4.5,3.5,-1,0,1,0,0,0,Leg_Length,0); 
	}
	
	else if //ԭ����ת 
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
		Dir_Set(33); //33��ԭ����ת
		Gait(0.5,4.5,3.5,-1,0,1,0,0,0,Leg_Length,0); 
	}

//	else if(flag_forward&&flag_left==1)//ֻ��ǰ��״̬�²������������ת��
//	{
//	legs_compensate.ALL.Forward_compensate_angle=180;
//	legs_compensate.ALL.Behind_compensate_angle=300;
//					
//	ChangeTheGainOfPID_KP_KI_KD(8.2,0.3,4.81,9.1,0.3,1.82);
//	Dir_Set(4);
//	Gait(0.5,15,7.5,-1,0,1,0,params,0,0); 
//	dir=0;
//	}
//	else if((flag_forward&&flag_right)==1)//ֻ��ǰ��״̬�²��������Ҳ���ת��
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
//	else if(flag_situ_left == 1)//ԭ��ת��
//	{
//	legs_compensate.ALL.Forward_compensate_angle=0;//ǰ����תƫ����-10
//	legs_compensate.ALL.Behind_compensate_angle=200;//������תƫ����-110
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
//	else if(reset == 1 )  //ң����������ʱreset��1 
//	{
//	legs_compensate.ALL.Forward_compensate_angle=0;
//	
//	legs_compensate.ID1.Behind_compensate_angle=0;//1
//	legs_compensate.ID3.Behind_compensate_angle=0;//��������
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
//	trot_time=1;//��ʼ�� �ù���λ�� ����Ϊ��վ���ʱ��
//	dir=3;
//	}
//	}
	if(reset == 1 )  //ң����������ʱreset��1 
	{

		Stand(X_Offset,Leg_Length); //����վ������
		trot_time=1;//��ʼ�� �ù���λ�� ����Ϊ��վ���ʱ��
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


