

#include "Allheaderfile.h"
#include "ticks.h"

//CAN_1 : PA11(RX)   PA12(TX)        CAN_2 : PB12(RX)      PB13(TX)
//遥控输出:PA3
//陀螺仪:PB10(RX)    PB11(TX)


Right_Rocker right_rocker_flag = {0}; //遥控器右摇杆前后左右标志量的集合
Left_Rocker left_rocker_flag ={0}; //遥控器左摇杆前后左右标志量的集合

int feed=1; //喂狗
int left_push_stick,right_push_stick; //遥控器左上，右上推杆变量初始化
/***********遥控相关***********/
int start=0;			//开始标志量
int stand=0;			//站立标志量
int crouch=0;    		//蹲下标志量
int bound=0;			//跳跃标志量
int stand_end=0;    	//代表站立结束进入操作阶段的标志量
int bound_prepare=0;
int bound_time_ctrl=0;
int after_bound=0;

int normal_control=0;	//正常控制标志量
int slow_control=0;

double Leg_Length=15;   //腿长
double X_Offset=0;		//足端在x轴上的偏移值

int reset=1;			//遥控器在中央时reset赋1 

int main()
{
	delay_init();    
	//uart_init(9600);	//调试陀螺仪用
	uart3_init(9600);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	LED_Init();					//LED 
	remote_control_init();
	KEY_Init(); 				

	IWDG_Init(4,500);  //Tout=((4*2^prer)*rlr)/32 (ms)  1s
	TIM5_Init(5000-1,84-1);				//5ms  
	TIM4_Int_Init(65535,0);
	
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_2tq,CAN_BS1_4tq,6,CAN_Mode_Normal);//CAN1 1Mbps 
	CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_2tq,CAN_BS1_4tq,6,CAN_Mode_Normal);//CAN2 1Mbps 
	PID_Init(&pidmsg);	
	
	while(1)
	{
		Controller();
	}
}
 
