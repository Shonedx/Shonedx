

#include "Allheaderfile.h"
#include "ticks.h"

//CAN_1 : PA11(RX)   PA12(TX)        CAN_2 : PB12(RX)      PB13(TX)
//ң�����:PA3
//������:PB10(RX)    PB11(TX)


Right_Rocker right_rocker_flag = {0}; //ң������ҡ��ǰ�����ұ�־���ļ���
Left_Rocker left_rocker_flag ={0}; //ң������ҡ��ǰ�����ұ�־���ļ���

int feed=1; //ι��
int left_push_stick,right_push_stick; //ң�������ϣ������Ƹ˱�����ʼ��
/***********ң�����***********/
int start=0;			//��ʼ��־��
int stand=0;			//վ����־��
int crouch=0;    		//���±�־��
int bound=0;			//��Ծ��־��
int stand_end=0;    	//����վ��������������׶εı�־��
int bound_prepare=0;
int bound_time_ctrl=0;
int after_bound=0;

int normal_control=0;	//�������Ʊ�־��
int slow_control=0;

double Leg_Length=15;   //�ȳ�
double X_Offset=0;		//�����x���ϵ�ƫ��ֵ

int reset=1;			//ң����������ʱreset��1 

int main()
{
	delay_init();    
	//uart_init(9600);	//������������
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
 
