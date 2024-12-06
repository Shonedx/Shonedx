#include "Allheaderfile.h"
#include<stdlib.h>
#include "stdio.h"
#include "math.h"

PidMsg pidmsg;
MOTOR_3508 motor_3508;
Dog_Motor_Angle Leg_angle={0};

 PidTypeDef Yaw_PID_Loop; //陀螺仪角相关
 PidTypeDef Roll_PID_Loop;
 PidTypeDef Pitch_PID_Loop;

int pid_pos_limit=6000; 

int Stand_ON_OFF=1; //用来控制站立的出腿和收腿运动，当置一时由初始状态出腿，置-1时收回腿

int Crouch_ON_OFF=1;//用来控制蹲下的出腿和收腿运动，当置一时由初始状态蹲下，置-1时站起

extern int left_push_stick, right_push_stick; //左上，右上推杆

float constrain(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void TIM5_Init(u32 arr,u32 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
	
	
	TIM_Cmd(TIM5,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void TIM5_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)!= RESET)
	{	
		
		if((stand==1|stand==2)&&crouch==0&&bound_prepare==0)
		{
			Leg_Length=15;
			X_Offset=0;
		}
		else if(crouch==1)
		{
			Leg_Length=10;
			X_Offset=-0;
		}
		if(bound_prepare==1&&after_bound==0)
		{	
				Leg_Length=10;
				X_Offset=-3;
				if(bound==1)
				{
					bound_time_ctrl++;
					if(bound_time_ctrl<=50)
					{ 
						Leg_Length=28;
						X_Offset=-8.4;	
					}
					else if(bound_time_ctrl>50)
					{							
						Leg_Length=10;
						X_Offset=-3;
						bound=0;
						after_bound=1;
					}
					
				}
			
		}
		
		if(stand==1&&crouch==0)
		{
			AllLeg_Set_angle(Stand_ON_OFF*1024,Stand_ON_OFF*1176);// offset控制腿高度 ，target控制腿偏向方向 1024 1176
			
		}
		else if(stand==2&&stand_end==0&&crouch==0)
		{	
			stand_end=1;		//这里置一使得该语句只执行一次，代表站立结束进入操作阶段
			start=1;
			Stand_ON_OFF=-1;
			M3508_ALL_ZERO_SET();
		}
		
		if(start==1)
		{
			Motor_Auto_Run();
		}
		feed_dog();
	}
	TIM_ClearITPendingBit(TIM5,TIM_IT_Update); //清除中断标志位
}


void feed_dog(void)
{
	if(feed)
	{
		IWDG_Feed();	
		LED1 =0;
	}
	else
	{
		LED1 = 1;
	}

}

void PID_Init(PidMsg *pid)
{
    // 速度环PID控制器初始化
   
        pid->M3508_SPEED_ID1.Kp =pid->M3508_SPEED_ID2.Kp =pid->M3508_SPEED_ID3.Kp =pid->M3508_SPEED_ID4.Kp =8.2f;//8.2
		pid->M3508_SPEED_ID5.Kp =pid->M3508_SPEED_ID6.Kp =pid->M3508_SPEED_ID7.Kp =pid->M3508_SPEED_ID8.Kp = 8.2f;
		
		pid->M3508_SPEED_ID1.Ki =pid->M3508_SPEED_ID2.Ki =pid->M3508_SPEED_ID3.Ki =pid->M3508_SPEED_ID4.Ki =0.2f;
		pid->M3508_SPEED_ID5.Ki =pid->M3508_SPEED_ID6.Ki =pid->M3508_SPEED_ID7.Ki =pid->M3508_SPEED_ID8.Ki = 0.2f;
        
		pid->M3508_SPEED_ID1.Kd =pid->M3508_SPEED_ID2.Kd =pid->M3508_SPEED_ID3.Kd =pid->M3508_SPEED_ID4.Kd =2.81f;//2.81
		pid->M3508_SPEED_ID5.Kd =pid->M3508_SPEED_ID6.Kd =pid->M3508_SPEED_ID7.Kd =pid->M3508_SPEED_ID8.Kd =2.81f;//2.81
       
		pid->M3508_SPEED_ID1.max_out =pid->M3508_SPEED_ID2.max_out =pid->M3508_SPEED_ID3.max_out =pid->M3508_SPEED_ID4.max_out =PWM_MAX;
		pid->M3508_SPEED_ID5.max_out =pid->M3508_SPEED_ID6.max_out =pid->M3508_SPEED_ID7.max_out =pid->M3508_SPEED_ID8.max_out =PWM_MAX;
		
		pid->M3508_SPEED_ID1.mode =pid->M3508_SPEED_ID2.mode =pid->M3508_SPEED_ID3.mode =pid->M3508_SPEED_ID4.mode =PID_DELTA;
		pid->M3508_SPEED_ID5.mode =pid->M3508_SPEED_ID6.mode =pid->M3508_SPEED_ID7.mode =pid->M3508_SPEED_ID8.mode =PID_DELTA;


    // 位置环PID控制器初始化
		pid->M3508_POS_ID1.Kp =pid->M3508_POS_ID2.Kp =pid->M3508_POS_ID3.Kp =pid->M3508_POS_ID4.Kp =7.1f;
		pid->M3508_POS_ID5.Kp =pid->M3508_POS_ID6.Kp =pid->M3508_POS_ID7.Kp =pid->M3508_POS_ID8.Kp = 7.1f;
		
		pid->M3508_POS_ID1.Ki =pid->M3508_POS_ID2.Ki =pid->M3508_POS_ID3.Ki =pid->M3508_POS_ID4.Ki =0.12f;
		pid->M3508_POS_ID5.Ki =pid->M3508_POS_ID6.Ki =pid->M3508_POS_ID7.Ki =pid->M3508_POS_ID8.Ki = 0.12f;
        
		pid->M3508_POS_ID1.Kd =pid->M3508_POS_ID2.Kd =pid->M3508_POS_ID3.Kd =pid->M3508_POS_ID4.Kd =1.82f;
		pid->M3508_POS_ID5.Kd =pid->M3508_POS_ID6.Kd =pid->M3508_POS_ID7.Kd =pid->M3508_POS_ID8.Kd =1.82f;
		
		pid->M3508_POS_ID1.max_iout =pid->M3508_POS_ID2.max_iout =pid->M3508_POS_ID3.max_iout =pid->M3508_POS_ID4.max_iout =90.0f;
		pid->M3508_POS_ID5.max_iout =pid->M3508_POS_ID6.max_iout =pid->M3508_POS_ID7.max_iout =pid->M3508_POS_ID8.max_iout =90.0f;
       
		pid->M3508_POS_ID1.max_out =pid->M3508_POS_ID2.max_out =pid->M3508_POS_ID3.max_out =pid->M3508_POS_ID4.max_out =pid_pos_limit;
		pid->M3508_POS_ID5.max_out =pid->M3508_POS_ID6.max_out =pid->M3508_POS_ID7.max_out =pid->M3508_POS_ID8.max_out =pid_pos_limit;
		
		pid->M3508_POS_ID1.mode =pid->M3508_POS_ID2.mode =pid->M3508_POS_ID3.mode =pid->M3508_POS_ID4.mode =PID_POSITION;
		pid->M3508_POS_ID5.mode =pid->M3508_POS_ID6.mode =pid->M3508_POS_ID7.mode =pid->M3508_POS_ID8.mode =PID_POSITION;
  

   
	// 站立位置环PID控制器初始化
		pid->M3508_STAND_ID1.Kp = pid->M3508_STAND_ID2.Kp = pid->M3508_STAND_ID3.Kp = pid->M3508_STAND_ID4.Kp = 8.0f;//8
		pid->M3508_STAND_ID5.Kp = pid->M3508_STAND_ID6.Kp = pid->M3508_STAND_ID7.Kp = pid->M3508_STAND_ID8.Kp = 8.0f;

		pid->M3508_STAND_ID1.Ki = pid->M3508_STAND_ID2.Ki = pid->M3508_STAND_ID3.Ki = pid->M3508_STAND_ID4.Ki = 0.2f;
		pid->M3508_STAND_ID5.Ki = pid->M3508_STAND_ID6.Ki = pid->M3508_STAND_ID7.Ki = pid->M3508_STAND_ID8.Ki = 0.2f;

		pid->M3508_STAND_ID1.Kd = pid->M3508_STAND_ID2.Kd = pid->M3508_STAND_ID3.Kd = pid->M3508_STAND_ID4.Kd = 1.82f;
		pid->M3508_STAND_ID5.Kd = pid->M3508_STAND_ID6.Kd = pid->M3508_STAND_ID7.Kd = pid->M3508_STAND_ID8.Kd = 1.82f;

		pid->M3508_STAND_ID1.max_iout = pid->M3508_STAND_ID2.max_iout = pid->M3508_STAND_ID3.max_iout = pid->M3508_STAND_ID4.max_iout = 90.0f;
		pid->M3508_STAND_ID5.max_iout = pid->M3508_STAND_ID6.max_iout = pid->M3508_STAND_ID7.max_iout = pid->M3508_STAND_ID8.max_iout = 90.0f;

		pid->M3508_STAND_ID1.max_out = pid->M3508_STAND_ID2.max_out = pid->M3508_STAND_ID3.max_out = pid->M3508_STAND_ID4.max_out = 200.0f;
		pid->M3508_STAND_ID5.max_out = pid->M3508_STAND_ID6.max_out = pid->M3508_STAND_ID7.max_out = pid->M3508_STAND_ID8.max_out = 200.0f;

		pid->M3508_STAND_ID1.mode = pid->M3508_STAND_ID2.mode = pid->M3508_STAND_ID3.mode = pid->M3508_STAND_ID4.mode = PID_POSITION;
		pid->M3508_STAND_ID5.mode = pid->M3508_STAND_ID6.mode = pid->M3508_STAND_ID7.mode = pid->M3508_STAND_ID8.mode = PID_POSITION;
    
}


float PID_Calc( PidTypeDef *pid,MOTOR_3508_MSG *motor_temp) // 速度环PID公式
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set =  motor_temp->target_speed;
    pid->fdb =  motor_temp->speed;
    pid->error[0] = pid->set - pid->fdb;

    if (pid->mode == PID_POSITION) //位置式
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->Iout = constrain(pid->Iout, -pid->max_iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        pid->out = constrain(pid->out, -pid->max_out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)//增量式
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        pid->out = constrain(pid->out, -pid->max_out, pid->max_out);
    }
    return pid->out;
}
float PID_Cal_STAND( PidTypeDef *PID ,float get)//起立姿态位置环
{
	PID->error[0] = PID->set - get;
	PID->error[1] = PID->error[0] - PID->error[2];
	
	PID->Pout  = PID->Kp * PID->error[0];
	PID->Iout += PID->Ki * PID->error[0];
	PID->Dout  = PID->Kd * PID->error[1];
	
	PID->Iout = (PID->Iout > PID->max_iout)?(PID->max_iout):((PID->Iout < -PID->max_iout)?(-PID->max_iout):(PID->Iout));
	
	PID->out = PID->Pout + PID->Iout + PID->Pout;
	
	PID->out = (PID->out > PID->max_out)?(PID->max_out):((PID->out < -PID->max_out)?(-PID->max_out):(PID->out));//设置PID->LIMIT=200，起始速度不会过快
	
	if(abs(PID->error[0]) <= abs(1))
	{
	  PID->Iout=0;
	  PID->out=0;
	}
	PID->error[2] = PID->error[0];
	return PID->out;
}
float PID_Cal_POSITION(PidTypeDef *PID, float get)//PID位置环
{
	PID->error[0] = PID->set - get;
	PID->error[1] = PID->error[0] - PID->error[2];
	
	PID->Pout  = PID->Kp * PID->error[0];
	PID->Iout += PID->Ki * PID->error[0];
	PID->Dout  = PID->Kd * PID->error[1];
	
	PID->Iout = (PID->Iout > PID->max_iout)?(PID->max_iout):((PID->Iout < -PID->max_iout)?(-PID->max_iout):(PID->Iout));
	
	PID->out = PID->Pout + PID->Iout + PID->Dout;
	
	PID->out = (PID->out > PID->max_out)?(PID->max_out):((PID->out < -PID->max_out)?(-PID->max_out):(PID->out));//设置PID->LIMIT=200，起始速度不会过快
	
	if(abs(PID->error[0]) <= abs(1))
	{
	  PID->Iout=0;
	  PID->out=0;
	}
	PID->error[2] = PID->error[0];
	return PID->out;
}

void Target_Pos_Setting(PidTypeDef *pid,float target)
{
  pid->set = target;
}
void SetPoint(PidTypeDef *pid,float target)
{
	pid->set=target;
}

void PID_Setting(PidTypeDef *pid,float kp,float kd,float ki)
{
  pid->Kp=kp;
	pid->Kd=kd;
	pid->Ki=ki;
}
void PID_Pos_Setting(PidTypeDef* pid,float kp,float kd,float ki)
{
  pid->Kp=kp;
	pid->Ki=ki;
	pid->Kd=kd;
}




//计算绝对角度处理

// 全局变量
uint8_t cnt[8] = {1, 1, 1, 1, 1, 1, 1, 1};

void Motor_Angle_Cal(MOTOR_3508_MSG *motor, float T, uint8_t index)
{
    float res1, res2;
    static float pos_old[8];
	float pos;
    int motor_err, motor_err_err;

    if (cnt[index])
    {
        pos_old[index] = motor->angle;
        cnt[index] = 0;
    }

    pos = motor->angle;
    motor_err = pos - pos_old[index];

    if (motor_err > 0)
    {
        res1 = motor_err - T; // 反转自减
        res2 = motor_err;
    }
    else
    {
        res1 = motor_err + T; // 正转，自加一个周期的角度值 (360)
        res2 = motor_err;
    }

    if (abs(res1) < abs(res2)) // 不管正反转，肯定是转的角度小的那个是真的
    {
        motor_err_err = res1;
    }
    else
    {
        motor_err_err = res2;
    }

    motor->POS_ABS += motor_err_err;
    pos_old[index] = pos;
}

// 调用示例
void Motor_Angle_Cal_1(float T)
{
    Motor_Angle_Cal(&motor_3508.ID1, T, 0);
}

void Motor_Angle_Cal_2(float T)
{
    Motor_Angle_Cal(&motor_3508.ID2, T, 1);
}

void Motor_Angle_Cal_3(float T)
{
    Motor_Angle_Cal(&motor_3508.ID3, T, 2);
}

void Motor_Angle_Cal_4(float T)
{
    Motor_Angle_Cal(&motor_3508.ID4, T, 3);
}

void Motor_Angle_Cal_5(float T)
{
    Motor_Angle_Cal(&motor_3508.ID5, T, 4);
}

void Motor_Angle_Cal_6(float T)
{
    Motor_Angle_Cal(&motor_3508.ID6, T, 5);
}

void Motor_Angle_Cal_7(float T)
{
    Motor_Angle_Cal(&motor_3508.ID7, T, 6);
}

void Motor_Angle_Cal_8(float T)
{
    Motor_Angle_Cal(&motor_3508.ID8, T, 7);
}


void ChangeTheGainOfPID_KP_KI_KD(float sp_kp,float sp_ki,float sp_kd,float pos_kp,float pos_ki,float pos_kd)
{
	/**速度环**/
	PID_Setting(&pidmsg.M3508_SPEED_ID1,sp_kp,sp_kd,sp_ki);
	PID_Setting(&pidmsg.M3508_SPEED_ID2,sp_kp,sp_kd,sp_ki);
	PID_Setting(&pidmsg.M3508_SPEED_ID3,sp_kp,sp_kd,sp_ki);
	PID_Setting(&pidmsg.M3508_SPEED_ID4,sp_kp,sp_kd,sp_ki);
	PID_Setting(&pidmsg.M3508_SPEED_ID5,sp_kp,sp_kd,sp_ki);
	PID_Setting(&pidmsg.M3508_SPEED_ID6,sp_kp,sp_kd,sp_ki);
	PID_Setting(&pidmsg.M3508_SPEED_ID7,sp_kp,sp_kd,sp_ki);
	PID_Setting(&pidmsg.M3508_SPEED_ID8,sp_kp,sp_kd,sp_ki);
	/**位置环**/
	PID_Pos_Setting(&pidmsg.M3508_POS_ID1,pos_kp,pos_kd,pos_ki);
	PID_Pos_Setting(&pidmsg.M3508_POS_ID2,pos_kp,pos_kd,pos_ki);
	PID_Pos_Setting(&pidmsg.M3508_POS_ID3,pos_kp,pos_kd,pos_ki);
	PID_Pos_Setting(&pidmsg.M3508_POS_ID4,pos_kp,pos_kd,pos_ki);
	PID_Pos_Setting(&pidmsg.M3508_POS_ID5,pos_kp,pos_kd,pos_ki);
	PID_Pos_Setting(&pidmsg.M3508_POS_ID6,pos_kp,pos_kd,pos_ki);
	PID_Pos_Setting(&pidmsg.M3508_POS_ID7,pos_kp,pos_kd,pos_ki);
	PID_Pos_Setting(&pidmsg.M3508_POS_ID8,pos_kp,pos_kd,pos_ki);
}
void LegPID_Change(u8 LegId,float sp_kp,float sp_kd,float sp_ki,float pos_kp,float pos_kd,float pos_ki)
{
  switch(LegId)
	{
	  case 0:
			PID_Setting(&pidmsg.M3508_SPEED_ID1,sp_kp,sp_kd,sp_ki);
			PID_Setting(&pidmsg.M3508_SPEED_ID2,sp_kp,sp_kd,sp_ki);
			PID_Pos_Setting(&pidmsg.M3508_POS_ID1,pos_kp,pos_kd,pos_ki);
			PID_Pos_Setting(&pidmsg.M3508_POS_ID2,pos_kp,pos_kd,pos_ki);
		break;
		
		case 1:
			PID_Setting(&pidmsg.M3508_SPEED_ID5,sp_kp,sp_kd,sp_ki);
			PID_Setting(&pidmsg.M3508_SPEED_ID6,sp_kp,sp_kd,sp_ki);
			PID_Pos_Setting(&pidmsg.M3508_POS_ID5,pos_kp,pos_kd,pos_ki);
			PID_Pos_Setting(&pidmsg.M3508_POS_ID6,pos_kp,pos_kd,pos_ki);
		break;
		
		case 2:
			PID_Setting(&pidmsg.M3508_SPEED_ID7,sp_kp,sp_kd,sp_ki);
			PID_Setting(&pidmsg.M3508_SPEED_ID8,sp_kp,sp_kd,sp_ki);
			PID_Pos_Setting(&pidmsg.M3508_POS_ID7,pos_kp,pos_kd,pos_ki);
			PID_Pos_Setting(&pidmsg.M3508_POS_ID8,pos_kp,pos_kd,pos_ki);
		break;
		
		case 3:
			PID_Setting(&pidmsg.M3508_SPEED_ID3,sp_kp,sp_kd,sp_ki);
			PID_Setting(&pidmsg.M3508_SPEED_ID4,sp_kp,sp_kd,sp_ki);
			PID_Pos_Setting(&pidmsg.M3508_POS_ID3,pos_kp,pos_kd,pos_ki);
			PID_Pos_Setting(&pidmsg.M3508_POS_ID4,pos_kp,pos_kd,pos_ki);
		  break;
	 }
}
