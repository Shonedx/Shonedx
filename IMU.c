#include "Allheaderfile.h"
#include "math.h"
//float times=0.0f;
// float zuo;
// float jian,jian1;
// float you;
// float yawout;
//u8 IMU_RX_BUF[IMU_REC_LEN];
u8 IMU_Control_Flag = 1;
float IMU_PID_Intensity = 4.0f;//rewrite
float yawwant = 0.0f;
float pitchwant = 0.0f;
float rollwant = 0.0f;

Detached_Params detached_params_imu;

Euler_angle imu_data;

extern PidTypeDef Yaw_PID_Loop; //PID相关
extern PidTypeDef Roll_PID_Loop;
extern PidTypeDef Pitch_PID_Loop;
 
//这里的PID完全可以替换成pid里的通用PID（前提是对通用PID做进一步的通用化）
//void zhijiaoyawchuli(void) //将角度控制在-45°到45°之间
//{  

//	if(Euler.yaw<45.0f&&Euler.yaw>-45.0f)
//	{
//		yawout=Euler.yaw;
//	}
//	else if(Euler.yaw>45.0f&&Euler.yaw<135.0f)
//	{
//		yawout=Euler.yaw-90.0f;
//	}	
//	else if(Euler.yaw>135.0f&&Euler.yaw<180.0f)
//	{
//		yawout=Euler.yaw-180.0f;
//	}
//	else if(Euler.yaw<-135.0f&&Euler.yaw>-180.0f)
//	{
//		yawout=Euler.yaw+180.0f;
//	}
//	else if(Euler.yaw<-45.0f&&Euler.yaw>-135.0f)
//	{
//		yawout=Euler.yaw+90.0f;
//	}
//	else if(Euler.yaw==0.0f||Euler.yaw==90.0f||Euler.yaw==180.0f||Euler.yaw==-180.0f||Euler.yaw==-90.0f)
//	{
//		yawout=0.0f;
//	}
//jian=yawout;
//}

//void xiejiaoyawchuli(void)//将角度控制在-45°到45°之间
//{  

//	if(Euler.yaw<90.0f&&Euler.yaw>0.0f)
//	{
//		yawout=Euler.yaw-45.0f;
//	}
//	else if(Euler.yaw>90.0f&&Euler.yaw<180.0f)
//	{
//		yawout=Euler.yaw-135.0f;
//	}	
//	else if(Euler.yaw<0.0f&&Euler.yaw>-90.0f)
//	{
//		yawout=Euler.yaw+45.0f;
//	}
//	else if(Euler.yaw<-90.0f&&Euler.yaw>-180.0f)
//	{
//		yawout=Euler.yaw+135.0f;
//	}

//	else if(Euler.yaw==45.0f||Euler.yaw==135.0f||Euler.yaw==-135.0f||Euler.yaw==-45.0f)
//	{
//		yawout=0.0f;
//	}
//jian1=yawout;
//}

int32_t IMU_PID( PidTypeDef *pid,float Now_Point)//位置式 for imu
{
    float Now_Error,d_Error;
	Now_Error=pid->set-Now_Point;
	pid->error[0]+=Now_Error;
	//积分限幅
    if(pid->error[0]>20) pid->error[0]=20;
	else if(pid->error[0]<-20) pid->error[0]=-20;
	
	d_Error=Now_Error-pid->error[1];
    pid->error[1]=Now_Error;
	pid->out=-(pid->Kp*Now_Error+pid->Ki*pid->error[0]+pid->Kd*d_Error);
	//速度目标值的限幅
	if(pid->out>45)  pid->out=45;
	if(pid->out<-45) pid->out=-45;
	return pid->out;
}

//extern Detached_Params Detached_params;
//IMU初始化
void IMU_Init(void)
{
	//detached_params_imu.detached_params0.stance_height=detached_params_imu.detached_params1.stance_height=detached_params_imu.detached_params2.stance_height=detached_params_imu.detached_params3.stance_height=18.0f;
	//standControl(0.0f,0.0f,&detached_params_imu);
//  AttitudeControl(0.0f,0.0f,0.0f,&state_Detached_params[0],1);

}
/*
	功能：基于IMU角度数据的姿态控制
	float roll_set,float pitch_set,float yaw_set：欧拉角（横滚、俯仰、水平）的目标值。
	GaitParams params：输入state_detached_params[i].detached_params_j，即某种步态的某条腿的姿态。该参数作为一种状态基准。j取0即可，因为四条腿参数都是一样的。
	paramID：用来将对状态基准做出改变后的新步态信息来调整state_detached_params[paramID]的值。paramID应与上一条中的i一样。
	问题：该函数充分暴露出了我们目前的控制的欠缺，即各个步态中，四条腿的参数配置都是一样的。
*/
// 在系统初始化时设置PID

void AttitudeControl(float roll_set,float pitch_set,float yaw_set,Detached_Params *Detached_params,int direction)
{
//	float normal_step_left,normal_step_right;
	float normal_stance_0,normal_stance_1,normal_stance_2,normal_stance_3;
	if(IMU_Control_Flag)
	{
		/*******IMUのPID相关*******/
		//PID目标设定（一般都是0，除了Pitch有时要求它是一定角度;另外还有可能是为了微调Yaw）
//		SetPoint(&Yaw_PID_Loop,yaw_set);
		SetPoint(&Pitch_PID_Loop,pitch_set);
		SetPoint(&Roll_PID_Loop,roll_set);
//		PID_Setting(&Yaw_PID_Loop,0.5,0.002,1.0);
		PID_Setting(&Pitch_PID_Loop,0.5,0.002,1.0);
		PID_Setting(&Roll_PID_Loop,0.5,0.002,1.0);		
		//PID计算（利用到了串口解算出的Euler_angle进行位置式PID计算）
		Pitch_PID_Loop.out = IMU_PID(&Pitch_PID_Loop,Euler.pitch);
		Roll_PID_Loop.out  = IMU_PID(&Roll_PID_Loop,Euler.roll);
//		Yaw_PID_Loop.out  = IMU_PID(&Yaw_PID_Loop,Euler.roll);
//		if(direction == 1)
//		{
//			Yaw_PID_Loop.out = IMU_PID(&Yaw_PID_Loop,yawout);
//		}
//		else
//		{
//			Yaw_PID_Loop.out = -IMU_PID(&Yaw_PID_Loop,yawout);		
//		}
		//死区设置
//		if((Yaw_PID_Loop.out<1.0f) && (Yaw_PID_Loop.out>-1.0f)) 	Yaw_PID_Loop.out=0;
		if((Pitch_PID_Loop.out<1.0f) && (Pitch_PID_Loop.out>-1.0f)) Pitch_PID_Loop.out=0;
		if((Roll_PID_Loop.out<1.0f) && (Roll_PID_Loop.out>-1.0f)) 	Roll_PID_Loop.out=0;
		/**********步态控制*********/
		//Yaw输出给步长参数
//		Detached_params->Gait_id=0;
//		 normal_step_left  = Detached_params->detached_params0.step_length + Yaw_PID_Loop.out;//左腿步长减小
//		 normal_step_right = Detached_params->detached_params1.step_length - Yaw_PID_Loop.out ;//右腿步长增加

//		//步长限幅
//		normal_step_left  = ((normal_step_left>StepLenthMax)  ? StepLenthMax : normal_step_left);
//		normal_step_right = ((normal_step_right>StepLenthMax) ? StepLenthMax : normal_step_right);
//		normal_step_left  = ((normal_step_left<StepLenthMin)  ? StepLenthMin : normal_step_left);
//		normal_step_right = ((normal_step_right<StepLenthMin) ? StepLenthMin : normal_step_right);
//		
//		zuo=normal_step_left; 
//		you=normal_step_right;
//		//最终赋值
//		Detached_params->detached_params0.step_length = normal_step_left;
//		Detached_params->detached_params1.step_length = normal_step_left;
//		Detached_params->detached_params2.step_length = normal_step_right;
//		Detached_params->detached_params3.step_length = normal_step_right;
//		
//		zuo=normal_step_left; 
//		you=normal_step_right;
		//腿号0123分别对应左前、左后、右前、右后，即1、2对应左腿，3、4对应右腿。注意配置对，否则正反馈。
		
		//狗腿长度控制（还有一种思路是只增不减，即保证最基本的情况，这样可以避免变数太多而增加调节负担）
		normal_stance_0 = Detached_params->detached_params0.stance_height + Pitch_PID_Loop.out - Roll_PID_Loop.out;//先+后－，是为了抑制前倾和右翻。
		normal_stance_1 = Detached_params->detached_params1.stance_height - Pitch_PID_Loop.out - Roll_PID_Loop.out;
		normal_stance_2 = Detached_params->detached_params2.stance_height + Pitch_PID_Loop.out + Roll_PID_Loop.out;
		normal_stance_3 = Detached_params->detached_params3.stance_height - Pitch_PID_Loop.out + Roll_PID_Loop.out;
		//狗腿长度上限控制
		normal_stance_0  = ( (normal_stance_0>=LegLenthMax) ? LegLenthMax : normal_stance_0 );
		normal_stance_1  = ( (normal_stance_1>=LegLenthMax) ? LegLenthMax : normal_stance_1 );
		normal_stance_2  = ( (normal_stance_2>=LegLenthMax) ? LegLenthMax : normal_stance_2 );
		normal_stance_3  = ( (normal_stance_3>=LegLenthMax) ? LegLenthMax : normal_stance_3 );
		//狗腿长度下限控制
		normal_stance_0  = ( (normal_stance_0<=LegLenthMin) ? LegLenthMin : normal_stance_0 );
		normal_stance_1  = ( (normal_stance_1<=LegLenthMin) ? LegLenthMin : normal_stance_1 );
		normal_stance_2  = ( (normal_stance_2<=LegLenthMin) ? LegLenthMin : normal_stance_2 );
		normal_stance_3  = ( (normal_stance_3<=LegLenthMin) ? LegLenthMin : normal_stance_3 );
		//Pitch和Roll输出给stance_height参数
		Detached_params->detached_params0.stance_height = normal_stance_0;
		Detached_params->detached_params1.stance_height = normal_stance_1;
		Detached_params->detached_params2.stance_height = normal_stance_2;
		Detached_params->detached_params3.stance_height = normal_stance_3;
	}
}

void standControl(float roll_set, float pitch_set, Detached_Params *Detached_params)
{
    double roll_offset, pitch_offset;
	float y;
    if (IMU_Control_Flag)
    {
        /*******IMU PID相关*******/
        // 设置PID目标（一般为0，除了Pitch有时要求一定角度）
        SetPoint(&Pitch_PID_Loop, pitch_set);
        SetPoint(&Roll_PID_Loop, roll_set);
        PID_Setting(&Pitch_PID_Loop, 1.0, 0.002, 1.0);
        PID_Setting(&Roll_PID_Loop, 1.0, 0.002, 1.0);     

        // PID计算（通过IMU获取的Euler角度进行PID计算）
        Pitch_PID_Loop.out = IMU_PID(&Pitch_PID_Loop, Euler.pitch);
        Roll_PID_Loop.out  = IMU_PID(&Roll_PID_Loop, Euler.roll);

        // 如果PID输出非常小，归零
		if((Pitch_PID_Loop.out<1.0f) && (Pitch_PID_Loop.out>-1.0f)) Pitch_PID_Loop.out=0.0f;
		if((Roll_PID_Loop.out<1.0f) && (Roll_PID_Loop.out>-1.0f)) 	Roll_PID_Loop.out=0.0f;

        // 计算roll和pitch的角度修正值
        roll_offset = 18.8f * sin(Roll_PID_Loop.out);   // roll轴角度修正
        pitch_offset = 25.4f * sin(Pitch_PID_Loop.out); // pitch轴角度修正

        // 处理四个腿部的高度和角度
        // 这部分保持不变，直接按原有逻辑处理
      
        
        y = -(Detached_params->detached_params0.stance_height + pitch_offset - roll_offset);
        Detached_params->detached_params0.stance_height = -y;
        Leg_angle.motorangle1 = -Legs.leg1.theta1 * Gaito;
        Leg_angle.motorangle2 = -Legs.leg1.theta2* Gaito;

        y = -(Detached_params->detached_params1.stance_height - pitch_offset + roll_offset);
        Detached_params->detached_params1.stance_height = -y;
        Leg_angle.motorangle3 = Legs.leg2.theta1 * Gaito;
        Leg_angle.motorangle4 = Legs.leg2.theta2 * Gaito;

        y = -(Detached_params->detached_params2.stance_height + pitch_offset + roll_offset);
        Detached_params->detached_params2.stance_height = -y;
        Leg_angle.motorangle5 = -Legs.leg3.theta1 * Gaito;
        Leg_angle.motorangle6 = -Legs.leg3.theta2 * Gaito;

        y = -(Detached_params->detached_params3.stance_height - pitch_offset - roll_offset);
        Detached_params->detached_params3.stance_height = -y;
        Leg_angle.motorangle7 = Legs.leg4.theta1 * Gaito;
        Leg_angle.motorangle8 = Legs.leg4.theta2 * Gaito;

        // 限制腿部高度的范围
        if (Detached_params->detached_params0.stance_height < -25.0f) 
            Detached_params->detached_params0.stance_height = -25.0f;
        else if (Detached_params->detached_params0.stance_height > -22.6f) 
            Detached_params->detached_params0.stance_height = -22.6f;

        if (Detached_params->detached_params1.stance_height < -25.0f) 
            Detached_params->detached_params1.stance_height = -25.0f;
        else if (Detached_params->detached_params1.stance_height > -22.6f) 
            Detached_params->detached_params1.stance_height = -22.6f;

        if (Detached_params->detached_params2.stance_height < -25.0f) 
            Detached_params->detached_params2.stance_height = -25.0f;
        else if (Detached_params->detached_params2.stance_height > -22.6f) 
            Detached_params->detached_params2.stance_height = -22.6f;

        if (Detached_params->detached_params3.stance_height < -25.0f) 
            Detached_params->detached_params3.stance_height = -25.0f;
        else if (Detached_params->detached_params3.stance_height > -22.6f) 
            Detached_params->detached_params3.stance_height = -22.6f;
    }
}
