#include "Allheaderfile.h"
#include "math.h"

       
//  LF(0)--------------------RF(3)
//  motor1       |             motor3             
//    (id:1)     |               (id:3)
//  motor2       |             motor4
//    (id:2)     |                (id:4)
//               |
//               |
//               |
//  LB(1)--------------------RB(2)
//   motor5                     motor7
//     (id:1)                     (id:3)
//   motor6                    motor8
//     (id:2)                     (id4)
//
// 2023-3-16  Motor_id_Sign

	GaitParams gait_params;
	LEGS Legs; //和每条腿有关
	MOVE_DIR Move_Dir;
	Cycloid_Gait cycloid_gait;
	
	double trot_time=1.5;
	double walk_time=1.5;
	
//步态相关	
 /******************************************************************************************************/
 //Tf为占空比，x_target为x轴目标值，z_target为z轴目标，x0为x轴起始坐标，z0为z轴起始坐标，xv0为摆动相抬腿前的腿的瞬间速度，zf为支撑相的z坐标
 // direction为相关方向的参数 ，change为改变步态状态相关参数
 void Gait(double Tf, double x_target, double z_target, double x0, double z0, double xv0, double zf, int direction, int change , double Leglength,int gait_state) 
{
    // 更新时间周期
//trot
	if(gait_state==0)
	{
		if(trot_time>=1.5)
			trot_time=0;
		else 
			trot_time+=t_length;
		
		if(trot_time>=1.5)//0.0017*599=1.0013
			trot_time=1.5;
	}
	 if (gait_state==1)
	{
		if(walk_time>=1.5)
			walk_time=0;
		else 
			walk_time+=t_length_w;
		
		if(walk_time>=1.5)
			walk_time=1.5;
	}
    // 根据 change 参数决定使用的步态模式
    switch (change) 
	{
        case 0:  // 正常 trotting
            trot(trot_time, Tf, x_target, z_target, x0, z0, xv0, zf,Leglength);
            break;
		case 1:  // 行走
			trot(walk_time, Tf, x_target, z_target, x0, z0, xv0, zf,Leglength);
			break;
//        case 2:  // 特殊步态（腿部单独控制）
//            trot_leg1(trot_time, Tf, x_target, z_target + 1, x0, z0, xv0, zf, params);
//            trot_leg(trot_time, Tf, x_target, z_target, x0, z0, xv0, zf, params);
//            break;
        default:
            // 默认步态控制
            trot(trot_time, Tf, x_target, z_target, x0, z0, xv0, zf,Leglength);
            break;
    }

    // 计算机器人姿态
	
   // cal_ges(Euler.pitch - Euler.init_pitch, Euler.roll - Euler.init_roll, 36, 32, 0, 0, 0);
    
    // 转换到逆运动学的角度（如果需要）
    CartesianToTheta_Cycloid_All_Legs();
    
    // 控制腿部运动
	Moveleg(direction);
}


//t为定时器里的时间，Tf为占空比，x_target为x轴目标值，z_target为z轴目标，x0为x轴起始坐标，z0为z轴起始坐标，xv0为摆动相抬腿前的腿的瞬间速度，zf为支撑相的z坐标，
//z轴坐标竖直向下为正，竖直向上为负，这里设狗站立时的腿长为Leglength，所以狗足端站立时的初始位置为(0,Leglength)
//Leglength为站立时的腿长，正常站立为15，下蹲为10
void trot(double t, double Tf, double x_target, double z_target, double x0, double z0, double xv0, double zf,double Leglength )
{
	
    if(t<Tf)
	{
		support_curve_generate_1(t,Tf,x_target,x0,z0);
		support_curve_generate_2(t,Tf,x_target,x0,z0);
		Legs.leg1.x=cycloid_gait.support_2.xf*Move_Dir.dir1;
		Legs.leg2.x=cycloid_gait.support_1.xf*Move_Dir.dir2;
		Legs.leg3.x=cycloid_gait.support_1.xf*Move_Dir.dir3;
		Legs.leg4.x=cycloid_gait.support_2.xf*Move_Dir.dir4;
		Legs.leg1.z=-cycloid_gait.support_2.zf+Leglength; //这里zf为负代表狗足端向上摆动
		Legs.leg2.z=-cycloid_gait.support_1.zf+Leglength;
		Legs.leg3.z=-cycloid_gait.support_1.zf+Leglength;
		Legs.leg4.z=-cycloid_gait.support_2.zf+Leglength;
	}
  if(2*Tf>t>=Tf)
	{
		swing_curve_generate(t-Tf,Tf,x_target,x0,z0); 
		support_curve_generate_1(t-Tf,Tf,x_target,x0,z0);
		Legs.leg1.x=cycloid_gait.support_1.xf*Move_Dir.dir1;
		Legs.leg2.x=cycloid_gait.curve.xf*Move_Dir.dir2;
		Legs.leg3.x=cycloid_gait.curve.xf*Move_Dir.dir3;
		Legs.leg4.x=cycloid_gait.support_1.xf*Move_Dir.dir4;
		Legs.leg1.z=-cycloid_gait.support_1.zf+Leglength;
		Legs.leg2.z=-cycloid_gait.curve.zf+Leglength;
		Legs.leg3.z=-cycloid_gait.curve.zf+Leglength;
		Legs.leg4.z=-cycloid_gait.support_1.zf+Leglength;
	}
	  if(t>=2*Tf)
	{
		swing_curve_generate(t-2*Tf,Tf,x_target,x0,z0); 
		support_curve_generate_2(t-2*Tf,Tf,x_target,x0,z0);
		Legs.leg1.x=cycloid_gait.curve.xf*Move_Dir.dir1;
		Legs.leg2.x=cycloid_gait.support_2.xf*Move_Dir.dir2;
		Legs.leg3.x=cycloid_gait.support_2.xf*Move_Dir.dir3;
		Legs.leg4.x=cycloid_gait.curve.xf*Move_Dir.dir4;
		Legs.leg1.z=-cycloid_gait.curve.zf+Leglength; 
		Legs.leg2.z=-cycloid_gait.support_2.zf+Leglength;
		Legs.leg3.z=-cycloid_gait.support_2.zf+Leglength;
		Legs.leg4.z=-cycloid_gait.curve.zf+Leglength;
	}

}
//leglength站立时的腿长，正常站立为15(暂定)，下蹲为10(暂定)，x_offset是腿在x轴上的的偏移值
void Stand(double x_offset,double Leglength)
{
	
	Legs.leg1.x=x_offset;
	Legs.leg2.x=x_offset;
	Legs.leg3.x=x_offset;
	Legs.leg4.x=x_offset;
	Legs.leg1.z=Leglength;
	Legs.leg2.z=Leglength;
	Legs.leg3.z=Leglength;
	Legs.leg4.z=Leglength;
	
	CartesianToTheta_Cycloid_All_Legs();
	Moveleg(0);
}
void Stand_After_Bound(double x_offset,double Leglength)
{
	
	Legs.leg1.x=x_offset;
	Legs.leg2.x=x_offset;
	Legs.leg3.x=-x_offset;
	Legs.leg4.x=-x_offset;
	Legs.leg1.z=Leglength;
	Legs.leg2.z=Leglength;
	Legs.leg3.z=Leglength;
	Legs.leg4.z=Leglength;
	
	CartesianToTheta_Cycloid_All_Legs();
	Moveleg(0);
}
//运动相关
/******************************************************************************************************/
// 逆解函数
void CartesianToTheta_Cycloid(Leg *leg)
{
    leg->L = sqrt(leg->x * leg->x + leg->z * leg->z);
    leg->psai1 = asin(leg->x / leg->L);
    leg->fai1 = acos((leg->L * leg->L + L1 * L1 - L2 * L2) / (2 * L1 * leg->L));
    leg->theta2 = 180.0f * (leg->fai1 - leg->psai1) / PI - 90.0f;
    leg->theta1 = 180.0f * (leg->fai1 + leg->psai1) / PI - 90.0f;
}
//逆解所有腿
void CartesianToTheta_Cycloid_All_Legs(void)
{
	CartesianToTheta_Cycloid(&Legs.leg1);
	CartesianToTheta_Cycloid(&Legs.leg2);
	CartesianToTheta_Cycloid(&Legs.leg3);
	CartesianToTheta_Cycloid(&Legs.leg4);
}

// 移动腿函数
void Moveleg(int direction)
{
    // 参数说明：
    // direction: 方向标志

    // 为每条腿设置角度
    // Legid 从 0 到 3 分别代表四条腿
    Angle_Setting_Cycloid(0, direction);  // 设置第 0 条腿的角度
    Angle_Setting_Cycloid(1, direction);  // 设置第 1 条腿的角度
    Angle_Setting_Cycloid(2, direction);  // 设置第 2 条腿的角度
    Angle_Setting_Cycloid(3, direction);  // 设置第 3 条腿的角度
}


/**************************************生成摆动期和支撑期曲线的函数********************************************************/
// 生成摆动期的曲线  t是当前时间，Tf是摆动期总时间 xt是目标x位移量，zh是z最高处，即目标z轴（竖直轴）位移量，x0,z0为初始位置
void swing_curve_generate(double t, double Tf, double xt,double x0,double z0)
{
    double xf, zf;
    double t_normalized = fmod(t/ Tf, 1.0);  // 标准化时间，使其在 [0, 1] 范围内

    // 摆线参数方程中的 r 是从起始点到目标点的距离的一半
    double r = 2*xt/2/PI;

    // 角度参数 theta
    double theta = 2 * PI * t_normalized;

    // 计算摆线轨迹的 x 和 z 坐标
    xf = -xt + r * (theta - sin(theta));
    zf = z0 + 1.5*r * (1 - cos(theta));

    if (zf < 0) {
        zf = 0;
    }

    // 确保 xf 不超过目标位置 xt
    if (xf > xt) {
        xf = xt;
    }
	 if (xf < -xt) {
        xf = -xt;
    }

    // 记录当前状态
    cycloid_gait.curve.xf = xf;   // 当前 x 坐标
    cycloid_gait.curve.zf = zf;   // 当前 z 坐标

}
//摆动前的第一个支撑期此时足端从原点向后移动到摆动的起始位置
void support_curve_generate_1(double t, double Tf, double x_target,double x0,double z0)
{
	double xf, zf;
	double t_normalized = fmod(t/ Tf, 1.0); 
	xf=x0-x_target*t_normalized;
	zf=z0;
	if(xf<-x_target)
		xf=-x_target;
	cycloid_gait.support_1.xf = xf;   // 当前 x 坐标
	cycloid_gait.support_1.zf = zf;   // 当前 z 坐标
}
//摆动后的第二个支撑期，足端从摆动后的落地向后移动到原点
void support_curve_generate_2(double t, double Tf, double x_target,double x0,double z0)
{
	double xf, zf;
	double t_normalized = fmod(t/ Tf, 1.0);
	xf=x_target-x_target*t_normalized;
	zf=z0;
	if(xf<0)
		xf=0;
	cycloid_gait.support_2.xf = xf;   // 当前 x 坐标
	cycloid_gait.support_2.zf = zf;   // 当前 z 坐标		
}

//设置相关摆动角
void Angle_Setting_Cycloid(int Legid, int direction)  // Moveleg 里被调用
{
    switch (direction)
    {
        case 0:	 //暂时只调case0,其他的以后调
        {
            switch (Legid)
            {
                case 0:
                    Leg_angle.motorangle1 = -Legs.leg1.theta1 * Gaito;  
                    Leg_angle.motorangle2 = -Legs.leg1.theta2 * Gaito;
                    break;
                case 1:
                    Leg_angle.motorangle3 =	Legs.leg2.theta2 * Gaito;  
                    Leg_angle.motorangle4 =	Legs.leg2.theta1 * Gaito;
                    break;
                case 2:
                    Leg_angle.motorangle5 = -Legs.leg3.theta1 * Gaito;
                    Leg_angle.motorangle6 =	-Legs.leg3.theta2 * Gaito;
                    break;
                case 3:
                    Leg_angle.motorangle7 =	Legs.leg4.theta2 * Gaito;  
                    Leg_angle.motorangle8 =	Legs.leg4.theta1 * Gaito;
                    break;
            }
        } break;

        case 1:
        {
            switch (Legid)
            {
                case 0:
                    Leg_angle.motorangle1 = Legs.leg1.theta1 * Gaito;
                    Leg_angle.motorangle2 = Legs.leg1.theta2 * Gaito;
                    break;
                case 1:
                    Leg_angle.motorangle3 = -Legs.leg2.theta1 * Gaito;
                    Leg_angle.motorangle4 = -Legs.leg2.theta2 * Gaito;
                    break;
                case 2:
                    Leg_angle.motorangle5 = Legs.leg3.theta1 * Gaito;
                    Leg_angle.motorangle6 = Legs.leg3.theta2 * Gaito;
                    break;
                case 3:
                    Leg_angle.motorangle7 = -Legs.leg4.theta1 * Gaito;
                    Leg_angle.motorangle8 = -Legs.leg4.theta2 * Gaito;
                    break;
            }
        } break;

        case 2:
        {
            switch (Legid)
            {
                case 0:
                    Leg_angle.motorangle1 = Legs.leg1.theta2 * Gaito;
                    Leg_angle.motorangle2 = Legs.leg1.theta1 * Gaito;
                    break;
                case 1:
                    Leg_angle.motorangle3 = -Legs.leg2.theta2 * Gaito;
                    Leg_angle.motorangle4 = -Legs.leg2.theta1 * Gaito;
                    break;
                case 2:
                    Leg_angle.motorangle5 = Legs.leg3.theta2 * Gaito;
                    Leg_angle.motorangle6 = Legs.leg3.theta1 * Gaito;
                    break;
                case 3:
                    Leg_angle.motorangle7 = -Legs.leg4.theta2 * Gaito;
                    Leg_angle.motorangle8 = -Legs.leg4.theta1 * Gaito;
                    break;
            }
        } break;

        case 3:
        {
            switch (Legid)
            {
                case 0:
                    Leg_angle.motorangle1 = Legs.leg1.theta2 * Gaito;
                    Leg_angle.motorangle2 = Legs.leg1.theta1 * Gaito;
                    break;
                case 1:
                    Leg_angle.motorangle3 = -Legs.leg2.theta1 * Gaito;
                    Leg_angle.motorangle4 = -Legs.leg2.theta2 * Gaito;
                    break;
                case 2:
                    Leg_angle.motorangle5 = Legs.leg3.theta2 * Gaito;
                    Leg_angle.motorangle6 = Legs.leg3.theta1 * Gaito;
                    break;
                case 3:
                    Leg_angle.motorangle7 = -Legs.leg4.theta1 * Gaito;
                    Leg_angle.motorangle8 = -Legs.leg4.theta2 * Gaito;
                    break;
            }
        } break;
    }
}

/******************************************************************************************************/

//设定相关方向
void Dir_Set(int direction)
{
  // 处理前进
  if (direction == 1)
  {
    double speed = (double)abs(left_y) / 660;
    Move_Dir.dir1 = Move_Dir.dir2 = Move_Dir.dir3 = Move_Dir.dir4 = speed;
  }
  // 差速左转
  else if (direction == 2)
  {
    double left_speed = (double)abs(left_y) / 660;
    double right_speed = (double)(660 - abs(left_x)) / 660;
    Move_Dir.dir2 = Move_Dir.dir4 = right_speed;
    Move_Dir.dir1 = Move_Dir.dir3 = left_speed;
  }
  // 差速右转
  else if (direction == 3)
  {  
	double left_speed = (double)abs(left_y) / 660;
    double right_speed = (double)(660 - abs(left_x)) / 660;
    Move_Dir.dir2 = Move_Dir.dir4 = left_speed;
    Move_Dir.dir1 = Move_Dir.dir3 = right_speed;
  }
  
  // 原地左转
  else if (direction == 22)
  {
    double turn_speed = (double)abs(right_x) / 660;
    Move_Dir.dir2 = Move_Dir.dir4 = -turn_speed;
    Move_Dir.dir1 = Move_Dir.dir3 = turn_speed;
  }
  // 原地右转
  else if (direction == 33)
  {
    double turn_speed = (double)abs(right_x) / 660;
    Move_Dir.dir2 = Move_Dir.dir4 = turn_speed;
    Move_Dir.dir1 = Move_Dir.dir3 = -turn_speed;
  }
  else
  {
    // 如果 direction 无效，可以设置一个默认状态或返回错误
    // 例如:
    // Move_DIR.dir1 = Move_DIR.dir2 = Move_DIR.dir3 = Move_DIR.dir4 = 0;
  }
}

//启动电机相关
/******************************************************************************************************/

//为每条腿设定角度
void AllLeg_Set_angle(int target_angle, int offset)
{
    // ID1
    Motor_Angle_Cal_1(360); // 得到绝对角度
    Target_Pos_Setting(&pidmsg.M3508_STAND_ID1, -target_angle + offset);
	//Target_Pos_Setting(&pidmsg.M3508_STAND_ID1, -2200); //调试站立角度用
    motor_3508.ID1.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID1,  motor_3508.ID1.POS_ABS);
    motor_3508.ID1.target_speed = motor_3508.ID1.angle_out; // 角度环输出作为速度环输入
    motor_3508.ID1.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID1,&motor_3508.ID1 );
    canbuf[0] = ((short)(motor_3508.ID1.corrent_output)) >> 8;
    canbuf[1] = ((short)(motor_3508.ID1.corrent_output)) & 0x00FF;

    // ID2
    Motor_Angle_Cal_2(360); // 得到绝对角度
    Target_Pos_Setting(&pidmsg.M3508_STAND_ID2, target_angle + offset);
	//Target_Pos_Setting(&pidmsg.M3508_STAND_ID2, -153);
    motor_3508.ID2.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID2,  motor_3508.ID2.POS_ABS);
    motor_3508.ID2.target_speed = motor_3508.ID2.angle_out; // 角度环输出作为速度环输入
    motor_3508.ID2.corrent_output = PID_Calc( &pidmsg.M3508_SPEED_ID2,&motor_3508.ID2);
    canbuf[2] = ((short)(motor_3508.ID2.corrent_output)) >> 8;
    canbuf[3] = ((short)(motor_3508.ID2.corrent_output)) & 0x00FF;

    // ID3
    Motor_Angle_Cal_3(360); // 得到绝对角度
    Target_Pos_Setting(&pidmsg.M3508_STAND_ID3, -target_angle - offset);
	//Target_Pos_Setting(&pidmsg.M3508_STAND_ID3,153);
    motor_3508.ID3.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID3,  motor_3508.ID3.POS_ABS);
    motor_3508.ID3.target_speed = motor_3508.ID3.angle_out; // 角度环输出作为速度环输入
    motor_3508.ID3.corrent_output = PID_Calc( &pidmsg.M3508_SPEED_ID3,&motor_3508.ID3);
    canbuf[4] = ((short)(motor_3508.ID3.corrent_output)) >> 8;
    canbuf[5] = ((short)(motor_3508.ID3.corrent_output)) & 0x00FF;

    // ID4
    Motor_Angle_Cal_4(360); // 得到绝对角度
    Target_Pos_Setting(&pidmsg.M3508_STAND_ID4, target_angle - offset);
    //Target_Pos_Setting(&pidmsg.M3508_STAND_ID4, 2200);
    motor_3508.ID4.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID4,motor_3508.ID4.POS_ABS);
    motor_3508.ID4.target_speed = motor_3508.ID4.angle_out; // 角度环输出作为速度环输入
    motor_3508.ID4.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID4,&motor_3508.ID4 ); 
    canbuf[6] = ((short)(motor_3508.ID4.corrent_output)) >> 8;
    canbuf[7] = ((short)(motor_3508.ID4.corrent_output)) & 0x00FF;

    // 发送CAN1消息
    CAN1_Send_Msg(canbuf, 8, 0);

    // ID5
    Motor_Angle_Cal_5(360); // 得到绝对角度
    Target_Pos_Setting(&pidmsg.M3508_STAND_ID5, target_angle + offset);
	//Target_Pos_Setting(&pidmsg.M3508_STAND_ID5, -1152);
    motor_3508.ID5.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID5, motor_3508.ID5.POS_ABS);
    motor_3508.ID5.target_speed = motor_3508.ID5.angle_out; // 角度环输出作为速度环输入
    motor_3508.ID5.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID5,&motor_3508.ID5 );
    canbuf2[0] = ((short)(motor_3508.ID5.corrent_output)) >> 8;
    canbuf2[1] = ((short)(motor_3508.ID5.corrent_output)) & 0x00FF;

    // ID6
    Motor_Angle_Cal_6(360); // 得到绝对角度
    Target_Pos_Setting(&pidmsg.M3508_STAND_ID6, -target_angle + offset);
	//Target_Pos_Setting(&pidmsg.M3508_STAND_ID6, -1152);
    motor_3508.ID6.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID6,  motor_3508.ID6.POS_ABS);
    motor_3508.ID6.target_speed = motor_3508.ID6.angle_out; // 角度环输出作为速度环输入
    motor_3508.ID6.corrent_output = PID_Calc( &pidmsg.M3508_SPEED_ID6,&motor_3508.ID6);
    canbuf2[2] = ((short)(motor_3508.ID6.corrent_output)) >> 8;
    canbuf2[3] = ((short)(motor_3508.ID6.corrent_output)) & 0x00FF;

    // ID7
    Motor_Angle_Cal_7(360); // 得到绝对角度
    Target_Pos_Setting(&pidmsg.M3508_STAND_ID7, target_angle - offset);
	//Target_Pos_Setting(&pidmsg.M3508_STAND_ID7, 2200);
    motor_3508.ID7.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID7,  motor_3508.ID7.POS_ABS);
    motor_3508.ID7.target_speed = motor_3508.ID7.angle_out; // 角度环输出作为速度环输入
    motor_3508.ID7.corrent_output = PID_Calc( &pidmsg.M3508_SPEED_ID7,&motor_3508.ID7);
    canbuf2[4] = ((short)(motor_3508.ID7.corrent_output)) >> 8;
    canbuf2[5] = ((short)(motor_3508.ID7.corrent_output)) & 0x00FF;

    // ID8
    Motor_Angle_Cal_8(360); // 得到绝对角度
   Target_Pos_Setting(&pidmsg.M3508_STAND_ID8, -target_angle - offset);
   //Target_Pos_Setting(&pidmsg.M3508_STAND_ID8, 153);
    motor_3508.ID8.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID8,  motor_3508.ID8.POS_ABS);
    motor_3508.ID8.target_speed = motor_3508.ID8.angle_out; // 角度环输出作为速度环输入
    motor_3508.ID8.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID8,&motor_3508.ID8 );
    canbuf2[6] = ((short)(motor_3508.ID8.corrent_output)) >> 8;
    canbuf2[7] = ((short)(motor_3508.ID8.corrent_output)) & 0x00FF;

    // 发送CAN2消息
    CAN2_Send_Msg(canbuf2, 8);
}

void Motor_Auto_Run(void) //驱动电机
{
    // ID1
    Motor_Angle_Cal_1(360); // 得到绝对角度
    Target_Pos_Setting(&pidmsg.M3508_POS_ID1, Leg_angle.motorangle1);
    motor_3508.ID1.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID1, motor_3508.ID1.POS_ABS);
    motor_3508.ID1.target_speed = motor_3508.ID1.angle_out; // 角度环输出作为速度环输入
    motor_3508.ID1.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID1, &motor_3508.ID1);
    canbuf[0] = ((short)(motor_3508.ID1.corrent_output)) >> 8;
    canbuf[1] = ((short)(motor_3508.ID1.corrent_output)) & 0x00FF;

    // ID2
    Motor_Angle_Cal_2(360); // 得到绝对角度
    Target_Pos_Setting(&pidmsg.M3508_POS_ID2, Leg_angle.motorangle2);
    motor_3508.ID2.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID2, motor_3508.ID2.POS_ABS);
    motor_3508.ID2.target_speed = motor_3508.ID2.angle_out; // 角度环输出作为速度环输入
    motor_3508.ID2.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID2, &motor_3508.ID2);
    canbuf[2] = ((short)(motor_3508.ID2.corrent_output)) >> 8;
    canbuf[3] = ((short)(motor_3508.ID2.corrent_output)) & 0x00FF;

    // ID3
    Motor_Angle_Cal_3(360); // 得到绝对角度
    Target_Pos_Setting(&pidmsg.M3508_POS_ID3, Leg_angle.motorangle3);
    motor_3508.ID3.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID3, motor_3508.ID3.POS_ABS);
    motor_3508.ID3.target_speed = motor_3508.ID3.angle_out; // 角度环输出作为速度环输入
    motor_3508.ID3.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID3, &motor_3508.ID3);
    canbuf[4] = ((short)(motor_3508.ID3.corrent_output)) >> 8;
    canbuf[5] = ((short)(motor_3508.ID3.corrent_output)) & 0x00FF;

    // ID4
    Motor_Angle_Cal_4(360); // 得到绝对角度
    Target_Pos_Setting(&pidmsg.M3508_POS_ID4, Leg_angle.motorangle4);
    motor_3508.ID4.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID4, motor_3508.ID4.POS_ABS);
    motor_3508.ID4.target_speed = motor_3508.ID4.angle_out; // 角度环输出作为速度环输入
    motor_3508.ID4.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID4, &motor_3508.ID4);
    canbuf[6] = ((short)(motor_3508.ID4.corrent_output)) >> 8;
    canbuf[7] = ((short)(motor_3508.ID4.corrent_output)) & 0x00FF;

    // 发送CAN1消息
    CAN1_Send_Msg(canbuf, 8, 0);

    // ID5
    Motor_Angle_Cal_5(360); // 得到绝对角度
    Target_Pos_Setting(&pidmsg.M3508_POS_ID5, Leg_angle.motorangle5);
    motor_3508.ID5.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID5, motor_3508.ID5.POS_ABS);
    motor_3508.ID5.target_speed = motor_3508.ID5.angle_out; // 角度环输出作为速度环输入
    motor_3508.ID5.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID5, &motor_3508.ID5);
    canbuf2[0] = ((short)(motor_3508.ID5.corrent_output)) >> 8;
    canbuf2[1] = ((short)(motor_3508.ID5.corrent_output)) & 0x00FF;

    // ID6
    Motor_Angle_Cal_6(360); // 得到绝对角度
    Target_Pos_Setting(&pidmsg.M3508_POS_ID6, Leg_angle.motorangle6);
    motor_3508.ID6.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID6, motor_3508.ID6.POS_ABS);
    motor_3508.ID6.target_speed = motor_3508.ID6.angle_out; // 角度环输出作为速度环输入
    motor_3508.ID6.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID6, &motor_3508.ID6);
    canbuf2[2] = ((short)(motor_3508.ID6.corrent_output)) >> 8;
    canbuf2[3] = ((short)(motor_3508.ID6.corrent_output)) & 0x00FF;

    // ID7
    Motor_Angle_Cal_7(360); // 得到绝对角度
    Target_Pos_Setting(&pidmsg.M3508_POS_ID7, Leg_angle.motorangle7);
    motor_3508.ID7.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID7, motor_3508.ID7.POS_ABS);
    motor_3508.ID7.target_speed = motor_3508.ID7.angle_out; // 角度环输出作为速度环输入
    motor_3508.ID7.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID7, &motor_3508.ID7);
    canbuf2[4] = ((short)(motor_3508.ID7.corrent_output)) >> 8;
    canbuf2[5] = ((short)(motor_3508.ID7.corrent_output)) & 0x00FF;

    // ID8
    Motor_Angle_Cal_8(360); // 得到绝对角度
    Target_Pos_Setting(&pidmsg.M3508_POS_ID8, Leg_angle.motorangle8);
    motor_3508.ID8.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID8, motor_3508.ID8.POS_ABS);
    motor_3508.ID8.target_speed = motor_3508.ID8.angle_out; // 角度环输出作为速度环输入
    motor_3508.ID8.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID8, &motor_3508.ID8);
    canbuf2[6] = ((short)(motor_3508.ID8.corrent_output)) >> 8;
    canbuf2[7] = ((short)(motor_3508.ID8.corrent_output)) & 0x00FF;

    // 发送CAN2消息
    CAN2_Send_Msg(canbuf2, 8);

    flag_full = 0;
}






