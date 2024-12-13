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
	LEGS Legs; //��ÿ�����й�
	MOVE_DIR Move_Dir;
	Cycloid_Gait cycloid_gait;
	
	double trot_time=1.5;
	double walk_time=1.5;
	
//��̬���	
 /******************************************************************************************************/
 //TfΪռ�ձȣ�x_targetΪx��Ŀ��ֵ��z_targetΪz��Ŀ�꣬x0Ϊx����ʼ���꣬z0Ϊz����ʼ���꣬xv0Ϊ�ڶ���̧��ǰ���ȵ�˲���ٶȣ�zfΪ֧�����z����
 // directionΪ��ط���Ĳ��� ��changeΪ�ı䲽̬״̬��ز���
 void Gait(double Tf, double x_target, double z_target, double x0, double z0, double xv0, double zf, int direction, int change , double Leglength,int gait_state) 
{
    // ����ʱ������
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
    // ���� change ��������ʹ�õĲ�̬ģʽ
    switch (change) 
	{
        case 0:  // ���� trotting
            trot(trot_time, Tf, x_target, z_target, x0, z0, xv0, zf,Leglength);
            break;
		case 1:  // ����
			trot(walk_time, Tf, x_target, z_target, x0, z0, xv0, zf,Leglength);
			break;
//        case 2:  // ���ⲽ̬���Ȳ��������ƣ�
//            trot_leg1(trot_time, Tf, x_target, z_target + 1, x0, z0, xv0, zf, params);
//            trot_leg(trot_time, Tf, x_target, z_target, x0, z0, xv0, zf, params);
//            break;
        default:
            // Ĭ�ϲ�̬����
            trot(trot_time, Tf, x_target, z_target, x0, z0, xv0, zf,Leglength);
            break;
    }

    // �����������̬
	
   // cal_ges(Euler.pitch - Euler.init_pitch, Euler.roll - Euler.init_roll, 36, 32, 0, 0, 0);
    
    // ת�������˶�ѧ�ĽǶȣ������Ҫ��
    CartesianToTheta_Cycloid_All_Legs();
    
    // �����Ȳ��˶�
	Moveleg(direction);
}


//tΪ��ʱ�����ʱ�䣬TfΪռ�ձȣ�x_targetΪx��Ŀ��ֵ��z_targetΪz��Ŀ�꣬x0Ϊx����ʼ���꣬z0Ϊz����ʼ���꣬xv0Ϊ�ڶ���̧��ǰ���ȵ�˲���ٶȣ�zfΪ֧�����z���꣬
//z��������ֱ����Ϊ������ֱ����Ϊ���������蹷վ��ʱ���ȳ�ΪLeglength�����Թ����վ��ʱ�ĳ�ʼλ��Ϊ(0,Leglength)
//LeglengthΪվ��ʱ���ȳ�������վ��Ϊ15���¶�Ϊ10
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
		Legs.leg1.z=-cycloid_gait.support_2.zf+Leglength; //����zfΪ������������ϰڶ�
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
//leglengthվ��ʱ���ȳ�������վ��Ϊ15(�ݶ�)���¶�Ϊ10(�ݶ�)��x_offset������x���ϵĵ�ƫ��ֵ
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
//�˶����
/******************************************************************************************************/
// ��⺯��
void CartesianToTheta_Cycloid(Leg *leg)
{
    leg->L = sqrt(leg->x * leg->x + leg->z * leg->z);
    leg->psai1 = asin(leg->x / leg->L);
    leg->fai1 = acos((leg->L * leg->L + L1 * L1 - L2 * L2) / (2 * L1 * leg->L));
    leg->theta2 = 180.0f * (leg->fai1 - leg->psai1) / PI - 90.0f;
    leg->theta1 = 180.0f * (leg->fai1 + leg->psai1) / PI - 90.0f;
}
//���������
void CartesianToTheta_Cycloid_All_Legs(void)
{
	CartesianToTheta_Cycloid(&Legs.leg1);
	CartesianToTheta_Cycloid(&Legs.leg2);
	CartesianToTheta_Cycloid(&Legs.leg3);
	CartesianToTheta_Cycloid(&Legs.leg4);
}

// �ƶ��Ⱥ���
void Moveleg(int direction)
{
    // ����˵����
    // direction: �����־

    // Ϊÿ�������ýǶ�
    // Legid �� 0 �� 3 �ֱ����������
    Angle_Setting_Cycloid(0, direction);  // ���õ� 0 ���ȵĽǶ�
    Angle_Setting_Cycloid(1, direction);  // ���õ� 1 ���ȵĽǶ�
    Angle_Setting_Cycloid(2, direction);  // ���õ� 2 ���ȵĽǶ�
    Angle_Setting_Cycloid(3, direction);  // ���õ� 3 ���ȵĽǶ�
}


/**************************************���ɰڶ��ں�֧�������ߵĺ���********************************************************/
// ���ɰڶ��ڵ�����  t�ǵ�ǰʱ�䣬Tf�ǰڶ�����ʱ�� xt��Ŀ��xλ������zh��z��ߴ�����Ŀ��z�ᣨ��ֱ�ᣩλ������x0,z0Ϊ��ʼλ��
void swing_curve_generate(double t, double Tf, double xt,double x0,double z0)
{
    double xf, zf;
    double t_normalized = fmod(t/ Tf, 1.0);  // ��׼��ʱ�䣬ʹ���� [0, 1] ��Χ��

    // ���߲��������е� r �Ǵ���ʼ�㵽Ŀ���ľ����һ��
    double r = 2*xt/2/PI;

    // �ǶȲ��� theta
    double theta = 2 * PI * t_normalized;

    // ������߹켣�� x �� z ����
    xf = -xt + r * (theta - sin(theta));
    zf = z0 + 1.5*r * (1 - cos(theta));

    if (zf < 0) {
        zf = 0;
    }

    // ȷ�� xf ������Ŀ��λ�� xt
    if (xf > xt) {
        xf = xt;
    }
	 if (xf < -xt) {
        xf = -xt;
    }

    // ��¼��ǰ״̬
    cycloid_gait.curve.xf = xf;   // ��ǰ x ����
    cycloid_gait.curve.zf = zf;   // ��ǰ z ����

}
//�ڶ�ǰ�ĵ�һ��֧���ڴ�ʱ��˴�ԭ������ƶ����ڶ�����ʼλ��
void support_curve_generate_1(double t, double Tf, double x_target,double x0,double z0)
{
	double xf, zf;
	double t_normalized = fmod(t/ Tf, 1.0); 
	xf=x0-x_target*t_normalized;
	zf=z0;
	if(xf<-x_target)
		xf=-x_target;
	cycloid_gait.support_1.xf = xf;   // ��ǰ x ����
	cycloid_gait.support_1.zf = zf;   // ��ǰ z ����
}
//�ڶ���ĵڶ���֧���ڣ���˴Ӱڶ�����������ƶ���ԭ��
void support_curve_generate_2(double t, double Tf, double x_target,double x0,double z0)
{
	double xf, zf;
	double t_normalized = fmod(t/ Tf, 1.0);
	xf=x_target-x_target*t_normalized;
	zf=z0;
	if(xf<0)
		xf=0;
	cycloid_gait.support_2.xf = xf;   // ��ǰ x ����
	cycloid_gait.support_2.zf = zf;   // ��ǰ z ����		
}

//������ذڶ���
void Angle_Setting_Cycloid(int Legid, int direction)  // Moveleg �ﱻ����
{
    switch (direction)
    {
        case 0:	 //��ʱֻ��case0,�������Ժ��
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

//�趨��ط���
void Dir_Set(int direction)
{
  // ����ǰ��
  if (direction == 1)
  {
    double speed = (double)abs(left_y) / 660;
    Move_Dir.dir1 = Move_Dir.dir2 = Move_Dir.dir3 = Move_Dir.dir4 = speed;
  }
  // ������ת
  else if (direction == 2)
  {
    double left_speed = (double)abs(left_y) / 660;
    double right_speed = (double)(660 - abs(left_x)) / 660;
    Move_Dir.dir2 = Move_Dir.dir4 = right_speed;
    Move_Dir.dir1 = Move_Dir.dir3 = left_speed;
  }
  // ������ת
  else if (direction == 3)
  {  
	double left_speed = (double)abs(left_y) / 660;
    double right_speed = (double)(660 - abs(left_x)) / 660;
    Move_Dir.dir2 = Move_Dir.dir4 = left_speed;
    Move_Dir.dir1 = Move_Dir.dir3 = right_speed;
  }
  
  // ԭ����ת
  else if (direction == 22)
  {
    double turn_speed = (double)abs(right_x) / 660;
    Move_Dir.dir2 = Move_Dir.dir4 = -turn_speed;
    Move_Dir.dir1 = Move_Dir.dir3 = turn_speed;
  }
  // ԭ����ת
  else if (direction == 33)
  {
    double turn_speed = (double)abs(right_x) / 660;
    Move_Dir.dir2 = Move_Dir.dir4 = turn_speed;
    Move_Dir.dir1 = Move_Dir.dir3 = -turn_speed;
  }
  else
  {
    // ��� direction ��Ч����������һ��Ĭ��״̬�򷵻ش���
    // ����:
    // Move_DIR.dir1 = Move_DIR.dir2 = Move_DIR.dir3 = Move_DIR.dir4 = 0;
  }
}

//����������
/******************************************************************************************************/

//Ϊÿ�����趨�Ƕ�
void AllLeg_Set_angle(int target_angle, int offset)
{
    // ID1
    Motor_Angle_Cal_1(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_STAND_ID1, -target_angle + offset);
	//Target_Pos_Setting(&pidmsg.M3508_STAND_ID1, -2200); //����վ���Ƕ���
    motor_3508.ID1.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID1,  motor_3508.ID1.POS_ABS);
    motor_3508.ID1.target_speed = motor_3508.ID1.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID1.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID1,&motor_3508.ID1 );
    canbuf[0] = ((short)(motor_3508.ID1.corrent_output)) >> 8;
    canbuf[1] = ((short)(motor_3508.ID1.corrent_output)) & 0x00FF;

    // ID2
    Motor_Angle_Cal_2(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_STAND_ID2, target_angle + offset);
	//Target_Pos_Setting(&pidmsg.M3508_STAND_ID2, -153);
    motor_3508.ID2.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID2,  motor_3508.ID2.POS_ABS);
    motor_3508.ID2.target_speed = motor_3508.ID2.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID2.corrent_output = PID_Calc( &pidmsg.M3508_SPEED_ID2,&motor_3508.ID2);
    canbuf[2] = ((short)(motor_3508.ID2.corrent_output)) >> 8;
    canbuf[3] = ((short)(motor_3508.ID2.corrent_output)) & 0x00FF;

    // ID3
    Motor_Angle_Cal_3(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_STAND_ID3, -target_angle - offset);
	//Target_Pos_Setting(&pidmsg.M3508_STAND_ID3,153);
    motor_3508.ID3.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID3,  motor_3508.ID3.POS_ABS);
    motor_3508.ID3.target_speed = motor_3508.ID3.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID3.corrent_output = PID_Calc( &pidmsg.M3508_SPEED_ID3,&motor_3508.ID3);
    canbuf[4] = ((short)(motor_3508.ID3.corrent_output)) >> 8;
    canbuf[5] = ((short)(motor_3508.ID3.corrent_output)) & 0x00FF;

    // ID4
    Motor_Angle_Cal_4(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_STAND_ID4, target_angle - offset);
    //Target_Pos_Setting(&pidmsg.M3508_STAND_ID4, 2200);
    motor_3508.ID4.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID4,motor_3508.ID4.POS_ABS);
    motor_3508.ID4.target_speed = motor_3508.ID4.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID4.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID4,&motor_3508.ID4 ); 
    canbuf[6] = ((short)(motor_3508.ID4.corrent_output)) >> 8;
    canbuf[7] = ((short)(motor_3508.ID4.corrent_output)) & 0x00FF;

    // ����CAN1��Ϣ
    CAN1_Send_Msg(canbuf, 8, 0);

    // ID5
    Motor_Angle_Cal_5(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_STAND_ID5, target_angle + offset);
	//Target_Pos_Setting(&pidmsg.M3508_STAND_ID5, -1152);
    motor_3508.ID5.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID5, motor_3508.ID5.POS_ABS);
    motor_3508.ID5.target_speed = motor_3508.ID5.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID5.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID5,&motor_3508.ID5 );
    canbuf2[0] = ((short)(motor_3508.ID5.corrent_output)) >> 8;
    canbuf2[1] = ((short)(motor_3508.ID5.corrent_output)) & 0x00FF;

    // ID6
    Motor_Angle_Cal_6(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_STAND_ID6, -target_angle + offset);
	//Target_Pos_Setting(&pidmsg.M3508_STAND_ID6, -1152);
    motor_3508.ID6.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID6,  motor_3508.ID6.POS_ABS);
    motor_3508.ID6.target_speed = motor_3508.ID6.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID6.corrent_output = PID_Calc( &pidmsg.M3508_SPEED_ID6,&motor_3508.ID6);
    canbuf2[2] = ((short)(motor_3508.ID6.corrent_output)) >> 8;
    canbuf2[3] = ((short)(motor_3508.ID6.corrent_output)) & 0x00FF;

    // ID7
    Motor_Angle_Cal_7(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_STAND_ID7, target_angle - offset);
	//Target_Pos_Setting(&pidmsg.M3508_STAND_ID7, 2200);
    motor_3508.ID7.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID7,  motor_3508.ID7.POS_ABS);
    motor_3508.ID7.target_speed = motor_3508.ID7.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID7.corrent_output = PID_Calc( &pidmsg.M3508_SPEED_ID7,&motor_3508.ID7);
    canbuf2[4] = ((short)(motor_3508.ID7.corrent_output)) >> 8;
    canbuf2[5] = ((short)(motor_3508.ID7.corrent_output)) & 0x00FF;

    // ID8
    Motor_Angle_Cal_8(360); // �õ����ԽǶ�
   Target_Pos_Setting(&pidmsg.M3508_STAND_ID8, -target_angle - offset);
   //Target_Pos_Setting(&pidmsg.M3508_STAND_ID8, 153);
    motor_3508.ID8.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID8,  motor_3508.ID8.POS_ABS);
    motor_3508.ID8.target_speed = motor_3508.ID8.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID8.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID8,&motor_3508.ID8 );
    canbuf2[6] = ((short)(motor_3508.ID8.corrent_output)) >> 8;
    canbuf2[7] = ((short)(motor_3508.ID8.corrent_output)) & 0x00FF;

    // ����CAN2��Ϣ
    CAN2_Send_Msg(canbuf2, 8);
}

void Motor_Auto_Run(void) //�������
{
    // ID1
    Motor_Angle_Cal_1(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_POS_ID1, Leg_angle.motorangle1);
    motor_3508.ID1.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID1, motor_3508.ID1.POS_ABS);
    motor_3508.ID1.target_speed = motor_3508.ID1.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID1.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID1, &motor_3508.ID1);
    canbuf[0] = ((short)(motor_3508.ID1.corrent_output)) >> 8;
    canbuf[1] = ((short)(motor_3508.ID1.corrent_output)) & 0x00FF;

    // ID2
    Motor_Angle_Cal_2(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_POS_ID2, Leg_angle.motorangle2);
    motor_3508.ID2.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID2, motor_3508.ID2.POS_ABS);
    motor_3508.ID2.target_speed = motor_3508.ID2.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID2.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID2, &motor_3508.ID2);
    canbuf[2] = ((short)(motor_3508.ID2.corrent_output)) >> 8;
    canbuf[3] = ((short)(motor_3508.ID2.corrent_output)) & 0x00FF;

    // ID3
    Motor_Angle_Cal_3(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_POS_ID3, Leg_angle.motorangle3);
    motor_3508.ID3.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID3, motor_3508.ID3.POS_ABS);
    motor_3508.ID3.target_speed = motor_3508.ID3.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID3.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID3, &motor_3508.ID3);
    canbuf[4] = ((short)(motor_3508.ID3.corrent_output)) >> 8;
    canbuf[5] = ((short)(motor_3508.ID3.corrent_output)) & 0x00FF;

    // ID4
    Motor_Angle_Cal_4(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_POS_ID4, Leg_angle.motorangle4);
    motor_3508.ID4.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID4, motor_3508.ID4.POS_ABS);
    motor_3508.ID4.target_speed = motor_3508.ID4.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID4.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID4, &motor_3508.ID4);
    canbuf[6] = ((short)(motor_3508.ID4.corrent_output)) >> 8;
    canbuf[7] = ((short)(motor_3508.ID4.corrent_output)) & 0x00FF;

    // ����CAN1��Ϣ
    CAN1_Send_Msg(canbuf, 8, 0);

    // ID5
    Motor_Angle_Cal_5(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_POS_ID5, Leg_angle.motorangle5);
    motor_3508.ID5.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID5, motor_3508.ID5.POS_ABS);
    motor_3508.ID5.target_speed = motor_3508.ID5.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID5.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID5, &motor_3508.ID5);
    canbuf2[0] = ((short)(motor_3508.ID5.corrent_output)) >> 8;
    canbuf2[1] = ((short)(motor_3508.ID5.corrent_output)) & 0x00FF;

    // ID6
    Motor_Angle_Cal_6(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_POS_ID6, Leg_angle.motorangle6);
    motor_3508.ID6.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID6, motor_3508.ID6.POS_ABS);
    motor_3508.ID6.target_speed = motor_3508.ID6.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID6.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID6, &motor_3508.ID6);
    canbuf2[2] = ((short)(motor_3508.ID6.corrent_output)) >> 8;
    canbuf2[3] = ((short)(motor_3508.ID6.corrent_output)) & 0x00FF;

    // ID7
    Motor_Angle_Cal_7(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_POS_ID7, Leg_angle.motorangle7);
    motor_3508.ID7.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID7, motor_3508.ID7.POS_ABS);
    motor_3508.ID7.target_speed = motor_3508.ID7.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID7.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID7, &motor_3508.ID7);
    canbuf2[4] = ((short)(motor_3508.ID7.corrent_output)) >> 8;
    canbuf2[5] = ((short)(motor_3508.ID7.corrent_output)) & 0x00FF;

    // ID8
    Motor_Angle_Cal_8(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_POS_ID8, Leg_angle.motorangle8);
    motor_3508.ID8.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID8, motor_3508.ID8.POS_ABS);
    motor_3508.ID8.target_speed = motor_3508.ID8.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID8.corrent_output = PID_Calc(&pidmsg.M3508_SPEED_ID8, &motor_3508.ID8);
    canbuf2[6] = ((short)(motor_3508.ID8.corrent_output)) >> 8;
    canbuf2[7] = ((short)(motor_3508.ID8.corrent_output)) & 0x00FF;

    // ����CAN2��Ϣ
    CAN2_Send_Msg(canbuf2, 8);

    flag_full = 0;
}






