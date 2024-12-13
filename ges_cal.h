#ifndef __POSTURE_CALCULATION_H
#define __POSTURE_CALCULATION_H
#include "Allheaderfile.h"
#include "sys.h"
#define L1 11.0 //14
#define L2 20.0 //30
#define L3 14.0
#define L4 30.0         //����������С�ȳ���
#define xa 3.5f            
#define xb -3.5f        //xa,xbΪ�����������,xa=-xb=motor_length/2(���ľ�/2)
#define PI 3.1415926f  //��,��Ҫת��Ϊf,ʹ��ɲ���floatֵ����
#define val 180.0/PI  //������ת������
#define AngleRaito 436.926337 //24��11.20 ���û�õ�
#define Gaito     3591/187//3508ת�ӱ�,�ǶȻ�����ʱ��Ҫ��ϣ���ĽǶȳ������
#define t_length  0.0025//ʱ�䲽�� 0.0025
#define t_length_w  0.0018 //0.0020
#define StatesMaxNum 20 
/***/

#define StepLenthMin 2.0f
//#define StepLenthMax (2*LegLenthExtremeMax*0.866f*0.9f)
#define StepLenthMax 17
#define LegLenthExtremeMax 29.2f //���,���ٳ���
#define LegLenthMax 30.0f //ʵ��������Ȼ���һ��ԣ�� //IMU��أ���ͬ
#define LegLenthMin 11.0f //����΢��ѹ�Źؽ�

#define LegStandLenth 15.0f //��׼վ�����ȳ�  //2023.6.21 18.5
#define LegCrouchLenth 10.0f //�¶��ȵĳ���

/*********************************************/
typedef struct
{
    float stance_height;  // ֧�Ÿ߶�
    float step_length;    // ����
    float up_amp;         // ��������
    float down_amp;       // �½�����
    float flight_percent; // ���н׶ΰٷֱ�
    float freq;           // Ƶ��
} GaitParams;

/**
 * �ܺͽṹ��
 */
typedef struct
{
    uint8_t gait_id;                // ����ID�����ڱ�ʶ��ͬ������
    GaitParams detached_params0;      // ��һ�鲽̬����
    GaitParams detached_params1;      // �ڶ��鲽̬����
    GaitParams detached_params2;      // �����鲽̬����
    GaitParams detached_params3;      // �����鲽̬����
} Detached_Params;

extern Detached_Params detached_params_imu;
/*********************************************/

/*********************************************/
typedef struct
{
    float x;
    float z;
    float L;
    float psai1;
    float fai1;
    float theta1;
    float theta2;
} Leg; //���Ȳ��˶��й�

typedef struct
{
	Leg leg1;
	Leg leg2;
	Leg leg3;
	Leg leg4;
} LEGS;	
/****************/
typedef struct
{
double dir1;
double dir2;
double dir3;
double dir4;

}MOVE_DIR; //�ƶ�������ؽṹ��
/****************/
typedef struct
{
double xf;
double zf;
double x_past;
double t_past;
}cycloid;

typedef struct
{
	cycloid curve; //�ڶ���
	cycloid support_1;
	cycloid support_2;//֧����
}Cycloid_Gait;
/****************/
extern	MOVE_DIR Move_Dir;
extern	GaitParams gait_params;
extern	LEGS Legs;
extern  Cycloid_Gait cycloid_gait;
/*********************************************/

/*******��̬********/ 
void Gait(double Tf, double x_target, double z_target, double x0, double z0, double xv0, double zf, int direction, int change, double Leglength,int gait_state);

void trot(double t, double Tf, double x_target, double z_target, double x0, double z0, double xv0, double zf, double Leglength );

void Stand(double x_offset,double Leglength);

void Stand_After_Bound(double x_offset,double Leglength);
/****************/

void CartesianToTheta_Cycloid(Leg *leg);//�˶�ѧ��⺯��

void CartesianToTheta_Cycloid_All_Legs(void); //�˶�ѧ���������

void Moveleg(int direction);

//void swing_curve_generate(double t, double Tf, double xt, double zh, double x0, double z0, double xv0);

//void support_curve_generate(double t, double Tf, double x_past, double t_past, double zf);

void swing_curve_generate(double t, double Tf, double xt,double x0,double z0);

void support_curve_generate_1(double t, double Tf, double x_target,double x0,double z0);

void support_curve_generate_2(double t, double Tf, double x_target,double x0,double z0);

void Dir_Set(int direction);

void Angle_Setting_Cycloid(int Legid, int direction);
	
void AllLeg_Set_angle(int target_angle, int offset);

void Motor_Auto_Run(void); //�������

#endif
