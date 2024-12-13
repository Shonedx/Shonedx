#ifndef __POSTURE_CALCULATION_H
#define __POSTURE_CALCULATION_H
#include "Allheaderfile.h"
#include "sys.h"
#define L1 11.0 //14
#define L2 20.0 //30
#define L3 14.0
#define L4 30.0         //两大腿与两小腿长度
#define xa 3.5f            
#define xb -3.5f        //xa,xb为两电机点坐标,xa=-xb=motor_length/2(轴心距/2)
#define PI 3.1415926f  //Π,需要转换为f,使其可参与float值运算
#define val 180.0/PI  //弧度制转换参数
#define AngleRaito 436.926337 //24年11.20 这个没用到
#define Gaito     3591/187//3508转子比,角度环控制时需要将希望的角度乘上这个
#define t_length  0.0025//时间步长 0.0025
#define t_length_w  0.0018 //0.0020
#define StatesMaxNum 20 
/***/

#define StepLenthMin 2.0f
//#define StepLenthMax (2*LegLenthExtremeMax*0.866f*0.9f)
#define StepLenthMax 17
#define LegLenthExtremeMax 29.2f //最长了,别再长了
#define LegLenthMax 30.0f //实际上离最长腿还有一定裕度 //IMU相关，下同
#define LegLenthMin 11.0f //有略微挤压髋关节

#define LegStandLenth 15.0f //标准站立的腿长  //2023.6.21 18.5
#define LegCrouchLenth 10.0f //下蹲腿的长度

/*********************************************/
typedef struct
{
    float stance_height;  // 支撑高度
    float step_length;    // 步长
    float up_amp;         // 上升幅度
    float down_amp;       // 下降幅度
    float flight_percent; // 飞行阶段百分比
    float freq;           // 频率
} GaitParams;

/**
 * 总和结构体
 */
typedef struct
{
    uint8_t gait_id;                // 姿势ID，用于标识不同的姿势
    GaitParams detached_params0;      // 第一组步态参数
    GaitParams detached_params1;      // 第二组步态参数
    GaitParams detached_params2;      // 第三组步态参数
    GaitParams detached_params3;      // 第四组步态参数
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
} Leg; //和腿部运动有关

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

}MOVE_DIR; //移动方向相关结构体
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
	cycloid curve; //摆动相
	cycloid support_1;
	cycloid support_2;//支撑相
}Cycloid_Gait;
/****************/
extern	MOVE_DIR Move_Dir;
extern	GaitParams gait_params;
extern	LEGS Legs;
extern  Cycloid_Gait cycloid_gait;
/*********************************************/

/*******步态********/ 
void Gait(double Tf, double x_target, double z_target, double x0, double z0, double xv0, double zf, int direction, int change, double Leglength,int gait_state);

void trot(double t, double Tf, double x_target, double z_target, double x0, double z0, double xv0, double zf, double Leglength );

void Stand(double x_offset,double Leglength);

void Stand_After_Bound(double x_offset,double Leglength);
/****************/

void CartesianToTheta_Cycloid(Leg *leg);//运动学逆解函数

void CartesianToTheta_Cycloid_All_Legs(void); //运动学逆解所有腿

void Moveleg(int direction);

//void swing_curve_generate(double t, double Tf, double xt, double zh, double x0, double z0, double xv0);

//void support_curve_generate(double t, double Tf, double x_past, double t_past, double zf);

void swing_curve_generate(double t, double Tf, double xt,double x0,double z0);

void support_curve_generate_1(double t, double Tf, double x_target,double x0,double z0);

void support_curve_generate_2(double t, double Tf, double x_target,double x0,double z0);

void Dir_Set(int direction);

void Angle_Setting_Cycloid(int Legid, int direction);
	
void AllLeg_Set_angle(int target_angle, int offset);

void Motor_Auto_Run(void); //驱动电机

#endif
