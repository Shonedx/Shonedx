#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"
#include "pid.h"

typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
	CAN_CHASSIS_ALL_ID_2 = 0x1FF,	 
} can_msg_id_e;//各id号地址

	
//CAN1接收RX0中断使能
extern u8 canbuf[8];
extern u8 canbuf2[8];
extern u8 flag_full;

#define CAN1_RX0_INT_ENABLE	1		//0,不使能;1,使能.			
#define CAN2_RX0_INT_ENABLE	1		//0,不使能;1,使能.	
										 							 				    
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化
u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化

u8 CAN1_Send_Msg(u8* msg,u8 len,u8 expend);						//发送数据
u8 CAN2_Send_Msg(u8* msg,u8 len);						//发送数据

u8 CAN1_Receive_Msg(u8 *buf);							//接收数据
u8 CAN2_Receive_Msg(u8 *buf);							//接收数据

void set_motor_3508_massage(MOTOR_3508_MSG * motor_msg,u8 * buf);

                                 

void M3508_ALL_ZERO_SET(void);



#endif



























