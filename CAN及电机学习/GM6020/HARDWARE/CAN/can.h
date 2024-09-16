#ifndef __CAN_H
#define __CAN_H	 

#include "sys.h"	
#include "motor.h"


//#define FEEDBACK_ID_BASE      0x205    6020
#define FEEDBACK_ID0_BASE      0x205    //云台电机1的CAN接收ID
#define FEEDBACK_ID1_BASE      0x206    //云台电机2的
#define FEEDBACK_ID2_BASE      0x201    //收球电机3508的
#define CONTROL_BASE0           0x100
#define CONTROL_BASE1           0x101

#define YAW_START      4000    //云台初始角度
#define PITCH_START    0

#define CAN_CONTROL_ID_BASE   0x1ff
#define CAN_CONTROL_ID_EXTEND 0x2ff
#define MOTOR_MAX_NUM         7
#define CAN2_PORT        CAN2
#define CAN2_GPIO_PORT   GPIOB
#define CAN2_RX_PIN      GPIO_Pin_12
#define CAN2_TX_PIN      GPIO_Pin_13


//CAN1接收RX0中断使能
#define CAN2_RX0_INT_ENABLE	1		//0,不使能;1,使能.	
#define CAN1_RX0_INT_ENABLE	1		//0,不使能;1,使能.
										 							 				    
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化

u8 set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);    
u8 set_ACK(uint8_t id_range, int Success_Flag);  

#endif

















