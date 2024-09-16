#ifndef __CAN_H
#define __CAN_H	 

#include "sys.h"	
#include "motor.h"


//#define FEEDBACK_ID_BASE      0x205    6020
#define FEEDBACK_ID0_BASE      0x205    //��̨���1��CAN����ID
#define FEEDBACK_ID1_BASE      0x206    //��̨���2��
#define FEEDBACK_ID2_BASE      0x201    //������3508��
#define CONTROL_BASE0           0x100
#define CONTROL_BASE1           0x101

#define YAW_START      4000    //��̨��ʼ�Ƕ�
#define PITCH_START    0

#define CAN_CONTROL_ID_BASE   0x1ff
#define CAN_CONTROL_ID_EXTEND 0x2ff
#define MOTOR_MAX_NUM         7
#define CAN2_PORT        CAN2
#define CAN2_GPIO_PORT   GPIOB
#define CAN2_RX_PIN      GPIO_Pin_12
#define CAN2_TX_PIN      GPIO_Pin_13


//CAN1����RX0�ж�ʹ��
#define CAN2_RX0_INT_ENABLE	1		//0,��ʹ��;1,ʹ��.	
#define CAN1_RX0_INT_ENABLE	1		//0,��ʹ��;1,ʹ��.
										 							 				    
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ��

u8 set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);    
u8 set_ACK(uint8_t id_range, int Success_Flag);  

#endif

















