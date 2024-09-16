#include "systemInit.h"


void system_Init(void)
{
	
	delay_init(168);		  
	LED_Init();	
	uart_init(115200);	
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);//CA`��ʼ������ģʽ,������42/(1+9+4)/3=1Mbps 	
	CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);

	//2006�ٶȻ�
	PID_Init(&S_Motor.SpeedLoop);
	PID_Set_KP_KI_KD(&S_Motor.SpeedLoop,5.2,0.22,0);   //2006 �ٶȻ�pid 
	
	
	//2006�ǶȻ�
	PID_Init(&S_Motor.AngleLoop);
	PID_Set_KP_KI_KD(&S_Motor.AngleLoop,185,0.01,105);     //2006 λ�û�pid
	
	TIM_Init(TIM14,5000-1,84-1,2,0); //5ms��ʱ�����ͷ�����֡

	
}