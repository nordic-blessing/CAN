#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "includes.h"
#include "os_app_hooks.h"
#include "task.h"
#include "timer.h"
#include "pid.h"
#include "motor.h"

int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	
	delay_init(168);  	                            					  //ʱ�ӳ�ʼ��
	delay_ms(2000);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 					  //�жϷ�������
	uart_init(115200);                              					  //���ڳ�ʼ��
	LED_Init();                                    						  //LED��ʼ��
	KEY_Init();                                     					  //������ʼ��
	PID_Set_Init();                                 					  //�ٶȻ��ǶȻ�PID�ṹ���ʼ��
	Motor_State_Init();
	TIM3_Int_Init(2000-1,84-1);
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,3,CAN_Mode_Normal);//CAN��ʼ����ͨģʽ,������1Mbps
	
	OSInit(&err);									//��ʼ��UCOSIII
	OS_CRITICAL_ENTER();                            //�����ٽ���
	
	//������ʼ����
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//������ƿ�
				 (CPU_CHAR	* )"start task", 		//��������
                 (OS_TASK_PTR )start_task, 			//������
                 (void		* )0,					//���ݸ��������Ĳ���
                 (OS_PRIO	  )START_TASK_PRIO,     //�������ȼ�
                 (CPU_STK   * )&START_TASK_STK[0],	//�����ջ����ַ
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//�����ջ�����λ
                 (CPU_STK_SIZE)START_STK_SIZE,		//�����ջ��С
                 (OS_MSG_QTY  )0,					//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,					//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                 (void   	* )0,					//�û�����Ĵ洢��
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                 (OS_ERR 	* )&err);	//��Ÿú�������ʱ�ķ���ֵ
								
	OS_CRITICAL_EXIT();	//�˳��ٽ���	 						 
	OSStart(&err);  //����UCOSIII
}
