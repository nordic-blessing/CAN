#ifndef __TASK_H
#define __TASK_H

#include "includes.h"
#include "led.h"
#include "motor.h"
#include "pid.h"
#include "os_app_hooks.h"
#include "key.h"
#include "can.h"

//�������ȼ�
#define START_TASK_PRIO      			4 	//��ʼ����
#define LED0_TASK_PRIO       			3 	//LED������, ���ڴ�ӡ��������
#define ANGLE_TASK_PRIO       		    2 	//�ǶȻ�����
#define SPEED_TASK_PRIO       		    1 	//�ٶȻ�����
#define TOTAL_TASK_PRIO       		    1 	//�ٶȻ�����

//�����ջ��С
#define START_STK_SIZE  				512u
#define LED0_STK_SIZE  				    128u
#define ANGLE_STK_SIZE  				512u
#define SPEED_STK_SIZE      			512u
#define TOTAL_STK_SIZE      			512u

//�����ջ	
extern  CPU_STK START_TASK_STK[START_STK_SIZE];
extern  CPU_STK LED0_TASK_STK[LED0_STK_SIZE];
extern  CPU_STK ANGLE_TASK_STK[ANGLE_STK_SIZE];
extern  CPU_STK SPEED_TASK_STK[SPEED_STK_SIZE];
extern  CPU_STK SPEED_TASK_STK[TOTAL_STK_SIZE];

//������ƿ�
extern OS_TCB StartTaskTCB;
extern OS_TCB Led0TaskTCB;
extern OS_TCB AngleTaskTCB;
extern OS_TCB SpeedTaskTCB;
extern OS_TCB TotalTaskTCB;

//�ź���
extern OS_SEM SPEED_SEM;
extern OS_SEM ANGLE_SEM;
extern OS_SEM TOTAL_SEM;

//������
void start_task(void *p_arg);	
void led0_task(void *p_arg);	
void angle_task(void *p_arg);
void speed_task(void *p_arg);
void total_task(void *p_arg);

/*�ⲿ����*/
extern moto_info_t motor_info[MOTOR_MAX_NUM];       //�����������



#endif  /* __TASK_H */
