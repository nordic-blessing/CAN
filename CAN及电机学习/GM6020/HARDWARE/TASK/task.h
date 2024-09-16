#ifndef __TASK_H
#define __TASK_H

#include "includes.h"
#include "led.h"
#include "motor.h"
#include "pid.h"
#include "os_app_hooks.h"
#include "key.h"
#include "can.h"

//任务优先级
#define START_TASK_PRIO      			4 	//开始任务
#define LED0_TASK_PRIO       			3 	//LED灯任务, 串口打印调试任务
#define ANGLE_TASK_PRIO       		    2 	//角度环任务
#define SPEED_TASK_PRIO       		    1 	//速度环任务
#define TOTAL_TASK_PRIO       		    1 	//速度环任务

//任务堆栈大小
#define START_STK_SIZE  				512u
#define LED0_STK_SIZE  				    128u
#define ANGLE_STK_SIZE  				512u
#define SPEED_STK_SIZE      			512u
#define TOTAL_STK_SIZE      			512u

//任务堆栈	
extern  CPU_STK START_TASK_STK[START_STK_SIZE];
extern  CPU_STK LED0_TASK_STK[LED0_STK_SIZE];
extern  CPU_STK ANGLE_TASK_STK[ANGLE_STK_SIZE];
extern  CPU_STK SPEED_TASK_STK[SPEED_STK_SIZE];
extern  CPU_STK SPEED_TASK_STK[TOTAL_STK_SIZE];

//任务控制块
extern OS_TCB StartTaskTCB;
extern OS_TCB Led0TaskTCB;
extern OS_TCB AngleTaskTCB;
extern OS_TCB SpeedTaskTCB;
extern OS_TCB TotalTaskTCB;

//信号量
extern OS_SEM SPEED_SEM;
extern OS_SEM ANGLE_SEM;
extern OS_SEM TOTAL_SEM;

//任务函数
void start_task(void *p_arg);	
void led0_task(void *p_arg);	
void angle_task(void *p_arg);
void speed_task(void *p_arg);
void total_task(void *p_arg);

/*外部变量*/
extern moto_info_t motor_info[MOTOR_MAX_NUM];       //电机数据数组



#endif  /* __TASK_H */
