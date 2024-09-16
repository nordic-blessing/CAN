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
	
	delay_init(168);  	                            					  //时钟初始化
	delay_ms(2000);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 					  //中断分组配置
	uart_init(115200);                              					  //串口初始化
	LED_Init();                                    						  //LED初始化
	KEY_Init();                                     					  //按键初始化
	PID_Set_Init();                                 					  //速度环角度环PID结构体初始化
	Motor_State_Init();
	TIM3_Int_Init(2000-1,84-1);
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,3,CAN_Mode_Normal);//CAN初始化普通模式,波特率1Mbps
	
	OSInit(&err);									//初始化UCOSIII
	OS_CRITICAL_ENTER();                            //进入临界区
	
	//创建开始任务
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//任务控制块
				 (CPU_CHAR	* )"start task", 		//任务名字
                 (OS_TASK_PTR )start_task, 			//任务函数
                 (void		* )0,					//传递给任务函数的参数
                 (OS_PRIO	  )START_TASK_PRIO,     //任务优先级
                 (CPU_STK   * )&START_TASK_STK[0],	//任务堆栈基地址
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//任务堆栈深度限位
                 (CPU_STK_SIZE)START_STK_SIZE,		//任务堆栈大小
                 (OS_MSG_QTY  )0,					//任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
                 (OS_TICK	  )0,					//当使能时间片轮转时的时间片长度，为0时为默认长度，
                 (void   	* )0,					//用户补充的存储区
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项
                 (OS_ERR 	* )&err);	//存放该函数错误时的返回值
								
	OS_CRITICAL_EXIT();	//退出临界区	 						 
	OSStart(&err);  //开启UCOSIII
}
