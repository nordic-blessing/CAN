#include "task.h"


//任务堆栈	
CPU_STK START_TASK_STK[START_STK_SIZE];
CPU_STK LED0_TASK_STK[LED0_STK_SIZE];
CPU_STK ANGLE_TASK_STK[ANGLE_STK_SIZE];        //角度环任务
CPU_STK SPEED_TASK_STK[SPEED_STK_SIZE];        //速度环任务
CPU_STK TOTAL_TASK_STK[TOTAL_STK_SIZE];        //总角度环任务

//任务控制块PCB
OS_TCB StartTaskTCB;
OS_TCB Led0TaskTCB;
OS_TCB AngleTaskTCB;
OS_TCB SpeedTaskTCB;
OS_TCB TotalTaskTCB;

//信号量
OS_SEM SPEED_SEM;
OS_SEM ANGLE_SEM;
OS_SEM TOTAL_SEM;

//逻辑全局变量  
int Zero_Middle_TakeBall_flag0 = 0;             //从零到中间再取球标志，0为云台不执行操作
int Move_Ready = 0;                   //移动标志，1为移开了
int m = 0;
uint16_t YAW_HOLD = 0;                //云台保持角
uint16_t PITCH_HOLD = 0;                //俯仰保持角
int16_t Total_AngleWant0 =  0;     //3800
int16_t Total_AngleWant1 =  0;        //0  -6650

//取球机构是否移开标志，0为没有移开

void start_task(void *p_arg)
{
    OS_ERR err;
    CPU_SR_ALLOC();
    p_arg = p_arg;

    CPU_Init();
    #if OS_CFG_STAT_TASK_EN > 0u
    OSStatTaskCPUUsageInit(&err);  	//统计任务                
    #endif
	
    #ifdef CPU_CFG_INT_DIS_MEAS_EN		//如果使能了测量中断关闭时间
    CPU_IntDisMeasMaxCurReset();	
    #endif

    #if	OS_CFG_SCHED_ROUND_ROBIN_EN  //当使用时间片轮转的时候
	 //使能时间片轮转调度功能,时间片长度为1个系统时钟节拍，既1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
    #endif		
	
	OS_CRITICAL_ENTER();	//进入临界区
	//创建信号量
	OSSemCreate((OS_SEM*	)&SPEED_SEM,
                 (CPU_CHAR*	)"SPEED_SEM",
                 (OS_SEM_CTR)1,		
                 (OS_ERR*	)&err);
								 
	OSSemCreate((OS_SEM*	)&ANGLE_SEM,
                 (CPU_CHAR*	)"ANGLE_SEM",
                 (OS_SEM_CTR)1,		
                 (OS_ERR*	)&err);
				 
	OSSemCreate((OS_SEM*	)&TOTAL_SEM,
                 (CPU_CHAR*	)"TOTAL_SEM",
                 (OS_SEM_CTR)1,		
                 (OS_ERR*	)&err);
	
	//创建LED0任务
	OSTaskCreate((OS_TCB 	* )&Led0TaskTCB,		
				 (CPU_CHAR	* )"led0 task", 		
                 (OS_TASK_PTR )led0_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )LED0_TASK_PRIO,     
                 (CPU_STK   * )&LED0_TASK_STK[0],	
                 (CPU_STK_SIZE)LED0_STK_SIZE/10,	
                 (CPU_STK_SIZE)LED0_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);				
				 

  //创建角度环任务
	OSTaskCreate((OS_TCB 	* )&AngleTaskTCB,		
				 (CPU_CHAR	* )"angle task", 		
                 (OS_TASK_PTR )angle_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )ANGLE_TASK_PRIO,     	
                 (CPU_STK   * )&ANGLE_TASK_STK[0],	
                 (CPU_STK_SIZE)ANGLE_STK_SIZE/10,	
                 (CPU_STK_SIZE)ANGLE_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);	
  

	//创建速度环任务							 
	OSTaskCreate((OS_TCB 	* )&SpeedTaskTCB,		
				 (CPU_CHAR	* )"speed task", 		
                 (OS_TASK_PTR )speed_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )SPEED_TASK_PRIO,     	
                 (CPU_STK   * )&SPEED_TASK_STK[0],	
                 (CPU_STK_SIZE)SPEED_STK_SIZE/10,	
                 (CPU_STK_SIZE)SPEED_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);	
								 
	//创建总角度控制任务							 
	OSTaskCreate((OS_TCB 	* )&TotalTaskTCB,		
				 (CPU_CHAR	* )"total task", 		
                 (OS_TASK_PTR )total_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )TOTAL_TASK_PRIO,     	
                 (CPU_STK   * )&TOTAL_TASK_STK[0],	
                 (CPU_STK_SIZE)TOTAL_STK_SIZE/10,	
                 (CPU_STK_SIZE)TOTAL_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);	
				 
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//挂起开始任务			 
	OS_CRITICAL_EXIT();	//退出临界区
}


void led0_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;	
	while(1)
	{
		LED0=~LED0;              //LED0闪烁
		
		/*打印开始*/
//		printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f\r\n",motor_info[0].set_voltage,
//		                                              motor_info[0].can_id,
//										              motor_info[0].rotor_angle,
//										              motor_info[0].rotor_speed,
//		                                              motor_info[0].temp,
//		                                              motor_info[0].torque_current,
//										              motor_info[0].last_angle,
//										              motor_info[0].offset_angle,
//										              motor_info[0].total_angle,
//										              motor_info[0].total_cnt,
//										              AngleWant[0]);
		
		printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%d\r\n",motor_info[1].set_voltage,
		                                                 motor_info[1].can_id,
										                 motor_info[1].rotor_angle,
										                 motor_info[1].rotor_speed,
		                                                 motor_info[1].temp,
		                                                 motor_info[1].torque_current,
										                 motor_info[1].last_angle,
										                 motor_info[1].offset_angle,
										                 motor_info[1].total_angle,
										                 motor_info[1].total_cnt,
										                 AngleWant[1],
													     OSStatTaskCPUUsage); 

//		printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f\r\n",motor_info[2].set_voltage,
//		                                              motor_info[2].can_id,
//										              motor_info[2].rotor_angle,
//										              motor_info[2].rotor_speed,
//		                                              motor_info[2].temp,
//		                                              motor_info[2].torque_current,
//										              motor_info[2].last_angle,
//										              motor_info[2].offset_angle,
//										              motor_info[2].total_angle,
//										              motor_info[2].total_cnt,
//													  motor_info[2].real_angle,
//										              AngleWant[2]);

//		printf("%d,%d\r\n",control_info[0].angle_yaw,control_info[0].angle_pitch);
//		printf("%d,%d\r\n",control_info[1].ball_take_flag, control_info[1].YAW_BIAS);
//		/*打印结束*/
		OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err); //延时1ms
	}

}


void angle_task(void *p_arg)             //单圈位置控制
{
	int i;
	int key = 0;          //按键扫描标志
	uint8_t can_send_flag = 1;   //CAN发送成功标志,0为成功
	OS_ERR err;
	p_arg = p_arg;
	OS_TaskSuspend((OS_TCB*)&AngleTaskTCB,&err);            //若需要可挂起自身
	for(i=0;i<4;i++)          //初始化设置角度期望值2000
	{
		AngleWant[i]=3000;	
	    motor_info[i].rotor_angle = AngleWant[i];
	}
	
	while(1)
	{
		can_send_flag = 1;
		OSSemPend(&ANGLE_SEM,0,OS_OPT_PEND_BLOCKING,0,&err); //等待角度换控制信号量
		
		key = KEY_Scan(0);                                   //按键控制角度值
		if (key == KEY1_PRES)    //按下key1角度加
		{
			AngleWant[0] += 100;	
		}
		else if (key == KEY2_PRES)    //按下key2角度归位
		{
			AngleWant[0] = 0;	
		}
		
		while(can_send_flag)
		{
			can_send_flag = Angle_Control(motor_info, AngleWant[0],AngleWant[1],AngleWant[2],AngleWant[3]);		     //CAN发送数据
		}
	}
}

void speed_task(void *p_arg)
{
	uint8_t can_send_flag = 1;   //CAN发送成功标志,0为成功
	int flaga = 0;               //取球动作切换标志
	int time = 50;               //取球间隔time * 4 ms
	int middle = 20000;       //在中间某个位置  
	int low = 95000;          //低取球位
    int up = 10000;           //往高归位
	OS_ERR err;
	motor_info[2].rotor_speed = 0;
	p_arg = p_arg;
    //OS_TaskSuspend((OS_TCB*)&SpeedTaskTCB,&err);          //若需要可挂起自身
	motor_info[2].total_angle = (control_info[1].ball_take_flag == 1) ? (95000):(0);      //设置初值避免上电就动
	while(1)
	{
		switch(control_info[1].ball_take_flag)     //初值为0
		{
			case 1:     //收球判断
				m = 1;
				break;
			case 3:     //自瞄判断，即移开收球机构
				m = 0;
				break;
		}
		
		if(m == 1)   //收球
		{
			Zero_Middle_TakeBall_flag0 = 1;           //执行收球动作，初值为0
			if(Move_Ready == 0)                       //云台俯仰已经归位
			{
				if(flaga < time)
				{
					SetPoint(&PID_3508Loop[0],10000);        //移到0位置，准备取球
					flaga++;
				}
				else if((flaga < 2 * time) && (flaga >= time))              //往下取球
				{
					SetPoint(&PID_3508Loop[0],low);        //移到low位置，进行取球
					flaga++;
				}
				else if(flaga >=  2 * time)       //往上归位
				{
					SetPoint(&PID_3508Loop[0],up);        //移到up位，归零
					flaga++;
					if(flaga > time * 3)   
					{
						flaga = 0;	
						m = 0;       //此时不收球
						Zero_Middle_TakeBall_flag0 = 0;           //执行收球动作完成
						control_info[1].ball_take_flag = 0;
					}
				}
			}
			else if(Move_Ready == 1)
			{
				SetPoint(&PID_3508Loop[0],middle);        //移到20000位置，此时应该不会干扰云台和俯仰移动
			}
		}
		else if(m == 0)     //不收球
		{
			SetPoint(&PID_3508Loop[0],middle);        //移到20000位置，此时应该不会干扰云台和俯仰移动
			Move_Ready = 1;                           //置云台俯仰可移动标志为1
		}
		
		SpeedWant[0] = PID_IncCalc(&PID_3508Loop[0],motor_info[2].total_angle);
		can_send_flag = 1;
		OSSemPend(&SPEED_SEM,0,OS_OPT_PEND_BLOCKING,0,&err);
		while(can_send_flag)
		{
			can_send_flag = Speed_Control(motor_info, SpeedWant[0],0,0,0); 
		}
	}
}

void total_task(void *p_arg)
{
//	int flagc = 1;
	int flaga = 0;            //到位计数标志
	int n = 0;                //移动功能标志位
//	int time = 500;
	int YAW_BALL_TAKE = 4000;             //云台取球位对应角度
	int PITCH_BALL_TAKE  = 0;             //俯仰取球位对应角度
	int YAW_BIAS = 0;                     //云台角度偏差
	int PITCH_BIAS = 0;                   //俯仰角度偏差
	int Auto_bias = 0;                    //自动瞄准时的镜头偏差
	int want = 0;
	OS_ERR err;
	p_arg = p_arg;
    //OS_TaskSuspend((OS_TCB*)&TotalTaskTCB,&err);          //若需要可挂起自身
	motor_info[0].total_angle = Total_AngleWant0;
	motor_info[1].total_angle = Total_AngleWant1;
	
	while(1)
	{
		switch(control_info[1].ball_take_flag)                //判断功能
		{
			case 3:
				n = 3;
				break;
			case 0:
				n = 1;
				break;
		}			
		
		OSSemPend(&ANGLE_SEM,0,OS_OPT_PEND_BLOCKING,0,&err);
		if(Zero_Middle_TakeBall_flag0 == 0 && Move_Ready == 1)             //云台不执行取球操作,俯仰此时应该是移开状态，即Move_Ready = 1   -> 操控状态
		{
			if(n != 3)               //手动瞄准
			{
				if(control_info[0].angle_yaw >6600)  control_info[0].angle_yaw = 6600;
				if(control_info[0].angle_pitch > 6100)  control_info[0].angle_pitch = 6100;
				if(control_info[0].angle_yaw < 0)  control_info[0].angle_yaw = 0;
				if(control_info[0].angle_pitch < 0)  control_info[0].angle_pitch = 0;
				Total_AngleWant0 = control_info[0].angle_yaw;
				Total_AngleWant1 = control_info[0].angle_pitch;
				Motor_Set_Round_angle(motor_info, 0, Total_AngleWant0, 0, Total_AngleWant1);    //电机转动圈数和总角度设置
			}
			else if(n == 3)          //自动瞄准
			{
				SetPoint(&PID_PIXLoop[0],0);                  //期望误差0
				Auto_bias = control_info[1].YAW_BIAS;         //获得水平偏差
				want = PID_PosLocCalc(&PID_PIXLoop[0],Auto_bias);   //PID计算
				want = -want + control_info[0].angle_yaw;
				if(want >6600)  want = 6600;
				if(want < 0)  want = 0;
				if(control_info[0].angle_pitch > 6100)  control_info[0].angle_pitch = 6100;
				if(control_info[0].angle_pitch < 0)  control_info[0].angle_pitch = 0;
//				printf("%d,%d,%d\r\n",Auto_bias,want,control_info[0].angle_pitch);
				Motor_Set_Round_angle(motor_info, 0, want, 0, control_info[0].angle_pitch);
			}	
		}
		else if(Zero_Middle_TakeBall_flag0 == 1 && Move_Ready == 1)        //云台执行取球操作,云台移动时俯仰应该移开 -> 取球预备
		{
			Motor_Set_Round_angle(motor_info, 0, YAW_BALL_TAKE, 0, PITCH_BALL_TAKE);    //电机转动圈数和总角度设置
			YAW_BIAS = motor_info[0].total_angle - YAW_BALL_TAKE;              //得到云台实时偏差
			PITCH_BIAS = motor_info[1].total_angle - PITCH_BALL_TAKE;          //得到俯仰实时偏差
			if(YAW_BIAS < 50 && PITCH_BIAS < 50)                          //稳定时flaga++
			{
				Move_Ready = 1;        //俯仰保持移开状态
				flaga++;                 //计数++
			}
			if(flaga > 100)                                                //稳定超过2*100 = 200ms则认为稳定到位   
			{
				Move_Ready = 0;        //收球可以归位标志 俯仰已经归位
				flaga = 0;               //计数置零
			}
		} 
		else if(Move_Ready == 0)                //俯仰归位标志，则此时云台俯仰保持不动
		{
			Motor_Set_Round_angle(motor_info, 0, YAW_BALL_TAKE, 0, PITCH_BALL_TAKE);    //目标值设定为当前角度
		}
	}
}

