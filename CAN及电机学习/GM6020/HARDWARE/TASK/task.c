#include "task.h"


//�����ջ	
CPU_STK START_TASK_STK[START_STK_SIZE];
CPU_STK LED0_TASK_STK[LED0_STK_SIZE];
CPU_STK ANGLE_TASK_STK[ANGLE_STK_SIZE];        //�ǶȻ�����
CPU_STK SPEED_TASK_STK[SPEED_STK_SIZE];        //�ٶȻ�����
CPU_STK TOTAL_TASK_STK[TOTAL_STK_SIZE];        //�ܽǶȻ�����

//������ƿ�PCB
OS_TCB StartTaskTCB;
OS_TCB Led0TaskTCB;
OS_TCB AngleTaskTCB;
OS_TCB SpeedTaskTCB;
OS_TCB TotalTaskTCB;

//�ź���
OS_SEM SPEED_SEM;
OS_SEM ANGLE_SEM;
OS_SEM TOTAL_SEM;

//�߼�ȫ�ֱ���  
int Zero_Middle_TakeBall_flag0 = 0;             //���㵽�м���ȡ���־��0Ϊ��̨��ִ�в���
int Move_Ready = 0;                   //�ƶ���־��1Ϊ�ƿ���
int m = 0;
uint16_t YAW_HOLD = 0;                //��̨���ֽ�
uint16_t PITCH_HOLD = 0;                //�������ֽ�
int16_t Total_AngleWant0 =  0;     //3800
int16_t Total_AngleWant1 =  0;        //0  -6650

//ȡ������Ƿ��ƿ���־��0Ϊû���ƿ�

void start_task(void *p_arg)
{
    OS_ERR err;
    CPU_SR_ALLOC();
    p_arg = p_arg;

    CPU_Init();
    #if OS_CFG_STAT_TASK_EN > 0u
    OSStatTaskCPUUsageInit(&err);  	//ͳ������                
    #endif
	
    #ifdef CPU_CFG_INT_DIS_MEAS_EN		//���ʹ���˲����жϹر�ʱ��
    CPU_IntDisMeasMaxCurReset();	
    #endif

    #if	OS_CFG_SCHED_ROUND_ROBIN_EN  //��ʹ��ʱ��Ƭ��ת��ʱ��
	 //ʹ��ʱ��Ƭ��ת���ȹ���,ʱ��Ƭ����Ϊ1��ϵͳʱ�ӽ��ģ���1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
    #endif		
	
	OS_CRITICAL_ENTER();	//�����ٽ���
	//�����ź���
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
	
	//����LED0����
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
				 

  //�����ǶȻ�����
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
  

	//�����ٶȻ�����							 
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
								 
	//�����ܽǶȿ�������							 
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
				 
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//����ʼ����			 
	OS_CRITICAL_EXIT();	//�˳��ٽ���
}


void led0_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;	
	while(1)
	{
		LED0=~LED0;              //LED0��˸
		
		/*��ӡ��ʼ*/
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
//		/*��ӡ����*/
		OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ1ms
	}

}


void angle_task(void *p_arg)             //��Ȧλ�ÿ���
{
	int i;
	int key = 0;          //����ɨ���־
	uint8_t can_send_flag = 1;   //CAN���ͳɹ���־,0Ϊ�ɹ�
	OS_ERR err;
	p_arg = p_arg;
	OS_TaskSuspend((OS_TCB*)&AngleTaskTCB,&err);            //����Ҫ�ɹ�������
	for(i=0;i<4;i++)          //��ʼ�����ýǶ�����ֵ2000
	{
		AngleWant[i]=3000;	
	    motor_info[i].rotor_angle = AngleWant[i];
	}
	
	while(1)
	{
		can_send_flag = 1;
		OSSemPend(&ANGLE_SEM,0,OS_OPT_PEND_BLOCKING,0,&err); //�ȴ��ǶȻ������ź���
		
		key = KEY_Scan(0);                                   //�������ƽǶ�ֵ
		if (key == KEY1_PRES)    //����key1�Ƕȼ�
		{
			AngleWant[0] += 100;	
		}
		else if (key == KEY2_PRES)    //����key2�Ƕȹ�λ
		{
			AngleWant[0] = 0;	
		}
		
		while(can_send_flag)
		{
			can_send_flag = Angle_Control(motor_info, AngleWant[0],AngleWant[1],AngleWant[2],AngleWant[3]);		     //CAN��������
		}
	}
}

void speed_task(void *p_arg)
{
	uint8_t can_send_flag = 1;   //CAN���ͳɹ���־,0Ϊ�ɹ�
	int flaga = 0;               //ȡ�����л���־
	int time = 50;               //ȡ����time * 4 ms
	int middle = 20000;       //���м�ĳ��λ��  
	int low = 95000;          //��ȡ��λ
    int up = 10000;           //���߹�λ
	OS_ERR err;
	motor_info[2].rotor_speed = 0;
	p_arg = p_arg;
    //OS_TaskSuspend((OS_TCB*)&SpeedTaskTCB,&err);          //����Ҫ�ɹ�������
	motor_info[2].total_angle = (control_info[1].ball_take_flag == 1) ? (95000):(0);      //���ó�ֵ�����ϵ�Ͷ�
	while(1)
	{
		switch(control_info[1].ball_take_flag)     //��ֵΪ0
		{
			case 1:     //�����ж�
				m = 1;
				break;
			case 3:     //�����жϣ����ƿ��������
				m = 0;
				break;
		}
		
		if(m == 1)   //����
		{
			Zero_Middle_TakeBall_flag0 = 1;           //ִ������������ֵΪ0
			if(Move_Ready == 0)                       //��̨�����Ѿ���λ
			{
				if(flaga < time)
				{
					SetPoint(&PID_3508Loop[0],10000);        //�Ƶ�0λ�ã�׼��ȡ��
					flaga++;
				}
				else if((flaga < 2 * time) && (flaga >= time))              //����ȡ��
				{
					SetPoint(&PID_3508Loop[0],low);        //�Ƶ�lowλ�ã�����ȡ��
					flaga++;
				}
				else if(flaga >=  2 * time)       //���Ϲ�λ
				{
					SetPoint(&PID_3508Loop[0],up);        //�Ƶ�upλ������
					flaga++;
					if(flaga > time * 3)   
					{
						flaga = 0;	
						m = 0;       //��ʱ������
						Zero_Middle_TakeBall_flag0 = 0;           //ִ�����������
						control_info[1].ball_take_flag = 0;
					}
				}
			}
			else if(Move_Ready == 1)
			{
				SetPoint(&PID_3508Loop[0],middle);        //�Ƶ�20000λ�ã���ʱӦ�ò��������̨�͸����ƶ�
			}
		}
		else if(m == 0)     //������
		{
			SetPoint(&PID_3508Loop[0],middle);        //�Ƶ�20000λ�ã���ʱӦ�ò��������̨�͸����ƶ�
			Move_Ready = 1;                           //����̨�������ƶ���־Ϊ1
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
	int flaga = 0;            //��λ������־
	int n = 0;                //�ƶ����ܱ�־λ
//	int time = 500;
	int YAW_BALL_TAKE = 4000;             //��̨ȡ��λ��Ӧ�Ƕ�
	int PITCH_BALL_TAKE  = 0;             //����ȡ��λ��Ӧ�Ƕ�
	int YAW_BIAS = 0;                     //��̨�Ƕ�ƫ��
	int PITCH_BIAS = 0;                   //�����Ƕ�ƫ��
	int Auto_bias = 0;                    //�Զ���׼ʱ�ľ�ͷƫ��
	int want = 0;
	OS_ERR err;
	p_arg = p_arg;
    //OS_TaskSuspend((OS_TCB*)&TotalTaskTCB,&err);          //����Ҫ�ɹ�������
	motor_info[0].total_angle = Total_AngleWant0;
	motor_info[1].total_angle = Total_AngleWant1;
	
	while(1)
	{
		switch(control_info[1].ball_take_flag)                //�жϹ���
		{
			case 3:
				n = 3;
				break;
			case 0:
				n = 1;
				break;
		}			
		
		OSSemPend(&ANGLE_SEM,0,OS_OPT_PEND_BLOCKING,0,&err);
		if(Zero_Middle_TakeBall_flag0 == 0 && Move_Ready == 1)             //��̨��ִ��ȡ�����,������ʱӦ�����ƿ�״̬����Move_Ready = 1   -> �ٿ�״̬
		{
			if(n != 3)               //�ֶ���׼
			{
				if(control_info[0].angle_yaw >6600)  control_info[0].angle_yaw = 6600;
				if(control_info[0].angle_pitch > 6100)  control_info[0].angle_pitch = 6100;
				if(control_info[0].angle_yaw < 0)  control_info[0].angle_yaw = 0;
				if(control_info[0].angle_pitch < 0)  control_info[0].angle_pitch = 0;
				Total_AngleWant0 = control_info[0].angle_yaw;
				Total_AngleWant1 = control_info[0].angle_pitch;
				Motor_Set_Round_angle(motor_info, 0, Total_AngleWant0, 0, Total_AngleWant1);    //���ת��Ȧ�����ܽǶ�����
			}
			else if(n == 3)          //�Զ���׼
			{
				SetPoint(&PID_PIXLoop[0],0);                  //�������0
				Auto_bias = control_info[1].YAW_BIAS;         //���ˮƽƫ��
				want = PID_PosLocCalc(&PID_PIXLoop[0],Auto_bias);   //PID����
				want = -want + control_info[0].angle_yaw;
				if(want >6600)  want = 6600;
				if(want < 0)  want = 0;
				if(control_info[0].angle_pitch > 6100)  control_info[0].angle_pitch = 6100;
				if(control_info[0].angle_pitch < 0)  control_info[0].angle_pitch = 0;
//				printf("%d,%d,%d\r\n",Auto_bias,want,control_info[0].angle_pitch);
				Motor_Set_Round_angle(motor_info, 0, want, 0, control_info[0].angle_pitch);
			}	
		}
		else if(Zero_Middle_TakeBall_flag0 == 1 && Move_Ready == 1)        //��ִ̨��ȡ�����,��̨�ƶ�ʱ����Ӧ���ƿ� -> ȡ��Ԥ��
		{
			Motor_Set_Round_angle(motor_info, 0, YAW_BALL_TAKE, 0, PITCH_BALL_TAKE);    //���ת��Ȧ�����ܽǶ�����
			YAW_BIAS = motor_info[0].total_angle - YAW_BALL_TAKE;              //�õ���̨ʵʱƫ��
			PITCH_BIAS = motor_info[1].total_angle - PITCH_BALL_TAKE;          //�õ�����ʵʱƫ��
			if(YAW_BIAS < 50 && PITCH_BIAS < 50)                          //�ȶ�ʱflaga++
			{
				Move_Ready = 1;        //���������ƿ�״̬
				flaga++;                 //����++
			}
			if(flaga > 100)                                                //�ȶ�����2*100 = 200ms����Ϊ�ȶ���λ   
			{
				Move_Ready = 0;        //������Թ�λ��־ �����Ѿ���λ
				flaga = 0;               //��������
			}
		} 
		else if(Move_Ready == 0)                //������λ��־�����ʱ��̨�������ֲ���
		{
			Motor_Set_Round_angle(motor_info, 0, YAW_BALL_TAKE, 0, PITCH_BALL_TAKE);    //Ŀ��ֵ�趨Ϊ��ǰ�Ƕ�
		}
	}
}

