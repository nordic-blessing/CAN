#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "pid.h"
#include "math.h"
#include "includes.h"

CanTxMsg TxMessage;    /*定义帧数据*/
CanRxMsg RxMessage;

moto_info_t motor_info[MOTOR_MAX_NUM];       //返回电机数据数组
Control_type control_info[2];                //外部控制结构
extern int m;

//CAN初始化
//tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:时间段2的时间单元.   范围:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:时间段1的时间单元.   范围:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :波特率分频器.范围:1~1024; tq=(brp)*tpclk1
//波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
//Fpclk1的时钟在初始化的时候设置为42M,如果设置CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//则波特率为:42M/((6+7+1)*6)=500Kbps
//返回值:0,初始化OK;
//其他,初始化失败;
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
  	GPIO_InitTypeDef GPIO_InitStructure;              //设置初始化结构体
	CAN_InitTypeDef  CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	
    #if CAN1_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
    #endif
	
    //使能相关时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PORTA时钟	                   											 
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	
    //初始化GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA11,PA12
	
	//引脚复用映射配置
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11复用为CAN1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12复用为CAN1
	  
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE; //睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	//模式设置 
  	CAN_InitStructure.CAN_SJW=tsjw;	    //重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1;     //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;     //Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;//分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN1, &CAN_InitStructure); // 初始化CAN1 
    
	//配置过滤器
 	CAN_FilterInitStructure.CAN_FilterNumber=0;	  												   //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;                                 //32位
  	CAN_FilterInitStructure.CAN_FilterIdHigh=((0x00000201<<21)&0xffff0000)>>16;                    // =0x4000
  	CAN_FilterInitStructure.CAN_FilterIdLow=((0x00000201<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;     //=0x0000
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//0xFE1F;                                  //0xFFFF,32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000;//0xFFFF;                                  //0xFFFF
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;                             //过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;                                           //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);                                                      //滤波器初始化

#if CAN1_RX0_INT_ENABLE                                           //CAN1_RX0中断如果使能
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);                        //FIFO0消息挂号中断允许	    

  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif

//CAN2
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

  GPIO_InitStructure.GPIO_Pin = CAN2_RX_PIN|CAN2_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(CAN2_GPIO_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2); //CAN_RX = PB12
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); //CAN_TX = PB13

  CAN_StructInit(&CAN_InitStructure);
  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = ENABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;//同步跳转时间段
  CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;//时间段1
  CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;//时间段2
  CAN_InitStructure.CAN_Prescaler = 6;//波特率分频器
  
  /* CAN 波特率配置 = 42MHz/[(1+3+3)*6] = 1MHz (APB1的时钟频率一般为42MHz)*/
  if(CAN_Init(CAN2, &CAN_InitStructure) == CANINITFAILED)
  {
    do {}
    while(1);
  }
  /* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber = 14;//CAN2的滤波器从14开始
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

  CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
  return 0;
}

#if CAN2_RX0_INT_ENABLE	                   //如果使能了CAN2_RX0中断,则定义接收中断服务函数
void CAN2_RX0_IRQHandler(void)
{
	static int flaga = 1;
	static int flaga1 = 1;
	static int flaga2 = 1;
	static int flagb = 1;
	static int flagb1 = 1;
	static int flagb2 = 1;
	uint8_t index = 0;
	OSIntEnter();                          //进入中断保护区
	
	CAN_Receive(CAN2, 0, &RxMessage);      //从CAN1接收数据，通过过滤器后放入FIFO0,存入RxMessage数据帧
	switch(RxMessage.StdId)
	{
		case FEEDBACK_ID0_BASE:
			index = RxMessage.StdId - FEEDBACK_ID0_BASE;
			break;
		case FEEDBACK_ID1_BASE:
			index = RxMessage.StdId - FEEDBACK_ID1_BASE + 1;
			break;
		case FEEDBACK_ID2_BASE:
			index = RxMessage.StdId - FEEDBACK_ID2_BASE + 2;
			break;
	}
	//初始偏差计算
	if(flaga && (RxMessage.StdId == 517))
	{
		motor_info[0].offset_angle = ((RxMessage.Data[0] << 8) | RxMessage.Data[1]);
		flaga = 0;
	}
	else if(flaga1 && (RxMessage.StdId == 518))
	{
		motor_info[1].offset_angle = ((RxMessage.Data[0] << 8) | RxMessage.Data[1]);
		flaga1 = 0;
	}
	else if(flaga2 && (RxMessage.StdId == 513))
	{
		motor_info[2].offset_angle = ((RxMessage.Data[0] << 8) | RxMessage.Data[1]);
		flaga2 = 0;
	}
	
    if ((RxMessage.StdId >= FEEDBACK_ID2_BASE) && (RxMessage.StdId <  FEEDBACK_ID2_BASE + MOTOR_MAX_NUM))                  // 判断CAN id是否符合要求
    {
		motor_info[index].can_id         =   RxMessage.StdId;
		motor_info[index].last_angle     =   motor_info[index].rotor_angle;
		motor_info[index].rotor_angle    = ((RxMessage.Data[0] << 8) | RxMessage.Data[1]);
		motor_info[index].rotor_speed    = ((RxMessage.Data[2] << 8) | RxMessage.Data[3]);
		motor_info[index].torque_current = ((RxMessage.Data[4] << 8) | RxMessage.Data[5]);
		motor_info[index].temp           =   RxMessage.Data[6];
	
	if(motor_info[index].rotor_angle - motor_info[index].last_angle>4096)
		motor_info[index].total_cnt--;
	else if(motor_info[index].rotor_angle - motor_info[index].last_angle<-4096)
		motor_info[index].total_cnt++;
	
	if(flagb && (RxMessage.StdId == 517))
	{
		motor_info[0].total_cnt = 0;
		flagb = 0;
	}
	else if(flagb1 && (RxMessage.StdId == 518))
	{
		motor_info[1].total_cnt = 0;
		flagb1 = 0;
	}
	else if(flagb2 && (RxMessage.StdId == 513))
	{
		motor_info[2].total_cnt = 0;
		flagb2 = 0;
	}
	motor_info[index].total_angle=motor_info[index].total_cnt*8192 + motor_info[index].rotor_angle - motor_info[index].offset_angle;
//	motor_info[2].real_angle=motor_info[2].total_angle/51.1f/8192.0f*360.0f;                 //3508  	
    }
	OSIntExit();                           //退出中断保护区 
}
#endif

//CAN2发送数据到电机
//返回值：1->发送失败    0->发送成功
u8 set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)          
{	
  uint8_t  tx_data[8];       //CAN数据发送数组
  uint32_t mail_box;         //数据邮箱
  int i = 0;                 //计数标志
  
  /*根据GM6020编写*/
  TxMessage.StdId=(id_range == 0)?(0x1ff):(0x200);	         // id_range为0时使用标准标识符，可以控制4个电机；为1时使用0x2ff，可以控制3个电机      6020
  TxMessage.IDE=0;		                                     // 使用标准标示符
  TxMessage.RTR=0;		                                     // 消息类型为数据帧，一帧8位
  TxMessage.DLC=8;		                                     // 发送一帧数据长度,8位数据可以控制4个电机

  /*数据处理*/
  tx_data[0] = (v1>>8)&0xff;
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] =    (v4)&0xff;
	
  for(i = 0;i < 8;i++)
  {
	TxMessage.Data[i] = tx_data[i];                           //将处理后的数据放入一帧数据，等待传输
  }
  mail_box= CAN_Transmit(CAN2, &TxMessage);                   //从CAN1发送一帧数据，得到发送数据邮箱
  
  i = 0;
  while((CAN_TransmitStatus(CAN2, mail_box)==CAN_TxStatus_Failed)&&(i<0XFFF))   i++;         //等待发送结束,未发送完成则i++
  
  if(i>=0XFFF)	  return 1;       		//等待时间过长，则认为发送失败，返回1
  else            return 0;	      		//发送成功
}

#if CAN1_RX0_INT_ENABLE	                 //如果使能了CAN1_RX0中断,则定义接收中断服务函数
void CAN1_RX0_IRQHandler(void)
{
	int index = 2;
	OSIntEnter();                        //进入中断保护区
	
	CAN_Receive(CAN1, 0, &RxMessage);    //从CAN1接收数据，通过过滤器后放入FIFO0,存入RxMessage数据帧 

	switch(RxMessage.StdId)              //判断ID号
	{
		case CONTROL_BASE0://判断是否是控制云台俯仰命令
			index = 0;            		 
			control_info[index].can_id = CONTROL_BASE0;
			control_info[index].angle_yaw   += RxMessage.Data[0] - RxMessage.Data[1];//低四位数据整合
			control_info[index].angle_pitch += RxMessage.Data[2] - RxMessage.Data[3];//高四位数据整合
			control_info[index].ball_take_flag = 0;				   //此时为不取球
			break;
		case CONTROL_BASE1://判断是否是收球、PID调节命令
			index = 1;            								   
			control_info[index].can_id = CONTROL_BASE1;
			control_info[index].ball_take_flag = RxMessage.Data[0];//开始收球
            switch(RxMessage.Data[0])
			{
				case 0:          //不收球，不执行动作
					break;
				case 1:          //收球，归零位
					control_info[index].angle_yaw = YAW_START;
					control_info[index].angle_pitch = PITCH_START;
				    m = 1;
					break;
				case 3:
					if(RxMessage.Data[1] == 0)       //发送的正偏差
					{
						control_info[index].YAW_BIAS = 	RxMessage.Data[2] + (RxMessage.Data[3] * 10) + (RxMessage.Data[4] * 100);				
					}
					else if(RxMessage.Data[1] == 1)  //发送的负偏差
					{
						control_info[index].YAW_BIAS = 	-(RxMessage.Data[2] + (RxMessage.Data[3] * 10) + (RxMessage.Data[4] * 100));
					}
					
					break;
			}
			break;
	}
	
//	printf("%d,%d,%d\r\n",control_info[0].angle_yaw,control_info[0].angle_pitch,control_info[1].ball_take_flag);
	OSIntExit();                           //退出中断保护区 
}

//CAN1发送数据到主控版
u8 set_ACK(uint8_t id_range, int Success_Flag)      //收球成功返回响应
{
  uint32_t mail_box;         //数据邮箱
  int i = 0;                 //计数标志
  
  TxMessage.StdId=(id_range == 0)?(0x007):(0x008); //发送ID号
  TxMessage.IDE=0;		                           // 使用标准标示符
  TxMessage.RTR=0;		                           // 消息类型为数据帧，一帧8位
  TxMessage.DLC=8;		                           // 发送一帧数据长度,8位数据

  /*数据放入数组 */
  TxMessage.Data[0] = 0;                           //将数据放入一帧数据，等待传输
  TxMessage.Data[1] = 0;                           //将数据放入一帧数据，等待传输
  TxMessage.Data[2] = 0;                           //将数据放入一帧数据，等待传输
  TxMessage.Data[3] = 0;                           //将数据放入一帧数据，等待传输
  TxMessage.Data[4] = 1;                           //将数据放入一帧数据，等待传输
  TxMessage.Data[5] = 1;                           //将数据放入一帧数据，等待传输
  TxMessage.Data[6] = 1;                           //将数据放入一帧数据，等待传输
  TxMessage.Data[7] = 1;                           //将数据放入一帧数据，等待传输

  mail_box= CAN_Transmit(CAN1, &TxMessage);        //从CAN1发送一帧数据，得到发送数据邮箱

  i = 0;
  while((CAN_TransmitStatus(CAN1, mail_box)==CAN_TxStatus_Failed)&&(i<0XFFF)) i++;//等待发送结束,未发送完成则i++

  if(i>=0XFFF)	  return 1;       //等待时间过长，则认为发送失败，返回1
  else            return 0;	      //发送成功
}
#endif
