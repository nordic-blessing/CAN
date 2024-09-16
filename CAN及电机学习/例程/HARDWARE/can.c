#include "can.h"

can_parameter canDat;
moto_measure_t motoS_chassis = {0}; 


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
//    其他,初始化失败; 

u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{

    GPIO_InitTypeDef GPIO_InitStructure; 
    CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

   	NVIC_InitTypeDef  NVIC_InitStructure;

    //使能相关时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PORTA时钟	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	
    //初始化GPIO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
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
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1 
    
		//过滤200~20F
		//配置过滤器
    CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;    //屏蔽位模式
																	 //CAN_FilterMode_IdList     列表模式
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;   //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=((0x00000201<<21)&0xffff0000)>>16;                        // =0x4000
  	CAN_FilterInitStructure.CAN_FilterIdLow=((0x00000201<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;         //=0x0000
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xFE1F;    //0xFFFF;    //32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0xFFFF;     //0xFFFF; 
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
		

	
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
	return 0;
}     


u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	NVIC_InitTypeDef  NVIC_InitStructure;
	//使能相关时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTA时钟	                   											 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//使能CAN2时钟	必须使能CAN1才能使用CAN2

	//初始化GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12| GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PA11,PA12

	//引脚复用映射配置
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2); //GPIOA11复用为CAN1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2); //GPIOA12复用为CAN1
	  
	//CAN单元设置
	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
	CAN_InitStructure.CAN_TXFP=ENABLE;	//优先级由报文标识符决定 
	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
	CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
	CAN_Init(CAN2, &CAN_InitStructure);   // 初始化CAN1 
	
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
	//接受来自2006电机ID为201~208
	CAN_FilterInitStructure.CAN_FilterNumber=14;	  //过滤器14
	CAN_FilterInitStructure.CAN_FilterIdHigh=((0x00000200<<21)&0xffff0000)>>16;
	CAN_FilterInitStructure.CAN_FilterIdLow=((0x00000200<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xFE1F;//32位MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xFFFF;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
	//接收控制板发送控制信息ID为：301~30F
	CAN_FilterInitStructure.CAN_FilterNumber=15;	  //过滤器15
	CAN_FilterInitStructure.CAN_FilterIdHigh=((0x00000300<<21)&0xffff0000)>>16;
	CAN_FilterInitStructure.CAN_FilterIdLow=((0x00000300<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xFE1F;//32位MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xFFFF;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化

	
	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    

	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     // 主优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // 次优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	return 0;
}    





//中断服务函数			  
//1ms进来一次
void CAN1_RX0_IRQHandler(void)
{
	static u8 offsetAngleFlag1;
	int i=0;
	CanRxMsg RxMessage;
	//清除中断标志位



	CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
	CAN_ClearFlag(CAN1, CAN_IT_FMP0);

	CAN_Receive(CAN1, 0, &RxMessage);
	
	switch(RxMessage.StdId)
	{
		case 0x201: for(i=0;i<8;i++)canDat.M1_datBuf[i]=RxMessage.Data[i];//根据canID识别哪个轮子返回的信息
					if(offsetAngleFlag1==0)//获取链轮电机上电初始编码器值
					{
						get_moto_offset(&motoS_chassis, canDat.M1_datBuf);
						offsetAngleFlag1=1;
					}
					else 
					{ 
						LED0 = !LED0;
						motoS_chassis.last_angle=motoS_chassis.angle;
						motoS_chassis.angle = (uint16_t)(RxMessage.Data[0]<<8 | RxMessage.Data[1]);
						//********************
						if(motoS_chassis.angle - motoS_chassis.last_angle > 4096)//经理论计算，此电机在1ms无法转过半圈，因而计算合理
						motoS_chassis.round_cnt--; 
						else if (motoS_chassis.angle - motoS_chassis.last_angle < -4096)
						motoS_chassis.round_cnt++;
						//************************
						motoS_chassis.total_angle =motoS_chassis.round_cnt*8192+ motoS_chassis.angle - motoS_chassis.offset_angle;
					}								 
					break;
					 
		default:break;
	}
}


//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
u8 CAN1_Send_Msg(s16 iq1, s16 iq2, s16 iq3, s16 iq4)	
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x200;	 // 标准标识符为0x200
  TxMessage.IDE=CAN_ID_STD;		  // 使用标准标识符
  TxMessage.RTR=CAN_RTR_DATA;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=8;							 // 消息长度为8  发送8个字节    

  TxMessage.Data[0]=iq1 >> 8;	
  TxMessage.Data[1]=iq1;
  TxMessage.Data[2]=iq2 >> 8;
  TxMessage.Data[3]=iq2;
  TxMessage.Data[4]=iq3 >> 8;
  TxMessage.Data[5]=iq3;
 	TxMessage.Data[6]=iq4 >> 8;
  TxMessage.Data[7]=iq4;
  
	mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 1;	
  return 0;		
}



u8 CAN2_Send_Msg(s16 iq1, s16 iq2, s16 iq3, s16 iq4)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x1FF;	 			// 标准标识符为0x1ff
  TxMessage.IDE=CAN_ID_STD;		  // 使用扩展标识符
  TxMessage.RTR=CAN_RTR_DATA;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=8;							 // 发送两帧信息

  TxMessage.Data[0]=iq1 >> 8;	
  TxMessage.Data[1]=iq1;
  TxMessage.Data[2]=iq2 >> 8;
  TxMessage.Data[3]=iq2;
  TxMessage.Data[4]=iq3 >> 8;
  TxMessage.Data[5]=iq3;
	TxMessage.Data[6]=iq4 >> 8;
  TxMessage.Data[7]=iq4;
 	
  
	mbox= CAN_Transmit(CAN2, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 1;	
  return 0;		
}




//必须在系统can初始化后执行，3508对编码器值具有记忆性，会记录最后一次编码器值并在上电时返回到控制器
void get_moto_offset(moto_measure_t *ptr, u8 *buf)
{
	ptr->angle = (uint16_t)(buf[0]<<8 | buf[1]) ;
	ptr->offset_angle = ptr->angle;
}


//接收3508电机发来的CAN信息
void get_moto_measure_3508(moto_measure_t *ptr, u8 *buf)
{
	ptr->last_angle = ptr->angle;//获取电机角度
	ptr->angle = (uint16_t)(buf[0]<<8 | buf[1]) ;
	ptr->speed_rpm  = (int16_t)(buf[2]<<8 | buf[3]);//获取电机转速
	ptr->real_current = (int16_t)(buf[4]<<8 | buf[5]);//获取电机电流
	ptr->hall = buf[6];
	if(ptr->angle - ptr->last_angle > 4096)//经理论计算，此电机在5ms无法转过半圈，因而计算合理
		ptr->round_cnt --;                                                                        //
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;                  //这个ptr->offset_angle 不清楚
	ptr->real_angle=ptr->total_angle/19.1935f/8192.0f*360.0f;//3508                             //这个19，1935还不知道为啥

}


//接收2006电机发来的CAN信息
void get_moto_measure_2006(moto_measure_t *ptr, u8 *buf)
{
	ptr->speed_rpm  = (int16_t)(buf[2]<<8 | buf[3]);//获取电机转速
	ptr->real_current = (int16_t)(buf[4]<<8 | buf[5]);//获取电机电流
	ptr->hall = buf[6];
	ptr->real_angle=ptr->total_angle/36.0f/8192.0f*360.0f;//2006
}

