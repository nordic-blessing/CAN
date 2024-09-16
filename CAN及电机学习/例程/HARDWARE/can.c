#include "can.h"

can_parameter canDat;
moto_measure_t motoS_chassis = {0}; 


//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:ʱ���1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024; tq=(brp)*tpclk1
//������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ42M,�������CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//������Ϊ:42M/((6+7+1)*6)=500Kbps
//����ֵ:0,��ʼ��OK;
//    ����,��ʼ��ʧ��; 

u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{

    GPIO_InitTypeDef GPIO_InitStructure; 
    CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

   	NVIC_InitTypeDef  NVIC_InitStructure;

    //ʹ�����ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
	
    //��ʼ��GPIO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��PA11,PA12
	
	  //���Ÿ���ӳ������
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11����ΪCAN1
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12����ΪCAN1
	  
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=tsjw;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 
    
		//����200~20F
		//���ù�����
    CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;    //����λģʽ
																	 //CAN_FilterMode_IdList     �б�ģʽ
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;   //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=((0x00000201<<21)&0xffff0000)>>16;                        // =0x4000
  	CAN_FilterInitStructure.CAN_FilterIdLow=((0x00000201<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;         //=0x0000
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xFE1F;    //0xFFFF;    //32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0xFFFF;     //0xFFFF; 
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
		

	
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // �����ȼ�Ϊ0
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
	//ʹ�����ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PORTAʱ��	                   											 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//ʹ��CAN2ʱ��	����ʹ��CAN1����ʹ��CAN2

	//��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12| GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PA11,PA12

	//���Ÿ���ӳ������
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2); //GPIOA11����ΪCAN1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2); //GPIOA12����ΪCAN1
	  
	//CAN��Ԫ����
	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
	CAN_InitStructure.CAN_TXFP=ENABLE;	//���ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
	CAN_InitStructure.CAN_SJW=tsjw;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
	CAN_Init(CAN2, &CAN_InitStructure);   // ��ʼ��CAN1 
	
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
	//��������2006���IDΪ201~208
	CAN_FilterInitStructure.CAN_FilterNumber=14;	  //������14
	CAN_FilterInitStructure.CAN_FilterIdHigh=((0x00000200<<21)&0xffff0000)>>16;
	CAN_FilterInitStructure.CAN_FilterIdLow=((0x00000200<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xFE1F;//32λMASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xFFFF;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
	//���տ��ư巢�Ϳ�����ϢIDΪ��301~30F
	CAN_FilterInitStructure.CAN_FilterNumber=15;	  //������15
	CAN_FilterInitStructure.CAN_FilterIdHigh=((0x00000300<<21)&0xffff0000)>>16;
	CAN_FilterInitStructure.CAN_FilterIdLow=((0x00000300<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xFE1F;//32λMASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xFFFF;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��

	
	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    

	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     // �����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // �����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	return 0;
}    





//�жϷ�����			  
//1ms����һ��
void CAN1_RX0_IRQHandler(void)
{
	static u8 offsetAngleFlag1;
	int i=0;
	CanRxMsg RxMessage;
	//����жϱ�־λ



	CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
	CAN_ClearFlag(CAN1, CAN_IT_FMP0);

	CAN_Receive(CAN1, 0, &RxMessage);
	
	switch(RxMessage.StdId)
	{
		case 0x201: for(i=0;i<8;i++)canDat.M1_datBuf[i]=RxMessage.Data[i];//����canIDʶ���ĸ����ӷ��ص���Ϣ
					if(offsetAngleFlag1==0)//��ȡ���ֵ���ϵ��ʼ������ֵ
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
						if(motoS_chassis.angle - motoS_chassis.last_angle > 4096)//�����ۼ��㣬�˵����1ms�޷�ת����Ȧ������������
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


//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//		 ����,ʧ��;
u8 CAN1_Send_Msg(s16 iq1, s16 iq2, s16 iq3, s16 iq4)	
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x200;	 // ��׼��ʶ��Ϊ0x200
  TxMessage.IDE=CAN_ID_STD;		  // ʹ�ñ�׼��ʶ��
  TxMessage.RTR=CAN_RTR_DATA;		  // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=8;							 // ��Ϣ����Ϊ8  ����8���ֽ�    

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
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)return 1;	
  return 0;		
}



u8 CAN2_Send_Msg(s16 iq1, s16 iq2, s16 iq3, s16 iq4)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x1FF;	 			// ��׼��ʶ��Ϊ0x1ff
  TxMessage.IDE=CAN_ID_STD;		  // ʹ����չ��ʶ��
  TxMessage.RTR=CAN_RTR_DATA;		  // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=8;							 // ������֡��Ϣ

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
  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)return 1;	
  return 0;		
}




//������ϵͳcan��ʼ����ִ�У�3508�Ա�����ֵ���м����ԣ����¼���һ�α�����ֵ�����ϵ�ʱ���ص�������
void get_moto_offset(moto_measure_t *ptr, u8 *buf)
{
	ptr->angle = (uint16_t)(buf[0]<<8 | buf[1]) ;
	ptr->offset_angle = ptr->angle;
}


//����3508���������CAN��Ϣ
void get_moto_measure_3508(moto_measure_t *ptr, u8 *buf)
{
	ptr->last_angle = ptr->angle;//��ȡ����Ƕ�
	ptr->angle = (uint16_t)(buf[0]<<8 | buf[1]) ;
	ptr->speed_rpm  = (int16_t)(buf[2]<<8 | buf[3]);//��ȡ���ת��
	ptr->real_current = (int16_t)(buf[4]<<8 | buf[5]);//��ȡ�������
	ptr->hall = buf[6];
	if(ptr->angle - ptr->last_angle > 4096)//�����ۼ��㣬�˵����5ms�޷�ת����Ȧ������������
		ptr->round_cnt --;                                                                        //
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;                  //���ptr->offset_angle �����
	ptr->real_angle=ptr->total_angle/19.1935f/8192.0f*360.0f;//3508                             //���19��1935����֪��Ϊɶ

}


//����2006���������CAN��Ϣ
void get_moto_measure_2006(moto_measure_t *ptr, u8 *buf)
{
	ptr->speed_rpm  = (int16_t)(buf[2]<<8 | buf[3]);//��ȡ���ת��
	ptr->real_current = (int16_t)(buf[4]<<8 | buf[5]);//��ȡ�������
	ptr->hall = buf[6];
	ptr->real_angle=ptr->total_angle/36.0f/8192.0f*360.0f;//2006
}

