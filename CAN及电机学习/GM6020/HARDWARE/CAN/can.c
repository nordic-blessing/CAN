#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "pid.h"
#include "math.h"
#include "includes.h"

CanTxMsg TxMessage;    /*����֡����*/
CanRxMsg RxMessage;

moto_info_t motor_info[MOTOR_MAX_NUM];       //���ص����������
Control_type control_info[2];                //�ⲿ���ƽṹ
extern int m;

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
//����,��ʼ��ʧ��;
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
  	GPIO_InitTypeDef GPIO_InitStructure;              //���ó�ʼ���ṹ��
	CAN_InitTypeDef  CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	
    #if CAN1_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
    #endif
	
    //ʹ�����ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
	
    //��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
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
  	CAN_InitStructure.CAN_AWUM=DISABLE; //˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= mode;	//ģʽ���� 
  	CAN_InitStructure.CAN_SJW=tsjw;	    //����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1;     //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;     //Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;//��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN1, &CAN_InitStructure); // ��ʼ��CAN1 
    
	//���ù�����
 	CAN_FilterInitStructure.CAN_FilterNumber=0;	  												   //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;                                 //32λ
  	CAN_FilterInitStructure.CAN_FilterIdHigh=((0x00000201<<21)&0xffff0000)>>16;                    // =0x4000
  	CAN_FilterInitStructure.CAN_FilterIdLow=((0x00000201<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;     //=0x0000
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//0xFE1F;                                  //0xFFFF,32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000;//0xFFFF;                                  //0xFFFF
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;                             //������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;                                           //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);                                                      //�˲�����ʼ��

#if CAN1_RX0_INT_ENABLE                                           //CAN1_RX0�ж����ʹ��
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);                        //FIFO0��Ϣ�Һ��ж�����	    

  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ1
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
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;//ͬ����תʱ���
  CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;//ʱ���1
  CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;//ʱ���2
  CAN_InitStructure.CAN_Prescaler = 6;//�����ʷ�Ƶ��
  
  /* CAN ���������� = 42MHz/[(1+3+3)*6] = 1MHz (APB1��ʱ��Ƶ��һ��Ϊ42MHz)*/
  if(CAN_Init(CAN2, &CAN_InitStructure) == CANINITFAILED)
  {
    do {}
    while(1);
  }
  /* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber = 14;//CAN2���˲�����14��ʼ
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

#if CAN2_RX0_INT_ENABLE	                   //���ʹ����CAN2_RX0�ж�,��������жϷ�����
void CAN2_RX0_IRQHandler(void)
{
	static int flaga = 1;
	static int flaga1 = 1;
	static int flaga2 = 1;
	static int flagb = 1;
	static int flagb1 = 1;
	static int flagb2 = 1;
	uint8_t index = 0;
	OSIntEnter();                          //�����жϱ�����
	
	CAN_Receive(CAN2, 0, &RxMessage);      //��CAN1�������ݣ�ͨ�������������FIFO0,����RxMessage����֡
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
	//��ʼƫ�����
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
	
    if ((RxMessage.StdId >= FEEDBACK_ID2_BASE) && (RxMessage.StdId <  FEEDBACK_ID2_BASE + MOTOR_MAX_NUM))                  // �ж�CAN id�Ƿ����Ҫ��
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
	OSIntExit();                           //�˳��жϱ����� 
}
#endif

//CAN2�������ݵ����
//����ֵ��1->����ʧ��    0->���ͳɹ�
u8 set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)          
{	
  uint8_t  tx_data[8];       //CAN���ݷ�������
  uint32_t mail_box;         //��������
  int i = 0;                 //������־
  
  /*����GM6020��д*/
  TxMessage.StdId=(id_range == 0)?(0x1ff):(0x200);	         // id_rangeΪ0ʱʹ�ñ�׼��ʶ�������Կ���4�������Ϊ1ʱʹ��0x2ff�����Կ���3�����      6020
  TxMessage.IDE=0;		                                     // ʹ�ñ�׼��ʾ��
  TxMessage.RTR=0;		                                     // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=8;		                                     // ����һ֡���ݳ���,8λ���ݿ��Կ���4�����

  /*���ݴ���*/
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
	TxMessage.Data[i] = tx_data[i];                           //�����������ݷ���һ֡���ݣ��ȴ�����
  }
  mail_box= CAN_Transmit(CAN2, &TxMessage);                   //��CAN1����һ֡���ݣ��õ�������������
  
  i = 0;
  while((CAN_TransmitStatus(CAN2, mail_box)==CAN_TxStatus_Failed)&&(i<0XFFF))   i++;         //�ȴ����ͽ���,δ���������i++
  
  if(i>=0XFFF)	  return 1;       		//�ȴ�ʱ�����������Ϊ����ʧ�ܣ�����1
  else            return 0;	      		//���ͳɹ�
}

#if CAN1_RX0_INT_ENABLE	                 //���ʹ����CAN1_RX0�ж�,��������жϷ�����
void CAN1_RX0_IRQHandler(void)
{
	int index = 2;
	OSIntEnter();                        //�����жϱ�����
	
	CAN_Receive(CAN1, 0, &RxMessage);    //��CAN1�������ݣ�ͨ�������������FIFO0,����RxMessage����֡ 

	switch(RxMessage.StdId)              //�ж�ID��
	{
		case CONTROL_BASE0://�ж��Ƿ��ǿ�����̨��������
			index = 0;            		 
			control_info[index].can_id = CONTROL_BASE0;
			control_info[index].angle_yaw   += RxMessage.Data[0] - RxMessage.Data[1];//����λ��������
			control_info[index].angle_pitch += RxMessage.Data[2] - RxMessage.Data[3];//����λ��������
			control_info[index].ball_take_flag = 0;				   //��ʱΪ��ȡ��
			break;
		case CONTROL_BASE1://�ж��Ƿ�������PID��������
			index = 1;            								   
			control_info[index].can_id = CONTROL_BASE1;
			control_info[index].ball_take_flag = RxMessage.Data[0];//��ʼ����
            switch(RxMessage.Data[0])
			{
				case 0:          //�����򣬲�ִ�ж���
					break;
				case 1:          //���򣬹���λ
					control_info[index].angle_yaw = YAW_START;
					control_info[index].angle_pitch = PITCH_START;
				    m = 1;
					break;
				case 3:
					if(RxMessage.Data[1] == 0)       //���͵���ƫ��
					{
						control_info[index].YAW_BIAS = 	RxMessage.Data[2] + (RxMessage.Data[3] * 10) + (RxMessage.Data[4] * 100);				
					}
					else if(RxMessage.Data[1] == 1)  //���͵ĸ�ƫ��
					{
						control_info[index].YAW_BIAS = 	-(RxMessage.Data[2] + (RxMessage.Data[3] * 10) + (RxMessage.Data[4] * 100));
					}
					
					break;
			}
			break;
	}
	
//	printf("%d,%d,%d\r\n",control_info[0].angle_yaw,control_info[0].angle_pitch,control_info[1].ball_take_flag);
	OSIntExit();                           //�˳��жϱ����� 
}

//CAN1�������ݵ����ذ�
u8 set_ACK(uint8_t id_range, int Success_Flag)      //����ɹ�������Ӧ
{
  uint32_t mail_box;         //��������
  int i = 0;                 //������־
  
  TxMessage.StdId=(id_range == 0)?(0x007):(0x008); //����ID��
  TxMessage.IDE=0;		                           // ʹ�ñ�׼��ʾ��
  TxMessage.RTR=0;		                           // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=8;		                           // ����һ֡���ݳ���,8λ����

  /*���ݷ������� */
  TxMessage.Data[0] = 0;                           //�����ݷ���һ֡���ݣ��ȴ�����
  TxMessage.Data[1] = 0;                           //�����ݷ���һ֡���ݣ��ȴ�����
  TxMessage.Data[2] = 0;                           //�����ݷ���һ֡���ݣ��ȴ�����
  TxMessage.Data[3] = 0;                           //�����ݷ���һ֡���ݣ��ȴ�����
  TxMessage.Data[4] = 1;                           //�����ݷ���һ֡���ݣ��ȴ�����
  TxMessage.Data[5] = 1;                           //�����ݷ���һ֡���ݣ��ȴ�����
  TxMessage.Data[6] = 1;                           //�����ݷ���һ֡���ݣ��ȴ�����
  TxMessage.Data[7] = 1;                           //�����ݷ���һ֡���ݣ��ȴ�����

  mail_box= CAN_Transmit(CAN1, &TxMessage);        //��CAN1����һ֡���ݣ��õ�������������

  i = 0;
  while((CAN_TransmitStatus(CAN1, mail_box)==CAN_TxStatus_Failed)&&(i<0XFFF)) i++;//�ȴ����ͽ���,δ���������i++

  if(i>=0XFFF)	  return 1;       //�ȴ�ʱ�����������Ϊ����ʧ�ܣ�����1
  else            return 0;	      //���ͳɹ�
}
#endif
