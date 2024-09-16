/**
  ******************************************************************************
  * @file    timer.c
  * @author  Scu_ditaya
  * @version V1.0
  * @date    2020.3.6
  * @brief   functions of time
  ******************************************************************************
  * @attention
  *
**/
/*include----------------------*/
#include "timer.h"

/**
  * @brief  ���ö�ʱ���ж�ʱ��
  * @param  TIMx:  x ��Χ�� 1-14. 
		@note TIM1 �� TIM 9,10,11;TIM2 �� TIM12,13,14 �ֱ�����ͬ���жϺ���
  * @param  arr: �Զ���װ��ֵ����Χ��1-65535
  * @param  psr: ��Ƶ������Χ��1-65535
  * @note   1.prepri��subpri
						2.Tout= ((arr+1)*(psc+1))/Tclk s, ������Ҫȷ�����ȼ��������ȼ�
							TIM2-7 �� TIM12-14, Tclk=84M
							TIM1,8,9,10,11 Tclk=168M
							eg: TIM2, if arr=999, psc=83, then Tout=(1000*84)/84M=1ms
  */
void TIM_Init(TIM_TypeDef * TIMx, uint16_t arr, uint16_t psr,uint16_t prepri,uint16_t subpri)
{
	TIM_TimeBaseInitTypeDef TIMx_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		
	switch((uint32_t)TIMx)
	{
		//APB2 TIM
		case TIM1_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_TIM10_IRQn ;     
			
			break;
		}
		case TIM8_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM8_UP_TIM13_IRQn ;    
			
			break;
		}
		case TIM9_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM1_BRK_TIM9_IRQn ;    
			
			break;
		}
		case TIM10_BASE:
		{
			RCC_APB2PeriphClockCmd(TIM1_UP_TIM10_IRQn, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM8_UP_TIM13_IRQn ;    
			
			break;
		}
		case TIM11_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM1_TRG_COM_TIM11_IRQn ;   
			
			break;
		}
		//APB1 TIM
		case TIM2_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn ;     

			break;
		}
		case TIM3_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn ;    

			break;
		}
		case TIM4_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn ;    

			break;
		}
		case TIM5_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn ;     

			break;
		}
		case TIM6_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM6_DAC_IRQn ;     

			break;
		}
		case TIM7_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn ;    

			break;
		}
		case TIM12_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM8_BRK_TIM12_IRQn ;    

			break;
		}
		case TIM13_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM8_UP_TIM13_IRQn ;   

			break;
		}
		case TIM14_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM8_TRG_COM_TIM14_IRQn ;     

			break;
		}
				
		default: break;
	}
	
	//��ʱ��TIMx��ʼ��
	TIMx_TimeBaseStructure.TIM_Period=arr;						//�����Զ���ת�ؼĴ������ڵ�ֵ
	TIMx_TimeBaseStructure.TIM_Prescaler=psr;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
	TIMx_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;      //����ʱ�ӷָ�
	TIMx_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //TIM���ϼ���
	TIM_TimeBaseInit(TIMx, &TIMx_TimeBaseStructure);            //��ʼ��TIMx
	TIM_ClearITPendingBit(TIMx, TIM_IT_Update);                 //��ʼ��ʱ���뽫����ж���0,�����ڿ�����ж�֮ǰ
	TIM_ITConfig(TIMx,TIM_IT_Update,ENABLE);                    //��������ж�
	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=prepri;		//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=subpri;            //�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;               //ʹ��IRQͨ��
	NVIC_Init(&NVIC_InitStructure);                             //��ʼ��NVIC�Ĵ���
	
	TIM_Cmd(TIMx,ENABLE);                                       //ʹ��TIMx
}
/**
  * @brief  accurency time delay dunction with TIMx
  * @param  TIMx: where x can be 1-14. 
  * @param  DelayMs: n us you want to delay, range from 1 to 65535
		@note	TIM2 and TIM5 are 32bit timer, you may change the code to delay longer;
					every time when time= 168/168M s,TIMCounter++ 
          the frequence is divided to 1M
  * @retval just use the timer load number
  */
void TIM_Delayus(TIM_TypeDef * TIMx, uint16_t Delayus)
{
  uint16_t  TIMCounter = Delayus;
	TIM_TimeBaseInitTypeDef TIMx_TimeBaseStructure;
	
	switch((uint32_t)TIMx)
	{
		//APB2 TIM
		case TIM1_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=168;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ

			break;
		}
		case TIM8_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=168;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ

			break;
		}
		case TIM9_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=168;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ

			break;
		}
		case TIM10_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=168;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ

			break;
		}
		case TIM11_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=168;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ

			break;
		}
		//APB1 TIM
		case TIM2_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=84;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM3_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=84;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM4_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=84;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM5_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=84;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM6_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=84;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM7_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=84;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM12_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=84;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM13_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=84;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM14_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=84;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		default: break;
	}

	TIMx_TimeBaseStructure.TIM_Period=1;						//�����Զ���ת�ؼĴ������ڵ�ֵ
	TIMx_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;      //����ʱ�ӷָ�
	TIMx_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //TIM���ϼ���
	TIM_TimeBaseInit(TIMx, &TIMx_TimeBaseStructure);            //��ʼ��TIM1
	 									
	TIM_Cmd(TIMx,ENABLE);
	TIM_SetCounter(TIMx, 65535-TIMCounter);
	
	while (TIMCounter<65535)
	{
		TIMCounter = TIM_GetCounter(TIMx);
	}

	TIM_Cmd(TIMx, DISABLE);	
}


/**
  * @brief  accurency time delay dunction with TIMx
  * @param  TIMx: where x can be 1-14. 
  * @param  DelayMs: n 100us you want to delay, range from 1 to 65535
		@note	TIM2 and TIM5 are 32bit timer, you may change the code to delay longer
  * @retval None
  */
void TIM_Delay100us(TIM_TypeDef * TIMx, uint16_t Delay100us)
{
  uint32_t  TIMCounter=0;
	TIM_TimeBaseInitTypeDef TIMx_TimeBaseStructure;
	TIMCounter= Delay100us;
	switch((uint32_t)TIMx)
	{
		//APB2 TIM
		case TIM1_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=16800;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ

			break;
		}
		case TIM8_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=16800;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ

			break;
		}
		case TIM9_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=16800;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ

			break;
		}
		case TIM10_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=16800;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ

			break;
		}
		case TIM11_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=16800;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ

			break;
		}
		//APB1 TIM
		case TIM2_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=8400;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM3_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=8400;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM4_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=8400;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM5_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=8400;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM6_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=8400;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM7_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=8400;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM12_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=8400;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM13_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=8400;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM14_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=8400;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		default: break;
	}

	TIMx_TimeBaseStructure.TIM_Period=1;						//�����Զ���ת�ؼĴ������ڵ�ֵ
	TIMx_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;      //����ʱ�ӷָ�
	TIMx_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //TIM���ϼ���
	TIM_TimeBaseInit(TIMx, &TIMx_TimeBaseStructure);            //��ʼ��TIM1
 					
	
	TIM_SetCounter(TIMx, 65535-TIMCounter);
	TIM_Cmd(TIMx,ENABLE);
	while (TIMCounter<65535)
	{
		TIMCounter = TIM_GetCounter(TIMx);
	}

	TIM_Cmd(TIMx, DISABLE);

}

/**
  * @brief  accurency time delay dunction with TIMx
  * @param  TIMx: where x can be 1-14. 
  * @param  DelayMs:
  * @retval None
  */
void TIM_Delayms(TIM_TypeDef * TIMx, uint32_t DelayMs)
{
    uint32_t i=0;

	for(i=0;i<DelayMs;i++)
	{
		TIM_Delay100us(TIMx,10);
	}
}

/**
  * @brief  ���ö�ʱ��PWM�����������ʱ����Ƶ������Ӧ�����������
    @note   TIM1:  GPIOE9/11/13/14
            TIM8:  GPIOC6/7/8/9
            TIM9:  GPIOE5/6��ͨ��1��2��
            TIM1:1,8,9,12,11 168M
  * @param  aar:��װ��ֵ
  * @param  ��Ƶ��
  * @note   ����������ֵ����PWM���Ƶ�ʣ�
            eg:TIM1,arr=20000,psc=2,��Ƶ��Ϊ��168M/arr/psc=4200Hz
  */
void TIM1_PWM_Init(u32 arr,u32 psc)  
{                              
    GPIO_InitTypeDef GPIO_InitStructure;  
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
    TIM_OCInitTypeDef  TIM_OCInitStructure;  
      
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);         //ʹ��TIMʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);       //ʹ����ӦPWM����
      
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);     //��GPIO����ΪPWM���
  	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
	  GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
	  GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
    /*��ʼ������*/      
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      
    GPIO_Init(GPIOE,&GPIO_InitStructure);              
    /*����PWM�������*/  
    TIM_TimeBaseStructure.TIM_Prescaler=psc;                   //��Ƶ��
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //���ϼ���
    TIM_TimeBaseStructure.TIM_Period=arr;                      //����װ��ֵ
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;      
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
    TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
      
  
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;            // PWM��ʱ������ģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;    //������Ը�
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset; 
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
  	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
  	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    /*ʹ��Ԥװ�ؼĴ���*/  
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); 
	  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable); 
  	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable); 
	  TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable); 
      
    TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPEʹ��
    TIM_Cmd(TIM1, ENABLE);            //ʹ��TIM4
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
     
	  TIM_SetCompare1(TIM1,0);
	  TIM_SetCompare2(TIM1,0);
	  TIM_SetCompare3(TIM1,0);
    TIM_SetCompare4(TIM1,0);
}


/*************************************************************************************/
int TIM2_Encoder_Read(void)
{
        int cnt = 0;
        cnt = TIM_GetCounter(TIM2) - 0x8000;    
        TIM_SetCounter(TIM2, 0x8000);
        return cnt;//���ݱ������������������
}
int TIM3_Encoder_Read(void)//��ȡǰ���ƶ�����
{
        int cnt = 0;
        cnt = TIM_GetCounter(TIM3)-0x8000;   
        TIM_SetCounter(TIM3,0x8000);	
        return -cnt;//���ݱ������������������
}
