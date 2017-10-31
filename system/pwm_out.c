/********************************************************************************
 * �ļ���  ��pwm_output.c
 * ����    ��         
 * ʵ��ƽ̨�����������STM32F4���ذ�
 * PWM�ӿ� 		
 * ��汾  ��
**********************************************************************************/
#include "pwm_out.h"
/*
static void TIM4_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // GPIOA  clock enable 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 

  //GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull 
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9 ;		   
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  // �����������
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_TIM4);

}

static void TIM4_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  
	
	//Time base configuration 		 
  TIM_TimeBaseStructure.TIM_Period = 20000;      //PWM����
  TIM_TimeBaseStructure.TIM_Prescaler = 84-1;	    //����Ԥ��Ƶ����Ϊ1MHz
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	//����ʱ�ӷ�Ƶϵ��������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ

  TIM_TimeBaseStructure.TIM_Period = 20000;  //PWM����

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_ARRPreloadConfig(TIM4, ENABLE);

  // PWM1 Mode configuration: Channel1 
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_Pulse = 1500;	   //��������ֵ�������������������ֵʱ����ƽ��������
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ



  TIM_OC3Init(TIM4, &TIM_OCInitStructure);	 //ʹ��ͨ��1
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);	 //ʹ��ͨ��2
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM4, ENABLE);			 // ʹ��TIM5���ؼĴ���ARR
	
  TIM_Cmd(TIM4, ENABLE);                   //ʹ�ܶ�ʱ��5
}

void TIM4_PWM_Init(void)
{
	TIM4_GPIO_Config();
	TIM4_Mode_Config();	
}

void TIM4_PWM_OUTPUT(u16 DR3,u16 DR4)
{
//	TIM_SetCompare1(TIM4,DR1);
//	TIM_SetCompare2(TIM4,DR2);
	TIM_SetCompare3(TIM4,DR3);
	TIM_SetCompare4(TIM4,DR4);
}


   */
/*
 * ��������TIM5_GPIO_Config
 * ����  ������TIM5�������PWMʱ�õ���I/O
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
 */
static void TIM5_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOA clock enable */
  RCC_APB2PeriphClockCmd  (RCC_APB2Periph_GPIOA,ENABLE );

  /*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;		   
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
}

/*
 * ��������TIM5_Mode_Config
 * ����  ������TIM5�����PWM�źŵ�ģʽ�������ڡ����ԡ�ռ�ձ�
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
 */
static void TIM5_Mode_Config(u16 CCR1_Val,u16 CCR2_Val,u16 CCR3_Val,u16 CCR4_Val)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;


  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5 , ENABLE);  
	
	/* Time base configuration */		 
  TIM_TimeBaseStructure.TIM_Period = 19999;      //PWM����
  TIM_TimeBaseStructure.TIM_Prescaler = 72-1;	    //����Ԥ��Ƶ����Ϊ1MHz
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	//����ʱ�ӷ�Ƶϵ��������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ

  TIM_TimeBaseStructure.TIM_Period = 19999;  //PWM����

  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_Pulse = 0;	   //��������ֵ�������������������ֵʱ����ƽ��������
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ


  TIM_OC1Init(TIM5, &TIM_OCInitStructure);	 //ʹ��ͨ��1
  TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
  TIM_OC2Init(TIM5, &TIM_OCInitStructure);	 //ʹ��ͨ��2
  TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
  TIM_OC3Init(TIM5, &TIM_OCInitStructure);	 //ʹ��ͨ��3
  TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
  TIM_OC4Init(TIM5, &TIM_OCInitStructure);	 //ʹ��ͨ��4
  TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM5, ENABLE);			 // ʹ��TIM5���ؼĴ���ARR
  TIM_Cmd(TIM5, ENABLE);                   //ʹ�ܶ�ʱ��5
}

/*
 * ��������TIM5_Mode_Config
 * ����  ��TIM5 ���PWM�źų�ʼ����ֻҪ�����������
 *         TIM5���ĸ�ͨ���ͻ���PWM�ź����
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 */
void TIM5_PWM_Init(void)
{
	TIM5_GPIO_Config();
	TIM5_Mode_Config(0,0,0,0);	
}

/*
 * ��������TIM5_PWM_OUTPUT
 * ����  ��TIM5����ɿ�PWM�ź�
 * ����  ��DR1
 * ���  ����
 * ����	 ���ⲿ����		   
 */

void TIM5_PWM_OUTPUT(u16 DR1,u16 DR2,u16 DR3,u16 DR4)  //PWM�����
{ 
	TIM_SetCompare1(TIM5,DR1);	   //DR���ݼ�¼��
	TIM_SetCompare2(TIM5,DR2);
	TIM_SetCompare3(TIM5,DR3);
	TIM_SetCompare4(TIM5,DR4);
}
