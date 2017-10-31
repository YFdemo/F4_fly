/********************************************************************************
 * 文件名  ：pwm_output.c
 * 描述    ：         
 * 实验平台：四轴飞行器STM32F4主控板
 * PWM接口 		
 * 库版本  ：
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
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  // 复用推挽输出
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
  TIM_TimeBaseStructure.TIM_Period = 20000;      //PWM周期
  TIM_TimeBaseStructure.TIM_Prescaler = 84-1;	    //设置预分频：即为1MHz
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	//设置时钟分频系数：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式

  TIM_TimeBaseStructure.TIM_Period = 20000;  //PWM周期

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_ARRPreloadConfig(TIM4, ENABLE);

  // PWM1 Mode configuration: Channel1 
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_Pulse = 1500;	   //设置跳变值，当计数器计数到这个值时，电平发生跳变
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平



  TIM_OC3Init(TIM4, &TIM_OCInitStructure);	 //使能通道1
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);	 //使能通道2
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM4, ENABLE);			 // 使能TIM5重载寄存器ARR
	
  TIM_Cmd(TIM4, ENABLE);                   //使能定时器5
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
 * 函数名：TIM5_GPIO_Config
 * 描述  ：配置TIM5复用输出PWM时用到的I/O
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
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
 * 函数名：TIM5_Mode_Config
 * 描述  ：配置TIM5输出的PWM信号的模式，如周期、极性、占空比
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
 */
static void TIM5_Mode_Config(u16 CCR1_Val,u16 CCR2_Val,u16 CCR3_Val,u16 CCR4_Val)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;


  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5 , ENABLE);  
	
	/* Time base configuration */		 
  TIM_TimeBaseStructure.TIM_Period = 19999;      //PWM周期
  TIM_TimeBaseStructure.TIM_Prescaler = 72-1;	    //设置预分频：即为1MHz
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	//设置时钟分频系数：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式

  TIM_TimeBaseStructure.TIM_Period = 19999;  //PWM周期

  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_Pulse = 0;	   //设置跳变值，当计数器计数到这个值时，电平发生跳变
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平


  TIM_OC1Init(TIM5, &TIM_OCInitStructure);	 //使能通道1
  TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
  TIM_OC2Init(TIM5, &TIM_OCInitStructure);	 //使能通道2
  TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
  TIM_OC3Init(TIM5, &TIM_OCInitStructure);	 //使能通道3
  TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
  TIM_OC4Init(TIM5, &TIM_OCInitStructure);	 //使能通道4
  TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM5, ENABLE);			 // 使能TIM5重载寄存器ARR
  TIM_Cmd(TIM5, ENABLE);                   //使能定时器5
}

/*
 * 函数名：TIM5_Mode_Config
 * 描述  ：TIM5 输出PWM信号初始化，只要调用这个函数
 *         TIM5的四个通道就会有PWM信号输出
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void TIM5_PWM_Init(void)
{
	TIM5_GPIO_Config();
	TIM5_Mode_Config(0,0,0,0);	
}

/*
 * 函数名：TIM5_PWM_OUTPUT
 * 描述  ：TIM5输出可控PWM信号
 * 输入  ：DR1
 * 输出  ：无
 * 调用	 ：外部调用		   
 */

void TIM5_PWM_OUTPUT(u16 DR1,u16 DR2,u16 DR3,u16 DR4)  //PWM波输出
{ 
	TIM_SetCompare1(TIM5,DR1);	   //DR数据记录仪
	TIM_SetCompare2(TIM5,DR2);
	TIM_SetCompare3(TIM5,DR3);
	TIM_SetCompare4(TIM5,DR4);
}
