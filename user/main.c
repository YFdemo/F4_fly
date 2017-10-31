#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"
#include "stm32f10x.h"
#include "MPU6050.h"
#include "IIC.h"
#include "control.h"
#include "imu.h"
#include "pwm_out.h"

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp) 	  ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
void ANO_DT_Send_Status(void);

int main()
{
	int i=0;

	delay_init();               //延时函数初始化
		 
	delay_ms(2000);
	delay_ms(2000);

	I2C_GPIO_Config();	 		//IIC接口初始化	

	NVIC_Configuration();       //设置NVIC中断分组2:2位抢占优先级，2位响应优先级

	Init_MPU6050();	  			//陀螺仪传感器初始化	
	
	uart_init(9600);           //串口初始化为9600

	TIM5_PWM_Init();			//pwm输出初始化
	
	TIM4_Cap_Init(0xffff,72-1); //以1Mhz的频率计数

	for(i=0;i<12000;i++)								//用以初始化电调航程的时间
	{
		 TIM5_PWM_OUTPUT(pwmout2,pwmout2,pwmout2,pwmout2);
		delay_ms(1);
	}

	Acc_Correct();
	Gyro_Correct();

	PID_controllerInit();		//PID控制初始化
	controlmiddleinit(pwmout1,pwmout2,pwmout3,pwmout4);
	
	while(1)
	{			 

				 IMUdataprepare();
				 IMUupdate(GyroFinal.X,GyroFinal.Y,GyroFinal.Z,AccFinal.X,AccFinal.Y,AccFinal.Z);	  //输入陀螺仪加速度计参数，得到三个方向的欧拉角
		
			//	 Roll=(float)atan2(AccFinal.Y,AccFinal.Z)*57.295779513;    //X轴角度值 
			//	 Pitch=-(float)atan2(AccFinal.X,AccFinal.Z)*57.295779513;  //Y轴角度值
			//	 	 YAW=-(float)atan2(AccFinal.Y,AccFinal.X)*57.295779513;  //Y轴角度值
			//	 SendData(Q_ANGLE.Yaw*10,Q_ANGLE.Pitch*10,Q_ANGLE.Roll*10,0);	 //配合diIMU上位机，单位度*10  红 蓝 青 黄	 //发送数据	
				 Getdesireddata(pwmout1,pwmout2,pwmout3,pwmout4);	  //得到期望数据

				 PID_CAL();
				 if(pwmout2>1140)
				 	TIM5_PWM_OUTPUT(MOTOR1,MOTOR2,MOTOR3,MOTOR4);
				 else
					TIM5_PWM_OUTPUT(1100,1100,1100,1100);												
			//	 TIM5_PWM_OUTPUT(pwmout1,pwmout2,pwmout2,pwmout2);
			//	 platform_control(Q_ANGLE.Roll,Q_ANGLE.Pitch,0,0);			   //平台控制
			//	 TIM4_PWM_OUTPUT(servo_Roll,servo_Pitch);					   //TIM4PWM波输出
			//		READ_MPU6050();
			//		printf("\n MOTOR1 %d",MOTOR1);
			//		printf("\n MOTOR2 %d",MOTOR2);
			//		printf("\n MOTOR3 %d",MOTOR3);
		//			printf("\n MOTOR4 %d",MOTOR4);
			//		printf("\n pwmout1 %d",pwmout1);
			//		printf("\n pwmout2 %d",pwmout2);
			//		printf("\n pwmout3 %d",pwmout3);
			//		printf("\n pwmout4 %d",pwmout4);
			//		printf("\n01: %f",Q_ANGLE.Pitch);
			//		printf("\n02: %f",Q_ANGLE.Roll);	
			//		printf("\n03: %f",Q_ANGLE.Yaw);
			//		printf("\n04: %X",16);
			//		ANO_DT_Send_Status();
			//		printf("\n陀螺仪X轴 %f",GyroFinal.X);	
			//		printf("\n陀螺仪Y轴 %f",GyroFinal.Y);	
			//		printf("\n陀螺仪Z轴 %f",GyroFinal.Z);
			//		printf("\n");

	}
}
void ANO_DT_Send_Status(void)
{
	u8 data_to_send[50];
	u8 sum = 0;
	u8 i;
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = 1;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = (s16)AccFinal.X;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (s16)AccFinal.Y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (s16)AccFinal.Z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (s16)GyroFinal.X;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (s16)GyroFinal.Y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (s16)GyroFinal.Z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = (s16)GyroFinal.X;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (s16)GyroFinal.Y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (s16)GyroFinal.Z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[3] = _cnt-4;
	

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	printf("%s",data_to_send);
}
