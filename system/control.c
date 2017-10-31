#include "control.h"
#include "imu.h"
#include "delay.h"

int Elemiddle=0;
int	Ailmiddle=0;
int	Rudmiddle=0;

pidsuite pidRoll;					   //横滚角pid
pidsuite pidPitch;					   //俯仰角pid
pidsuite pidYaw;
	 
int Motor_Thr=0;					   //油门
int Motor_Ele=0;					   //俯仰期望
int Motor_Ail=0;					   //横滚期望
int Motor_Rud=0;					   //航向期望

int MOTOR1;
int MOTOR2;
int MOTOR3;	
int MOTOR4;

int servo_Roll=1500; 
int servo_Pitch=1500;

float pid_roll;
float pid_pitch;
float pid_yaw;

/*------------------------------------------pid结构初始化-----------------------------------------*/
//输入参数：结构体指针，期望值，kp,ki,kd
void pidInit(pidsuite* pid, const float desired, const float kp,
             const float ki, const float kd)
{

  pid->error = 0;
  pid->prevError = 0;
  pid->integ = 0;
  pid->deriv = 0;
  pid->desired = desired;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  
  pid->iLimit = DEFAULT_PID_INTEGRATION_LIMIT;
}

/*----------------------------------------------pid输出更新------------------------------------------*/
//输入参数：pid结构体指针，测量值 ,期望值
//输出：pid输出
float pidUpdate(pidsuite* pid, const float measured,float expect,float gyro)
{
  float output;
  static float lastoutput=0;

  pid->desired=expect;			 				//获取期望角度

  pid->error = pid->desired - measured;	 	  //偏差：期望-测量值
  
  pid->integ += pid->error * IMU_UPDATE_DT;	  //偏差积分
 
  if (pid->integ > pid->iLimit)				  //作积分限制
  {
    pid->integ = pid->iLimit;
  }
  else if (pid->integ < -pid->iLimit)
  {
    pid->integ = -pid->iLimit;
  }				 

 // pid->deriv = (pid->error - pid->prevError) / IMU_UPDATE_DT;		//微分	 应该可用陀螺仪角速度代替
  pid->deriv = -gyro;
  if(fabs(pid->error)>Piddeadband)									//pid死区
  {
		  pid->outP = pid->kp * pid->error;								 //方便独立观察
		  pid->outI = pid->ki * pid->integ;
		  pid->outD = pid->kd * pid->deriv;
		
		  output = (pid->kp * pid->error) +
		           (pid->ki * pid->integ) +
		           (pid->kd * pid->deriv);
  }
  else
  {
  		  output=lastoutput;
  }

  pid->prevError = pid->error;							 		//更新前一次偏差
  lastoutput=output;

  return output;
}

/*----------------------------------------------pid积分部分限制值-------------------------------------------*/
void pidSetIntegralLimit(pidsuite* pid, const float limit)
{
  pid->iLimit = limit;
}


void PID_controllerInit(void)
{
    pidInit(&pidRoll, 0, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD);
    pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD);
	pidInit(&pidYaw,0,PID_YAW_KP,PID_YAW_KI,PID_YAW_KD);

	pidSetIntegralLimit(&pidRoll, PID_ROLL_INTEGRATION_LIMIT);
    pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pidYaw, PID_YAW_INTEGRATION_LIMIT);
}


/*------------------------------------pid控制------------------------------------*/
#define PIDMIX(X,Y,Z) Motor_Thr + pid_pitch*X + pid_roll*Y + pid_yaw*Z
void PID_CAL(void)		  //PID参数计算
{

  	pid_roll = pidUpdate(&pidRoll,Q_ANGLE.Roll,Motor_Ail,GyroFinal.X);
	pid_pitch = pidUpdate(&pidPitch,Q_ANGLE.Pitch,Motor_Ele,GyroFinal.Y);
	pid_yaw = pidUpdate(&pidYaw,Q_ANGLE.Yaw,Motor_Rud,GyroFinal.Z);


	MOTOR1=MOTORLimit(PIDMIX(+1,+1,+1));	 	//  1	  2
    MOTOR2=MOTORLimit(PIDMIX(-1,+1,-1));		//     x
    MOTOR3=MOTORLimit(PIDMIX(+1,-1,-1));		//  3     4
    MOTOR4=MOTORLimit(PIDMIX(-1,-1,+1));		//

}

/*-----------------------------------------电机输出限制-------------------------------------------*/
int MOTORLimit(float value)
{
  	  if(value>MOTORMaxLimit)
	    {				  
		  	value=MOTORMaxLimit;
		}
	  else if(value<MOTORMinLimit)
		{
			value=MOTORMinLimit;
		}
	  else 
		{
		   value=value;
		}

	  return value;
}	

void Getdesireddata(int ch1,int ch2,int ch3,int ch4)		   //pwmout1:横滚	pwmout2:油门	pwmout3:俯仰	pwmout4:航向														   				 	  
{
		if(ch2<ThrMinLimit){	ch1=ThrMinLimit;}			   //油门限制
		if(ch2>ThrMaxLimit){	ch1=ThrMaxLimit;}	

		if(ch1<AttitudeMinLimit){	ch2=AttitudeMinLimit;}
		if(ch1>AttitudeMaxLimit){	ch2=AttitudeMaxLimit;}

		if(ch3<AttitudeMinLimit){	ch3=AttitudeMinLimit;}
		if(ch3>AttitudeMaxLimit){	ch3=AttitudeMaxLimit;}

		if(ch4<AttitudeMinLimit){	ch4=AttitudeMinLimit;}
		if(ch4>AttitudeMaxLimit){	ch4=AttitudeMaxLimit;}  	  


		Motor_Thr=ch2;					   			   	   //油门期望-遥控器通道3
		Motor_Ele=(int)(ch3-Elemiddle);					   //俯仰期望-遥控器通道2		 值域-100~100
	    Motor_Ail=(int)(ch1-Ailmiddle);					   //横滚期望-遥控器通道4
		Motor_Rud=(int)(ch4-Rudmiddle);					   //航向期望-遥控器通道1

		Motor_Ele=-Motor_Ele/20;				//转化单位为角度-31~31度	 此处STM32F407上传感器与机体的横滚与俯仰是反的
		Motor_Ail=-Motor_Ail/20;

		if(Motor_Rud<10&&Motor_Rud>-10)
		{
			  Motor_Rud=0;
		}
		else
		{Motor_Rud=Motor_Rud/8;}				//-125~125度
				
}


void controlmiddleinit(int pwmout1,int pwmout2,int pwmout3,int pwmout4)	  //	  pwmout1:横滚	  pwmout2:油门
{														   				  //	  pwmout3:俯仰	  pwmout4:航向
		unsigned char i;
		for(i=0;i<10;i++)
		{
			Elemiddle+=pwmout3;											//俯仰
			delay_ms(30);
		}
		Elemiddle=Elemiddle/10;	

		for(i=0;i<10;i++)
		{
			Ailmiddle+=pwmout1;	
			delay_ms(25);
		}
		Ailmiddle=Ailmiddle/10;	

		for(i=0;i<10;i++)
		{
			Rudmiddle+=pwmout4;	
			delay_ms(25);
		}
		Rudmiddle=Rudmiddle/10;	
} 
