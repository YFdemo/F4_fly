#include "control.h"
#include "imu.h"
#include "delay.h"

int Elemiddle=0;
int	Ailmiddle=0;
int	Rudmiddle=0;

pidsuite pidRoll;					   //�����pid
pidsuite pidPitch;					   //������pid
pidsuite pidYaw;
	 
int Motor_Thr=0;					   //����
int Motor_Ele=0;					   //��������
int Motor_Ail=0;					   //�������
int Motor_Rud=0;					   //��������

int MOTOR1;
int MOTOR2;
int MOTOR3;	
int MOTOR4;

int servo_Roll=1500; 
int servo_Pitch=1500;

float pid_roll;
float pid_pitch;
float pid_yaw;

/*------------------------------------------pid�ṹ��ʼ��-----------------------------------------*/
//����������ṹ��ָ�룬����ֵ��kp,ki,kd
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

/*----------------------------------------------pid�������------------------------------------------*/
//���������pid�ṹ��ָ�룬����ֵ ,����ֵ
//�����pid���
float pidUpdate(pidsuite* pid, const float measured,float expect,float gyro)
{
  float output;
  static float lastoutput=0;

  pid->desired=expect;			 				//��ȡ�����Ƕ�

  pid->error = pid->desired - measured;	 	  //ƫ�����-����ֵ
  
  pid->integ += pid->error * IMU_UPDATE_DT;	  //ƫ�����
 
  if (pid->integ > pid->iLimit)				  //����������
  {
    pid->integ = pid->iLimit;
  }
  else if (pid->integ < -pid->iLimit)
  {
    pid->integ = -pid->iLimit;
  }				 

 // pid->deriv = (pid->error - pid->prevError) / IMU_UPDATE_DT;		//΢��	 Ӧ�ÿ��������ǽ��ٶȴ���
  pid->deriv = -gyro;
  if(fabs(pid->error)>Piddeadband)									//pid����
  {
		  pid->outP = pid->kp * pid->error;								 //��������۲�
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

  pid->prevError = pid->error;							 		//����ǰһ��ƫ��
  lastoutput=output;

  return output;
}

/*----------------------------------------------pid���ֲ�������ֵ-------------------------------------------*/
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


/*------------------------------------pid����------------------------------------*/
#define PIDMIX(X,Y,Z) Motor_Thr + pid_pitch*X + pid_roll*Y + pid_yaw*Z
void PID_CAL(void)		  //PID��������
{

  	pid_roll = pidUpdate(&pidRoll,Q_ANGLE.Roll,Motor_Ail,GyroFinal.X);
	pid_pitch = pidUpdate(&pidPitch,Q_ANGLE.Pitch,Motor_Ele,GyroFinal.Y);
	pid_yaw = pidUpdate(&pidYaw,Q_ANGLE.Yaw,Motor_Rud,GyroFinal.Z);


	MOTOR1=MOTORLimit(PIDMIX(+1,+1,+1));	 	//  1	  2
    MOTOR2=MOTORLimit(PIDMIX(-1,+1,-1));		//     x
    MOTOR3=MOTORLimit(PIDMIX(+1,-1,-1));		//  3     4
    MOTOR4=MOTORLimit(PIDMIX(-1,-1,+1));		//

}

/*-----------------------------------------����������-------------------------------------------*/
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

void Getdesireddata(int ch1,int ch2,int ch3,int ch4)		   //pwmout1:���	pwmout2:����	pwmout3:����	pwmout4:����														   				 	  
{
		if(ch2<ThrMinLimit){	ch1=ThrMinLimit;}			   //��������
		if(ch2>ThrMaxLimit){	ch1=ThrMaxLimit;}	

		if(ch1<AttitudeMinLimit){	ch2=AttitudeMinLimit;}
		if(ch1>AttitudeMaxLimit){	ch2=AttitudeMaxLimit;}

		if(ch3<AttitudeMinLimit){	ch3=AttitudeMinLimit;}
		if(ch3>AttitudeMaxLimit){	ch3=AttitudeMaxLimit;}

		if(ch4<AttitudeMinLimit){	ch4=AttitudeMinLimit;}
		if(ch4>AttitudeMaxLimit){	ch4=AttitudeMaxLimit;}  	  


		Motor_Thr=ch2;					   			   	   //��������-ң����ͨ��3
		Motor_Ele=(int)(ch3-Elemiddle);					   //��������-ң����ͨ��2		 ֵ��-100~100
	    Motor_Ail=(int)(ch1-Ailmiddle);					   //�������-ң����ͨ��4
		Motor_Rud=(int)(ch4-Rudmiddle);					   //��������-ң����ͨ��1

		Motor_Ele=-Motor_Ele/20;				//ת����λΪ�Ƕ�-31~31��	 �˴�STM32F407�ϴ����������ĺ���븩���Ƿ���
		Motor_Ail=-Motor_Ail/20;

		if(Motor_Rud<10&&Motor_Rud>-10)
		{
			  Motor_Rud=0;
		}
		else
		{Motor_Rud=Motor_Rud/8;}				//-125~125��
				
}


void controlmiddleinit(int pwmout1,int pwmout2,int pwmout3,int pwmout4)	  //	  pwmout1:���	  pwmout2:����
{														   				  //	  pwmout3:����	  pwmout4:����
		unsigned char i;
		for(i=0;i<10;i++)
		{
			Elemiddle+=pwmout3;											//����
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
