#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "stm32f10x.h"

#define DEFAULT_PID_INTEGRATION_LIMIT  100.0
#define IMU_UPDATE_FREQ   250		//250hz
#define IMU_UPDATE_DT     (float)(1.0/IMU_UPDATE_FREQ)		//���ֳ���

#define PID_ROLL_KP  1.5//0.4//0.46		   3.25						  	3.5
#define PID_ROLL_KI  0.0 //0//0.0 //2.0	 0.1
#define PID_ROLL_KD  40.0//0.15//0.1	   20   25   30		27	   40	 50
#define PID_ROLL_INTEGRATION_LIMIT    80.0 // 20.0

#define PID_PITCH_KP  1.5//0.4 
#define PID_PITCH_KI  0.0//0.0		  0.1
#define PID_PITCH_KD  40.0//0.09
#define PID_PITCH_INTEGRATION_LIMIT   25.0 // 20.0

#define PID_YAW_KP 0.0				  //0.5~1.0			 2����
#define PID_YAW_KI 0.0
#define PID_YAW_KD 0.0					   //����ǰ�Ӹ���  ??
#define PID_YAW_INTEGRATION_LIMIT   20.0 // 20.0

#define Piddeadband 0		  //1~2

#define ServoRoll_Middle 1500
#define ServoPitch_Middle 1500	 			//��̨���������

#define MOTORMaxLimit 2000
#define MOTORMinLimit 1000	 		
#define ThrMaxLimit 2000
#define ThrMinLimit 1000	 	
#define AttitudeMaxLimit 2000
#define AttitudeMinLimit 1000	 	

extern int MOTOR1;
extern int MOTOR2;
extern int MOTOR3;	
extern int MOTOR4;

extern int servo_Roll;
extern int servo_Pitch;

extern int Motor_Thr;
extern int Motor_Ele;					   //��������
extern int Motor_Ail;					   //�������
extern int Motor_Rud;					   //��������

extern float pid_roll;
extern float pid_pitch;
extern float pid_yaw;

typedef struct
{
  float desired;     //< ����������ֵ
  float error;        //< ����ֵ-ʵ��ֵ
  float prevError;    //< ǰһ��ƫ��
  float integ;        //< ���ֲ���
  float deriv;        //< ΢�ֲ���
  float kp;           //< ��������
  float ki;           //< ���ֲ���
  float kd;           //< ΢�ֲ���
  float outP;         //< pid�������֣�������
  float outI;         //< pid���ֲ��֣�������
  float outD;         //< pid΢�ֲ��֣�������
  float iLimit;      //< ��������
} pidsuite;

extern pidsuite pidRoll;
extern pidsuite pidPitch;
extern pidsuite pidYaw;


void pidInit(pidsuite* pid, const float desired, const float kp, const float ki, const float kd);
float pidUpdate(pidsuite* pid, const float measured,float expect,float gyro);
void pidSetIntegralLimit(pidsuite* pid, const float limit);
void PID_controllerInit(void);
void PID_CAL(void);
int MOTORLimit(float value);
void Getdesireddata(int ch1,int ch2,int ch3,int ch4);
void controlmiddleinit(int pwmout1,int pwmout2,int pwmout3,int pwmout4);

void platform_control(float roll_data,						//������̬��
					  float pitch_data,
					  int platform_r_des,  					//��̨�ں�����Ƕ�
					  int platform_p_des);					//��̨�ڸ������Ƕ�

#endif
