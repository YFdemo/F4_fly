/**********************************************************************************
 * �ļ���  ��imu.c
 * ����    �����������������ںϴ����õ�pitch roll ��̬��
 * ʵ��ƽ̨�����������STM32���ذ�
 * ��汾  ��ST3.5 
**********************************************************************************/	
#include "imu.h"																		  
#include "MPU6050.h"
#include "IIC.h"
#include "delay.h"

/*-------------------------------------------------------------------------------------------------------------------*/
char databuffer[15];       //�������ݻ�����
SENSOR_DATA Gyrobuf;						   //���������ݽṹ��short��x,y,z//������
SENSOR_DATA Accbuf;							 //���ٶȼ�x,y,z

SENSOR_DATA Gyrooffset;							 //����	������//������X,Y,Z
SENSOR_DATA	Accoffset;							  //���ٶȼ�X,Y��Z

IMU_DATA GyroFinal;							 //����������ֵ
IMU_DATA AccFinal;							 //���ٶȼ�����ֵ

S_FLOAT_ANGLE Q_ANGLE;					//�����̬�ǽṹ��//����Pitch������Roll��ƫ��Yaw
                 
float		exInt=0,eyInt=0,ezInt=0;
#define Kp 100.0f
#define Ki 0.003f
#define halfT 0.002 //����ʱ���һ�룬���ǵ�Ӧ����2ms
/*-------------------------------------------------------------------------------------------------------------------*/					

void Get_MPU6050data(void)
{
   databuffer[0]=Single_Read(MPU6050_Addr,GYRO_XOUT_L); 
   databuffer[1]=Single_Read(MPU6050_Addr,GYRO_XOUT_H);
   Gyrobuf.X=(databuffer[1]<<8)|databuffer[0];						   //��ȡX������

   databuffer[2]=Single_Read(MPU6050_Addr,GYRO_YOUT_L);
   databuffer[3]=Single_Read(MPU6050_Addr,GYRO_YOUT_H);
   Gyrobuf.Y=(databuffer[3]<<8)|databuffer[2];						   //��ȡY������

   databuffer[4]=Single_Read(MPU6050_Addr,GYRO_ZOUT_L);
   databuffer[5]=Single_Read(MPU6050_Addr,GYRO_ZOUT_H);
   Gyrobuf.Z=(databuffer[5]<<8)|databuffer[4];					       //��ȡZ������
  
/*---------------------------------------------------------------------------------------*/  
   databuffer[6]=Single_Read(MPU6050_Addr,ACCEL_XOUT_L); 
   databuffer[7]=Single_Read(MPU6050_Addr,ACCEL_XOUT_H);
   Accbuf.X=(databuffer[7]<<8)|databuffer[6];					       //��ȡX������

   databuffer[8]=Single_Read(MPU6050_Addr,ACCEL_YOUT_L);
   databuffer[9]=Single_Read(MPU6050_Addr,ACCEL_YOUT_H);
   Accbuf.Y=(databuffer[9]<<8)|databuffer[8]; 						   //��ȡY������

   databuffer[10]=Single_Read(MPU6050_Addr,ACCEL_ZOUT_L);
   databuffer[11]=Single_Read(MPU6050_Addr,ACCEL_ZOUT_H);
   Accbuf.Z=(databuffer[11]<<8)|databuffer[10]; 					   //��ȡZ������
}

/*-----------------------------------ƽ�����У����У������������尲װ�����ĵ����ƫ�----------------------------------------*/
void Zero_Correct(void)
{
  Q_ANGLE.Pitch=Q_ANGLE.Pitch-PITCH_OFFEST; 					// pitch #define PITCH_OFFEST -1.0    #define ROLL_OFFEST -1.0
  Q_ANGLE.Roll=Q_ANGLE.Roll-ROLL_OFFEST; 						// roll 
}

/*-----------------------------------���ٶȼƻ�׼У��(error)----------------------------------------*/
void Acc_Correct(void)
{
	unsigned char i=0;
	unsigned char numAcc=200;

	int Angleaccx=0;
	int Angleaccy=0;
	int Angleaccz=0;							  //���ٶȼ�У���м����

	for(i=0;i<numAcc;i++)
	{		
		Get_MPU6050data();						   //���MPU6050����
		Angleaccx+=Accbuf.X;
		Angleaccy+=Accbuf.Y;
		Angleaccz+=Accbuf.Z;
		delay_ms(2);							 //�ӳ�2ms
	}	

	Accoffset.X= Angleaccx/numAcc;					   
	Accoffset.Y= Angleaccy/numAcc;
	Accoffset.Z= Angleaccy/numAcc;				   //�õ����ٶȼƻ�׼
		
}


/*-----------------------------------�����ǻ�׼У��----------------------------------------*/
void Gyro_Correct(void)
{
	unsigned char i=0;
	unsigned char numGyro=200;

	int Gyrox=0;
	int Gyroy=0;
	int Gyroz=0;							  //������У���м����

	for(i=0;i<numGyro;i++)
	{
		Get_MPU6050data();					 //���MPU6050����
		Gyrox+=Gyrobuf.X;
		Gyroy+=Gyrobuf.Y;
		Gyroz+=Gyrobuf.Z;
		delay_ms(2);
	}	

	Gyrooffset.X= Gyrox/numGyro;					   
	Gyrooffset.Y= Gyroy/numGyro;
	Gyrooffset.Z= Gyroz/numGyro;		     //�õ������Ǽƻ�׼
}
/*-----------------------------------MPU6050���ݳ�����---------------------------------------*/
void IMUdataprepare(void)			 //���IMU׼������
{  
	Get_MPU6050data();							  //���MPU6050����
	GyroFinal.X=(Gyrobuf.X-Gyrooffset.X)*0.061*0.0174;
	GyroFinal.Y=(Gyrobuf.Y-Gyrooffset.Y)*0.061*0.0174;
	GyroFinal.Z=(Gyrobuf.Z-Gyrooffset.Z)*0.061*0.0174;		//����ֵ��ȥ��׼ֵ���Ե�λ�����������ǽ��ٶ�
	//1/16.4=0.061
	//�˴���0.0174��3.14/180,����ÿ���Ϊ����ÿ�롣
	
	AccFinal.X=(float)((Accbuf.X-Accoffset.X)*0.244)*0.0098;		
	AccFinal.Y=(float)((Accbuf.Y-Accoffset.Y)*0.244)*0.0098;		
	AccFinal.Z=(float)((Accbuf.Z-Accoffset.Z)*0.244)*0.0098;	

}


		
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
  static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; 

  float norm=0.0f;
  float vx, vy, vz;// wx, wy, wz;
  float ex, ey, ez;


  norm = sqrt(ax*ax + ay*ay +az*az);       //����ά������ģ               
  ax = ax /norm;		                //���������� �Ѽ��ٶȼƵ���ά����ת�ɵ�λ������
  ay = ay / norm;
  az = az / norm;
    
  vx = 2*(q1*q3 - q0*q2);						// ���Ʒ��������						
  vy = 2*(q0*q1 + q2*q3);						//������λ����
  vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
//���ǰ���Ԫ������ɡ��������Ҿ����еĵ����е�����Ԫ�ء�
//�������Ҿ����ŷ���ǵĶ��壬��������ϵ������������ת����������ϵ��������������Ԫ�ء�
//���������vx\y\z����ʵ���ǵ�ǰ��ŷ���ǣ�����Ԫ�����Ļ����������ϵ�ϣ����������������λ������


  // Error is sum of cross product between estimated direction and measured direction of field vectors
  ex = (ay*vz - az*vy);          //�������ٶ������Ĳ�����ó���һ����������                 					
  ey = (az*vx - ax*vz);
  ez = (ax*vy - ay*vx);
//axyz�ǻ����������ϵ�ϣ����ٶȼƲ����������������Ҳ����ʵ�ʲ����������������
//axyz�ǲ����õ�������������vxyz�����ݻ��ֺ����̬����������������������Ƕ��ǻ����������ϵ�ϵ�����������
//������֮�������������������ݻ��ֺ����̬�ͼӼƲ��������̬֮�����
//������������������������Ҳ�������������ˣ�����ʾ��exyz�����������������Ĳ����
//�����������Ծ���λ�ڻ�������ϵ�ϵģ������ݻ������Ҳ���ڻ�������ϵ�����Ҳ���Ĵ�С�����ݻ����������ȣ����������������ݡ���������Լ��ö�������һ�£����������ǶԻ���ֱ�ӻ��֣����Զ����ݵľ�������ֱ�������ڶԻ�������ϵ�ľ�����

  
  exInt = exInt + ex * Ki ;					//�����Ӧ�û��ַ���			 
  eyInt = eyInt + ey * Ki ;
  ezInt = ezInt + ez * Ki ;


  gx = gx + Kp*ex + exInt;					//У�������ǲ���ֵ	   �ò���������PI����������ƫ							
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;		   
											   
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;							// ������Ԫ����	 ��Ԫ��΢�ַ���	��Ԫ�������㷨��һ�����ⷨ
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;			   	   
										   
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);		// ��������Ԫ ����Ԫ����ģ
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  //ת��Ϊŷ����
  Q_ANGLE.Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 					// pitch
  Q_ANGLE.Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; 	// roll
  Q_ANGLE.Yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 57.3; // yaw
}


void AGMIMUupdate(float gx, float gy, float gz, float ax, float ay, float az,float mx, float my, float mz)
{
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

  // �Ȱ���Щ�õõ���ֵ���
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
 
  float delta_2=0;

  const static float FACTOR = 0.002;

  norm = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;

  norm = sqrt(mx*mx + my*my + mz*mz);          
  mx = mx / norm;
  my = my / norm;
  mz = mz / norm;

  // compute reference direction of flux			 ת������������ϵ��ȥȷ������͵���ķ���
  hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);         
  bx = sqrt((hx*hx) + (hy*hy));					   //������任��Ϊ��0������Ĥû�б�
  bz = hz;     
  
  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;				//����ת������������ϵ��ȥ�Ͳ���ֵ���Ա�
//  wx = (float)(2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2));
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//  wz = (float)(2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2));  		 //��ת������������ϵ��ȥ�Ͳ���ֵ���Ա�
  
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);				 //�ۺϴźͼ��ٶȵ����  ��˿���û��
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);


 // halfT=GET_NOWTIME();			  //���μ����ʱ��������λ��	                                                             

  gx = gx + ex*FACTOR/halfT; 					//У�������ǲ���ֵ	   �ò���������PI����������ƫ							
  gy = gy + ey*FACTOR/halfT; 
  gz = gz + ez*FACTOR/halfT;	 
	  
  delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);	
  // integrate quaternion rate and normalise  �����漰��һ����ʽQ=0.5*w*q  (Q��ʾ��Ԫ���ı仯�ʣ�w��ʾ���ٶȣ�q��ʾ��Ԫ��) Q��T��������
  q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;			// ������Ԫ����	 ��Ԫ��΢�ַ���	��Ԫ�������㷨�����ױϿ���
  q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;			//���渳ֵdelta_2=0....��̫�������
  q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT;			 
/* 												   
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;			// ������Ԫ����	 ��Ԫ��΢�ַ���	��Ԫ�������㷨��һ�����ⷨ
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;			   	   */
										   
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);		// ��������Ԫ
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  //ת��Ϊŷ����
  Q_ANGLE.Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 					// pitch
  Q_ANGLE.Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; 	// roll
  Q_ANGLE.Yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 57.3; // yaw
}
