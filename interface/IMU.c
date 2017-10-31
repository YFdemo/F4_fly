/**********************************************************************************
 * 文件名  ：imu.c
 * 描述    ：将传感器的数据融合处理，得到pitch roll 姿态角
 * 实验平台：四轴飞行器STM32主控板
 * 库版本  ：ST3.5 
**********************************************************************************/	
#include "imu.h"																		  
#include "MPU6050.h"
#include "IIC.h"
#include "delay.h"

/*-------------------------------------------------------------------------------------------------------------------*/
char databuffer[15];       //接收数据缓存区
SENSOR_DATA Gyrobuf;						   //传感器数据结构体short，x,y,z//陀螺仪
SENSOR_DATA Accbuf;							 //加速度计x,y,z

SENSOR_DATA Gyrooffset;							 //补偿	，抵消//陀螺仪X,Y,Z
SENSOR_DATA	Accoffset;							  //加速度计X,Y，Z

IMU_DATA GyroFinal;							 //陀螺仪最终值
IMU_DATA AccFinal;							 //加速度计最终值

S_FLOAT_ANGLE Q_ANGLE;					//输出姿态角结构体//俯仰Pitch，滚动Roll，偏航Yaw
                 
float		exInt=0,eyInt=0,ezInt=0;
#define Kp 100.0f
#define Ki 0.003f
#define halfT 0.002 //采样时间的一半，我们的应该是2ms
/*-------------------------------------------------------------------------------------------------------------------*/					

void Get_MPU6050data(void)
{
   databuffer[0]=Single_Read(MPU6050_Addr,GYRO_XOUT_L); 
   databuffer[1]=Single_Read(MPU6050_Addr,GYRO_XOUT_H);
   Gyrobuf.X=(databuffer[1]<<8)|databuffer[0];						   //读取X轴数据

   databuffer[2]=Single_Read(MPU6050_Addr,GYRO_YOUT_L);
   databuffer[3]=Single_Read(MPU6050_Addr,GYRO_YOUT_H);
   Gyrobuf.Y=(databuffer[3]<<8)|databuffer[2];						   //读取Y轴数据

   databuffer[4]=Single_Read(MPU6050_Addr,GYRO_ZOUT_L);
   databuffer[5]=Single_Read(MPU6050_Addr,GYRO_ZOUT_H);
   Gyrobuf.Z=(databuffer[5]<<8)|databuffer[4];					       //读取Z轴数据
  
/*---------------------------------------------------------------------------------------*/  
   databuffer[6]=Single_Read(MPU6050_Addr,ACCEL_XOUT_L); 
   databuffer[7]=Single_Read(MPU6050_Addr,ACCEL_XOUT_H);
   Accbuf.X=(databuffer[7]<<8)|databuffer[6];					       //读取X轴数据

   databuffer[8]=Single_Read(MPU6050_Addr,ACCEL_YOUT_L);
   databuffer[9]=Single_Read(MPU6050_Addr,ACCEL_YOUT_H);
   Accbuf.Y=(databuffer[9]<<8)|databuffer[8]; 						   //读取Y轴数据

   databuffer[10]=Single_Read(MPU6050_Addr,ACCEL_ZOUT_L);
   databuffer[11]=Single_Read(MPU6050_Addr,ACCEL_ZOUT_H);
   Accbuf.Z=(databuffer[11]<<8)|databuffer[10]; 					   //读取Z轴数据
}

/*-----------------------------------平衡零点校正（校正传感器与机体安装产生的的相对偏差）----------------------------------------*/
void Zero_Correct(void)
{
  Q_ANGLE.Pitch=Q_ANGLE.Pitch-PITCH_OFFEST; 					// pitch #define PITCH_OFFEST -1.0    #define ROLL_OFFEST -1.0
  Q_ANGLE.Roll=Q_ANGLE.Roll-ROLL_OFFEST; 						// roll 
}

/*-----------------------------------加速度计基准校正(error)----------------------------------------*/
void Acc_Correct(void)
{
	unsigned char i=0;
	unsigned char numAcc=200;

	int Angleaccx=0;
	int Angleaccy=0;
	int Angleaccz=0;							  //加速度计校正中间变量

	for(i=0;i<numAcc;i++)
	{		
		Get_MPU6050data();						   //获得MPU6050数据
		Angleaccx+=Accbuf.X;
		Angleaccy+=Accbuf.Y;
		Angleaccz+=Accbuf.Z;
		delay_ms(2);							 //延迟2ms
	}	

	Accoffset.X= Angleaccx/numAcc;					   
	Accoffset.Y= Angleaccy/numAcc;
	Accoffset.Z= Angleaccy/numAcc;				   //得到加速度计基准
		
}


/*-----------------------------------陀螺仪基准校正----------------------------------------*/
void Gyro_Correct(void)
{
	unsigned char i=0;
	unsigned char numGyro=200;

	int Gyrox=0;
	int Gyroy=0;
	int Gyroz=0;							  //陀螺仪校正中间变量

	for(i=0;i<numGyro;i++)
	{
		Get_MPU6050data();					 //获得MPU6050数据
		Gyrox+=Gyrobuf.X;
		Gyroy+=Gyrobuf.Y;
		Gyroz+=Gyrobuf.Z;
		delay_ms(2);
	}	

	Gyrooffset.X= Gyrox/numGyro;					   
	Gyrooffset.Y= Gyroy/numGyro;
	Gyrooffset.Z= Gyroz/numGyro;		     //得到陀螺仪计基准
}
/*-----------------------------------MPU6050数据初处理---------------------------------------*/
void IMUdataprepare(void)			 //获得IMU准备数据
{  
	Get_MPU6050data();							  //获得MPU6050数据
	GyroFinal.X=(Gyrobuf.X-Gyrooffset.X)*0.061*0.0174;
	GyroFinal.Y=(Gyrobuf.Y-Gyrooffset.Y)*0.061*0.0174;
	GyroFinal.Z=(Gyrobuf.Z-Gyrooffset.Z)*0.061*0.0174;		//读出值减去基准值乘以单位，计算陀螺仪角速度
	//1/16.4=0.061
	//此处的0.0174是3.14/180,将度每秒变为弧度每秒。
	
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


  norm = sqrt(ax*ax + ay*ay +az*az);       //求三维向量的模               
  ax = ax /norm;		                //测量正常化 把加速度计的三维向量转成单位向量。
  ay = ay / norm;
  az = az / norm;
    
  vx = 2*(q1*q3 - q0*q2);						// 估计方向的重力						
  vy = 2*(q0*q1 + q2*q3);						//重力单位向量
  vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
//这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
//根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
//所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。


  // Error is sum of cross product between estimated direction and measured direction of field vectors
  ex = (ay*vz - az*vy);          //两个加速度向量的叉积，得出另一个方向的误差                 					
  ey = (az*vx - ax*vz);
  ez = (ax*vy - ay*vx);
//axyz是机体坐标参照系上，加速度计测出来的重力向量，也就是实际测出来的重力向量。
//axyz是测量得到的重力向量，vxyz是陀螺积分后的姿态来推算出的重力向量，它们都是机体坐标参照系上的重力向量。
//那它们之间的误差向量，就是陀螺积分后的姿态和加计测出来的姿态之间的误差。
//向量间的误差，可以用向量叉积（也叫向量外积、叉乘）来表示，exyz就是两个重力向量的叉积。
//这个叉积向量仍旧是位于机体坐标系上的，而陀螺积分误差也是在机体坐标系，而且叉积的大小与陀螺积分误差成正比，正好拿来纠正陀螺。（你可以自己拿东西想象一下）由于陀螺是对机体直接积分，所以对陀螺的纠正量会直接体现在对机体坐标系的纠正。

  
  exInt = exInt + ex * Ki ;					//计算和应用积分反馈			 
  eyInt = eyInt + ey * Ki ;
  ezInt = ezInt + ez * Ki ;


  gx = gx + Kp*ex + exInt;					//校正陀螺仪测量值	   用叉积误差来做PI修正陀螺零偏							
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;		   
											   
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;							// 整合四元数率	 四元数微分方程	四元数更新算法，一阶龙库法
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;			   	   
										   
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);		// 正常化四元 ，四元数的模
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  //转换为欧拉角
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

  // 先把这些用得到的值算好
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

  // compute reference direction of flux			 转换到地理坐标系中去确定北向和地向的分量
  hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);         
  bx = sqrt((hx*hx) + (hy*hy));					   //东向将其变换成为了0，但是膜没有变
  bz = hz;     
  
  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;				//重力转换到机体坐标系中去和测量值作对比
//  wx = (float)(2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2));
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//  wz = (float)(2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2));  		 //磁转换到机体坐标系中去和测量值作对比
  
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);				 //综合磁和加速度的误差  叉乘看到没有
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);


 // halfT=GET_NOWTIME();			  //两次计算的时间间隔，单位秒	                                                             

  gx = gx + ex*FACTOR/halfT; 					//校正陀螺仪测量值	   用叉积误差来做PI修正陀螺零偏							
  gy = gy + ey*FACTOR/halfT; 
  gz = gz + ez*FACTOR/halfT;	 
	  
  delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);	
  // integrate quaternion rate and normalise  这里涉及到一个公式Q=0.5*w*q  (Q表示四元数的变化率，w表示角速度，q表示四元数) Q×T就是增量
  q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;			// 整合四元数率	 四元数微分方程	四元数更新算法，二阶毕卡法
  q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;			//上面赋值delta_2=0....不太理解意义
  q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT;			 
/* 												   
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;			// 整合四元数率	 四元数微分方程	四元数更新算法，一阶龙库法
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;			   	   */
										   
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);		// 正常化四元
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  //转换为欧拉角
  Q_ANGLE.Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 					// pitch
  Q_ANGLE.Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; 	// roll
  Q_ANGLE.Yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 57.3; // yaw
}
