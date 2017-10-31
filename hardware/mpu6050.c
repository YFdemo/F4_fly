#include "MPU6050.h"
#include "IIC.h"
#include  <math.h> 

char TX_DATA[10];  	 //显示据缓存区
char BUF[15];       //接收数据缓存区
short GTRO_X,GTRO_Y,GTRO_Z,ACCEL_X,ACCEL_Y,ACCEL_Z,T_T;		 //X,Y,Z轴，温度


void DATA_printf(char *s,short temp_data)    //转字符串
{
	*s='\0';
	if(temp_data<0){
	temp_data=-temp_data;
    *s='-';
	}
	else *s=' ';
	  *++s =temp_data/1000+0x30;
	  temp_data=temp_data%1000; 
    *++s =temp_data/100+0x30;
    temp_data=temp_data%100;     //取余运算
    *++s =temp_data/10+0x30;
    temp_data=temp_data%10;      //取余运算
    *++s =temp_data+0x30; 
    	
}

void Init_MPU6050(void)
{
  	Single_Write(MPU6050_Addr,PWR_MGMT_1, 0x00);	//解除休眠状态
	Single_Write(MPU6050_Addr,SMPLRT_DIV, 0x03);	//取样率4分频，1k/4=250hz
	Single_Write(MPU6050_Addr,CONFIG, 0x02);	  	//低通滤波94hz
	Single_Write(MPU6050_Addr,GYRO_CONFIG, 0x18);	//2000dps
	Single_Write(MPU6050_Addr,ACCEL_CONFIG, 0x01); 	//+-8g,5Hz高通滤波	 
}
//******读取MPU6050数据****************************************
void READ_MPU6050(void)
{
   BUF[0]=Single_Read(MPU6050_Addr,GYRO_XOUT_L); 	 //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改,MPU6050  0xD0,陀螺仪xl地址
   BUF[1]=Single_Read(MPU6050_Addr,GYRO_XOUT_H);
   GTRO_X=	(BUF[1]<<8)|BUF[0];
   GTRO_X/=16.4; 						   //读取计算X轴数据

   BUF[2]=Single_Read(MPU6050_Addr,GYRO_YOUT_L);
   BUF[3]=Single_Read(MPU6050_Addr,GYRO_YOUT_H);
   GTRO_Y=	(BUF[3]<<8)|BUF[2];
   GTRO_Y/=16.4; 						   //读取计算Y轴数据
   BUF[4]=Single_Read(MPU6050_Addr,GYRO_ZOUT_L);
   BUF[5]=Single_Read(MPU6050_Addr,GYRO_ZOUT_H);
   GTRO_Z=	(BUF[5]<<8)|BUF[4];
   GTRO_Z/=16.4; 					       //读取计算Z轴数据
   
   BUF[6]=Single_Read(MPU6050_Addr,ACCEL_XOUT_L); 
   BUF[7]=Single_Read(MPU6050_Addr,ACCEL_XOUT_H);
   ACCEL_X=	(BUF[7]<<8)|BUF[6];
   ACCEL_X/=163.84; 						   //读取计算X轴数据

   BUF[8]=Single_Read(MPU6050_Addr,ACCEL_YOUT_L);
   BUF[9]=Single_Read(MPU6050_Addr,ACCEL_YOUT_H);
   ACCEL_Y=	(BUF[9]<<8)|BUF[8];
   ACCEL_Y/=163.84; 						   //读取计算Y轴数据
   BUF[10]=Single_Read(MPU6050_Addr,ACCEL_ZOUT_L);
   BUF[11]=Single_Read(MPU6050_Addr,ACCEL_ZOUT_H);
   ACCEL_Z=	(BUF[11]<<8)|BUF[10];
   ACCEL_Z/=163.84; 					       //读取计算Z轴数据
	 
   BUF[12]=Single_Read(MPU6050_Addr,TEMP_OUT_L); 
   BUF[13]=Single_Read(MPU6050_Addr,TEMP_OUT_H); 
   T_T=(BUF[13]<<8)|BUF[12];
   T_T = (-5)+((double) (T_T + 13200)) / 280;// 读取计算出温度
}
 //********串口发送数据***************************************	
/*unsigned int GetData(unsigned char REG_Address)
{
	char H,L;
	H=Single_Read(MPU6050_Addr,REG_Address);
	L=Single_Read(MPU6050_Addr,REG_Address+1);
	return (H<<8)+L;   //合成数据
}							   */
