#ifndef __IMU_H
#define	__IMU_H
#include "stm32F10x.h"
#include <math.h>


#define PITCH_OFFEST -1.0
#define ROLL_OFFEST -1.0

typedef struct float_angle{
				float Roll;
				float Pitch;
				float Yaw;}S_FLOAT_ANGLE;

typedef struct  sensor_data{
				short X;
				short Y;
				short Z;}SENSOR_DATA;

typedef struct  imu_data{
				float X;
				float Y;
				float Z;}IMU_DATA;

extern S_FLOAT_ANGLE Q_ANGLE;
extern IMU_DATA GyroFinal;
extern IMU_DATA	AccFinal;

void Get_MPU6050data(void);
void Gyro_Correct(void);
void Acc_Correct(void);
void Zero_Correct(void);

void IMUdataprepare(void);

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);									//≤Ë≤ªÀºŒ¢Àƒ÷·
void AGMIMUupdate(float gx, float gy, float gz, float ax, float ay, float az,float mx, float my, float mz);

#endif /* __IMU_H */
