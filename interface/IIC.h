#ifndef _IIC_H
#define _IIC_H

#include "stm32f10x.h"


/*ģ��IIC�˿�������붨��*/
#define SCL_H         GPIO_SetBits(GPIOB,GPIO_Pin_10)
#define SCL_L         GPIO_ResetBits(GPIOB,GPIO_Pin_10)
   
#define SDA_H         GPIO_SetBits(GPIOB,GPIO_Pin_11)
#define SDA_L         GPIO_ResetBits(GPIOB,GPIO_Pin_11)

#define SCL_read      GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)
#define SDA_read      GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)

typedef enum {FALSE = 0, TRUE = !FALSE} bool;

void I2C_GPIO_Config(void);		 				//GPIO����

void I2C_delay(void);
void delay5ms(void);

bool I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NoAck(void);
bool I2C_WaitAck(void);

void I2C_SendByte(u8 SendByte);	   					//���ݴӸ�λ����λ//
unsigned char I2C_RadeByte(void);  					//���ݴӸ�λ����λ//

bool Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);
//���ֽ�д��*******************************************
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address);
//���ֽڶ�ȡ*****************************************
	
#endif
