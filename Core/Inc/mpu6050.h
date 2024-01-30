#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "main.h"
//IO方向设置
#define MPU_SDA_IN()  {GPIOA->CRL&=0XFFF0FFFF;GPIOA->CRL|=8<<16;}
#define MPU_SDA_OUT() {GPIOA->CRL&=0XFFF0FFFF;GPIOA->CRL|=3<<16;}

//IO操作函数	 
#define MPU_IIC_SCL    PAout(5) //SCL
#define MPU_IIC_SDA    PAout(4) //SDA	 
#define MPU_READ_SDA   PAin(4)  //输入SDA 

extern void MPU6050_initialize(void); 
extern unsigned char MPU_I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
extern uint8_t MPU_IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
extern unsigned char MPU_IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
#endif

//------------------End of File----------------------------
