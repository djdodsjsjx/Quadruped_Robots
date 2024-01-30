#include "mpu6050.h"
#include "tim.h"

/**************************实现函数********************************************
*函数原型:		void IIC_Start(void)
*功　　能:		产生IIC起始信号
*******************************************************************************/
static int MPU_IIC_Start(void) {
	MPU_SDA_OUT();     //sda线输出
	MPU_IIC_SDA=1;
	if (!MPU_READ_SDA) return 0;	
	MPU_IIC_SCL=1;
	HAL_Delay_us(1);
 	MPU_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	if (MPU_READ_SDA) return 0;
	HAL_Delay_us(1);
	MPU_IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
	return 1;
}

/**************************实现函数********************************************
*函数原型:		void IIC_Stop(void)
*功　　能:	    //产生IIC停止信号
*******************************************************************************/	  
static void MPU_IIC_Stop(void) {
	MPU_SDA_OUT();//sda线输出
	MPU_IIC_SCL=0;
	MPU_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	HAL_Delay_us(1);
	MPU_IIC_SCL=1; 
	MPU_IIC_SDA=1;//发送I2C总线结束信号
	HAL_Delay_us(1);							   	
}

/**************************实现函数********************************************
*函数原型:		uint8_t IIC_Wait_Ack(void)
*功　　能:	    等待应答信号到来 
//返回值：1，接收应答失败
//        0，接收应答成功
*******************************************************************************/
static int MPU_IIC_Wait_Ack(void) {
	uint8_t ucErrTime = 0;
	MPU_SDA_IN();      //SDA设置为输入  
	MPU_IIC_SDA = 1;
	HAL_Delay_us(1);	   
	MPU_IIC_SCL = 1;
	HAL_Delay_us(1);	 
	while (MPU_READ_SDA) {
		ucErrTime++;
		if(ucErrTime > 50)
		{
			MPU_IIC_Stop();
			return 0;
		}
	  HAL_Delay_us(1);
	}
	MPU_IIC_SCL = 0;//时钟输出0 	   
	return 1;  
} 

/**************************实现函数********************************************
*函数原型:		void IIC_Ack(void)
*功　　能:	    产生ACK应答
*******************************************************************************/
static void MPU_IIC_Ack(void) {
	MPU_IIC_SCL = 0;
	MPU_SDA_OUT();
	MPU_IIC_SDA = 0;
	HAL_Delay_us(1);
	MPU_IIC_SCL = 1;
	HAL_Delay_us(1);
	MPU_IIC_SCL = 0;
}
	
/**************************实现函数********************************************
*函数原型:		void IIC_NAck(void)
*功　　能:	    产生NACK应答
*******************************************************************************/	    
static void MPU_IIC_NAck(void) {
	MPU_IIC_SCL = 0;
	MPU_SDA_OUT();
	MPU_IIC_SDA = 1;
	HAL_Delay_us(1);
	MPU_IIC_SCL = 1;
	HAL_Delay_us(1);
	MPU_IIC_SCL = 0;
}
/**************************实现函数********************************************
*函数原型:		void IIC_Send_Byte(uint8_t txd)
*功　　能:	    IIC发送一个字节
*******************************************************************************/		  
static void MPU_IIC_Send_Byte(uint8_t txd) {                        
    uint8_t t;   
	MPU_SDA_OUT(); 	    
    MPU_IIC_SCL = 0;//拉低时钟开始数据传输
    for (t = 0; t < 8; ++t) {              
        MPU_IIC_SDA = (txd & 0x80) >> 7;
        txd <<= 1; 	  
		HAL_Delay_us(1);   
		MPU_IIC_SCL = 1;
		HAL_Delay_us(1); 
		MPU_IIC_SCL = 0;	
		HAL_Delay_us(1);
    }	 
} 	 

/**************************实现函数********************************************
*函数原型:		uint8_t IIC_Read_Byte(unsigned char ack)
*功　　能:	    //读1个字节，ack=1时，发送ACK，ack=0，发送nACK 
*******************************************************************************/  
static uint8_t MPU_IIC_Read_Byte(unsigned char ack) {
	unsigned char i, receive = 0;
	MPU_SDA_IN();//SDA设置为输入
    for (i = 0; i < 8; ++i) {
        MPU_IIC_SCL = 0; 
        HAL_Delay_us(5);
		MPU_IIC_SCL = 1;
        receive <<= 1;
        if(MPU_READ_SDA) {
			receive++;  
		} 
		HAL_Delay_us(5); 
    }					 
    if (ack) MPU_IIC_Ack(); //发送ACK 
    else MPU_IIC_NAck();//发送nACK  
    return receive;
}

/**************************实现函数********************************************
*函数原型:		unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*功　　能:	    读取指定设备 指定寄存器的一个值
输入	I2C_Addr  目标设备地址
		addr	   寄存器地址
返回   读出来的值
*******************************************************************************/ 
extern unsigned char MPU_I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr) {
	unsigned char res=0;
	
	MPU_IIC_Start();	
	MPU_IIC_Send_Byte(I2C_Addr << 1);	   //发送写命令
	res++;
	MPU_IIC_Wait_Ack();
	MPU_IIC_Send_Byte(addr); 
	res++;  //发送地址
	MPU_IIC_Wait_Ack();	  
	//IIC_Stop();//产生一个停止条件	
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((I2C_Addr << 1)+1); 
	res++;          //进入接收模式			   
	MPU_IIC_Wait_Ack();
	res = MPU_IIC_Read_Byte(0);	   
    MPU_IIC_Stop();//产生一个停止条件

	return res;
}


/**************************实现函数********************************************
*函数原型:		uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
*功　　能:	    读取指定设备 指定寄存器的 length个值
输入	dev  目标设备地址
		reg	  寄存器地址
		length 要读的字节数
		*data  读出的数据将要存放的指针
返回   读出来的字节数量
*******************************************************************************/ 
extern uint8_t MPU_IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data) {
    uint8_t count = 0;
	
	MPU_IIC_Start();
	MPU_IIC_Send_Byte(dev << 1);	   //发送写命令
	MPU_IIC_Wait_Ack();
	MPU_IIC_Send_Byte(reg);   //发送地址
    MPU_IIC_Wait_Ack();	  
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((dev << 1) + 1);  //进入接收模式	
	MPU_IIC_Wait_Ack();
	
    for(count = 0; count < length; count++){	 
		if(count != length-1) data[count] = MPU_IIC_Read_Byte(1);  //带ACK的读数据
		else  data[count]=MPU_IIC_Read_Byte(0);	 //最后一个字节NACK
	}
    MPU_IIC_Stop();//产生一个停止条件
    return count;
}

/**************************实现函数********************************************
*函数原型:		uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
*功　　能:	    将多个字节写入指定设备 指定寄存器
输入	dev  目标设备地址
		reg	  寄存器地址
		length 要写的字节数
		*data  将要写的数据的首地址
返回   返回是否成功
*******************************************************************************/ 
static uint8_t MPU_IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data) {
 	uint8_t count = 0;
	MPU_IIC_Start();
	MPU_IIC_Send_Byte(dev << 1);	   //发送写命令
	MPU_IIC_Wait_Ack();
	MPU_IIC_Send_Byte(reg);   //发送地址
    MPU_IIC_Wait_Ack();	  
	for(count = 0; count < length; count++){
		MPU_IIC_Send_Byte(data[count]); 
		MPU_IIC_Wait_Ack(); 
	}
	MPU_IIC_Stop();//产生一个停止条件

    return 1; //status == 0;
}

/**************************实现函数********************************************
*函数原型:		uint8_t IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data)
*功　　能:	    读取指定设备 指定寄存器的一个值
输入	dev  目标设备地址
		reg	   寄存器地址
		*data  读出的数据将要存放的地址
返回   1
*******************************************************************************/ 


static uint8_t MPU_IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data) {
	*data=MPU_I2C_ReadOneByte(dev, reg);
    return 1;
}

/**************************实现函数********************************************
*函数原型:		unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*功　　能:	    写入指定设备 指定寄存器一个字节
输入	dev  目标设备地址
		reg	   寄存器地址
		data  将要写入的字节
返回   1
*******************************************************************************/ 
extern unsigned char MPU_IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data) {
    return MPU_IICwriteBytes(dev, reg, 1, &data);
}

/**************************实现函数********************************************
*函数原型:		uint8_t IICwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位
输入	dev  目标设备地址
		reg	   寄存器地址
		bitStart  目标字节的起始位
		length   位长度
		data    存放改变目标字节位的值
返回   成功 为1 
 		失败为0
*******************************************************************************/ 

static uint8_t MPU_IICwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data) {
    uint8_t b;
    if (MPU_IICreadByte(dev, reg, &b) != 0) {
        uint8_t mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return MPU_IICwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}


/**************************实现函数********************************************
*函数原型:		uint8_t IICwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入	dev  目标设备地址
		reg	   寄存器地址
		bitNum  要修改目标字节的bitNum位
		data  为0 时，目标位将被清0 否则将被置位
返回   成功 为1 
 		失败为0
*******************************************************************************/ 
static uint8_t MPU_IICwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    MPU_IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return MPU_IICwriteByte(dev, reg, b);
}


// /**************************实现函数********************************************
// *函数原型:		bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
// *功　　能:		
// *******************************************************************************/
// static int MPU_i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data) {
// 	int i;
//     if (!MPU_IIC_Start()) return 1;
//     MPU_IIC_Send_Byte(addr << 1);
//     if (!MPU_IIC_Wait_Ack()) {
//         MPU_IIC_Stop();
//         return 1;
//     }
//     MPU_IIC_Send_Byte(reg);
//     MPU_IIC_Wait_Ack();
// 	for (i = 0; i < len; i++) {
// 		MPU_IIC_Send_Byte(data[i]);
// 		if (!MPU_IIC_Wait_Ack()) {
// 			MPU_IIC_Stop();
// 			return 0;
// 		}
// 	}
//     MPU_IIC_Stop();
//     return 0;
// }
// /**************************实现函数********************************************
// *函数原型:		bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
// *功　　能:		
// *******************************************************************************/
// static int MPU_i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf) {
//     if (!MPU_IIC_Start()) return 1;
//     MPU_IIC_Send_Byte(addr << 1);
//     if (!MPU_IIC_Wait_Ack()) {
//         MPU_IIC_Stop();
//         return 1;
//     }
//     MPU_IIC_Send_Byte(reg);
//     MPU_IIC_Wait_Ack();
//     MPU_IIC_Start();
//     MPU_IIC_Send_Byte((addr << 1) + 1);
//     MPU_IIC_Wait_Ack();
//     while (len) {
//         if (len == 1)
//             *buf = MPU_IIC_Read_Byte(0);
//         else
//             *buf = MPU_IIC_Read_Byte(1);
//         buf++;
//         len--;
//     }
//     MPU_IIC_Stop();
//     return 0;
// }


uint8_t buffer[14];

static void MPU6050_setClockSource(uint8_t source) {
    MPU_IICwriteBits(0x68, 0x6B, 2, 3, source);
}

static void MPU6050_setFullScaleGyroRange(uint8_t range) {
    MPU_IICwriteBits(0x68, 0x1B, 4, 2, range);
}

static void MPU6050_setFullScaleAccelRange(uint8_t range) {
    MPU_IICwriteBits(0x68, 0x1C, 4, 2, range);
}


static void MPU6050_resetEnabled(uint8_t enabled) {
    MPU_IICwriteBit(0x68, 0x6B, 7, enabled);
	HAL_Delay(100);
}


static void MPU6050_set_Gyro_Accel_outRange(uint8_t range) {
    MPU_IICwriteBits(0x68, 0x6C, 5, 6, range);
}

static void MPU6050_set_SMPRT_DIV(uint8_t range) {
	MPU_IICwriteByte(0x68,0x19,range);
}

static void MPU6050_set_Gyro_DLPF_outRange(uint8_t range) {
    MPU_IICwriteBits(0x68, 0x1A, 2, 3, range);
}


static void MPU6050_INT_Disabled(uint8_t enabled) {
    MPU_IICwriteBit(0x68, 0x38, 0, enabled);
}


static void MPU6050_setSleepEnabled(uint8_t enabled) {
    MPU_IICwriteBit(0x68, 0x6B, 6, enabled);
}


static void MPU6050_testConnection(void) {
	uint8_t mpu_test_i = 0;
	while(buffer[0] != 0x68 && mpu_test_i < 10) {
		MPU_IICreadBytes(0x68, 0x75, 1, buffer);
		mpu_test_i++;
	}
	if(mpu_test_i >= 10) printf("Could not connect to MPU6050 \n\r");
	//	 while(1);
}

static void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    MPU_IICwriteBit(0x68, 0x6A, 5, enabled);
}

static void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    MPU_IICwriteBit(0x68, 0x37, 1, enabled);
}


extern void MPU6050_initialize(void) {
	MPU6050_testConnection();     //检测MPU6050 是否已经连接
	MPU6050_resetEnabled(1);   //复位mpu6050  需延时100ms
    MPU6050_setClockSource(2); //设置时钟
	MPU6050_set_Gyro_Accel_outRange(0);  // 设置输出三轴陀螺仪和三轴加速度数据
	MPU6050_INT_Disabled(0);    //禁止中断
	MPU6050_set_SMPRT_DIV(0);   //采样分频
    MPU6050_setFullScaleGyroRange(3);//陀螺仪最大量程 +-1000度每秒
    MPU6050_setFullScaleAccelRange(0);	//加速度度最大量程 +-2G
  	MPU6050_set_Gyro_DLPF_outRange(2);  //设置陀螺仪和加速度计的低通滤波器
    MPU6050_setSleepEnabled(0); //进入工作状态
	MPU6050_setI2CMasterModeEnabled(0);	 //不让MPU6050 控制AUXI2C
	MPU6050_setI2CBypassEnabled(1);	 //主控制器的I2C与	MPU6050的AUXI2C	直通。控制器可以直接访问HMC5883L
}


//------------------End of File----------------------------
