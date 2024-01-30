#include "filter.h"
#include <math.h>
#include "mpu6050.h"
#include "DT.h"

extern int16_t ROBOT_ROL, ROBOT_PIT;

float K1_X =0.02; 
float angle_X, angle_dot_X; 	
float Q_angle_X=0.001;// 过程噪声的协方差
float Q_gyro_X=0.003;//0.03 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
float R_angle_X=0.5;// 测量噪声的协方差 既测量偏差
float dt_X=0.005;//                 
char  C_0_X = 1;
float Q_bias_X, Angle_err_X;
float PCt_0_X, PCt_1_X, E_X;
float K_0_X, K_1_X, t_0_X, t_1_X;
float Pdot_X[4] ={0,0,0,0};
float PP_X[2][2] = { { 1, 0 },{ 0, 1 } };

float K1_Y =0.02; 
float angle_Y, angle_dot_Y; 	
float Q_angle_Y=0.001;// 过程噪声的协方差
float Q_gyro_Y=0.003;//0.03 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
float R_angle_Y=0.5;// 测量噪声的协方差 既测量偏差
float dt_Y=0.005;//                 
char  C_0_Y = 1;
float Q_bias_Y, Angle_err_Y;
float PCt_0_Y, PCt_1_Y, E_Y;
float K_0_Y, K_1_Y, t_0_Y, t_1_Y;
float Pdot_Y[4] ={0,0,0,0};
float PP_Y[2][2] = { { 1, 0 },{ 0, 1 } };
	
float K1_Z =0.02; 
float angle_Z, angle_dot_Z; 	
float Q_angle_Z=0.001;// 过程噪声的协方差
float Q_gyro_Z=0.003;//0.03 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
float R_angle_Z=0.5;// 测量噪声的协方差 既测量偏差
float dt_Z=0.005;//                 
char  C_0_Z = 1;
float Q_bias_Z, Angle_err_Z;
float PCt_0_Z, PCt_1_Z, E_Z;
float K_0_Z, K_1_Z, t_0_Z, t_1_Z;
float Pdot_Z[4] ={0,0,0,0};
float PP_Z[2][2] = { { 1, 0 },{ 0, 1 } };
/**************************************************************************
函数功能：简易卡尔曼滤波
入口参数：加速度、角速度
返回  值：无
**************************************************************************/

float Kalman_Filter_X(float Accel_X,float Gyro_X) {

	angle_X+=(Gyro_X - Q_bias_X) * dt_X; //先验估计
	Pdot_X[0]=Q_angle_X- PP_X[0][1] - PP_X[1][0]; // Pk-先验估计误差协方差的微分

	Pdot_X[1]=-PP_X[1][1];
	Pdot_X[2]=-PP_X[1][1];
	Pdot_X[3]=Q_gyro_X;
	PP_X[0][0] += Pdot_X[0] * dt_X;   // Pk-先验估计误差协方差微分的积分
	PP_X[0][1] += Pdot_X[1] * dt_X;   // =先验估计误差协方差
	PP_X[1][0] += Pdot_X[2] * dt_X;
	PP_X[1][1] += Pdot_X[3] * dt_X;
		
	Angle_err_X = Accel_X - angle_X;	//zk-先验估计
	
	PCt_0_X = C_0_X * PP_X[0][0];
	PCt_1_X = C_0_X * PP_X[1][0];
	
	E_X = R_angle_X + C_0_X * PCt_0_X;
	
	K_0_X = PCt_0_X / E_X;
	K_1_X = PCt_1_X / E_X;
	
	t_0_X = PCt_0_X;
	t_1_X = C_0_X * PP_X[0][1];

	PP_X[0][0] -= K_0_X * t_0_X;		 //后验估计误差协方差
	PP_X[0][1] -= K_0_X * t_1_X;
	PP_X[1][0] -= K_1_X * t_0_X;
	PP_X[1][1] -= K_1_X * t_1_X;
		
	angle_X	+= K_0_X * Angle_err_X;	 //后验估计
	Q_bias_X	+= K_1_X * Angle_err_X;	 //后验估计
	angle_dot_X   = Gyro_X - Q_bias_X;	 //输出值(后验估计)的微分=角速度
	return angle_X;
}

float Kalman_Filter_Y(float Accel_Y,float Gyro_Y) {

	angle_Y+=(Gyro_Y - Q_bias_Y) * dt_Y; //先验估计
	Pdot_Y[0]=Q_angle_Y- PP_Y[0][1] - PP_Y[1][0]; // Pk-先验估计误差协方差的微分

	Pdot_Y[1]=-PP_Y[1][1];
	Pdot_Y[2]=-PP_Y[1][1];
	Pdot_Y[3]=Q_gyro_Y;
	PP_Y[0][0] += Pdot_Y[0] * dt_Y;   // Pk-先验估计误差协方差微分的积分
	PP_Y[0][1] += Pdot_Y[1] * dt_Y;   // =先验估计误差协方差
	PP_Y[1][0] += Pdot_Y[2] * dt_Y;
	PP_Y[1][1] += Pdot_Y[3] * dt_Y;
		
	Angle_err_Y = Accel_Y - angle_Y;	//zk-先验估计
	
	PCt_0_Y = C_0_Y * PP_Y[0][0];
	PCt_1_Y = C_0_Y * PP_Y[1][0];
	
	E_Y = R_angle_Y + C_0_Y * PCt_0_Y;
	
	K_0_Y = PCt_0_Y / E_Y;
	K_1_Y = PCt_1_Y / E_Y;
	
	t_0_Y = PCt_0_Y;
	t_1_Y = C_0_Y * PP_Y[0][1];

	PP_Y[0][0] -= K_0_Y * t_0_Y;		 //后验估计误差协方差
	PP_Y[0][1] -= K_0_Y * t_1_Y;
	PP_Y[1][0] -= K_1_Y * t_0_Y;
	PP_Y[1][1] -= K_1_Y * t_1_Y;
		
	angle_Y	+= K_0_Y * Angle_err_Y;	 //后验估计
	Q_bias_Y	+= K_1_Y * Angle_err_Y;	 //后验估计
	angle_dot_Y  = Gyro_Y - Q_bias_Y;	 //输出值(后验估计)的微分=角速度
	return angle_Y;
}



float Kalman_Filter_Z(float Accel_Z,float Gyro_Z) {

	angle_Z+=(Gyro_Z - Q_bias_Z) * dt_Z; //先验估计
	Pdot_Z[0]=Q_angle_Z- PP_Z[0][1] - PP_Z[1][0]; // Pk-先验估计误差协方差的微分

	Pdot_Z[1]=-PP_Z[1][1];
	Pdot_Z[2]=-PP_Z[1][1];
	Pdot_Z[3]=Q_gyro_Z;
	PP_Z[0][0] += Pdot_Z[0] * dt_Z;   // Pk-先验估计误差协方差微分的积分
	PP_Z[0][1] += Pdot_Z[1] * dt_Z;   // =先验估计误差协方差
	PP_Z[1][0] += Pdot_Z[2] * dt_Z;
	PP_Z[1][1] += Pdot_Z[3] * dt_Z;
		
	Angle_err_Z = Accel_Z - angle_Z;	//zk-先验估计
	
	PCt_0_Z = C_0_Z * PP_Z[0][0];
	PCt_1_Z = C_0_Z * PP_Z[1][0];
	
	E_Z = R_angle_Z + C_0_Z * PCt_0_Z;
	
	K_0_Z = PCt_0_Z / E_Z;
	K_1_Z = PCt_1_Z / E_Z;
	
	t_0_Z = PCt_0_Z;
	t_1_Z = C_0_Z * PP_Z[0][1];

	PP_Z[0][0] -= K_0_Z * t_0_Z;		 //后验估计误差协方差
	PP_Z[0][1] -= K_0_Z * t_1_Z;
	PP_Z[1][0] -= K_1_Z * t_0_Z;
	PP_Z[1][1] -= K_1_Z * t_1_Z;
		
	angle_Z	+= K_0_Z * Angle_err_Z;	 //后验估计
	Q_bias_Z	+= K_1_Z * Angle_err_Z;	 //后验估计
	angle_dot_Z  = Gyro_Z - Q_bias_Z;	 //输出值(后验估计)的微分=角速度
	return angle_Z;
}

//===========   获取横滚角和俯仰角   ============
void Filter_Get_Angle(void) {
	float Accel_Y,Accel_X,Gyro_Y,Gyro_X;
	float MPU_Accel_X,MPU_Accel_Z,MPU_Accel_Y,MPU_Gyro_Y,MPU_Gyro_X,MPU_Gyro_Z;

	MPU_Gyro_Y = (MPU_I2C_ReadOneByte(0x68,0x45)<<8)+MPU_I2C_ReadOneByte(0x68,0x46);    //读取Y轴陀螺仪
	MPU_Gyro_X = (MPU_I2C_ReadOneByte(0x68,0x43)<<8)+MPU_I2C_ReadOneByte(0x68,0x44); 
	MPU_Gyro_Z = (MPU_I2C_ReadOneByte(0x68,0x47)<<8)+MPU_I2C_ReadOneByte(0x68,0x48); 
	if(MPU_Gyro_Y>32767)  (MPU_Gyro_Y==32768) ? (MPU_Gyro_Y=-32768): (MPU_Gyro_Y-=65536);     
	if(MPU_Gyro_X>32767)  (MPU_Gyro_X==32768) ? (MPU_Gyro_X=-32768): (MPU_Gyro_X-=65536); 
	if(MPU_Gyro_Z>32767)  (MPU_Gyro_Z==32768) ? (MPU_Gyro_Z=-32768): (MPU_Gyro_Z-=65536); 
	Gyro_Y=MPU_Gyro_Y/16.4;                                   
	Gyro_X=MPU_Gyro_X/16.4;                                                                      

	MPU_Accel_Z = (MPU_I2C_ReadOneByte(0x68,0x3F)<<8)+MPU_I2C_ReadOneByte(0x68,0x40);    //读取Z轴陀螺仪
	MPU_Accel_X = (MPU_I2C_ReadOneByte(0x68,0x3B)<<8)+MPU_I2C_ReadOneByte(0x68,0x3C); //读取X轴加速度记
	MPU_Accel_Y = (MPU_I2C_ReadOneByte(0x68,0x3D)<<8)+MPU_I2C_ReadOneByte(0x68,0x3E);
	if(MPU_Accel_Y>32767)  (MPU_Accel_Y==32768) ? (MPU_Accel_Y=-32768): (MPU_Accel_Y-=65536);     
	if(MPU_Accel_X>32767)  (MPU_Accel_X==32768) ? (MPU_Accel_X=-32768): (MPU_Accel_X-=65536); 
	if(MPU_Accel_Z>32767)  (MPU_Accel_Z==32768) ? (MPU_Accel_Z=-32768): (MPU_Accel_Z-=65536); 

	Accel_Y = atan2(MPU_Accel_X, MPU_Accel_Z) * 180.0/PI;                 //计算与地面的夹角	
	Accel_X = atan2(MPU_Accel_Y, MPU_Accel_Z) * 180.0/PI; 


	ROBOT_PIT = -Kalman_Filter_Y(Accel_Y, -Gyro_Y);//卡尔曼滤波	
	ROBOT_ROL = -Kalman_Filter_X(Accel_X, Gyro_X);

	// DT_Send03(Dog_ROL,Dog_PIT,0,1);				
}
