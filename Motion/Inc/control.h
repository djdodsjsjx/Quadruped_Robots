#ifndef __CONTROL_H
#define __CONTROL_H
#include "main.h"

//校准模式和运动模式切换
// #define ROBOT_SET   //舵机安装校准
#ifndef ROBOT_SET
#define ROBOT_MOTION 
#endif

//PID调试
// #define PID_TEST

// #define USE_MPU6050
#ifndef USE_MPU6050
#define USE_MPU9250
#endif // !USE_MPU6050
extern uint8_t Uart1_aRxBuffer;
extern uint8_t Uart3_aRxBuffer;;

extern int16_t ROBOT_ROL, ROBOT_PIT, ROBOT_YAW;

extern void Stand_Task(void);
extern void Robot_Control(uint8_t cmd, uint8_t data);
extern void Temp_UPdate(void);
void User_Init(void);
void ESP_Control(void);
void Send_To_Comptuer(int16_t data);
void Cmd_Test(void);
#ifdef PID_TEST
void Tempin_Update(void);
void Tempout_Update(void);
#endif // DEBUG
#endif

