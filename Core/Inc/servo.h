#ifndef __SERVO_H
#define __SERVO_H
#include "stdint.h"

extern float servo_init[12];

void Angle_To_Pwm(float* a);
void Servo_Test(void);
void Servo_Init(void);

// extern void Angle_One_Output(uint8_t ID ,uint16_t degrees);

#endif
