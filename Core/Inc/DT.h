#ifndef __DT_H
#define __DT_H
#include "main.h"

void DT_Send(const int16_t* a, uint8_t len);
void DT_SendF1(int16_t a_1, int16_t a_2,int16_t a_3,int16_t a_4,int16_t a_5,int16_t a_6);
void DT_Send01(int16_t acc_x, int16_t acc_y,int16_t acc_z,int16_t gyrox,int16_t gyroy,int16_t gyroz, uint8_t sta);
void DT_Send03(int16_t rol, int16_t pit,int16_t yaw, uint8_t sta);
#endif


