#ifndef __POSTURE_CONTROL_H
#define __POSTURE_CONTROL_H
#include "main.h"

#define POS_PIT_MAX 20      //俯仰角极限值
#define	IMU_PIT_Kp_G 0.007      //俯仰角kp值


extern Coordinates* Pos_Stab(uint8_t cmd);
extern Coordinates* Pos_Adjust(float pit, float rol, float pos_x, float speed);
extern Coordinates* Pos_Adjust2(float pit, float rol, float speed);
extern Coordinates* Get_Pos_Coor(void);
extern Coordinates* Pos_Test(void);
extern void POS_PID_INIT(void);
	
#endif
