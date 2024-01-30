#include "calculate.h"
#include "servo.h"
#include <math.h>
#include "DT.h"

#define MY_POW(a) pow(a, 2)
#define l1_12 BOBY_L1_FRONT
#define l1_34 BOBY_L1_BACK
#define l2_12 BOBY_L2_FRONT
#define l2_34 BOBY_L2_BACK
#define l3 BOBY_L3

void Caculate(Coordinates* a) {
	float lxz[4], dyz[4], lyz[4];
	float angle[12];   //转动角度
	dyz[0] = sqrt(MY_POW(a->y[0]) + MY_POW(a->z[0]));
	lyz[0] = sqrt(MY_POW(dyz[0]) - MY_POW(l3));
	lxz[0] = sqrt(MY_POW(lyz[0]) + MY_POW(a->x[0]));
	angle[0] = atan(lyz[0] / l3) + atan(a->z[0] / a->y[0]);
	angle[1] = acos((MY_POW(l1_12) + MY_POW(lxz[0]) - MY_POW(l2_12)) / (2*l1_12 * lxz[0])) - atan(a->x[0] / lyz[0]);
	angle[2] = PI - acos((MY_POW(l1_12) + MY_POW(l2_12) - MY_POW(lxz[0])) / (2*l1_12 * l2_12));

	// int16_t data[3] = {dyz[0], lyz[0], lxz[0]};
	// DT_Send(data, 3);
	dyz[1] = sqrt(MY_POW(a->y[1]) + MY_POW(a->z[1]));
	lyz[1] = sqrt(MY_POW(dyz[1]) - MY_POW(l3));
	lxz[1] = sqrt(MY_POW(lyz[1]) + MY_POW(a->x[1]));
	angle[3] = atan(lyz[1] / l3) - atan(a->z[1] / a->y[1]);
	angle[4] = acos((MY_POW(l1_12) + MY_POW(lxz[1]) - MY_POW(l2_12)) / (2*l1_12*lxz[1])) - atan(a->x[1] / lyz[1]);
	angle[5] = PI-acos((MY_POW(l1_12) + MY_POW(l2_12) - MY_POW(lxz[1])) / (2*l1_12 * l2_12));

	dyz[2] = sqrt(MY_POW(a->y[2]) + MY_POW(a->z[2]));
	lyz[2] = sqrt(MY_POW(dyz[2]) - MY_POW(l3));
	lxz[2] = sqrt(MY_POW(lyz[2]) + MY_POW(a->x[2]));
	angle[6] = atan(lyz[2] / l3) - atan(a->z[2] / a->y[2]);
	angle[7] = acos((MY_POW(l1_34) + MY_POW(lxz[2]) - MY_POW(l2_34)) / (2 * l1_34 * lxz[2])) - atan(a->x[2] / lyz[2]);
	angle[8] = PI-acos((MY_POW(l1_34) + MY_POW(l2_34) - MY_POW(lxz[2])) / (2 * l1_34 * l2_34));
	
	dyz[3] = sqrt(MY_POW(a->y[3]) + MY_POW(a->z[3]));
	lyz[3] = sqrt(MY_POW(dyz[3]) - MY_POW(l3));
	lxz[3] = sqrt(MY_POW(lyz[3]) + MY_POW(a->x[3]));
	angle[9] = atan(lyz[3] / l3) + atan(a->z[3] / a->y[3]);
	angle[10] = acos((MY_POW(l1_34) + MY_POW(lxz[3]) - MY_POW(l2_34)) / (2 * l1_34 * lxz[3])) - atan(a->x[3] / lyz[3]);	
	angle[11] = PI - acos((MY_POW(l1_34) + MY_POW(l2_34) - MY_POW(lxz[3])) / (2 * l1_34 * l2_34));

	for (int i = 0; i < 12; ++i) 
		angle[i] = 180 * angle[i] / PI;
	
	// DT_SendF1(angle[0], angle[1], angle[2], angle[3], angle[4], angle[5]);

	angle[0] = servo_init[0] + angle[0];
	angle[1] = servo_init[1] - (90 - angle[1]);
	angle[2] = servo_init[2] - (90 - angle[2]);

	angle[3] = servo_init[3] - angle[3];
	angle[4] = servo_init[4] + (90 - angle[4]);
	angle[5] = servo_init[5] + (90 - angle[5]);

	angle[6] = servo_init[6] + angle[6];
	angle[7] = servo_init[7] + (90 - angle[7]);
	angle[8] = servo_init[8] + (90 - angle[8]);

	angle[9] = servo_init[9] - angle[9];
	angle[10] = servo_init[10] - (90 - angle[10]);
	angle[11] = servo_init[11] - (90 - angle[11]);

	// DT_SendF1(angle_out[0], angle_out[1], angle_out[2], angle_out[3], angle_out[4], angle_out[5]);
	
	Angle_To_Pwm(angle);
	
}

