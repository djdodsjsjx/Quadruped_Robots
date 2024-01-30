#include "gait.h" 
#include "openmv.h"
#include "DT.h"

extern int16_t ROBOT_ROL, ROBOT_YAW;

uint8_t PA_PO_Status_TIMES = 0, Turn_Left_Status_TIMES = 0, Turn_Right_Status_TIMES = 0;
uint8_t PA_PO_FLAG = 0, Turn_Left_FLAG = 0, Turn_Right_FLAG = 0;

static float t = 0; 
static float xf[4] = {10, 10, -20, -20}, xs[4] = {-20, -20, -50, -50}, yf[4] = {0}, ys[4] = {0}, h[4] = {20, 20, 20, 20};

extern float Get_Gait_Time(void) {
	return t;
}

static Coordinates* Trot(void) {
	static Coordinates trot_coor = {0};
	if (t <= MOTION_TS * FAAI) {
		// if (ABS(t) == TROT_SPEED) HAL_Delay(100);
		float sigma = 2 * PI * t / (FAAI * MOTION_TS);
        for (int i = 0; i < 4; ++i) {
			if (i % 2 == 0) {      //摆动相
				trot_coor.z[i] = h[i]*(1-cos(sigma))/2;
				trot_coor.x[i] = (xf[i]-xs[i]) * ((sigma-sin(sigma)) / (2*PI)) + xs[i];
				trot_coor.y[i] = (yf[i]-ys[i]) * ((sigma-sin(sigma)) / (2*PI)) + ys[i];
			} else {               //支撑相
				trot_coor.z[i] = -5;
				trot_coor.x[i] = (xs[i]-xf[i]) * ((sigma-sin(sigma)) / (2*PI)) + xf[i];
				trot_coor.y[i] = (ys[i]-yf[i]) * ((sigma-sin(sigma)) / (2*PI)) + yf[i];
			}
        }
	} else if (t < MOTION_TS) {
		// if (ABS(t - MOTION_TS * FAAI) <= TROT_SPEED) HAL_Delay(100);
		float sigma = 2 * PI * (t - MOTION_TS * FAAI) / (FAAI * MOTION_TS);
        for (int i = 0; i < 4; ++i) {
			if (i % 2 == 1) {      //摆动相
				trot_coor.z[i] = h[i]*(1-cos(sigma))/2;
				trot_coor.x[i] = (xf[i]-xs[i]) * ((sigma-sin(sigma)) / (2*PI)) + xs[i];
				trot_coor.y[i] = (yf[i]-ys[i]) * ((sigma-sin(sigma)) / (2*PI)) + ys[i];
			} else {               //支撑相
				trot_coor.z[i] = -5;
				trot_coor.x[i] = (xs[i]-xf[i]) * ((sigma-sin(sigma)) / (2*PI)) + xf[i];
				trot_coor.y[i] = (ys[i]-yf[i]) * ((sigma-sin(sigma)) / (2*PI)) + yf[i];
			}
        }
	}
	return &trot_coor;
}


extern int16_t ROBOT_PIT;
extern Coordinates* Pa_Po_Gait(float speed) {
	if (t >= MOTION_TS) {
		t = 0;
		PA_PO_Status_TIMES++;
		float pit = LIMIT(ROBOT_PIT, -20, 20);
		float ang = LIMIT(opmv.lt.angle, -30, 30);
		float dev = LIMIT(opmv.lt.deviation,-50,50);
		xf[0] = 20;   xs[0] = -10;
		xf[1] = 10;   xs[1] = -20;
		xf[2] = -20;   xs[2] = -50;
		xf[3] = -20;   xs[3] = -50;
		// 左偏：+      右斜：+
		ys[0] = - OPENMV_Angle_Kp_G * ang + OPENMV_Deviation_Kp_G * dev;  yf[0] = - ys[0];
		ys[1] = - OPENMV_Angle_Kp_G * ang + OPENMV_Deviation_Kp_G * dev;  yf[1] = - ys[1];
		ys[2] = OPENMV_Angle_Kp_G * ang + OPENMV_Deviation_Kp_G * dev;    yf[2] = - ys[2];
		ys[3] = OPENMV_Angle_Kp_G * ang + OPENMV_Deviation_Kp_G * dev;	  yf[3] = - ys[3];
		h[0] = 20 - 0.5f * pit;
		h[1] = 20 - 0.5f * pit;
		h[2] = 20 + 0.35f * pit;
		h[3] = 20 + 0.35f * pit;
		return Trot();  
	} 
	t = t + speed;
	return Trot();  
}

/*
	speed: 前进速度
	times_on: 开启计数模式
	times：计数次数
*/
extern Coordinates* Front_Gait(float speed, uint8_t times_on, uint8_t times) {
	static uint8_t times_cur = 0;
	if ((times_on && times_cur < times) || !times_on) {
		if (!times_on) times_cur = 0;
		if (t >= MOTION_TS) {
			t = 0;
			++times_cur;
			xs[0] = -20; xf[0] = 10;
			xs[1] = -20; xf[1] = 10;
			xs[2] = -50; xf[2] = -20;
			xs[3] = -50; xf[3] = -20;

			ys[0] = 0; yf[0] = 0;
			ys[1] = 0; yf[1] = 0;
			ys[2] = 0; yf[2] = 0;
			ys[3] = 0; yf[3] = 0;
			return Trot(); 
		} 
		t = t + speed;
	} 
	return Trot();
}

extern Coordinates* Front_Gait2(float speed, uint8_t times, uint8_t* button_times) {
	static uint8_t times_cur = 0, times_tar;
	if (times != 0 && (*button_times) != 0) times_tar += times, (*button_times)--;
	if (times == 0 || times_cur < times_tar) {
		if (t >= MOTION_TS) {
			t = 0;
			++times_cur;
			xs[0] = -20; xf[0] = 10;
			xs[1] = -20; xf[1] = 10;
			xs[2] = -50; xf[2] = -20;
			xs[3] = -50; xf[3] = -20;

			ys[0] = 0; yf[0] = 0;
			ys[1] = 0; yf[1] = 0;
			ys[2] = 0; yf[2] = 0;
			ys[3] = 0; yf[3] = 0;
			return Trot(); 
		} 
		t = t + speed;
	} 
	return Trot();
}

/*
	speed: 前进速度
	times_on: 开启计数模式
	times：计数次数
*/
extern Coordinates* Back_Gait(float speed, uint8_t times_on, uint8_t times) {
	static uint8_t times_cur = 0;
	if (!times_on) times_cur = 0;
	if ((times_on && times_cur < times) || !times_on) {
		if (t >= MOTION_TS) {
			t = 0;
			++times_cur;
			xs[0] = 10; xf[0] = -20;
			xs[1] = 10; xf[1] = -20;
			xs[2] = -20; xf[2] = -50;
			xs[3] = -20; xf[3] = -50;

			ys[0] = 0; yf[0] = 0;
			ys[1] = 0; yf[1] = 0;
			ys[2] = 0; yf[2] = 0;
			ys[3] = 0; yf[3] = 0;
			return Trot(); 
		} 
		t = t + speed;
	}
	return Trot();
}

extern Coordinates* Back_Gait2(float speed,uint8_t times, uint8_t* button_times) {
	static uint8_t times_cur = 0, times_tar;
	if (times != 0 && (*button_times) != 0) times_tar += times, (*button_times)--;
	if (times == 0 || times_cur < times_tar) {
		if (t >= MOTION_TS) {
			t = 0;
			++times_cur;
			xs[0] = 10; xf[0] = -20;
			xs[1] = 10; xf[1] = -20;
			xs[2] = -20; xf[2] = -50;
			xs[3] = -20; xf[3] = -50;

			ys[0] = 0; yf[0] = 0;
			ys[1] = 0; yf[1] = 0;
			ys[2] = 0; yf[2] = 0;
			ys[3] = 0; yf[3] = 0;
			return Trot(); 
		} 
		t = t + speed;
	}
	return Trot();
}


extern Coordinates* Turn_Left_Gait(float speed, uint8_t set_angle_on) {
	static uint8_t angle_times = 0;            //旋转90度的次数
	static uint16_t yaw_tar = 0;
	if (!set_angle_on) angle_times = 0;
	if (set_angle_on && angle_times < 1) yaw_tar = (ROBOT_YAW + 90) % 360, ++angle_times;
	if (((yaw_tar - ROBOT_YAW > 10) || (yaw_tar <= 90 && ROBOT_YAW >= 270 && yaw_tar + 360 - ROBOT_YAW > 10)) || 
		!set_angle_on) {
		if (t >= MOTION_TS) {
			t = 0;
			xs[0] = 5; xf[0] = 5;
			xs[1] = -5; xf[1] = -5;
			xs[2] = -35; xf[2] = -35;
			xs[3] = -35; xf[3] = -35;

			ys[0] = -15; yf[0] = 15;
			ys[1] = -15; yf[1] = 15;
			ys[2] = 15; yf[2] = -15;
			ys[3] = 15; yf[3] = -15;
			return Trot(); 
		} 
		t = t + speed;
	}
	return Trot();
}

extern Coordinates* Turn_Left_Gait2(float speed, uint16_t angles, uint8_t* button_times) {
	static uint16_t yaw_tar = 0;
	angles = angles * 10;
	if (angles != 0 && (*button_times) != 0) yaw_tar = (ROBOT_YAW + angles) % 360, (*button_times)--;
	if (((yaw_tar - ROBOT_YAW > 15) || (yaw_tar <= 90 && ROBOT_YAW >= 270 && yaw_tar + 360 - ROBOT_YAW > 15)) || angles == 0) {
		if (t >= MOTION_TS) {
			t = 0;
			xs[0] = 5; xf[0] = 5;
			xs[1] = -5; xf[1] = -5;
			xs[2] = -35; xf[2] = -35;
			xs[3] = -35; xf[3] = -35;

			ys[0] = -15; yf[0] = 15;
			ys[1] = -15; yf[1] = 15;
			ys[2] = 15; yf[2] = -15;
			ys[3] = 15; yf[3] = -15;
			return Trot(); 
		} 
		t = t + speed;
	}
	return Trot();
}

extern Coordinates* Step_Gait(float speed) {
	if (t >= MOTION_TS) {
		Turn_Left_Status_TIMES++;
		t = 0;
		for (uint8_t i = 0; i < 4; ++i) {
			xs[i] = 0;
			xf[i] = 0;
			ys[i] = 0;
			yf[i] = 0;
		}
		return Trot(); 
	} 
	t = t + speed;
	return Trot();
}

extern Coordinates* Turn_Right_Gait(float speed, uint8_t set_angle_on) {
	static uint8_t angle_times = 0;            //旋转90度的次数
	static uint16_t yaw_tar = 0;
	if (!set_angle_on) angle_times = 0;
	if (set_angle_on && angle_times < 1) yaw_tar = (ROBOT_YAW + 270) % 360, ++angle_times;
	if (((ROBOT_YAW - yaw_tar > 10) || (yaw_tar >= 270 && ROBOT_YAW <= 90 && ROBOT_YAW + 360 - yaw_tar > 10)) || 
		!set_angle_on) {
		if (t >= MOTION_TS) {
			Turn_Right_Status_TIMES++;
			t = 0;
			xs[0] = 5; xf[0] = 5;
			xs[1] = -5; xf[1] = -5;
			xs[2] = -35; xf[2] = -35;
			xs[3] = -35; xf[3] = -35;

			ys[0] = 15; yf[0] = -15;
			ys[1] = 15; yf[1] = -15;
			ys[2] = -15; yf[2] = 15;
			ys[3] = -15; yf[3] = 15;
			return Trot(); 
		} 
		t = t + speed;
	}
	return Trot();
}

extern Coordinates* Turn_Right_Gait2(float speed, uint16_t angles, uint8_t* button_times) {
	static uint16_t yaw_tar = 0;
	angles = angles * 10;
	if (angles != 0 && (*button_times) != 0) yaw_tar = (ROBOT_YAW + (360 - angles)) % 360, (*button_times)--;
	if (((ROBOT_YAW - yaw_tar > 5) || (yaw_tar >= 270 && ROBOT_YAW <= 90 && ROBOT_YAW + 360 - yaw_tar > 5)) || 
		angles == 0) {
		if (t >= MOTION_TS) {
			Turn_Right_Status_TIMES++;
			t = 0;
			xs[0] = 5; xf[0] = 5;
			xs[1] = -5; xf[1] = -5;
			xs[2] = -35; xf[2] = -35;
			xs[3] = -35; xf[3] = -35;

			ys[0] = 15; yf[0] = -15;
			ys[1] = 15; yf[1] = -15;
			ys[2] = -15; yf[2] = 15;
			ys[3] = -15; yf[3] = 15;
			return Trot(); 
		} 
		t = t + speed;
	}
	return Trot();
}