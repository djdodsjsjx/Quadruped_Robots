#include "Posture_control.h"
#include "filter.h"
#include "calculate.h"
#include "DT.h"
#include "pid.h"

#define h BOBY_H
#define b BOBY_B
#define l BOBY_L
#define w BOBY_W

float pos_pid_pit_out = 0;
float pos_pid_pit_target = -2;
PID_TypeDef pos_pid_pit;

float pos_pid_posx_out = 0;
float pos_pid_last_posx_out = 0;
float pos_pid_posx_target = 0;
PID_TypeDef pos_pid_posx;

float pit;
//pit:俯仰角角 rol:横滚角 yaw:偏航角  
//pos_x:x方向偏移 pos_y:y方向偏移  pos_z:z方向偏移
static Coordinates* Pos_Control(float pit,float rol,float yaw,float pos_x,float pos_y,float pos_z) {
	static Coordinates posture_coor = {0};
	float P = pit * PI / 180;
	float R = rol * PI / 180;
	float Y = yaw * PI / 180;
	posture_coor.x[0] = l/2 - pos_x - (l*cos(P)*cos(Y))/2 + (b*cos(P)*sin(Y))/2;
	posture_coor.y[0] = w/2 - pos_y - (b*(cos(R)*cos(Y) - sin(P)*sin(R)*sin(Y)))/2 - (l*(cos(R)*sin(Y) + cos(Y)*sin(P)*sin(R)))/2;
	posture_coor.z[0] = - h - pos_z - (b*(cos(Y)*sin(R) + cos(R)*sin(P)*sin(Y)))/2 - (l*(sin(R)*sin(Y) - cos(R)*cos(Y)*sin(P)))/2;

	posture_coor.x[1] = l/2 - pos_x - (l*cos(P)*cos(Y))/2 - (b*cos(P)*sin(Y))/2;
	posture_coor.y[1] = (b*(cos(R)*cos(Y) - sin(P)*sin(R)*sin(Y)))/2 - w/2 - pos_y - (l*(cos(R)*sin(Y) + cos(Y)*sin(P)*sin(R)))/2;
	posture_coor.z[1] = (b*(cos(Y)*sin(R) + cos(R)*sin(P)*sin(Y)))/2 - pos_z - h - (l*(sin(R)*sin(Y) - cos(R)*cos(Y)*sin(P)))/2;

	posture_coor.x[2] = (l*cos(P)*cos(Y))/2 - pos_x - l/2 - (b*cos(P)*sin(Y))/2;
	posture_coor.y[2] = (b*(cos(R)*cos(Y) - sin(P)*sin(R)*sin(Y)))/2 - w/2 - pos_y + (l*(cos(R)*sin(Y) + cos(Y)*sin(P)*sin(R)))/2;
	posture_coor.z[2] = (b*(cos(Y)*sin(R) + cos(R)*sin(P)*sin(Y)))/2 - pos_z - h + (l*(sin(R)*sin(Y) - cos(R)*cos(Y)*sin(P)))/2;
	
	posture_coor.x[3] = (l*cos(P)*cos(Y))/2 - pos_x - l/2 + (b*cos(P)*sin(Y))/2;
	posture_coor.y[3] = w/2 - pos_y - (b*(cos(R)*cos(Y) - sin(P)*sin(R)*sin(Y)))/2 + (l*(cos(R)*sin(Y) + cos(Y)*sin(P)*sin(R)))/2;
	posture_coor.z[3] = (l*(sin(R)*sin(Y) - cos(R)*cos(Y)*sin(P)))/2 - pos_z - (b*(cos(Y)*sin(R) + cos(R)*sin(P)*sin(Y)))/2 - h;
	return &posture_coor;
}

extern int16_t ROBOT_ROL, ROBOT_PIT;
//自稳
extern Coordinates* Pos_Stab(uint8_t cmd) {
	// P调节 
	// static float pos_pit_out = 0;   
	// if (ABS(ROBOT_PIT) > 3) pos_pit_out += -IMU_PIT_Kp_G * ROBOT_PIT;
	// pos_pit_out = LIMIT(pos_pit_out, -POS_PIT_MAX, POS_PIT_MAX);

	// //pid调节
	pit = ROBOT_PIT;
	// if (ABS(ROBOT_PIT) > 1) PID_Compute(&pos_pid_pit);
	// DT_SendF1(pos_pid_pit_out, -ROBOT_PIT, 0, 0, 0, 0)

	static float esp_high_cur = 0;
	if (cmd == 0x0B) esp_high_cur += 0.05;
	else if (cmd == 0x0C) esp_high_cur -= 0.05;
	else if (cmd == 0x0D) pos_pid_pit_out -= 0.05;
	else if (cmd == 0x0E) pos_pid_pit_out += 0.05;
	else if (cmd == 0 && ABS(ROBOT_PIT) > 3) {
		PID_Compute(&pos_pid_pit);                     //俯仰角修正

		pos_pid_posx_target = -ROBOT_PIT * 1.5;        //x方向移动
		PID_Compute(&pos_pid_posx);
		pos_pid_last_posx_out = pos_pid_posx_out;
	} else if (cmd == 1) {                 //复位
		esp_high_cur = 0;
		pos_pid_pit_out = 0;
		pos_pid_posx_out = 0;
	}
	esp_high_cur = LIMIT(esp_high_cur, -20, 20);
	pos_pid_pit_out = LIMIT(pos_pid_pit_out, -15, 15);

	// DT_SendF1(pos_pid_posx_out, pos_pid_pit_out, ROBOT_PIT, 0, 0, 0);
	return Pos_Control(pos_pid_pit_out, 0, 0, pos_pid_posx_out, 0, esp_high_cur);   
	// return Pos_Control(pos_pit_out, 0, 0, 0, 0, esp_high_cur);   
	// return Pos_Control(0, 0, 0, 0, 0, 0);
}


/*
	自己设计角度，机器人调节至该角度
*/
extern Coordinates* Pos_Adjust(float pit, float rol, float pos_x, float speed) {
	static float pit_cur = 0, rol_cur = 0, pos_x_cur = 0;
	if (pit - pit_cur > 1) pit_cur += speed;
	else if (pit_cur - pit > 1) pit_cur -= speed;
	if (rol - rol_cur > 1) rol_cur += speed;
	else if (rol_cur - rol > 1) rol_cur -= speed;
	if (pos_x - pos_x_cur > 1) pos_x_cur += speed;
	else if (pos_x_cur - pos_x > 1) pos_x_cur -= speed;
	DT_SendF1(pit_cur, rol_cur, 0, 0, 0, 0);
	return Pos_Control(pit_cur, rol_cur, 0, pos_x_cur, 0, 0);
}

/*
	根据陀螺仪测角度，更改机器人的角度
*/
extern Coordinates* Pos_Adjust2(float pit, float rol, float speed) {
	static float pit_cur = 0, rol_cur = 0;
	if (ROBOT_PIT - pit > 3) pit_cur -= speed;
	else if (pit - ROBOT_PIT > 3) pit_cur += speed;
	if (ROBOT_ROL - rol > 3) rol_cur -= speed;
	else if (rol - ROBOT_ROL > 3) rol_cur += speed;
	return Pos_Control(pit_cur, rol_cur, 0, 0, 0, 0);
}

extern Coordinates* Pos_Test(void) {
	static int8_t i = 0, up = 1, down = 1;
	static uint16_t last = 0, now = 0;
	now = HAL_GetTick();
	if (now - last > 25) {
		if(i == -10 || up == 1) down = 0, up = 1, ++i;
		if(i == 10 || down == 1) down = 1, up = 0, --i;
		last = HAL_GetTick();
	}
	return Pos_Control(i, 0, 0, 0, 0, 0);
}

extern void POS_PID_INIT(void) {
	PID_Init(&pos_pid_pit, &pit, &pos_pid_pit_out, &pos_pid_pit_target, 10, 0.02, 0, 0, -20, 20);
	PID_Init(&pos_pid_posx, &pos_pid_last_posx_out, &pos_pid_posx_out, &pos_pid_posx_target, 10, 0.02, 0, 0, -40, 40);
}

