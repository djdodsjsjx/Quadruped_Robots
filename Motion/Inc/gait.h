#ifndef __GAIT_H
#define __GAIT_H

#include "main.h"

#define TROT_SPEED 0.007
#define TURN_SPEED 0.007  
#define MOTION_TS 1     
#define FAAI 0.5

#define	IMU_POSX_12_Kp_G 0
#define	IMU_POSX_34_Kp_G 0
// #define OPENMV_Angle_Kp_G 0.3
// #define	OPENMV_Deviation_Kp_G 0.2
#define OPENMV_Angle_Kp_G 0.4
#define	OPENMV_Deviation_Kp_G 0.2
#define OPENMV_ANGLE_KP_X 0
// extern u8 TROT_TIMES;

extern uint8_t PA_PO_Status_TIMES, Turn_Left_Status_TIMES, Turn_Right_Status_TIMES;
extern uint8_t PA_PO_FLAG, Turn_Left_FLAG, Turn_Right_FLAG;

extern float Get_Gait_Time(void);
extern Coordinates* Stand_Gait(void);
extern Coordinates* Pa_Po_Gait(float speed);
extern Coordinates* Front_Gait(float speed, uint8_t times_on, uint8_t times);
extern Coordinates* Front_Gait2(float speed,uint8_t times, uint8_t* button_times);
extern Coordinates* Back_Gait(float speed, uint8_t times_on, uint8_t times);
extern Coordinates* Back_Gait2(float speed, uint8_t times, uint8_t* button_times);
extern Coordinates* Turn_Left_Gait(float speed, uint8_t set_angle_on);  //左转
extern Coordinates* Turn_Left_Gait2(float speed,uint16_t angles, uint8_t* times);
extern Coordinates* Turn_Right_Gait(float speed, uint8_t set_angle_on); //右转
extern Coordinates* Turn_Right_Gait2(float speed,uint16_t angles, uint8_t* times);
#endif
