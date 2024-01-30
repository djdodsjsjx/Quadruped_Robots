#include "control.h"
#include "Posture_control.h"
#include "calculate.h"
#include "servo.h"
#include "openmv.h"
#include "gait.h"
#include "tim.h"
#include "DT.h"
#include "pid.h"
#include "usart.h"
#include "oled.h"
#include "filter.h"
#include "esp32.h"

#ifdef USE_MPU9250
#include "mpu9250.h"
//rol:机器人的横滚角  pit：机器人的俯仰角  yaw：机器人的偏航角
int16_t ROBOT_ROL = 0, ROBOT_PIT = 0, ROBOT_YAW = 0;
//通过陀螺仪转向
extern int16_t yaw_target;
int16_t yaw_target;
static Coordinates* State_Detection2(float speed) {
	static uint32_t first_time = 0;
	if (PA_PO_FLAG == 1) {                    //在直线状态下更新转向的角度
		switch (opmv.lt.sta) {
			case 2: {                          //左转指令
				if (first_time == 0) first_time = HAL_GetTick();      //第一次检测到转向指令
			  	if (HAL_GetTick() - first_time >= 500) {   
					yaw_target = (int16_t)(ROBOT_YAW + 90) % 360;      //加90度
					Turn_Left_FLAG = 1;
					Turn_Right_FLAG = 0;
					first_time = 0;                  //确认转向后，将初始时间清零，等待下一次判断
				}
				break;
			}
			case 3: {                          //右转指令
				if (first_time == 0) first_time = HAL_GetTick();
			  	if (HAL_GetTick() - first_time >= 500) {
					yaw_target = (int16_t)(ROBOT_YAW + 270) % 360;     //减90度
					Turn_Right_FLAG = 1;
					Turn_Left_FLAG = 0;
					first_time = 0;
				}
				break; 
			}
			default:{
				yaw_target = ROBOT_YAW; 
				first_time = 0;          //若在500ms内不满足转向指令，将初始时间清零，等待下一次转向指令
				break;
			} 
		}	
	}
	if ( (Turn_Left_FLAG == 1) && ( (yaw_target - ROBOT_YAW > 15) || (yaw_target <= 90 &&  ROBOT_YAW >= 270 && yaw_target + 360 - ROBOT_YAW > 15) ) ) {        //左转
		PA_PO_FLAG = 0;
		return Turn_Left_Gait(speed, 0);
	} else if ((Turn_Right_FLAG == 1) && ((ROBOT_YAW - yaw_target > 15) || (yaw_target >= 270 && ROBOT_YAW <= 90 && ROBOT_YAW + 360 - yaw_target > 15))) {     //右转
		PA_PO_FLAG = 0;
		return Turn_Right_Gait(speed, 0);
		// return Stand_Gait();
	} else {						//直线
		PA_PO_FLAG = 1;
		Turn_Left_FLAG = 0;
		Turn_Right_FLAG = 0;
	}
	return Pa_Po_Gait(speed);
}

#elif USE_MPU6050
#include "filter.h"
int16_t ROBOT_ROL, ROBOT_PIT;
#endif // USE_MPU9250

//估计左转周期
uint8_t opmv_sta_times = 10;         //检测次数
uint8_t Turn_Left_Right_TIMES = 6;   //转弯周期数
static Coordinates* State_Detection(float speed) {
	if (PA_PO_FLAG == 1)	{
		switch (opmv.lt.sta) {
			case 2: {                          //左转
				while(opmv_sta_times > 0) {
					HAL_Delay_us(50);
					if(opmv.lt.sta != 2) break;
					opmv_sta_times--;
				}
				if(opmv_sta_times == 0) {
					Turn_Left_FLAG = 1;
					Turn_Left_Status_TIMES = 0;
					PA_PO_FLAG = 0;
					PA_PO_Status_TIMES = 0;
				}
				break; 
			}
			case 3: {                          //右转
				while(opmv_sta_times > 0) {
					HAL_Delay_us(50);
					if(opmv.lt.sta != 3) break;
					opmv_sta_times--;
				}
				if(opmv_sta_times == 0)	{
					Turn_Right_FLAG = 1;
					Turn_Right_Status_TIMES = 0;
					PA_PO_FLAG = 0;
					PA_PO_Status_TIMES = 0;
				}
				break; 
			}
			default:{
				PA_PO_FLAG = 1;
				break;
			} 
		}
	}
	//转弯->直线
	if(PA_PO_FLAG == 1 || Turn_Left_Status_TIMES > Turn_Left_Right_TIMES || Turn_Right_Status_TIMES > Turn_Left_Right_TIMES){         
		Turn_Left_FLAG = 0;
		Turn_Right_FLAG = 0;
		PA_PO_FLAG = 1;
		return Pa_Po_Gait(speed);
	}
	//左转
	if(Turn_Left_FLAG == 1 && Turn_Left_Status_TIMES <= Turn_Left_Right_TIMES){
		if(PA_PO_Status_TIMES < 4){            //待转弯
			return Pa_Po_Gait(speed);
		}
		else{
			return Turn_Left_Gait(speed, 0);
		}
		
	}
	//右转
	if(Turn_Right_FLAG == 1 && Turn_Right_Status_TIMES <= Turn_Left_Right_TIMES){
		if(PA_PO_Status_TIMES < 4){
			return Pa_Po_Gait(speed);
		}
		else{
			return Turn_Left_Gait(speed, 0);
		}
	}
	return Pa_Po_Gait(speed);
}

static Coordinates* State_Detection3(float speed) {
	//在直线状态下判断转向指令
	if (PA_PO_FLAG == 1 && opmv.lt.pos_y != 0 && ABS(opmv.lt.pos_y) < 15) {         
		if (opmv.lt.sta == 2) {
			Turn_Left_FLAG = 1;
			PA_PO_FLAG = 0;
			return Turn_Left_Gait(speed, 0);
		} else if (opmv.lt.sta == 3) {
			Turn_Right_FLAG = 1;
			PA_PO_FLAG = 0;
			return Turn_Right_Gait(speed, 0);
		}
	}
	//在转向状态下，判断是否切换至直线循迹
	if (Turn_Left_FLAG == 1 || Turn_Right_FLAG == 1) {
		if ((opmv.lt.sta == 1) && (ABS(opmv.lt.angle) < 20)) {
			Turn_Left_FLAG = 0;
			Turn_Right_FLAG = 0;
			PA_PO_FLAG = 1;
		} else if (opmv.lt.sta == 2) {
			return Turn_Left_Gait(speed, 0);
		} else if (opmv.lt.sta == 3) {
			return Turn_Right_Gait(speed, 0);
		}
	}
	return Pa_Po_Gait(speed);
}


extern uint8_t esp_cmd_count[40];    //接收的指令次数
extern uint8_t esp_cmd, esp_data;
/*
	speed：速度
	cmd：控制运行指令
	data：运行中控制指令
*/
static void Motion_Task(float speed, uint8_t cmd, uint8_t data) {
	static int16_t esp_yaw = 0;
	static float speed_control = 0.f;
	if (esp_cmd == 17 && data == 17) speed_control += 0.00001f;       //加速
	else if (esp_cmd == 18 && data == 18) speed_control -= 0.00001f;  //减速
	speed += speed_control;
	speed = LIMIT(speed, 0.004, 0.013);
	Coordinates* a = Pos_Stab(cmd);            //姿态控制
	Coordinates* b;
	switch (cmd) {
		case 0: b = State_Detection3(speed); break;       //爬坡模式
		// case 0: Caculate(a); return ;
		// case 5: b = Front_Gait(speed, 0, 0);   break;		 //前进
		// case 5: Front_Gait2(speed, 1, data, &esp_cmd_count[cmd]); break;		 //前进
		// case 6: b = Back_Gait(speed, 0, 0);   break;		 //后退
		// case 7: b = Turn_Left_Gait(speed, 0);   break;   //左转
		// case 8: b = Turn_Right_Gait(speed, 0);  break;   //右转
		case 5: b = Front_Gait2(speed, data, &esp_cmd_count[cmd]); break;
		case 6: b = Back_Gait2(speed, data, &esp_cmd_count[cmd]); break;
		case 7: b = Turn_Left_Gait2(speed, data, &esp_cmd_count[cmd]); break;
		case 8: b = Turn_Right_Gait2(speed, data, &esp_cmd_count[cmd]);  break;
		default: {
			if (cmd == 1) speed_control = 0;     //复位
			Caculate(a); return ;  //默认站立
		}
	}
    Coordinates res = {0};
    for (int i = 0; i < 4; ++i) {
        res.x[i] = a->x[i] + b->x[i];
        res.y[i] = a->y[i] + b->y[i];
        res.z[i] = a->z[i] + b->z[i];
    }
	// DT_SendF1(res.x[0], res.y[0], res.z[0], ROBOT_PIT, 0, 0);
	Caculate(&res);
}

//抬脚
void Lift_Feet(uint8_t cmd) {
	Coordinates* a;
	static uint8_t up = 1, down = 1;
	static float high = 0;
	if (high < 50 || up == 1) down = 0, up = 1, high += 0.3;
	if (high > 80 || down == 1) down = 1, up = 0, high -= 0.3;

	if (cmd == 15) {              //抬左脚
		a = Pos_Adjust(-5, 20, -30, 0.1);
		a->z[0] += high;
		a->x[0] += 30;
	} else if (cmd == 16) {       //抬右脚
		a = Pos_Adjust(-5, -20, -30, 0.1);
		a->z[1] += high;
		a->x[1] += 30;
	} 
	Caculate(a);
}

//扭动身体
void Stance_swing(float speed) {
	static uint8_t step = 0, times = 0;  //step: 运行的阶段   times:每个阶段运行的次数
	Coordinates* a;
	if (step == 0) {
		a = Pos_Adjust2(0, 15, speed);
		if (ABS((ROBOT_ROL - 15)) < 5) ++step;
	} else if (step == 1) {
		a = Pos_Adjust2(0, -15, speed);
		if (ABS((ROBOT_ROL + 15)) < 5) step = 0, ++times;
		if (times == 2) step = 2, times = 0;
	} else if (step == 2) {
		a = Pos_Adjust2(0, 0, speed);
		if (ABS(ROBOT_ROL) < 5) ++step;
	} else if (step == 3) {
		a = Pos_Adjust2(10, 0, speed);
		if (ABS((ROBOT_PIT - 10)) < 5) ++step;
	} else if (step == 4) {
		a = Pos_Adjust2(-10, 0, speed);
		if (ABS((ROBOT_PIT + 10)) < 5) step = 2, ++times;
		if (times == 2) step = 5, times = 0;
	} else if (step == 5) {
		a = Pos_Adjust2(0, 0, speed);
	}
	Caculate(a);
}

/**
	cmd : 控制运行指令 
	data : 运行控制指令
	爬坡 ： cmd = 0, data = 0
**/
extern void Robot_Control(uint8_t cmd, uint8_t data) {
#ifdef ROBOT_SET    
    Servo_Init();
#endif

#ifdef ROBOT_MOTION
	if (HAL_GetTick() > 3000) {
		Motion_Task(0.007, cmd, data);
		// Stance_swing(0.15);
	}
#endif
} 

#ifdef PID_TEST   //PID测试
double Temp = 1, PIDOut, TempSetpoint = 100;
void Tempin_Update(void) {
	PID_Compute(&TPID);
	Temp += *(TPID.MyOutput);
	DT_SendF1(Temp, TempSetpoint, 0, 0, 0, 0);
}

void Tempout_Update(void) {     //更新输出值，使输出值按三角波变化
	static uint8_t up = 1, down = 1;
	if (TempSetpoint == 100 || up == 1) down = 0, up = 1, TempSetpoint += 5;
	if (TempSetpoint == 200 || down == 1) up = 0, down = 1, TempSetpoint -= 5;
	// PID(&TPID, &Temp, &PIDOut, &TempSetpoint, 0.5, 0, 0.005, _PID_P_ON_E, _PID_CD_DIRECT);
	// PID_Init(&TPID, &Temp, &PIDOut, &TempSetpoint);
}
#endif

void User_Init(void) {
  OLED_Init();
#ifdef PID_TEST
	PID_Init(&TPID, &Temp, &PIDOut, &TempSetpoint);
#endif 
  HAL_UART_Receive_IT(&huart1, &Uart1_aRxBuffer, 1);
  HAL_UART_Receive_IT(&huart3, &Uart3_aRxBuffer, 1);
  POS_PID_INIT();
#ifdef USE_MPU9250
	MPU9250_INIT();
	PA_PO_FLAG = 1;
#elif USE_MPU6050
	MPU6050_initialize();
#endif // DEBUG
}


//串口接收控制
uint8_t Uart1_aRxBuffer;	
uint8_t Uart3_aRxBuffer;			

//串口回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
  if(huart->Instance == USART1) {               //pc
	ESP_Byte_Get2(Uart1_aRxBuffer);
	// Receive_Test2(Uart1_aRxBuffer2);
    HAL_UART_Receive_IT(&huart1, &Uart1_aRxBuffer, 1);
  } else if(huart->Instance == USART3) {        //jetson xavier nx
    // OpenMV_Byte_Get(Uart3_aRxBuffer);
	// ESP_Byte_Get2(Uart3_aRxBuffer);
	Jet_Byte_Get(Uart3_aRxBuffer);
	// HAL_UART_Transmit_IT(&huart1, &Uart3_aRxBuffer, 1);
    HAL_UART_Receive_IT(&huart3, &Uart3_aRxBuffer, 1);
  }
}

//接收控制指令
void ESP_Control(void) {
	float gait_time = Get_Gait_Time();
	static uint8_t cmd = 4, data;
	if (HAL_GetTick() > 3000) {
		if ((gait_time > 0.98 && gait_time < 1) || gait_time == 0) {
			if (esp_cmd == 17 || esp_cmd == 18) data = esp_cmd;
			else {
				cmd = esp_cmd;
				data = esp_data;
			}
		}
		switch (cmd)
		{
			// case 1: HAL_NVIC_SystemReset(); break;      //复位
			case 2: break;							     //暂停	
			// case 3: Servo_Init(); break;				//校准模式
			case 3: break;				//校准模式
			case 15: 
			case 16: Lift_Feet(cmd); break;
			case 21: Stance_swing(0.15); break;
			default: {      //运行模式
				Robot_Control(cmd, data);
				break;
			}		   
		}
	}
}

/*
	0：爬坡模式
	1：复位
	2：暂停
	3：校准模式
	4：站立
	5：前进
	6：后退
	7：左转
	8：右转
	9：左转90度
	10：右转90度
	11：上升
	12：下降
	13：抬头
	14：低头
	15：抬左前脚
	16：抬右前脚
	17：加速
	18：减速
	19：前进指定次数
	20：后退指定次数
	21：扭动身体
*/
void Cmd_Test(void) {
	static uint8_t step = 0;
	static uint32_t last_time = 3000;
	if (HAL_GetTick() > 3000) {
		if (step == 0) {
			Robot_Control(0, 0);
			if (HAL_GetTick() - last_time > 5000) {
				++step,	last_time = HAL_GetTick();
			}
		} else if (step == 1) {
			Robot_Control(5, 0);
			if (HAL_GetTick() - last_time > 5000) {
				++step, last_time = HAL_GetTick();
			}
		} else if (step == 2) {
			Robot_Control(5, 17);
			if (HAL_GetTick() - last_time > 5000) {
				++step, last_time = HAL_GetTick();
			}
		} else if (step == 3) {
			Robot_Control(5, 18);
			if (HAL_GetTick() - last_time > 5000) {
				++step, last_time = HAL_GetTick();
			}
		} else if (step == 4) {
			Robot_Control(1, 0);
			if (HAL_GetTick() - last_time > 5000) {
				++step, last_time = HAL_GetTick();
			}
		} else if (step == 5) {
			Robot_Control(5, 0);
			if (HAL_GetTick() - last_time > 5000) {
				++step, last_time = HAL_GetTick();
			}
		}
	}
}


