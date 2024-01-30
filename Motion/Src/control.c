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
//rol:�����˵ĺ����  pit�������˵ĸ�����  yaw�������˵�ƫ����
int16_t ROBOT_ROL = 0, ROBOT_PIT = 0, ROBOT_YAW = 0;
//ͨ��������ת��
extern int16_t yaw_target;
int16_t yaw_target;
static Coordinates* State_Detection2(float speed) {
	static uint32_t first_time = 0;
	if (PA_PO_FLAG == 1) {                    //��ֱ��״̬�¸���ת��ĽǶ�
		switch (opmv.lt.sta) {
			case 2: {                          //��תָ��
				if (first_time == 0) first_time = HAL_GetTick();      //��һ�μ�⵽ת��ָ��
			  	if (HAL_GetTick() - first_time >= 500) {   
					yaw_target = (int16_t)(ROBOT_YAW + 90) % 360;      //��90��
					Turn_Left_FLAG = 1;
					Turn_Right_FLAG = 0;
					first_time = 0;                  //ȷ��ת��󣬽���ʼʱ�����㣬�ȴ���һ���ж�
				}
				break;
			}
			case 3: {                          //��תָ��
				if (first_time == 0) first_time = HAL_GetTick();
			  	if (HAL_GetTick() - first_time >= 500) {
					yaw_target = (int16_t)(ROBOT_YAW + 270) % 360;     //��90��
					Turn_Right_FLAG = 1;
					Turn_Left_FLAG = 0;
					first_time = 0;
				}
				break; 
			}
			default:{
				yaw_target = ROBOT_YAW; 
				first_time = 0;          //����500ms�ڲ�����ת��ָ�����ʼʱ�����㣬�ȴ���һ��ת��ָ��
				break;
			} 
		}	
	}
	if ( (Turn_Left_FLAG == 1) && ( (yaw_target - ROBOT_YAW > 15) || (yaw_target <= 90 &&  ROBOT_YAW >= 270 && yaw_target + 360 - ROBOT_YAW > 15) ) ) {        //��ת
		PA_PO_FLAG = 0;
		return Turn_Left_Gait(speed, 0);
	} else if ((Turn_Right_FLAG == 1) && ((ROBOT_YAW - yaw_target > 15) || (yaw_target >= 270 && ROBOT_YAW <= 90 && ROBOT_YAW + 360 - yaw_target > 15))) {     //��ת
		PA_PO_FLAG = 0;
		return Turn_Right_Gait(speed, 0);
		// return Stand_Gait();
	} else {						//ֱ��
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

//������ת����
uint8_t opmv_sta_times = 10;         //������
uint8_t Turn_Left_Right_TIMES = 6;   //ת��������
static Coordinates* State_Detection(float speed) {
	if (PA_PO_FLAG == 1)	{
		switch (opmv.lt.sta) {
			case 2: {                          //��ת
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
			case 3: {                          //��ת
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
	//ת��->ֱ��
	if(PA_PO_FLAG == 1 || Turn_Left_Status_TIMES > Turn_Left_Right_TIMES || Turn_Right_Status_TIMES > Turn_Left_Right_TIMES){         
		Turn_Left_FLAG = 0;
		Turn_Right_FLAG = 0;
		PA_PO_FLAG = 1;
		return Pa_Po_Gait(speed);
	}
	//��ת
	if(Turn_Left_FLAG == 1 && Turn_Left_Status_TIMES <= Turn_Left_Right_TIMES){
		if(PA_PO_Status_TIMES < 4){            //��ת��
			return Pa_Po_Gait(speed);
		}
		else{
			return Turn_Left_Gait(speed, 0);
		}
		
	}
	//��ת
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
	//��ֱ��״̬���ж�ת��ָ��
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
	//��ת��״̬�£��ж��Ƿ��л���ֱ��ѭ��
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


extern uint8_t esp_cmd_count[40];    //���յ�ָ�����
extern uint8_t esp_cmd, esp_data;
/*
	speed���ٶ�
	cmd����������ָ��
	data�������п���ָ��
*/
static void Motion_Task(float speed, uint8_t cmd, uint8_t data) {
	static int16_t esp_yaw = 0;
	static float speed_control = 0.f;
	if (esp_cmd == 17 && data == 17) speed_control += 0.00001f;       //����
	else if (esp_cmd == 18 && data == 18) speed_control -= 0.00001f;  //����
	speed += speed_control;
	speed = LIMIT(speed, 0.004, 0.013);
	Coordinates* a = Pos_Stab(cmd);            //��̬����
	Coordinates* b;
	switch (cmd) {
		case 0: b = State_Detection3(speed); break;       //����ģʽ
		// case 0: Caculate(a); return ;
		// case 5: b = Front_Gait(speed, 0, 0);   break;		 //ǰ��
		// case 5: Front_Gait2(speed, 1, data, &esp_cmd_count[cmd]); break;		 //ǰ��
		// case 6: b = Back_Gait(speed, 0, 0);   break;		 //����
		// case 7: b = Turn_Left_Gait(speed, 0);   break;   //��ת
		// case 8: b = Turn_Right_Gait(speed, 0);  break;   //��ת
		case 5: b = Front_Gait2(speed, data, &esp_cmd_count[cmd]); break;
		case 6: b = Back_Gait2(speed, data, &esp_cmd_count[cmd]); break;
		case 7: b = Turn_Left_Gait2(speed, data, &esp_cmd_count[cmd]); break;
		case 8: b = Turn_Right_Gait2(speed, data, &esp_cmd_count[cmd]);  break;
		default: {
			if (cmd == 1) speed_control = 0;     //��λ
			Caculate(a); return ;  //Ĭ��վ��
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

//̧��
void Lift_Feet(uint8_t cmd) {
	Coordinates* a;
	static uint8_t up = 1, down = 1;
	static float high = 0;
	if (high < 50 || up == 1) down = 0, up = 1, high += 0.3;
	if (high > 80 || down == 1) down = 1, up = 0, high -= 0.3;

	if (cmd == 15) {              //̧���
		a = Pos_Adjust(-5, 20, -30, 0.1);
		a->z[0] += high;
		a->x[0] += 30;
	} else if (cmd == 16) {       //̧�ҽ�
		a = Pos_Adjust(-5, -20, -30, 0.1);
		a->z[1] += high;
		a->x[1] += 30;
	} 
	Caculate(a);
}

//Ť������
void Stance_swing(float speed) {
	static uint8_t step = 0, times = 0;  //step: ���еĽ׶�   times:ÿ���׶����еĴ���
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
	cmd : ��������ָ�� 
	data : ���п���ָ��
	���� �� cmd = 0, data = 0
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

#ifdef PID_TEST   //PID����
double Temp = 1, PIDOut, TempSetpoint = 100;
void Tempin_Update(void) {
	PID_Compute(&TPID);
	Temp += *(TPID.MyOutput);
	DT_SendF1(Temp, TempSetpoint, 0, 0, 0, 0);
}

void Tempout_Update(void) {     //�������ֵ��ʹ���ֵ�����ǲ��仯
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


//���ڽ��տ���
uint8_t Uart1_aRxBuffer;	
uint8_t Uart3_aRxBuffer;			

//���ڻص�����
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

//���տ���ָ��
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
			// case 1: HAL_NVIC_SystemReset(); break;      //��λ
			case 2: break;							     //��ͣ	
			// case 3: Servo_Init(); break;				//У׼ģʽ
			case 3: break;				//У׼ģʽ
			case 15: 
			case 16: Lift_Feet(cmd); break;
			case 21: Stance_swing(0.15); break;
			default: {      //����ģʽ
				Robot_Control(cmd, data);
				break;
			}		   
		}
	}
}

/*
	0������ģʽ
	1����λ
	2����ͣ
	3��У׼ģʽ
	4��վ��
	5��ǰ��
	6������
	7����ת
	8����ת
	9����ת90��
	10����ת90��
	11������
	12���½�
	13��̧ͷ
	14����ͷ
	15��̧��ǰ��
	16��̧��ǰ��
	17������
	18������
	19��ǰ��ָ������
	20������ָ������
	21��Ť������
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


