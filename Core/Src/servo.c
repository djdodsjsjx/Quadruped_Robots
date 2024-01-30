#include "servo.h"
#include "tim.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_tim.h"
#include "DT.h"

uint16_t angle_pwmout[12] = {0};
float servo_init[12] = { \
	65, 160, 80,  
	55, 5,   90, 
	53, 18,  90,  
	50, 168, 88   
};

//开启中断
static void Servo_PWM_Start(void) {
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4);
}

//舵机角度->PWM脉冲
void Angle_To_Pwm(float* a) { 
	for (int i = 0; i < 12; ++i) {
		if(i % 3 == 0) {
			a[i] = LIMIT(a[i], 5, 115);
			angle_pwmout[i] = (uint16_t)(900 + a[i] * 10.0f);
		} else {
			a[i] = LIMIT(a[i], 5, 175);
			angle_pwmout[i] = (uint16_t)(500 + a[i] * 100/9.0f);
		}
	}
	Servo_PWM_Start();
}

void Servo_Init(void) {
	Angle_To_Pwm(servo_init);
}

void Servo_Test(void) {
	uint16_t a[12];
	for (int i = 0; i < 12; ++i) a[i] = 90;
	for (int i = 50; i <= 140; i += 5) {
		a[1] = i;
		a[5] = i;
		Angle_To_Pwm(a);
		HAL_Delay(25);
	}
	for (int i = 140; i >= 50; i -= 5) {
		a[1] = i;
		a[5] = i;
		Angle_To_Pwm(a);
		HAL_Delay(25);
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
  	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, angle_pwmout[0]);
  } else if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, angle_pwmout[1]);
  } else if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, angle_pwmout[2]);
  } else if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, angle_pwmout[3]);
  } else if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, angle_pwmout[4]);
  } else if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, angle_pwmout[5]);
  } else if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, angle_pwmout[6]);
  } else if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, angle_pwmout[7]);
  } else if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, angle_pwmout[8]);
  } else if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, angle_pwmout[9]);
  } else if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, angle_pwmout[10]);
  } else if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, angle_pwmout[11]);
  }
}
