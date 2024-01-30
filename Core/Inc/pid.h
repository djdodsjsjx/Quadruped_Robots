 /*
------------------------------------------------------------------------------
~ File   : pid.h
~ Author : Majid Derhambakhsh
~ Version: V1.0.0
~ Created: 02/11/2021 03:43:00 AM
~ Brief  :
~ Support:
		   E-Mail : Majid.do16@gmail.com (subject : Embedded Library Support)

		   Github : https://github.com/Majid-Derhambakhsh
------------------------------------------------------------------------------
~ Description:

~ Attention  :

~ Changes    :
------------------------------------------------------------------------------
*/

#ifndef __PID_H_
#define __PID_H_

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Include ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "main.h"
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ------------------------ Library ------------------------ */
#define _PID_LIBRARY_VERSION    1.0.0

/* ------------------------ Public ------------------------- */
#define _PID_8BIT_PWM_MAX       UINT8_MAX
#define _PID_SAMPLE_TIME_MS_DEF 100
#define GetTime() HAL_GetTick()

#ifndef _FALSE

	#define _FALSE 0

#endif

#ifndef _TRUE

	#define _TRUE 1

#endif


/* --------------------------------------------------------- */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* PID Mode */
typedef enum
{
	
	_PID_MODE_MANUAL    = 0,
	_PID_MODE_AUTOMATIC = 1
	
}PIDMode_TypeDef;

/* PID P On x */
typedef enum
{
	
	_PID_P_ON_M = 0, /* Proportional on Measurement */
	_PID_P_ON_E = 1
	
}PIDPON_TypeDef;

/* PID Control direction */
typedef enum
{
	
	_PID_CD_DIRECT  = 0,
	_PID_CD_REVERSE = 1
	
}PIDCD_TypeDef;

/* PID Structure */
typedef struct
{
	
	PIDPON_TypeDef  POnE;
	PIDMode_TypeDef InAuto;

	PIDPON_TypeDef  POn;
	PIDCD_TypeDef   ControllerDirection;

	uint32_t        LastTime;		      //上一次的时间
	uint32_t        SampleTime;           //采样时间

	float          DispKp;
	float          DispKi;
	float          DispKd;

	float          Kp;
	float          Ki;
	float          Kd;

	float          *MyInput;           //试图控制的变量(当前值)
	float          *MyOutput;          //由 pid 调整的变量(输出值)
	float          *MySetpoint;        //我们想要输入保持的值(目标值)

	float          OutputSum;          //积分累加值
	float          LastInput;			//上一次输出

	float          OutMin;				//输出下限值
	float          OutMax;				//输出上限值
	
}PID_TypeDef;

extern PID_TypeDef TPID;


void PID_Init(PID_TypeDef *uPID, float* Temp, float* PIDOut, float* TempSetpoint, uint32_t SampleTime, 
			  float kp, float ki, float kd, float output_limit_min, float output_limit_max);

void PID(PID_TypeDef *uPID, float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, PIDPON_TypeDef POn, PIDCD_TypeDef ControllerDirection);
void PID2(PID_TypeDef *uPID, float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, PIDCD_TypeDef ControllerDirection);

/* ::::::::::: Computing ::::::::::: */
uint8_t PID_Compute(PID_TypeDef *uPID);

/* ::::::::::: PID Mode :::::::::::: */
void            PID_SetMode(PID_TypeDef *uPID, PIDMode_TypeDef Mode);
PIDMode_TypeDef PID_GetMode(PID_TypeDef *uPID);

/* :::::::::: PID Limits ::::::::::: */
void PID_SetOutputLimits(PID_TypeDef *uPID, float Min, float Max);

/* :::::::::: PID Tunings :::::::::: */
void PID_SetTunings(PID_TypeDef *uPID, float Kp, float Ki, float Kd);
void PID_SetTunings2(PID_TypeDef *uPID, float Kp, float Ki, float Kd, PIDPON_TypeDef POn);

/* ::::::::: PID Direction ::::::::: */
void          PID_SetControllerDirection(PID_TypeDef *uPID, PIDCD_TypeDef Direction);
PIDCD_TypeDef PID_GetDirection(PID_TypeDef *uPID);

/* ::::::::: PID Sampling :::::::::: */
void PID_SetSampleTime(PID_TypeDef *uPID, int32_t NewSampleTime);

/* ::::::: Get Tunings Param ::::::: */
float PID_GetKp(PID_TypeDef *uPID);
float PID_GetKi(PID_TypeDef *uPID);
float PID_GetKd(PID_TypeDef *uPID);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#endif /* __PID_H_ */

