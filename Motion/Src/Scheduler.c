#include "Scheduler.h"
#include "tim.h"
#include "filter.h"
#include "show.h"
#include "mpu9250.h"
#include "usart.h"
#include "DT.h"
#include "control.h"
#include "pid.h"

extern int16_t ROBOT_ROL, ROBOT_PIT, ROBOT_YAW;
static void Loop_1000Hz(void)	//1msִ��һ��
{
	
}

static void Loop_500Hz(void)	//2msִ��һ��
{	
	// ESP_Control();
	Robot_Control(0, 0);
	// Cmd_Test(); 
}

static void Loop_200Hz(void)	//5msִ��һ��
{
#ifdef USE_MPU9250
	AHRS_Get_Angle();         //mpu9250
#elif USE_MPU6050
	Filter_Get_Angle();   //4.7ms
#endif

}

static void Loop_100Hz(void)	//10msִ��һ��
{
	// Robot_Control();         
#ifdef PID_TEST
	Tempin_Update();
#endif 

}

static void Loop_50Hz(void)	//20msִ��һ��
{	
	// DT_Send03(ROBOT_ROL, ROBOT_PIT, ROBOT_YAW, 1);
	// DT_SendF1(ROBOT_PIT, 0, 0, 0, 0, 0);
}

static void Loop_20Hz(void)	//50msִ��һ��
{
	// Send_To_Comptuer(50);

}

static void Loop_10Hz(void)	//100msִ��һ��
{
}

static void Loop_2Hz(void)	//500msִ��һ��
{
	
	oled_show(12); 
#ifdef PID_TEST
	Tempout_Update();
#endif // DEBUG
}

//ϵͳ�������ã�������ִͬ��Ƶ�ʵġ��̡߳�
static sched_task_t sched_tasks[] = 
{
	{Loop_1000Hz,1000,  0, 0},
	{Loop_500Hz , 500,  0, 0},
	{Loop_200Hz , 200,  0, 0},
	{Loop_100Hz , 100,  0, 0},
	{Loop_50Hz  ,  50,  0, 0},
	{Loop_20Hz  ,  20,  0, 0},
	{Loop_10Hz  ,  10,  0, 0},
	{Loop_2Hz   ,   2,  0, 0},
};
//�������鳤�ȣ��ж��߳�����
#define TASK_NUM (sizeof(sched_tasks)/sizeof(sched_task_t))

void Scheduler_Setup(void)
{
	uint8_t index = 0;
	//��ʼ�������
	for(index=0;index < TASK_NUM;index++)
	{
		//����ÿ���������ʱ����
		sched_tasks[index].interval_ticks = 1000U/sched_tasks[index].rate_hz;
		//�������Ϊ1��Ҳ����1ms
		if(sched_tasks[index].interval_ticks < 1)
		{
			sched_tasks[index].interval_ticks = 1;
		}
	}
}
//��������ŵ�main������while(1)�У���ͣ�ж��Ƿ����߳�Ӧ��ִ��
void Scheduler_Run(void)
{
	//ѭ���ж������̣߳��Ƿ�Ӧ��ִ��
	for(uint8_t index = 0; index < TASK_NUM; index++)
	{
		//��ȡϵͳ��ǰʱ�䣬��λMS
		uint32_t tnow = HAL_GetTick();
		//�����жϣ������ǰʱ���ȥ��һ��ִ�е�ʱ�䣬���ڵ��ڸ��̵߳�ִ�����ڣ���ִ���߳�
		if(tnow - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks)
		{			
			//�����̵߳�ִ��ʱ�䣬������һ���ж�
			sched_tasks[index].last_run = tnow;
			//ִ���̺߳�����ʹ�õ��Ǻ���ָ��
			sched_tasks[index].task_func();
			 			
		}
			
	}
}

	

