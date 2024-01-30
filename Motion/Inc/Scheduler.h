#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "main.h"

typedef struct
{
    void(*task_func)(void);      //任务函数
    uint16_t rate_hz;            //任务频率
    uint16_t interval_ticks;     //任务周期
    uint32_t last_run;           //当前任务上一次运行的时间
}sched_task_t;

void Scheduler_Setup(void);
void Scheduler_Run(void);

#endif

