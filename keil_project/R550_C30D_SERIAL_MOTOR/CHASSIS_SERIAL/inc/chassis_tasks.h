#ifndef CHASSIS_TASKS_H
#define CHASSIS_TASKS_H

#include "system.h"

#define CHASSIS_CONTROL_TASK_PRIO  6
#define CHASSIS_CONTROL_STACK_SIZE 512
#define CHASSIS_SERIAL_TASK_PRIO      5
#define CHASSIS_SERIAL_STACK_SIZE     512

/* 在 start_task 中调用一次，创建控制任务和串口通信任务。 */
int chassis_tasks_start(void);

void chassis_control_task(void *argument);
void chassis_serial_task(void *argument);

#endif
