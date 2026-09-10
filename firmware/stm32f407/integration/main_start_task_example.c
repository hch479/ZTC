/*
 * 这不是第二个 main.c，而是给 USER/main.c 对照修改的最小示例。
 * 详细步骤见 docs/03_STM32接入说明.md。
 */

#include "system.h"
#include "chassis_tasks.h"

void start_task_chassis_example(void *pvParameters)
{
    (void)pvParameters;
    taskENTER_CRITICAL();

    /*
     * 关键点：不要再创建原 Balance_task，因为它也会读取编码器并写 PWM。
     * 同一个硬件只能有一个最终电机输出任务。
     */
    (void)chassis_tasks_start();

    /* IMU、OLED、LED 等不写电机 PWM 的原任务可以继续保留。 */
    if (SysVal.HardWare_Ver == V1_0)
    {
        xTaskCreate(MPU6050_task, "IMU_task", IMU_STK_SIZE,
                    NULL, IMU_TASK_PRIO, NULL);
    }
    else if (SysVal.HardWare_Ver == V1_1)
    {
        xTaskCreate(ICM20948_task, "IMU_task", IMU_STK_SIZE,
                    NULL, IMU_TASK_PRIO, NULL);
    }
    xTaskCreate(show_task, "show_task", SHOW_STK_SIZE,
                NULL, SHOW_TASK_PRIO, NULL);
    xTaskCreate(led_task, "led_task", LED_STK_SIZE,
                NULL, LED_TASK_PRIO, NULL);

    taskEXIT_CRITICAL();
    vTaskDelete(NULL);
}
