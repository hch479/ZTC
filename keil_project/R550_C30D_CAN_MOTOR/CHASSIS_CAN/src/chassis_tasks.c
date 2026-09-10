#include "chassis_tasks.h"

#include "c30d_chassis_port.h"
#include "chassis_app.h"

#include "semphr.h"

#define CONTROL_PERIOD_MS 10U
#define CAN_POLL_PERIOD_MS 2U
#define FAST_TELEMETRY_PERIOD_MS 20U
#define ENCODER_TELEMETRY_PERIOD_MS 50U
#define STATUS_PERIOD_MS 100U
#define HEARTBEAT_PERIOD_MS 1000U

static chassis_app_t g_chassis_app;
static SemaphoreHandle_t g_chassis_mutex = NULL;

static void send_frame(const chassis_can_frame_t *frame)
{
    /* 当前策略允许丢失个别遥测帧；下一周期会重新发送最新数据。 */
    (void)c30d_port_can_send(frame);
}

int chassis_tasks_start(void)
{
    chassis_params_t stored_params;
    int load_result;
    BaseType_t control_result;
    BaseType_t can_result;

    load_result = c30d_port_load_parameters(&stored_params);
    if (load_result > 0)
    {
        chassis_app_init(&g_chassis_app, &stored_params);
    }
    else if (load_result == 0)
    {
        /* 传 NULL 表示 Flash 为空，首次启动采用安全默认值，不把它当故障。 */
        chassis_app_init(&g_chassis_app, NULL);
    }
    else
    {
        /* Flash 有损坏内容：使用默认值，同时上报 BAD_PARAMETER 锁存故障。 */
        chassis_app_init(&g_chassis_app, &stored_params);
    }

    g_chassis_mutex = xSemaphoreCreateMutex();
    if (g_chassis_mutex == NULL)
    {
        return 0;
    }

    /*
     * 先创建无电机输出的 CAN 任务，最后才创建控制任务。
     * 即使第二次创建因内存不足失败，也不会留下一个可输出 PWM 的孤立任务。
     */
    can_result = xTaskCreate(chassis_can_task,
                             "chassis_can",
                             CHASSIS_CAN_STACK_SIZE,
                             NULL,
                             CHASSIS_CAN_TASK_PRIO,
                             NULL);
    if (can_result != pdPASS)
    {
        return 0;
    }
    control_result = xTaskCreate(chassis_control_task,
                                 "chassis_ctrl",
                                 CHASSIS_CONTROL_STACK_SIZE,
                                 NULL,
                                 CHASSIS_CONTROL_TASK_PRIO,
                                 NULL);
    return (control_result == pdPASS) && (can_result == pdPASS);
}

void chassis_control_task(void *argument)
{
    TickType_t last_wake_time;
    uint32_t previous_ms;
    uint32_t now_ms;
    float dt_s;
    int16_t left_delta;
    int16_t right_delta;
    int16_t left_pwm;
    int16_t right_pwm;
    uint16_t servo_pwm;
    uint8_t chassis_type;

    (void)argument;
    last_wake_time = xTaskGetTickCount();
    previous_ms = c30d_port_get_time_ms();

    for (;;)
    {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
        now_ms = c30d_port_get_time_ms();
        dt_s = (float)(now_ms - previous_ms) / 1000.0f;
        previous_ms = now_ms;

        /* Read_Encoder 读取后会清零计数器，因此只能由本任务调用。 */
        left_delta = c30d_port_read_left_encoder();
        right_delta = c30d_port_read_right_encoder();

        if (xSemaphoreTake(g_chassis_mutex, pdMS_TO_TICKS(2U)) == pdTRUE)
        {
            chassis_app_control_step(&g_chassis_app,
                                     now_ms,
                                     dt_s,
                                     left_delta,
                                     right_delta,
                                     c30d_port_read_battery_mv(),
                                     c30d_port_hardware_is_enabled());
            left_pwm = g_chassis_app.left_pwm;
            right_pwm = g_chassis_app.right_pwm;
            servo_pwm = g_chassis_app.servo_pwm;
            chassis_type = g_chassis_app.params.chassis_type;
            xSemaphoreGive(g_chassis_mutex);

            /* 差速车型没有转向舵机，传零保持与原工程行为一致。 */
            if (chassis_type != (uint8_t)CHASSIS_TYPE_ACKERMANN)
            {
                servo_pwm = 0U;
            }
            c30d_port_set_output(left_pwm, right_pwm, servo_pwm);
        }
        else
        {
            /* 无法及时取得共享数据说明调度异常，安全起见本周期输出零。 */
            c30d_port_set_output(0, 0, 0U);
        }
    }
}

void chassis_can_task(void *argument)
{
    TickType_t last_wake_time;
    uint32_t now_ms;
    uint32_t last_fast_ms = 0U;
    uint32_t last_encoder_ms = 0U;
    uint32_t last_status_ms = 0U;
    uint32_t last_heartbeat_ms = 0U;
    chassis_can_frame_t received;
    chassis_can_frame_t reply;
    chassis_can_frame_t first;
    chassis_can_frame_t second;
    chassis_action_t action;
    uint8_t reply_ready;
    chassis_params_t params_to_save;

    (void)argument;
    last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CAN_POLL_PERIOD_MS));
        now_ms = c30d_port_get_time_ms();

        /* 一次唤醒把 FIFO 中已有报文全部读完，降低突发参数报文丢失概率。 */
        while (c30d_port_can_receive(&received))
        {
            action = CHASSIS_ACTION_NONE;
            reply_ready = 0U;
            if (xSemaphoreTake(g_chassis_mutex, pdMS_TO_TICKS(2U)) == pdTRUE)
            {
                action = chassis_app_process_can_frame(&g_chassis_app,
                                                       &received,
                                                       now_ms,
                                                       &reply,
                                                       &reply_ready);
                if (action == CHASSIS_ACTION_SAVE_PARAMETERS)
                {
                    params_to_save = g_chassis_app.params;
                }
                xSemaphoreGive(g_chassis_mutex);
            }

            if (action == CHASSIS_ACTION_SAVE_PARAMETERS)
            {
                if (!c30d_port_save_parameters(&params_to_save))
                {
                    /* 用相同序号补发失败应答，上位机以最后一帧为准。 */
                    chassis_encode_parameter_reply(0xFEU,
                                                   CHASSIS_REPLY_STORAGE_ERROR,
                                                   (float)CHASSIS_SYSTEM_SAVE_PARAMETERS,
                                                   reply.data[6],
                                                   &reply);
                    reply_ready = 1U;
                }
            }
            if (reply_ready != 0U)
            {
                send_frame(&reply);
            }
        }

        if ((now_ms - last_fast_ms) >= FAST_TELEMETRY_PERIOD_MS)
        {
            if (xSemaphoreTake(g_chassis_mutex, pdMS_TO_TICKS(2U)) == pdTRUE)
            {
                chassis_app_make_wheel_speed_frame(&g_chassis_app, &first);
                chassis_app_make_motor_output_frame(&g_chassis_app, &second);
                xSemaphoreGive(g_chassis_mutex);
                send_frame(&first);
                send_frame(&second);
            }
            last_fast_ms = now_ms;
        }

        if ((now_ms - last_encoder_ms) >= ENCODER_TELEMETRY_PERIOD_MS)
        {
            if (xSemaphoreTake(g_chassis_mutex, pdMS_TO_TICKS(2U)) == pdTRUE)
            {
                chassis_app_make_encoder_frames(&g_chassis_app, &first, &second);
                xSemaphoreGive(g_chassis_mutex);
                send_frame(&first);
                send_frame(&second);
            }
            last_encoder_ms = now_ms;
        }

        if ((now_ms - last_status_ms) >= STATUS_PERIOD_MS)
        {
            if (xSemaphoreTake(g_chassis_mutex, pdMS_TO_TICKS(2U)) == pdTRUE)
            {
                chassis_app_make_status_frame(&g_chassis_app, &first);
                xSemaphoreGive(g_chassis_mutex);
                send_frame(&first);
            }
            last_status_ms = now_ms;
        }

        if ((now_ms - last_heartbeat_ms) >= HEARTBEAT_PERIOD_MS)
        {
            if (xSemaphoreTake(g_chassis_mutex, pdMS_TO_TICKS(2U)) == pdTRUE)
            {
                chassis_app_make_heartbeat_frame(&g_chassis_app, now_ms, &first);
                xSemaphoreGive(g_chassis_mutex);
                send_frame(&first);
            }
            last_heartbeat_ms = now_ms;
        }
    }
}
