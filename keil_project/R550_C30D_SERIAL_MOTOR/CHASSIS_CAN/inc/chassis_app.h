#ifndef CHASSIS_APP_H
#define CHASSIS_APP_H

#include <stdint.h>

#include "chassis_can_protocol.h"
#include "chassis_params.h"
#include "wheel_controller.h"

typedef enum
{
    CHASSIS_ACTION_NONE = 0,
    CHASSIS_ACTION_SAVE_PARAMETERS = 1
} chassis_action_t;

typedef struct
{
    chassis_params_t params;
    wheel_controller_t left_controller;
    wheel_controller_t right_controller;
    chassis_motion_command_t command;

    uint32_t last_command_ms;
    uint32_t left_stall_ms;
    uint32_t right_stall_ms;
    uint32_t control_overrun_count;

    int32_t left_total_count;
    int32_t right_total_count;
    int16_t left_delta_count;
    int16_t right_delta_count;
    float left_distance_m;
    float right_distance_m;

    float x_m;
    float y_m;
    float yaw_rad;
    float linear_speed_mps;
    float angular_speed_radps;

    float left_target_mps;
    float right_target_mps;
    float left_raw_speed_mps;
    float right_raw_speed_mps;
    float straight_left_start_m;
    float straight_right_start_m;

    uint16_t battery_mv;
    uint16_t latched_faults;
    uint16_t dynamic_faults;
    uint16_t servo_pwm;
    int16_t left_pwm;
    int16_t right_pwm;

    uint8_t state;
    uint8_t has_received_command;
    uint8_t straight_sync_active;
    uint8_t left_encoder_spike_count;
    uint8_t right_encoder_spike_count;
    uint8_t telemetry_sequence;
} chassis_app_t;

/* 如果传入参数无效，函数会采用默认参数并置位 BAD_PARAMETER。 */
void chassis_app_init(chassis_app_t *app, const chassis_params_t *stored_params);

/*
 * 处理一帧来自 RDK X5 的 CAN 数据。
 * reply_ready 为 1 时，调用者应发送 reply。
 * 返回值用于通知接口层执行保存 Flash 等与具体硬件相关的动作。
 */
chassis_action_t chassis_app_process_can_frame(chassis_app_t *app,
                                               const chassis_can_frame_t *frame,
                                               uint32_t now_ms,
                                               chassis_can_frame_t *reply,
                                               uint8_t *reply_ready);

/* 100 Hz 控制任务每周期调用一次。编码器增量必须已换算为“向前为正”。 */
void chassis_app_control_step(chassis_app_t *app,
                              uint32_t now_ms,
                              float dt_s,
                              int16_t left_encoder_delta,
                              int16_t right_encoder_delta,
                              uint16_t battery_mv,
                              uint8_t hardware_enable);

void chassis_app_clear_faults(chassis_app_t *app);
void chassis_app_reset_odometry(chassis_app_t *app);

/* 以下函数生成上报帧，不访问硬件，可由 CAN 任务直接使用。 */
void chassis_app_make_wheel_speed_frame(const chassis_app_t *app,
                                        chassis_can_frame_t *frame);
void chassis_app_make_motor_output_frame(const chassis_app_t *app,
                                         chassis_can_frame_t *frame);
void chassis_app_make_encoder_frames(chassis_app_t *app,
                                     chassis_can_frame_t *left_frame,
                                     chassis_can_frame_t *right_frame);
void chassis_app_make_status_frame(const chassis_app_t *app,
                                   chassis_can_frame_t *frame);
void chassis_app_make_heartbeat_frame(const chassis_app_t *app,
                                      uint32_t now_ms,
                                      chassis_can_frame_t *frame);

#endif
