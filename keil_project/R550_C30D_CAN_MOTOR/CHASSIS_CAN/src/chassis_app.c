#include "chassis_app.h"

#include <math.h>
#include <string.h>

#define CHASSIS_PI 3.14159265358979323846f
#define CHASSIS_FIRMWARE_MAJOR 1U
#define CHASSIS_FIRMWARE_MINOR 0U

static float clamp_float(float value, float minimum, float maximum)
{
    if (value > maximum)
    {
        return maximum;
    }
    if (value < minimum)
    {
        return minimum;
    }
    return value;
}

static uint16_t clamp_servo_pwm(float value, const chassis_params_t *params)
{
    value = clamp_float(value,
                        (float)params->servo_min_pwm,
                        (float)params->servo_max_pwm);
    return (uint16_t)(value + 0.5f);
}

static float count_to_distance_m(const chassis_params_t *params, int32_t count)
{
    const float perimeter = CHASSIS_PI * params->wheel_diameter_m;
    return ((float)count * perimeter) / params->counts_per_wheel_rev;
}

static float delta_to_speed_mps(const chassis_params_t *params,
                                int16_t delta,
                                float dt_s)
{
    if (dt_s <= 0.0f)
    {
        return 0.0f;
    }
    return count_to_distance_m(params, (int32_t)delta) / dt_s;
}

static float wrap_angle(float angle)
{
    while (angle > CHASSIS_PI)
    {
        angle -= 2.0f * CHASSIS_PI;
    }
    while (angle < -CHASSIS_PI)
    {
        angle += 2.0f * CHASSIS_PI;
    }
    return angle;
}

static void set_reply(chassis_can_frame_t *reply,
                      uint8_t *reply_ready,
                      uint8_t parameter_id,
                      uint8_t status,
                      float value,
                      uint8_t sequence)
{
    if ((reply != 0) && (reply_ready != 0))
    {
        chassis_encode_parameter_reply(parameter_id, status, value, sequence, reply);
        *reply_ready = 1U;
    }
}

void chassis_app_init(chassis_app_t *app, const chassis_params_t *stored_params)
{
    if (app == 0)
    {
        return;
    }

    memset(app, 0, sizeof(*app));
    if ((stored_params != 0) && chassis_params_validate(stored_params) &&
        chassis_params_crc_is_valid(stored_params))
    {
        app->params = *stored_params;
    }
    else
    {
        chassis_params_set_defaults(&app->params);
        if (stored_params != 0)
        {
            app->latched_faults |= CHASSIS_FAULT_BAD_PARAMETER;
        }
    }

    wheel_controller_reset(&app->left_controller);
    wheel_controller_reset(&app->right_controller);
    app->state = (uint8_t)CHASSIS_STATE_IDLE;
    app->servo_pwm = clamp_servo_pwm(app->params.servo_center_pwm, &app->params);
}

void chassis_app_clear_faults(chassis_app_t *app)
{
    if (app == 0)
    {
        return;
    }
    app->latched_faults = 0U;
    app->left_stall_ms = 0U;
    app->right_stall_ms = 0U;
    app->left_encoder_spike_count = 0U;
    app->right_encoder_spike_count = 0U;
    wheel_controller_reset(&app->left_controller);
    wheel_controller_reset(&app->right_controller);
}

void chassis_app_reset_odometry(chassis_app_t *app)
{
    if (app == 0)
    {
        return;
    }
    app->left_total_count = 0;
    app->right_total_count = 0;
    app->left_delta_count = 0;
    app->right_delta_count = 0;
    app->left_distance_m = 0.0f;
    app->right_distance_m = 0.0f;
    app->x_m = 0.0f;
    app->y_m = 0.0f;
    app->yaw_rad = 0.0f;
    app->straight_sync_active = 0U;
}

chassis_action_t chassis_app_process_can_frame(chassis_app_t *app,
                                               const chassis_can_frame_t *frame,
                                               uint32_t now_ms,
                                               chassis_can_frame_t *reply,
                                               uint8_t *reply_ready)
{
    chassis_motion_command_t motion;
    chassis_parameter_command_t parameter;
    chassis_system_command_t system_command;
    float value = 0.0f;

    if (reply_ready != 0)
    {
        *reply_ready = 0U;
    }
    if ((app == 0) || (frame == 0))
    {
        return CHASSIS_ACTION_NONE;
    }

    if (frame->id == CHASSIS_CAN_ID_MOTION_COMMAND)
    {
        if (chassis_decode_motion_command(frame, &motion))
        {
            app->command = motion;
            app->last_command_ms = now_ms;
            app->has_received_command = 1U;
            if (motion.emergency_stop != 0U)
            {
                /* 急停锁存，后续普通速度命令不能自动解除。 */
                app->latched_faults |= CHASSIS_FAULT_ESTOP;
            }
        }
        return CHASSIS_ACTION_NONE;
    }

    if (frame->id == CHASSIS_CAN_ID_PARAMETER_CMD)
    {
        if (!chassis_decode_parameter_command(frame, &parameter))
        {
            return CHASSIS_ACTION_NONE;
        }

        if (parameter.operation == (uint8_t)CHASSIS_PARAM_READ)
        {
            if (chassis_params_get_value(&app->params, parameter.parameter_id, &value))
            {
                set_reply(reply, reply_ready, parameter.parameter_id,
                          CHASSIS_REPLY_OK, value, parameter.sequence);
            }
            else
            {
                set_reply(reply, reply_ready, parameter.parameter_id,
                          CHASSIS_REPLY_BAD_PARAMETER, 0.0f, parameter.sequence);
            }
        }
        else if (parameter.operation == (uint8_t)CHASSIS_PARAM_WRITE)
        {
            /* 运行中不允许改变控制参数，避免一个轮子突然改变输出。 */
            if (app->state == (uint8_t)CHASSIS_STATE_RUNNING)
            {
                set_reply(reply, reply_ready, parameter.parameter_id,
                          CHASSIS_REPLY_DENIED, parameter.value, parameter.sequence);
            }
            else if (chassis_params_set_value(&app->params,
                                              parameter.parameter_id,
                                              parameter.value))
            {
                wheel_controller_reset(&app->left_controller);
                wheel_controller_reset(&app->right_controller);
                chassis_params_get_value(&app->params, parameter.parameter_id, &value);
                set_reply(reply, reply_ready, parameter.parameter_id,
                          CHASSIS_REPLY_OK, value, parameter.sequence);
            }
            else
            {
                set_reply(reply, reply_ready, parameter.parameter_id,
                          CHASSIS_REPLY_BAD_VALUE, parameter.value, parameter.sequence);
            }
        }
        else
        {
            set_reply(reply, reply_ready, parameter.parameter_id,
                      CHASSIS_REPLY_BAD_PARAMETER, 0.0f, parameter.sequence);
        }
        return CHASSIS_ACTION_NONE;
    }

    if (frame->id == CHASSIS_CAN_ID_SYSTEM_COMMAND)
    {
        if (!chassis_decode_system_command(frame, &system_command))
        {
            return CHASSIS_ACTION_NONE;
        }

        if (system_command.opcode == (uint8_t)CHASSIS_SYSTEM_CLEAR_FAULT)
        {
            /* 必须先下发“禁止运行且非急停”的速度命令，再允许清故障。 */
            if ((app->command.enable == 0U) && (app->command.emergency_stop == 0U))
            {
                chassis_app_clear_faults(app);
                set_reply(reply, reply_ready, 0xFEU, CHASSIS_REPLY_OK,
                          (float)system_command.opcode, system_command.sequence);
            }
            else
            {
                set_reply(reply, reply_ready, 0xFEU, CHASSIS_REPLY_DENIED,
                          (float)system_command.opcode, system_command.sequence);
            }
        }
        else if (system_command.opcode == (uint8_t)CHASSIS_SYSTEM_RESET_ODOMETRY)
        {
            if (app->state != (uint8_t)CHASSIS_STATE_RUNNING)
            {
                chassis_app_reset_odometry(app);
                set_reply(reply, reply_ready, 0xFEU, CHASSIS_REPLY_OK,
                          (float)system_command.opcode, system_command.sequence);
            }
            else
            {
                set_reply(reply, reply_ready, 0xFEU, CHASSIS_REPLY_DENIED,
                          (float)system_command.opcode, system_command.sequence);
            }
        }
        else if (system_command.opcode == (uint8_t)CHASSIS_SYSTEM_SAVE_PARAMETERS)
        {
            if (app->state != (uint8_t)CHASSIS_STATE_RUNNING)
            {
                set_reply(reply, reply_ready, 0xFEU, CHASSIS_REPLY_OK,
                          (float)system_command.opcode, system_command.sequence);
                return CHASSIS_ACTION_SAVE_PARAMETERS;
            }
            set_reply(reply, reply_ready, 0xFEU, CHASSIS_REPLY_DENIED,
                      (float)system_command.opcode, system_command.sequence);
        }
        else
        {
            set_reply(reply, reply_ready, 0xFEU, CHASSIS_REPLY_BAD_PARAMETER,
                      (float)system_command.opcode, system_command.sequence);
        }
    }
    return CHASSIS_ACTION_NONE;
}

static void update_encoder_fault(uint8_t *counter,
                                 float measured_speed,
                                 float physical_limit,
                                 uint16_t fault_bit,
                                 uint16_t *latched_faults)
{
    if (fabsf(measured_speed) > physical_limit)
    {
        if (*counter < 255U)
        {
            ++(*counter);
        }
    }
    else
    {
        *counter = 0U;
    }

    /* 连续三次异常才锁存，避免一个受干扰的计数就让整车停机。 */
    if (*counter >= 3U)
    {
        *latched_faults |= fault_bit;
    }
}

static void update_stall_fault(uint32_t *elapsed_ms,
                               float target_speed,
                               float measured_speed,
                               int16_t pwm,
                               int16_t pwm_limit,
                               uint32_t step_ms,
                               const chassis_params_t *params,
                               uint16_t fault_bit,
                               uint16_t *latched_faults)
{
    const float pwm_threshold = params->stall_pwm_ratio * (float)pwm_limit;

    if ((fabsf(target_speed) >= params->stall_target_speed_mps) &&
        ((fabsf(measured_speed) <= params->stall_measured_speed_mps) ||
         ((target_speed * measured_speed) < -0.002f)) &&
        (fabsf((float)pwm) >= pwm_threshold))
    {
        if (*elapsed_ms < 60000U)
        {
            *elapsed_ms += step_ms;
        }
    }
    else
    {
        *elapsed_ms = 0U;
    }

    if (*elapsed_ms >= params->stall_time_ms)
    {
        *latched_faults |= fault_bit;
    }
}

void chassis_app_control_step(chassis_app_t *app,
                              uint32_t now_ms,
                              float dt_s,
                              int16_t left_encoder_delta,
                              int16_t right_encoder_delta,
                              uint16_t battery_mv,
                              uint8_t hardware_enable)
{
    float linear_command;
    float angular_command;
    float left_target = 0.0f;
    float right_target = 0.0f;
    float left_distance;
    float right_distance;
    float sync_error;
    float sync_correction;
    float maximum_wheel_speed;
    float scale;
    float steering_angle = 0.0f;
    float delta_yaw;
    float average_yaw;
    float left_speed_for_control;
    float right_speed_for_control;
    uint32_t elapsed_since_command;
    uint32_t step_ms;
    uint8_t output_allowed = 0U;
    uint16_t severe_faults;

    if (app == 0)
    {
        return;
    }

    /* 控制周期明显异常时记录故障；过大的 dt 不直接用于积分和里程计。 */
    if ((dt_s < 0.005f) || (dt_s > 0.020f) || (dt_s != dt_s))
    {
        app->control_overrun_count++;
        app->dynamic_faults |= CHASSIS_FAULT_CONTROL_OVERRUN;
        dt_s = 0.010f;
    }
    else
    {
        app->dynamic_faults &= (uint16_t)~CHASSIS_FAULT_CONTROL_OVERRUN;
    }
    step_ms = (uint32_t)(dt_s * 1000.0f + 0.5f);

    app->left_raw_speed_mps = delta_to_speed_mps(&app->params,
                                                 left_encoder_delta, dt_s);
    app->right_raw_speed_mps = delta_to_speed_mps(&app->params,
                                                  right_encoder_delta, dt_s);
    app->battery_mv = battery_mv;

    /*
     * 单个异常计数不进入里程计和 PI。既能避免里程计突然跳变，也能避免
     * PI 因为一次强干扰立即打出反向大 PWM。连续异常仍由下面的计数器锁存。
     */
    if (fabsf(app->left_raw_speed_mps) <= app->params.physical_speed_limit_mps)
    {
        app->left_delta_count = left_encoder_delta;
        /* 明确采用 32 位环绕，避免有符号累计计数溢出的未定义行为。 */
        app->left_total_count = (int32_t)((uint32_t)app->left_total_count +
                                         (uint32_t)(int32_t)left_encoder_delta);
        app->left_distance_m += count_to_distance_m(&app->params,
                                                    (int32_t)left_encoder_delta);
        left_speed_for_control = app->left_raw_speed_mps;
    }
    else
    {
        app->left_delta_count = 0;
        left_speed_for_control = app->left_controller.filtered_speed_mps;
    }
    if (fabsf(app->right_raw_speed_mps) <= app->params.physical_speed_limit_mps)
    {
        app->right_delta_count = right_encoder_delta;
        app->right_total_count = (int32_t)((uint32_t)app->right_total_count +
                                          (uint32_t)(int32_t)right_encoder_delta);
        app->right_distance_m += count_to_distance_m(&app->params,
                                                     (int32_t)right_encoder_delta);
        right_speed_for_control = app->right_raw_speed_mps;
    }
    else
    {
        app->right_delta_count = 0;
        right_speed_for_control = app->right_controller.filtered_speed_mps;
    }

    update_encoder_fault(&app->left_encoder_spike_count,
                         app->left_raw_speed_mps,
                         app->params.physical_speed_limit_mps,
                         CHASSIS_FAULT_LEFT_ENCODER,
                         &app->latched_faults);
    update_encoder_fault(&app->right_encoder_spike_count,
                         app->right_raw_speed_mps,
                         app->params.physical_speed_limit_mps,
                         CHASSIS_FAULT_RIGHT_ENCODER,
                         &app->latched_faults);

    /* 先重新计算每周期变化的故障位，锁存故障保存在 latched_faults 中。 */
    app->dynamic_faults &= CHASSIS_FAULT_CONTROL_OVERRUN;
    if (hardware_enable == 0U)
    {
        app->dynamic_faults |= CHASSIS_FAULT_HARDWARE_DISABLED;
    }
    if ((battery_mv != 0U) && (battery_mv < app->params.low_battery_mv))
    {
        app->latched_faults |= CHASSIS_FAULT_LOW_BATTERY;
    }

    elapsed_since_command = now_ms - app->last_command_ms;
    if ((app->has_received_command != 0U) &&
        (elapsed_since_command > app->params.command_timeout_ms))
    {
        app->dynamic_faults |= CHASSIS_FAULT_COMMAND_TIMEOUT;
    }

    severe_faults = app->latched_faults &
                    (CHASSIS_FAULT_LOW_BATTERY |
                     CHASSIS_FAULT_LEFT_ENCODER |
                     CHASSIS_FAULT_RIGHT_ENCODER |
                     CHASSIS_FAULT_LEFT_STALL |
                     CHASSIS_FAULT_RIGHT_STALL |
                     CHASSIS_FAULT_BAD_PARAMETER);

    if ((app->latched_faults & CHASSIS_FAULT_ESTOP) != 0U)
    {
        app->state = (uint8_t)CHASSIS_STATE_ESTOP;
    }
    else if (severe_faults != 0U)
    {
        app->state = (uint8_t)CHASSIS_STATE_FAULT;
    }
    else if ((app->dynamic_faults & CHASSIS_FAULT_COMMAND_TIMEOUT) != 0U)
    {
        app->state = (uint8_t)CHASSIS_STATE_TIMEOUT;
    }
    else if ((hardware_enable == 0U) || (app->has_received_command == 0U) ||
             (app->command.enable == 0U))
    {
        app->state = (uint8_t)CHASSIS_STATE_IDLE;
    }
    else
    {
        app->state = (uint8_t)CHASSIS_STATE_RUNNING;
        output_allowed = 1U;
    }

    linear_command = clamp_float(app->command.linear_mps,
                                 -app->params.maximum_linear_speed_mps,
                                 app->params.maximum_linear_speed_mps);
    angular_command = clamp_float(app->command.angular_radps,
                                  -app->params.maximum_angular_speed_radps,
                                  app->params.maximum_angular_speed_radps);

    if (output_allowed != 0U)
    {
        if (app->params.chassis_type == (uint8_t)CHASSIS_TYPE_ACKERMANN)
        {
            /* 阿克曼车不能原地自转，线速度接近零时不执行角速度命令。 */
            if (fabsf(linear_command) < 0.02f)
            {
                angular_command = 0.0f;
                linear_command = 0.0f;
            }
            else
            {
                steering_angle = atanf(app->params.wheelbase_m *
                                       angular_command / linear_command);
                steering_angle = clamp_float(
                    steering_angle,
                    -app->params.maximum_steering_angle_rad,
                    app->params.maximum_steering_angle_rad);

                /* 舵角被限幅后，同步降低角速度目标，使驱动轮速度与真实舵角一致。 */
                angular_command = linear_command * tanf(steering_angle) /
                                  app->params.wheelbase_m;
            }
        }

        /* 两个驱动轮在车体中心速度两侧各补偿半个轮距的转动速度。 */
        left_target = linear_command -
                      angular_command * app->params.wheel_track_m * 0.5f;
        right_target = linear_command +
                       angular_command * app->params.wheel_track_m * 0.5f;

        /* 转弯时某一轮可能超限，等比例缩小可保持曲率不变。 */
        maximum_wheel_speed = fabsf(left_target);
        if (fabsf(right_target) > maximum_wheel_speed)
        {
            maximum_wheel_speed = fabsf(right_target);
        }
        if (maximum_wheel_speed > app->params.maximum_linear_speed_mps)
        {
            scale = app->params.maximum_linear_speed_mps / maximum_wheel_speed;
            left_target *= scale;
            right_target *= scale;
        }

        left_distance = app->left_distance_m;
        right_distance = app->right_distance_m;
        if ((fabsf(angular_command) <= app->params.straight_angular_threshold_radps) &&
            (fabsf(linear_command) >= 0.05f))
        {
            if (app->straight_sync_active == 0U)
            {
                app->straight_left_start_m = left_distance;
                app->straight_right_start_m = right_distance;
                app->straight_sync_active = 1U;
            }
            sync_error = (left_distance - app->straight_left_start_m) -
                         (right_distance - app->straight_right_start_m);
            sync_correction = clamp_float(
                app->params.straight_sync_kp * sync_error,
                -app->params.straight_sync_max_mps,
                app->params.straight_sync_max_mps);
            left_target -= sync_correction;
            right_target += sync_correction;
        }
        else
        {
            app->straight_sync_active = 0U;
        }
    }
    else
    {
        app->straight_sync_active = 0U;
    }

    app->left_target_mps = left_target;
    app->right_target_mps = right_target;
    app->left_pwm = wheel_controller_update(&app->left_controller,
                                             &app->params.left,
                                             &app->params,
                                             left_target,
                                             left_speed_for_control,
                                             dt_s,
                                             output_allowed);
    app->right_pwm = wheel_controller_update(&app->right_controller,
                                              &app->params.right,
                                              &app->params,
                                              right_target,
                                              right_speed_for_control,
                                              dt_s,
                                              output_allowed);

    if (app->params.chassis_type == (uint8_t)CHASSIS_TYPE_ACKERMANN)
    {
        app->servo_pwm = clamp_servo_pwm(app->params.servo_center_pwm +
                                         app->params.servo_pwm_per_rad * steering_angle,
                                         &app->params);
    }
    else
    {
        app->servo_pwm = clamp_servo_pwm(app->params.servo_center_pwm, &app->params);
    }

    if (output_allowed != 0U)
    {
        update_stall_fault(&app->left_stall_ms,
                           app->left_controller.ramped_target_mps,
                           app->left_controller.filtered_speed_mps,
                           app->left_pwm,
                           app->params.left.output_limit,
                           step_ms,
                           &app->params,
                           CHASSIS_FAULT_LEFT_STALL,
                           &app->latched_faults);
        update_stall_fault(&app->right_stall_ms,
                           app->right_controller.ramped_target_mps,
                           app->right_controller.filtered_speed_mps,
                           app->right_pwm,
                           app->params.right.output_limit,
                           step_ms,
                           &app->params,
                           CHASSIS_FAULT_RIGHT_STALL,
                           &app->latched_faults);
    }
    else
    {
        app->left_stall_ms = 0U;
        app->right_stall_ms = 0U;
    }

    /* 堵转在本周期刚被发现时，立即撤销输出，不等下一个周期。 */
    if ((app->latched_faults &
         (CHASSIS_FAULT_LEFT_STALL | CHASSIS_FAULT_RIGHT_STALL)) != 0U)
    {
        app->left_pwm = 0;
        app->right_pwm = 0;
        app->state = (uint8_t)CHASSIS_STATE_FAULT;
        wheel_controller_reset(&app->left_controller);
        wheel_controller_reset(&app->right_controller);
    }

    /* 里程计使用经过低通滤波的实测轮速，而不是目标速度。 */
    app->linear_speed_mps =
        0.5f * (app->left_controller.filtered_speed_mps +
                app->right_controller.filtered_speed_mps);
    app->angular_speed_radps =
        (app->right_controller.filtered_speed_mps -
         app->left_controller.filtered_speed_mps) /
        app->params.wheel_track_m;
    delta_yaw = app->angular_speed_radps * dt_s;
    average_yaw = app->yaw_rad + 0.5f * delta_yaw;
    app->x_m += app->linear_speed_mps * cosf(average_yaw) * dt_s;
    app->y_m += app->linear_speed_mps * sinf(average_yaw) * dt_s;
    app->yaw_rad = wrap_angle(app->yaw_rad + delta_yaw);
}

void chassis_app_make_wheel_speed_frame(const chassis_app_t *app,
                                        chassis_can_frame_t *frame)
{
    chassis_encode_wheel_speed(app->left_controller.ramped_target_mps,
                               app->left_controller.filtered_speed_mps,
                               app->right_controller.ramped_target_mps,
                               app->right_controller.filtered_speed_mps,
                               frame);
}

void chassis_app_make_motor_output_frame(const chassis_app_t *app,
                                         chassis_can_frame_t *frame)
{
    chassis_encode_motor_output(app->left_pwm,
                                app->right_pwm,
                                app->left_controller.last_error_mps,
                                app->right_controller.last_error_mps,
                                frame);
}

void chassis_app_make_encoder_frames(chassis_app_t *app,
                                     chassis_can_frame_t *left_frame,
                                     chassis_can_frame_t *right_frame)
{
    app->telemetry_sequence++;
    chassis_encode_encoder(CHASSIS_CAN_ID_LEFT_ENCODER,
                           app->left_total_count,
                           app->left_delta_count,
                           app->telemetry_sequence,
                           left_frame);
    chassis_encode_encoder(CHASSIS_CAN_ID_RIGHT_ENCODER,
                           app->right_total_count,
                           app->right_delta_count,
                           app->telemetry_sequence,
                           right_frame);
}

void chassis_app_make_status_frame(const chassis_app_t *app,
                                   chassis_can_frame_t *frame)
{
    uint32_t count = app->control_overrun_count;
    if (count > 255U)
    {
        count = 255U;
    }
    chassis_encode_status(app->state,
                          (uint16_t)(app->latched_faults | app->dynamic_faults),
                          app->battery_mv,
                          app->command.sequence,
                          (uint8_t)count,
                          frame);
}

void chassis_app_make_heartbeat_frame(const chassis_app_t *app,
                                      uint32_t now_ms,
                                      chassis_can_frame_t *frame)
{
    chassis_encode_heartbeat(now_ms,
                             app->state,
                             CHASSIS_FIRMWARE_MAJOR,
                             CHASSIS_FIRMWARE_MINOR,
                             frame);
}
