#include "wheel_controller.h"

#include <math.h>
#include <string.h>

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

//当前斜坡目标逐步靠近最终目标
static float approach_target(float current,
                             float target,
                             float acceleration,
                             float deceleration,
                             float dt_s)
{
    float limit;
    float difference = target - current;

    /*
     * 同向且目标绝对值更大时属于加速；减速、换向都先使用更快的减速度。
     * 这样急停前的正常停车不会拖得太长，换向也不会突然反打。
     */
    if (((current >= 0.0f) && (target >= 0.0f) && (target > current)) ||
        ((current <= 0.0f) && (target <= 0.0f) && (target < current)))
    {
        limit = acceleration * dt_s;
    }
    else
    {
        limit = deceleration * dt_s;
    }

    if (difference > limit)
    {
        difference = limit;
    }
    else if (difference < -limit)
    {
        difference = -limit;
    }
    return current + difference;
}

void wheel_controller_reset(wheel_controller_t *controller)
{
    if (controller != 0)
    {
        memset(controller, 0, sizeof(*controller));
    }
}

//计算pwm
int16_t wheel_controller_update(wheel_controller_t *controller,
                                const wheel_control_params_t *control_params,
                                const chassis_params_t *chassis_params,
                                float target_mps,
                                float measured_mps,
                                float dt_s,
                                uint8_t allow_output)
{
    float error;
    float feedforward;
    float proportional;
    float integral_candidate;
    float output_before_limit;
    float output_after_limit;
    float deadzone = 0.0f;

    if ((controller == 0) || (control_params == 0) || (chassis_params == 0) ||
        (dt_s <= 0.0f))
    {
        return 0;
    }

    /* 一阶低通滤波可降低单周期编码器计数跳变导致的 PWM 抖动。 */
    controller->filtered_speed_mps +=
        chassis_params->speed_filter_alpha *
        (measured_mps - controller->filtered_speed_mps);

    if (allow_output == 0U)
    {
        /* 禁止输出时清空历史量，恢复运行后不会带着旧积分突然冲击。 */
        controller->ramped_target_mps = 0.0f;
        controller->integral_output = 0.0f;
        controller->last_error_mps = 0.0f;
        controller->output_pwm = 0;
        return 0;
    }

    target_mps = clamp_float(target_mps,
                             -chassis_params->maximum_linear_speed_mps,
                             chassis_params->maximum_linear_speed_mps);
    controller->ramped_target_mps = approach_target(
        controller->ramped_target_mps,
        target_mps,
        chassis_params->acceleration_mps2,
        chassis_params->deceleration_mps2,
        dt_s);

    /* 目标已经回到零时主动清积分，避免静止状态仍有电流和啸叫。 */
    if ((fabsf(target_mps) < 0.001f) &&
        (fabsf(controller->ramped_target_mps) < 0.002f))
    {
        controller->ramped_target_mps = 0.0f;
        controller->integral_output = 0.0f;
        controller->last_error_mps = -controller->filtered_speed_mps;
        controller->output_pwm = 0;
        return 0;
    }

    if (controller->ramped_target_mps > 0.0f)
    {
        deadzone = control_params->dead_forward;
    }
    else if (controller->ramped_target_mps < 0.0f)
    {
        deadzone = -control_params->dead_reverse;
    }

    error = controller->ramped_target_mps - controller->filtered_speed_mps;
    feedforward = control_params->kff * controller->ramped_target_mps + deadzone;
    proportional = control_params->kp * error;
    integral_candidate = controller->integral_output +
                         control_params->ki * error * dt_s;
    output_before_limit = feedforward + proportional + integral_candidate;
    output_after_limit = clamp_float(output_before_limit,
                                     -(float)control_params->output_limit,
                                     (float)control_params->output_limit);

    /*
     * 条件积分抗饱和：输出顶到上限且误差仍想让它更大时，暂停积分。
     * 当误差方向有助于退出饱和时，仍允许积分更新。
     */
    if ((output_before_limit == output_after_limit) ||
        ((output_before_limit > output_after_limit) && (error < 0.0f)) ||
        ((output_before_limit < output_after_limit) && (error > 0.0f)))
    {
        controller->integral_output = integral_candidate;
    }

    output_before_limit = feedforward + proportional + controller->integral_output;
    output_after_limit = clamp_float(output_before_limit,
                                     -(float)control_params->output_limit,
                                     (float)control_params->output_limit);
    controller->last_error_mps = error;
    controller->output_pwm = (int16_t)((output_after_limit >= 0.0f) ?
                                       (output_after_limit + 0.5f) :
                                       (output_after_limit - 0.5f));
    return controller->output_pwm;
}
