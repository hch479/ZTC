#include "chassis_params.h"

#include "chassis_can_protocol.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static int in_range(float value, float minimum, float maximum)
{
    return (value == value) && (value >= minimum) && (value <= maximum);
}

void chassis_params_set_defaults(chassis_params_t *params)
{
    if (params == 0)
    {
        return;
    }

    memset(params, 0, sizeof(*params));
    params->magic = CHASSIS_PARAMETER_MAGIC;
    params->version = CHASSIS_PARAMETER_VERSION;
    params->size = (uint16_t)sizeof(*params);

    /*
     * 默认值以“车轮离地可安全验证”为目标，不代表最终调好的参数。
     * kff 给出大部分基础输出，PI 只修正负载和左右差异，比较容易理解和调试。
     */
    params->left.kp = 3000.0f;
    params->left.ki = 1500.0f;
    params->left.kff = 9000.0f;
    params->left.dead_forward = 700.0f;
    params->left.dead_reverse = 700.0f;
    params->left.output_limit = CHASSIS_PWM_LIMIT_DEFAULT;
    params->right = params->left;

    /* 下面三项必须按实际购买的小车测量或查铭牌后再确认。 */
    /* R550 Mini 原资料：黑色轮 65 mm、轮距 162 mm、轴距 144 mm。 */
    params->wheel_diameter_m = 0.065f;
    params->wheel_track_m = 0.162f;
    params->wheelbase_m = 0.144f;
    params->counts_per_wheel_rev = 60000.0f; /* 500 线 * 4 倍频 * 30 减速比。 */

    params->maximum_linear_speed_mps = 0.60f;
    params->maximum_angular_speed_radps = 1.50f;
    params->acceleration_mps2 = 0.50f;
    params->deceleration_mps2 = 0.80f;
    params->speed_filter_alpha = 0.35f;

    params->straight_sync_kp = 0.40f;
    params->straight_sync_max_mps = 0.08f;
    params->straight_angular_threshold_radps = 0.03f;

    params->maximum_steering_angle_rad = 0.52f;
    params->servo_center_pwm = 1500.0f;
    params->servo_pwm_per_rad = 636.56f;
    params->servo_min_pwm = 1000U;
    params->servo_max_pwm = 2000U;

    params->command_timeout_ms = 200U;
    params->low_battery_mv = 9800U;
    params->physical_speed_limit_mps = 2.00f;

    params->stall_target_speed_mps = 0.15f;
    params->stall_measured_speed_mps = 0.02f;
    params->stall_pwm_ratio = 0.80f;
    params->stall_time_ms = 600U;

    /* 当前参考工程是 R550 Mini，默认按阿克曼结构；差速车型改为 0。 */
    params->chassis_type = (uint8_t)CHASSIS_TYPE_ACKERMANN;
    chassis_params_update_crc(params);
}

int chassis_params_validate(const chassis_params_t *params)
{
    if (params == 0)
    {
        return 0;
    }
    if ((params->magic != CHASSIS_PARAMETER_MAGIC) ||
        (params->version != CHASSIS_PARAMETER_VERSION) ||
        (params->size != sizeof(*params)))
    {
        return 0;
    }
    if (!in_range(params->left.kp, 0.0f, 50000.0f) ||
        !in_range(params->left.ki, 0.0f, 50000.0f) ||
        !in_range(params->left.kff, 0.0f, 50000.0f) ||
        !in_range(params->right.kp, 0.0f, 50000.0f) ||
        !in_range(params->right.ki, 0.0f, 50000.0f) ||
        !in_range(params->right.kff, 0.0f, 50000.0f))
    {
        return 0;
    }
    if (!in_range(params->left.dead_forward, 0.0f, 10000.0f) ||
        !in_range(params->left.dead_reverse, 0.0f, 10000.0f) ||
        !in_range(params->right.dead_forward, 0.0f, 10000.0f) ||
        !in_range(params->right.dead_reverse, 0.0f, 10000.0f))
    {
        return 0;
    }
    if ((params->left.output_limit <= 0) ||
        (params->left.output_limit > CHASSIS_PWM_LIMIT_DEFAULT) ||
        (params->right.output_limit <= 0) ||
        (params->right.output_limit > CHASSIS_PWM_LIMIT_DEFAULT))
    {
        return 0;
    }
    if (!in_range(params->wheel_diameter_m, 0.02f, 0.50f) ||
        !in_range(params->wheel_track_m, 0.05f, 2.00f) ||
        !in_range(params->wheelbase_m, 0.05f, 2.00f) ||
        !in_range(params->counts_per_wheel_rev, 1.0f, 1000000.0f))
    {
        return 0;
    }
    if (!in_range(params->maximum_linear_speed_mps, 0.02f, 5.0f) ||
        !in_range(params->maximum_angular_speed_radps, 0.05f, 10.0f) ||
        !in_range(params->acceleration_mps2, 0.02f, 10.0f) ||
        !in_range(params->deceleration_mps2, 0.02f, 20.0f) ||
        !in_range(params->speed_filter_alpha, 0.01f, 1.0f))
    {
        return 0;
    }
    if (!in_range(params->straight_sync_kp, 0.0f, 10.0f) ||
        !in_range(params->straight_sync_max_mps, 0.0f, 1.0f) ||
        !in_range(params->straight_angular_threshold_radps, 0.0f, 1.0f))
    {
        return 0;
    }
    if (!in_range(params->maximum_steering_angle_rad, 0.05f, 1.40f) ||
        !in_range(params->servo_center_pwm, 500.0f, 2500.0f) ||
        !in_range(fabsf(params->servo_pwm_per_rad), 50.0f, 3000.0f) ||
        (params->servo_min_pwm >= params->servo_max_pwm))
    {
        return 0;
    }
    if ((params->command_timeout_ms < 50U) || (params->command_timeout_ms > 5000U) ||
        (params->low_battery_mv < 6000U) || (params->low_battery_mv > 30000U) ||
        !in_range(params->physical_speed_limit_mps, 0.10f, 10.0f))
    {
        return 0;
    }
    if (!in_range(params->stall_target_speed_mps, 0.01f, 2.0f) ||
        !in_range(params->stall_measured_speed_mps, 0.0f, 1.0f) ||
        !in_range(params->stall_pwm_ratio, 0.10f, 1.0f) ||
        (params->stall_time_ms < 100U) || (params->stall_time_ms > 10000U))
    {
        return 0;
    }
    if ((params->chassis_type != (uint8_t)CHASSIS_TYPE_DIFFERENTIAL) &&
        (params->chassis_type != (uint8_t)CHASSIS_TYPE_ACKERMANN))
    {
        return 0;
    }
    return 1;
}

uint32_t chassis_params_calculate_crc(const chassis_params_t *params)
{
    const uint8_t *bytes = (const uint8_t *)params;
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    uint8_t bit;
    const uint32_t length = (uint32_t)offsetof(chassis_params_t, crc32);

    if (params == 0)
    {
        return 0U;
    }

    /* 标准反射形式 CRC-32，多项式 0xEDB88320。 */
    for (i = 0U; i < length; ++i)
    {
        crc ^= bytes[i];
        for (bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 1U) != 0U)
            {
                crc = (crc >> 1U) ^ 0xEDB88320UL;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}

void chassis_params_update_crc(chassis_params_t *params)
{
    if (params != 0)
    {
        params->crc32 = chassis_params_calculate_crc(params);
    }
}

int chassis_params_crc_is_valid(const chassis_params_t *params)
{
    return (params != 0) && (params->crc32 == chassis_params_calculate_crc(params));
}

int chassis_params_get_value(const chassis_params_t *params,
                             uint8_t parameter_id,
                             float *value)
{
    if ((params == 0) || (value == 0))
    {
        return 0;
    }

    switch (parameter_id)
    {
        case CHASSIS_PARAM_LEFT_KP: *value = params->left.kp; break;
        case CHASSIS_PARAM_LEFT_KI: *value = params->left.ki; break;
        case CHASSIS_PARAM_LEFT_KFF: *value = params->left.kff; break;
        case CHASSIS_PARAM_RIGHT_KP: *value = params->right.kp; break;
        case CHASSIS_PARAM_RIGHT_KI: *value = params->right.ki; break;
        case CHASSIS_PARAM_RIGHT_KFF: *value = params->right.kff; break;
        case CHASSIS_PARAM_LEFT_DEAD_FORWARD: *value = params->left.dead_forward; break;
        case CHASSIS_PARAM_LEFT_DEAD_REVERSE: *value = params->left.dead_reverse; break;
        case CHASSIS_PARAM_RIGHT_DEAD_FORWARD: *value = params->right.dead_forward; break;
        case CHASSIS_PARAM_RIGHT_DEAD_REVERSE: *value = params->right.dead_reverse; break;
        case CHASSIS_PARAM_ACCELERATION: *value = params->acceleration_mps2; break;
        case CHASSIS_PARAM_DECELERATION: *value = params->deceleration_mps2; break;
        case CHASSIS_PARAM_FILTER_ALPHA: *value = params->speed_filter_alpha; break;
        case CHASSIS_PARAM_SYNC_KP: *value = params->straight_sync_kp; break;
        case CHASSIS_PARAM_SYNC_MAX: *value = params->straight_sync_max_mps; break;
        case CHASSIS_PARAM_WHEEL_TRACK: *value = params->wheel_track_m; break;
        case CHASSIS_PARAM_WHEEL_DIAMETER: *value = params->wheel_diameter_m; break;
        case CHASSIS_PARAM_COUNTS_PER_REV: *value = params->counts_per_wheel_rev; break;
        case CHASSIS_PARAM_MAX_LINEAR_SPEED: *value = params->maximum_linear_speed_mps; break;
        case CHASSIS_PARAM_LOW_BATTERY_MV: *value = (float)params->low_battery_mv; break;
        case CHASSIS_PARAM_COMMAND_TIMEOUT_MS: *value = (float)params->command_timeout_ms; break;
        case CHASSIS_PARAM_STALL_TARGET_SPEED: *value = params->stall_target_speed_mps; break;
        case CHASSIS_PARAM_STALL_MEASURED_SPEED: *value = params->stall_measured_speed_mps; break;
        case CHASSIS_PARAM_STALL_PWM_RATIO: *value = params->stall_pwm_ratio; break;
        case CHASSIS_PARAM_STALL_TIME_MS: *value = (float)params->stall_time_ms; break;
        case CHASSIS_PARAM_CHASSIS_TYPE: *value = (float)params->chassis_type; break;
        case CHASSIS_PARAM_WHEELBASE: *value = params->wheelbase_m; break;
        case CHASSIS_PARAM_MAX_STEERING_ANGLE: *value = params->maximum_steering_angle_rad; break;
        case CHASSIS_PARAM_SERVO_CENTER_PWM: *value = params->servo_center_pwm; break;
        case CHASSIS_PARAM_SERVO_PWM_PER_RAD: *value = params->servo_pwm_per_rad; break;
        default: return 0;
    }
    return 1;
}

int chassis_params_set_value(chassis_params_t *params,
                             uint8_t parameter_id,
                             float value)
{
    chassis_params_t candidate;

    if ((params == 0) || (value != value))
    {
        return 0;
    }

    candidate = *params;
    switch (parameter_id)
    {
        case CHASSIS_PARAM_LEFT_KP: candidate.left.kp = value; break;
        case CHASSIS_PARAM_LEFT_KI: candidate.left.ki = value; break;
        case CHASSIS_PARAM_LEFT_KFF: candidate.left.kff = value; break;
        case CHASSIS_PARAM_RIGHT_KP: candidate.right.kp = value; break;
        case CHASSIS_PARAM_RIGHT_KI: candidate.right.ki = value; break;
        case CHASSIS_PARAM_RIGHT_KFF: candidate.right.kff = value; break;
        case CHASSIS_PARAM_LEFT_DEAD_FORWARD: candidate.left.dead_forward = value; break;
        case CHASSIS_PARAM_LEFT_DEAD_REVERSE: candidate.left.dead_reverse = value; break;
        case CHASSIS_PARAM_RIGHT_DEAD_FORWARD: candidate.right.dead_forward = value; break;
        case CHASSIS_PARAM_RIGHT_DEAD_REVERSE: candidate.right.dead_reverse = value; break;
        case CHASSIS_PARAM_ACCELERATION: candidate.acceleration_mps2 = value; break;
        case CHASSIS_PARAM_DECELERATION: candidate.deceleration_mps2 = value; break;
        case CHASSIS_PARAM_FILTER_ALPHA: candidate.speed_filter_alpha = value; break;
        case CHASSIS_PARAM_SYNC_KP: candidate.straight_sync_kp = value; break;
        case CHASSIS_PARAM_SYNC_MAX: candidate.straight_sync_max_mps = value; break;
        case CHASSIS_PARAM_WHEEL_TRACK: candidate.wheel_track_m = value; break;
        case CHASSIS_PARAM_WHEEL_DIAMETER: candidate.wheel_diameter_m = value; break;
        case CHASSIS_PARAM_COUNTS_PER_REV: candidate.counts_per_wheel_rev = value; break;
        case CHASSIS_PARAM_MAX_LINEAR_SPEED: candidate.maximum_linear_speed_mps = value; break;
        case CHASSIS_PARAM_LOW_BATTERY_MV:
            if ((value < 0.0f) || (value > 65535.0f)) return 0;
            candidate.low_battery_mv = (uint16_t)(value + 0.5f);
            break;
        case CHASSIS_PARAM_COMMAND_TIMEOUT_MS:
            if ((value < 0.0f) || (value > 65535.0f)) return 0;
            candidate.command_timeout_ms = (uint16_t)(value + 0.5f);
            break;
        case CHASSIS_PARAM_STALL_TARGET_SPEED: candidate.stall_target_speed_mps = value; break;
        case CHASSIS_PARAM_STALL_MEASURED_SPEED: candidate.stall_measured_speed_mps = value; break;
        case CHASSIS_PARAM_STALL_PWM_RATIO: candidate.stall_pwm_ratio = value; break;
        case CHASSIS_PARAM_STALL_TIME_MS:
            if ((value < 0.0f) || (value > 65535.0f)) return 0;
            candidate.stall_time_ms = (uint16_t)(value + 0.5f);
            break;
        case CHASSIS_PARAM_CHASSIS_TYPE:
            if ((value != 0.0f) && (value != 1.0f)) return 0;
            candidate.chassis_type = (uint8_t)value;
            break;
        case CHASSIS_PARAM_WHEELBASE: candidate.wheelbase_m = value; break;
        case CHASSIS_PARAM_MAX_STEERING_ANGLE: candidate.maximum_steering_angle_rad = value; break;
        case CHASSIS_PARAM_SERVO_CENTER_PWM: candidate.servo_center_pwm = value; break;
        case CHASSIS_PARAM_SERVO_PWM_PER_RAD: candidate.servo_pwm_per_rad = value; break;
        default: return 0;
    }

    chassis_params_update_crc(&candidate);
    if (!chassis_params_validate(&candidate))
    {
        return 0;
    }
    *params = candidate;
    return 1;
}
