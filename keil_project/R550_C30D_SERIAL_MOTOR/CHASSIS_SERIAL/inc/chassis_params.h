#ifndef CHASSIS_PARAMS_H
#define CHASSIS_PARAMS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_PARAMETER_MAGIC   0x43483330UL /* ASCII: CH30 */
#define CHASSIS_PARAMETER_VERSION 1U
#define CHASSIS_PWM_LIMIT_DEFAULT 16700

typedef enum
{
    CHASSIS_TYPE_DIFFERENTIAL = 0,
    CHASSIS_TYPE_ACKERMANN = 1
} chassis_type_t;

/* 单个车轮速度控制器使用的参数。 */
typedef struct
{
    float kp;                 /* 比例系数，单位约为 PWM/(m/s)。 */
    float ki;                 /* 积分系数，单位约为 PWM/(m/s*s)。 */
    float kff;                /* 速度前馈系数，单位约为 PWM/(m/s)。 */
    float dead_forward;       /* 克服正转静摩擦所需的基础 PWM。 */
    float dead_reverse;       /* 克服反转静摩擦所需的基础 PWM。 */
    int16_t output_limit;     /* 对 C30D 原程序应不大于 16700。 */
} wheel_control_params_t;

/*
 * 该结构中的数值都使用明确的 SI 单位。
 * 浮点数仅在 100 Hz 任务中运算，STM32F407 带单精度 FPU，可以胜任。
 */
typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;

    wheel_control_params_t left;
    wheel_control_params_t right;

    float wheel_diameter_m;
    float wheel_track_m;
    float wheelbase_m;
    float counts_per_wheel_rev;

    float maximum_linear_speed_mps;
    float maximum_angular_speed_radps;
    float acceleration_mps2;
    float deceleration_mps2;
    float speed_filter_alpha;

    float straight_sync_kp;
    float straight_sync_max_mps;
    float straight_angular_threshold_radps;

    float maximum_steering_angle_rad;
    float servo_center_pwm;
    float servo_pwm_per_rad;
    uint16_t servo_min_pwm;
    uint16_t servo_max_pwm;

    uint16_t command_timeout_ms;
    uint16_t low_battery_mv;
    float physical_speed_limit_mps;

    float stall_target_speed_mps;
    float stall_measured_speed_mps;
    float stall_pwm_ratio;
    uint16_t stall_time_ms;

    uint8_t chassis_type;
    uint8_t reserved0;
    uint16_t reserved1;

    uint32_t crc32;
} chassis_params_t;

void chassis_params_set_defaults(chassis_params_t *params);
int chassis_params_validate(const chassis_params_t *params);
uint32_t chassis_params_calculate_crc(const chassis_params_t *params);
void chassis_params_update_crc(chassis_params_t *params);
int chassis_params_crc_is_valid(const chassis_params_t *params);

/* 通过 CAN 读取或修改单个参数。返回 1 表示成功。 */
int chassis_params_get_value(const chassis_params_t *params,
                             uint8_t parameter_id,
                             float *value);
int chassis_params_set_value(chassis_params_t *params,
                             uint8_t parameter_id,
                             float value);

#ifdef __cplusplus
}
#endif

#endif

