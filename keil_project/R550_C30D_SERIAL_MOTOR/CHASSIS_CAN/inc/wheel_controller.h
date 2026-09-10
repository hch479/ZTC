#ifndef WHEEL_CONTROLLER_H
#define WHEEL_CONTROLLER_H

#include <stdint.h>

#include "chassis_params.h"

typedef struct
{
    float ramped_target_mps;
    float filtered_speed_mps;
    float integral_output;
    float last_error_mps;
    int16_t output_pwm;
} wheel_controller_t;

void wheel_controller_reset(wheel_controller_t *controller);

/*
 * 更新一个车轮的速度闭环。
 * target_mps：上层计算出的车轮目标速度。
 * measured_mps：根据本周期编码器增量得到的原始速度。
 * dt_s：控制周期，正常为 0.01 s。
 * allow_output：0 时内部状态复位并返回 0 PWM。
 */
int16_t wheel_controller_update(wheel_controller_t *controller,
                                const wheel_control_params_t *control_params,
                                const chassis_params_t *chassis_params,
                                float target_mps,
                                float measured_mps,
                                float dt_s,
                                uint8_t allow_output);

#endif

