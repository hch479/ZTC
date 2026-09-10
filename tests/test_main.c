#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "chassis_app.h"
#include "chassis_can_protocol.h"
#include "chassis_params.h"
#include "wheel_controller.h"

static int float_is_close(float a, float b, float tolerance)
{
    return fabsf(a - b) <= tolerance;
}

static void test_crc_known_vector(void)
{
    /* CRC-8/ATM 对 ASCII "123456789" 的标准校验值是 0xF4。 */
    const uint8_t input[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    assert(chassis_crc8(input, (uint8_t)sizeof(input)) == 0xF4U);
}

static void test_motion_command_round_trip(void)
{
    chassis_motion_command_t source;
    chassis_motion_command_t decoded;
    chassis_can_frame_t frame;

    memset(&source, 0, sizeof(source));
    source.linear_mps = 0.321f;
    source.angular_radps = -0.456f;
    source.enable = 1U;
    source.sequence = 77U;

    assert(chassis_encode_motion_command(&source, &frame));
    assert(frame.id == CHASSIS_CAN_ID_MOTION_COMMAND);
    assert(chassis_decode_motion_command(&frame, &decoded));
    assert(float_is_close(decoded.linear_mps, 0.321f, 0.0011f));
    assert(float_is_close(decoded.angular_radps, -0.456f, 0.0011f));
    assert(decoded.enable == 1U);
    assert(decoded.emergency_stop == 0U);
    assert(decoded.sequence == 77U);

    /* 任意一位被修改后都不应通过应用层 CRC 检查。 */
    frame.data[2] ^= 0x01U;
    assert(!chassis_decode_motion_command(&frame, &decoded));
}

static void test_imu_telemetry_layout(void)
{
    chassis_can_frame_t vector;
    chassis_can_frame_t status;

    chassis_encode_imu_vector(CHASSIS_CAN_ID_IMU_ACCELERATION,
                              -123, 456, 16384, 27U, &vector);
    assert(vector.id == CHASSIS_CAN_ID_IMU_ACCELERATION);
    assert(vector.data[0] == 0x85U && vector.data[1] == 0xFFU);
    assert(vector.data[2] == 0xC8U && vector.data[3] == 0x01U);
    assert(vector.data[4] == 0x00U && vector.data[5] == 0x40U);
    assert(vector.data[6] == 27U);
    assert(chassis_frame_crc_is_valid(&vector));

    chassis_encode_imu_status(1U, 1U, 500U, 500U, 91U, &status);
    assert(status.id == CHASSIS_CAN_ID_IMU_STATUS);
    assert(status.data[0] == 1U);
    assert((status.data[1] & CHASSIS_IMU_CALIBRATED_MASK) != 0U);
    assert(status.data[2] == 0xF4U && status.data[3] == 0x01U);
    assert(status.data[4] == 0xF4U && status.data[5] == 0x01U);
    assert(status.data[6] == 91U);
    assert(chassis_frame_crc_is_valid(&status));
}

static void test_parameters(void)
{
    chassis_params_t params;
    float value = 0.0f;

    chassis_params_set_defaults(&params);
    assert(chassis_params_validate(&params));
    assert(chassis_params_crc_is_valid(&params));
    assert(params.chassis_type == CHASSIS_TYPE_ACKERMANN);
    assert(float_is_close(params.wheel_diameter_m, 0.065f, 0.0001f));

    assert(chassis_params_set_value(&params, CHASSIS_PARAM_LEFT_KP, 4200.0f));
    assert(chassis_params_get_value(&params, CHASSIS_PARAM_LEFT_KP, &value));
    assert(float_is_close(value, 4200.0f, 0.01f));
    assert(chassis_params_crc_is_valid(&params));

    /* 超出验证范围的值必须被拒绝，原值保持不变。 */
    assert(!chassis_params_set_value(&params, CHASSIS_PARAM_FILTER_ALPHA, 1.5f));
    assert(float_is_close(params.speed_filter_alpha, 0.35f, 0.001f));
    assert(!chassis_params_set_value(&params, CHASSIS_PARAM_LOW_BATTERY_MV,
                                     1000000000.0f));
    assert(!chassis_params_set_value(&params, CHASSIS_PARAM_CHASSIS_TYPE, 0.5f));
}

static void test_wheel_controller(void)
{
    chassis_params_t params;
    wheel_controller_t controller;
    int16_t pwm;
    int i;

    chassis_params_set_defaults(&params);
    wheel_controller_reset(&controller);

    /* 连续运行后应产生正 PWM，且永远不超过输出限制。 */
    pwm = 0;
    for (i = 0; i < 100; ++i)
    {
        pwm = wheel_controller_update(&controller,
                                      &params.left,
                                      &params,
                                      0.30f,
                                      0.0f,
                                      0.01f,
                                      1U);
        assert(pwm <= params.left.output_limit);
        assert(pwm >= -params.left.output_limit);
    }
    assert(pwm > 0);

    /* 安全状态禁止输出时必须在当前调用立即返回零并清积分。 */
    pwm = wheel_controller_update(&controller,
                                  &params.left,
                                  &params,
                                  0.30f,
                                  0.0f,
                                  0.01f,
                                  0U);
    assert(pwm == 0);
    assert(controller.integral_output == 0.0f);
}

static void send_motion(chassis_app_t *app,
                        float linear,
                        float angular,
                        uint8_t enable,
                        uint8_t estop,
                        uint8_t sequence,
                        uint32_t now_ms)
{
    chassis_motion_command_t command;
    chassis_can_frame_t frame;
    chassis_can_frame_t reply;
    uint8_t reply_ready = 0U;

    memset(&command, 0, sizeof(command));
    command.linear_mps = linear;
    command.angular_radps = angular;
    command.enable = enable;
    command.emergency_stop = estop;
    command.sequence = sequence;
    assert(chassis_encode_motion_command(&command, &frame));
    (void)chassis_app_process_can_frame(app, &frame, now_ms, &reply, &reply_ready);
    assert(reply_ready == 0U);
}

static void send_system(chassis_app_t *app, uint8_t opcode, uint32_t now_ms)
{
    chassis_system_command_t command;
    chassis_can_frame_t frame;
    chassis_can_frame_t reply;
    uint8_t reply_ready = 0U;

    memset(&command, 0, sizeof(command));
    command.opcode = opcode;
    command.sequence = 9U;
    assert(chassis_encode_system_command(&command, &frame));
    (void)chassis_app_process_can_frame(app, &frame, now_ms, &reply, &reply_ready);
    assert(reply_ready == 1U);
    assert(reply.id == CHASSIS_CAN_ID_PARAMETER_REPLY);
}

static void test_state_machine_and_odometry(void)
{
    chassis_params_t params;
    chassis_app_t app;
    int i;

    chassis_params_set_defaults(&params);
    chassis_app_init(&app, &params);
    assert(app.state == CHASSIS_STATE_IDLE);

    send_motion(&app, 0.20f, 0.0f, 1U, 0U, 1U, 10U);
    for (i = 0; i < 20; ++i)
    {
        /* 两边相同计数应沿 x 正方向行驶，y 和 yaw 接近零。 */
        chassis_app_control_step(&app, 20U + (uint32_t)i * 10U,
                                 0.01f, 38, 38, 12000U, 1U);
    }
    assert(app.state == CHASSIS_STATE_RUNNING);
    assert(app.left_pwm > 0);
    assert(app.right_pwm > 0);
    assert(app.x_m > 0.0f);
    assert(fabsf(app.y_m) < 0.001f);
    assert(fabsf(app.yaw_rad) < 0.001f);

    /* 超过 200 ms 未更新命令后，本周期必须进入 TIMEOUT 并撤销 PWM。 */
    chassis_app_control_step(&app, 250U, 0.01f, 0, 0, 12000U, 1U);
    assert(app.state == CHASSIS_STATE_TIMEOUT);
    assert(app.left_pwm == 0);
    assert(app.right_pwm == 0);
    assert((app.dynamic_faults & CHASSIS_FAULT_COMMAND_TIMEOUT) != 0U);

    /* 新的合法命令能从超时恢复，不需要人工清故障。 */
    send_motion(&app, 0.10f, 0.0f, 1U, 0U, 2U, 260U);
    chassis_app_control_step(&app, 270U, 0.01f, 20, 20, 12000U, 1U);
    assert(app.state == CHASSIS_STATE_RUNNING);
}

static void test_estop_is_latched(void)
{
    chassis_params_t params;
    chassis_app_t app;

    chassis_params_set_defaults(&params);
    chassis_app_init(&app, &params);
    send_motion(&app, 0.20f, 0.0f, 1U, 1U, 1U, 10U);
    chassis_app_control_step(&app, 20U, 0.01f, 0, 0, 12000U, 1U);
    assert(app.state == CHASSIS_STATE_ESTOP);
    assert(app.left_pwm == 0);

    /* 先置为禁止运行，再发送带钥匙的清故障命令。 */
    send_motion(&app, 0.0f, 0.0f, 0U, 0U, 2U, 30U);
    send_system(&app, CHASSIS_SYSTEM_CLEAR_FAULT, 31U);
    chassis_app_control_step(&app, 40U, 0.01f, 0, 0, 12000U, 1U);
    assert(app.state == CHASSIS_STATE_IDLE);
    assert(app.latched_faults == 0U);
}

static void test_corrupted_stored_parameters_raise_fault(void)
{
    chassis_params_t params;
    chassis_app_t app;

    chassis_params_set_defaults(&params);
    params.crc32 ^= 1U;
    chassis_app_init(&app, &params);
    assert((app.latched_faults & CHASSIS_FAULT_BAD_PARAMETER) != 0U);
    assert(app.state == CHASSIS_STATE_IDLE);
}

int main(void)
{
    test_crc_known_vector();
    test_motion_command_round_trip();
    test_imu_telemetry_layout();
    test_parameters();
    test_wheel_controller();
    test_state_machine_and_odometry();
    test_estop_is_latched();
    test_corrupted_stored_parameters_raise_fault();
    puts("All chassis core tests passed.");
    return 0;
}
