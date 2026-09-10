#include "chassis_can_protocol.h"

#include <string.h>

static int float_is_finite_number(float value)
{
    /* NaN 是唯一不等于自身的浮点数；无穷大还会被后续范围判断拒绝。 */
    return (value == value) && (value <= 3.402823466e+38F) &&
           (value >= -3.402823466e+38F);
}

static int16_t clamp_float_to_i16(float value)
{
    if (value >= 32767.0f)
    {
        return 32767;
    }
    if (value <= -32768.0f)
    {
        return -32768;
    }
    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void put_i16_le(uint8_t *destination, int16_t value)
{
    uint16_t raw = (uint16_t)value;
    destination[0] = (uint8_t)(raw & 0xFFU);
    destination[1] = (uint8_t)((raw >> 8U) & 0xFFU);
}

static int16_t get_i16_le(const uint8_t *source)
{
    uint16_t raw = (uint16_t)source[0] | ((uint16_t)source[1] << 8U);
    return (int16_t)raw;
}

static void put_u16_le(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)(value & 0xFFU);
    destination[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void put_i32_le(uint8_t *destination, int32_t value)
{
    uint32_t raw = (uint32_t)value;
    destination[0] = (uint8_t)(raw & 0xFFU);
    destination[1] = (uint8_t)((raw >> 8U) & 0xFFU);
    destination[2] = (uint8_t)((raw >> 16U) & 0xFFU);
    destination[3] = (uint8_t)((raw >> 24U) & 0xFFU);
}

static void put_u32_le(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)(value & 0xFFU);
    destination[1] = (uint8_t)((value >> 8U) & 0xFFU);
    destination[2] = (uint8_t)((value >> 16U) & 0xFFU);
    destination[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static void put_float_le(uint8_t *destination, float value)
{
    uint32_t raw = 0U;
    memcpy(&raw, &value, sizeof(raw));
    put_u32_le(destination, raw);
}

static float get_float_le(const uint8_t *source)
{
    uint32_t raw = (uint32_t)source[0] |
                   ((uint32_t)source[1] << 8U) |
                   ((uint32_t)source[2] << 16U) |
                   ((uint32_t)source[3] << 24U);
    float value = 0.0f;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

/* CRC-8/ATM：多项式 0x07，初始值 0x00，不反射，不异或输出。 */
uint8_t chassis_crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0U;
    uint8_t i;
    uint8_t bit;

    if (data == 0)
    {
        return 0U;
    }

    for (i = 0U; i < length; ++i)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1U) ^ 0x07U);
            }
            else
            {
                crc = (uint8_t)(crc << 1U);
            }
        }
    }
    return crc;
}

uint8_t chassis_frame_crc_is_valid(const chassis_can_frame_t *frame)
{
    if ((frame == 0) || (frame->length != CHASSIS_CAN_DLC))
    {
        return 0U;
    }
    return (uint8_t)(chassis_crc8(frame->data, 7U) == frame->data[7]);
}

int chassis_encode_motion_command(const chassis_motion_command_t *command,
                                  chassis_can_frame_t *frame)
{
    int16_t linear_mm_s;
    int16_t angular_mrad_s;

    if ((command == 0) || (frame == 0) ||
        !float_is_finite_number(command->linear_mps) ||
        !float_is_finite_number(command->angular_radps))
    {
        return 0;
    }

    linear_mm_s = clamp_float_to_i16(command->linear_mps * 1000.0f);
    angular_mrad_s = clamp_float_to_i16(command->angular_radps * 1000.0f);

    frame->id = CHASSIS_CAN_ID_MOTION_COMMAND;
    frame->length = CHASSIS_CAN_DLC;
    put_i16_le(&frame->data[0], linear_mm_s);
    put_i16_le(&frame->data[2], angular_mrad_s);
    frame->data[4] = 0U;
    if (command->enable != 0U)
    {
        frame->data[4] |= CHASSIS_CMD_ENABLE_MASK;
    }
    if (command->emergency_stop != 0U)
    {
        frame->data[4] |= CHASSIS_CMD_ESTOP_MASK;
    }
    frame->data[5] = 1U; /* 当前协议版本。 */
    frame->data[6] = command->sequence;
    frame->data[7] = chassis_crc8(frame->data, 7U);
    return 1;
}

int chassis_decode_motion_command(const chassis_can_frame_t *frame,
                                  chassis_motion_command_t *command)
{
    if ((frame == 0) || (command == 0) ||
        (frame->id != CHASSIS_CAN_ID_MOTION_COMMAND) ||
        (chassis_frame_crc_is_valid(frame) == 0U) || (frame->data[5] != 1U))
    {
        return 0;
    }

    command->linear_mps = (float)get_i16_le(&frame->data[0]) / 1000.0f;
    command->angular_radps = (float)get_i16_le(&frame->data[2]) / 1000.0f;
    command->enable = (uint8_t)((frame->data[4] & CHASSIS_CMD_ENABLE_MASK) != 0U);
    command->emergency_stop = (uint8_t)((frame->data[4] & CHASSIS_CMD_ESTOP_MASK) != 0U);
    command->sequence = frame->data[6];
    return 1;
}

int chassis_encode_parameter_command(const chassis_parameter_command_t *command,
                                     chassis_can_frame_t *frame)
{
    if ((command == 0) || (frame == 0) ||
        !float_is_finite_number(command->value))
    {
        return 0;
    }
    frame->id = CHASSIS_CAN_ID_PARAMETER_CMD;
    frame->length = CHASSIS_CAN_DLC;
    frame->data[0] = command->parameter_id;
    frame->data[1] = command->operation;
    put_float_le(&frame->data[2], command->value);
    frame->data[6] = command->sequence;
    frame->data[7] = chassis_crc8(frame->data, 7U);
    return 1;
}

int chassis_decode_parameter_command(const chassis_can_frame_t *frame,
                                     chassis_parameter_command_t *command)
{
    if ((frame == 0) || (command == 0) ||
        (frame->id != CHASSIS_CAN_ID_PARAMETER_CMD) ||
        (chassis_frame_crc_is_valid(frame) == 0U))
    {
        return 0;
    }
    command->parameter_id = frame->data[0];
    command->operation = frame->data[1];
    command->value = get_float_le(&frame->data[2]);
    command->sequence = frame->data[6];
    return float_is_finite_number(command->value) ? 1 : 0;
}

int chassis_encode_system_command(const chassis_system_command_t *command,
                                  chassis_can_frame_t *frame)
{
    if ((command == 0) || (frame == 0))
    {
        return 0;
    }
    frame->id = CHASSIS_CAN_ID_SYSTEM_COMMAND;
    frame->length = CHASSIS_CAN_DLC;
    frame->data[0] = command->opcode;
    frame->data[1] = CHASSIS_SYSTEM_KEY0;
    frame->data[2] = CHASSIS_SYSTEM_KEY1;
    frame->data[3] = command->sequence;
    frame->data[4] = command->argument0;
    frame->data[5] = command->argument1;
    frame->data[6] = command->argument2;
    frame->data[7] = chassis_crc8(frame->data, 7U);
    return 1;
}

int chassis_decode_system_command(const chassis_can_frame_t *frame,
                                  chassis_system_command_t *command)
{
    if ((frame == 0) || (command == 0) ||
        (frame->id != CHASSIS_CAN_ID_SYSTEM_COMMAND) ||
        (chassis_frame_crc_is_valid(frame) == 0U) ||
        (frame->data[1] != CHASSIS_SYSTEM_KEY0) ||
        (frame->data[2] != CHASSIS_SYSTEM_KEY1))
    {
        return 0;
    }
    command->opcode = frame->data[0];
    command->sequence = frame->data[3];
    command->argument0 = frame->data[4];
    command->argument1 = frame->data[5];
    command->argument2 = frame->data[6];
    return 1;
}

void chassis_encode_wheel_speed(float left_target_mps,
                                float left_measured_mps,
                                float right_target_mps,
                                float right_measured_mps,
                                chassis_can_frame_t *frame)
{
    frame->id = CHASSIS_CAN_ID_WHEEL_SPEED;
    frame->length = CHASSIS_CAN_DLC;
    put_i16_le(&frame->data[0], clamp_float_to_i16(left_target_mps * 1000.0f));
    put_i16_le(&frame->data[2], clamp_float_to_i16(left_measured_mps * 1000.0f));
    put_i16_le(&frame->data[4], clamp_float_to_i16(right_target_mps * 1000.0f));
    put_i16_le(&frame->data[6], clamp_float_to_i16(right_measured_mps * 1000.0f));
}

void chassis_encode_motor_output(int16_t left_pwm,
                                 int16_t right_pwm,
                                 float left_error_mps,
                                 float right_error_mps,
                                 chassis_can_frame_t *frame)
{
    frame->id = CHASSIS_CAN_ID_MOTOR_OUTPUT;
    frame->length = CHASSIS_CAN_DLC;
    put_i16_le(&frame->data[0], left_pwm);
    put_i16_le(&frame->data[2], right_pwm);
    put_i16_le(&frame->data[4], clamp_float_to_i16(left_error_mps * 1000.0f));
    put_i16_le(&frame->data[6], clamp_float_to_i16(right_error_mps * 1000.0f));
}

void chassis_encode_encoder(uint32_t id,
                            int32_t total_count,
                            int16_t delta_count,
                            uint8_t sequence,
                            chassis_can_frame_t *frame)
{
    frame->id = id;
    frame->length = CHASSIS_CAN_DLC;
    put_i32_le(&frame->data[0], total_count);
    put_i16_le(&frame->data[4], delta_count);
    frame->data[6] = sequence;
    frame->data[7] = chassis_crc8(frame->data, 7U);
}

void chassis_encode_status(uint8_t state,
                           uint16_t fault_flags,
                           uint16_t battery_mv,
                           uint8_t last_command_sequence,
                           uint8_t overrun_count,
                           chassis_can_frame_t *frame)
{
    frame->id = CHASSIS_CAN_ID_STATUS;
    frame->length = CHASSIS_CAN_DLC;
    frame->data[0] = state;
    put_u16_le(&frame->data[1], fault_flags);
    put_u16_le(&frame->data[3], battery_mv);
    frame->data[5] = last_command_sequence;
    frame->data[6] = overrun_count;
    frame->data[7] = chassis_crc8(frame->data, 7U);
}

void chassis_encode_parameter_reply(uint8_t parameter_id,
                                    uint8_t reply_status,
                                    float value,
                                    uint8_t sequence,
                                    chassis_can_frame_t *frame)
{
    frame->id = CHASSIS_CAN_ID_PARAMETER_REPLY;
    frame->length = CHASSIS_CAN_DLC;
    frame->data[0] = parameter_id;
    frame->data[1] = reply_status;
    put_float_le(&frame->data[2], value);
    frame->data[6] = sequence;
    frame->data[7] = chassis_crc8(frame->data, 7U);
}

void chassis_encode_heartbeat(uint32_t uptime_ms,
                              uint8_t state,
                              uint8_t firmware_major,
                              uint8_t firmware_minor,
                              chassis_can_frame_t *frame)
{
    frame->id = CHASSIS_CAN_ID_HEARTBEAT;
    frame->length = CHASSIS_CAN_DLC;
    put_u32_le(&frame->data[0], uptime_ms);
    frame->data[4] = state;
    frame->data[5] = firmware_major;
    frame->data[6] = firmware_minor;
    frame->data[7] = chassis_crc8(frame->data, 7U);
}

void chassis_encode_imu_vector(uint32_t id,
                               int16_t x,
                               int16_t y,
                               int16_t z,
                               uint8_t sequence,
                               chassis_can_frame_t *frame)
{
    frame->id = id;
    frame->length = CHASSIS_CAN_DLC;
    put_i16_le(&frame->data[0], x);
    put_i16_le(&frame->data[2], y);
    put_i16_le(&frame->data[4], z);
    frame->data[6] = sequence;
    frame->data[7] = chassis_crc8(frame->data, 7U);
}

void chassis_encode_imu_status(uint8_t sensor_type,
                               uint8_t calibrated,
                               uint16_t calibration_samples,
                               uint16_t calibration_target,
                               uint8_t sequence,
                               chassis_can_frame_t *frame)
{
    frame->id = CHASSIS_CAN_ID_IMU_STATUS;
    frame->length = CHASSIS_CAN_DLC;
    frame->data[0] = sensor_type;
    frame->data[1] = (calibrated != 0U) ? CHASSIS_IMU_CALIBRATED_MASK : 0U;
    put_u16_le(&frame->data[2], calibration_samples);
    put_u16_le(&frame->data[4], calibration_target);
    frame->data[6] = sequence;
    frame->data[7] = chassis_crc8(frame->data, 7U);
}
