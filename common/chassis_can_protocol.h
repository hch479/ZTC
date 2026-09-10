#ifndef CHASSIS_CAN_PROTOCOL_H
#define CHASSIS_CAN_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 所有报文都使用标准 11 位 CAN ID，数据长度固定为 8 字节。
 * 多字节整数采用小端顺序：低字节在前，高字节在后。
 */
#define CHASSIS_CAN_DLC 8U

#define CHASSIS_CAN_ID_MOTION_COMMAND   0x101U
#define CHASSIS_CAN_ID_PARAMETER_CMD    0x102U
#define CHASSIS_CAN_ID_SYSTEM_COMMAND   0x103U

#define CHASSIS_CAN_ID_WHEEL_SPEED      0x181U
#define CHASSIS_CAN_ID_MOTOR_OUTPUT     0x182U
#define CHASSIS_CAN_ID_LEFT_ENCODER     0x183U
#define CHASSIS_CAN_ID_RIGHT_ENCODER    0x184U
#define CHASSIS_CAN_ID_STATUS           0x185U
#define CHASSIS_CAN_ID_PARAMETER_REPLY  0x186U
#define CHASSIS_CAN_ID_IMU_ACCELERATION 0x187U
#define CHASSIS_CAN_ID_IMU_GYROSCOPE    0x188U
#define CHASSIS_CAN_ID_IMU_STATUS       0x189U
#define CHASSIS_CAN_ID_HEARTBEAT        0x700U

#define CHASSIS_IMU_CALIBRATED_MASK 0x01U

/* 运动命令中 flags 字节的位定义。 */
#define CHASSIS_CMD_ENABLE_MASK 0x01U
#define CHASSIS_CMD_ESTOP_MASK  0x02U

/* 系统命令需要同时带两个固定钥匙字节，降低随机误触发的可能性。 */
#define CHASSIS_SYSTEM_KEY0 0xA5U
#define CHASSIS_SYSTEM_KEY1 0x5AU

typedef enum
{
    CHASSIS_STATE_BOOT = 0,
    CHASSIS_STATE_IDLE = 1,
    CHASSIS_STATE_RUNNING = 2,
    CHASSIS_STATE_TIMEOUT = 3,
    CHASSIS_STATE_FAULT = 4,
    CHASSIS_STATE_ESTOP = 5
} chassis_state_t;

typedef enum
{
    CHASSIS_FAULT_NONE = 0x0000U,
    CHASSIS_FAULT_COMMAND_TIMEOUT = 0x0001U,
    CHASSIS_FAULT_ESTOP = 0x0002U,
    CHASSIS_FAULT_LOW_BATTERY = 0x0004U,
    CHASSIS_FAULT_LEFT_ENCODER = 0x0008U,
    CHASSIS_FAULT_RIGHT_ENCODER = 0x0010U,
    CHASSIS_FAULT_LEFT_STALL = 0x0020U,
    CHASSIS_FAULT_RIGHT_STALL = 0x0040U,
    CHASSIS_FAULT_HARDWARE_DISABLED = 0x0080U,
    CHASSIS_FAULT_CONTROL_OVERRUN = 0x0100U,
    CHASSIS_FAULT_BAD_PARAMETER = 0x0200U
} chassis_fault_t;

typedef enum
{
    CHASSIS_PARAM_READ = 0,
    CHASSIS_PARAM_WRITE = 1
} chassis_parameter_operation_t;

typedef enum
{
    CHASSIS_PARAM_LEFT_KP = 1,
    CHASSIS_PARAM_LEFT_KI = 2,
    CHASSIS_PARAM_LEFT_KFF = 3,
    CHASSIS_PARAM_RIGHT_KP = 4,
    CHASSIS_PARAM_RIGHT_KI = 5,
    CHASSIS_PARAM_RIGHT_KFF = 6,
    CHASSIS_PARAM_LEFT_DEAD_FORWARD = 7,
    CHASSIS_PARAM_LEFT_DEAD_REVERSE = 8,
    CHASSIS_PARAM_RIGHT_DEAD_FORWARD = 9,
    CHASSIS_PARAM_RIGHT_DEAD_REVERSE = 10,
    CHASSIS_PARAM_ACCELERATION = 11,
    CHASSIS_PARAM_DECELERATION = 12,
    CHASSIS_PARAM_FILTER_ALPHA = 13,
    CHASSIS_PARAM_SYNC_KP = 14,
    CHASSIS_PARAM_SYNC_MAX = 15,
    CHASSIS_PARAM_WHEEL_TRACK = 16,
    CHASSIS_PARAM_WHEEL_DIAMETER = 17,
    CHASSIS_PARAM_COUNTS_PER_REV = 18,
    CHASSIS_PARAM_MAX_LINEAR_SPEED = 19,
    CHASSIS_PARAM_LOW_BATTERY_MV = 20,
    CHASSIS_PARAM_COMMAND_TIMEOUT_MS = 21,
    CHASSIS_PARAM_STALL_TARGET_SPEED = 22,
    CHASSIS_PARAM_STALL_MEASURED_SPEED = 23,
    CHASSIS_PARAM_STALL_PWM_RATIO = 24,
    CHASSIS_PARAM_STALL_TIME_MS = 25,
    CHASSIS_PARAM_CHASSIS_TYPE = 26,
    CHASSIS_PARAM_WHEELBASE = 27,
    CHASSIS_PARAM_MAX_STEERING_ANGLE = 28,
    CHASSIS_PARAM_SERVO_CENTER_PWM = 29,
    CHASSIS_PARAM_SERVO_PWM_PER_RAD = 30
} chassis_parameter_id_t;

typedef enum
{
    CHASSIS_SYSTEM_CLEAR_FAULT = 1,
    CHASSIS_SYSTEM_SAVE_PARAMETERS = 2,
    CHASSIS_SYSTEM_RESET_ODOMETRY = 3
} chassis_system_opcode_t;

typedef enum
{
    CHASSIS_REPLY_OK = 0,
    CHASSIS_REPLY_BAD_CRC = 1,
    CHASSIS_REPLY_BAD_PARAMETER = 2,
    CHASSIS_REPLY_BAD_VALUE = 3,
    CHASSIS_REPLY_DENIED = 4,
    CHASSIS_REPLY_STORAGE_ERROR = 5
} chassis_reply_status_t;

typedef struct
{
    uint32_t id;
    uint8_t length;
    uint8_t data[CHASSIS_CAN_DLC];
} chassis_can_frame_t;

typedef struct
{
    float linear_mps;
    float angular_radps;
    uint8_t enable;
    uint8_t emergency_stop;
    uint8_t sequence;
} chassis_motion_command_t;

typedef struct
{
    uint8_t parameter_id;
    uint8_t operation;
    float value;
    uint8_t sequence;
} chassis_parameter_command_t;

typedef struct
{
    uint8_t opcode;
    uint8_t sequence;
    uint8_t argument0;
    uint8_t argument1;
    uint8_t argument2;
} chassis_system_command_t;

uint8_t chassis_crc8(const uint8_t *data, uint8_t length);
uint8_t chassis_frame_crc_is_valid(const chassis_can_frame_t *frame);

int chassis_encode_motion_command(const chassis_motion_command_t *command,
                                  chassis_can_frame_t *frame);
int chassis_decode_motion_command(const chassis_can_frame_t *frame,
                                  chassis_motion_command_t *command);

int chassis_encode_parameter_command(const chassis_parameter_command_t *command,
                                     chassis_can_frame_t *frame);
int chassis_decode_parameter_command(const chassis_can_frame_t *frame,
                                     chassis_parameter_command_t *command);

int chassis_encode_system_command(const chassis_system_command_t *command,
                                  chassis_can_frame_t *frame);
int chassis_decode_system_command(const chassis_can_frame_t *frame,
                                  chassis_system_command_t *command);

void chassis_encode_wheel_speed(float left_target_mps,
                                float left_measured_mps,
                                float right_target_mps,
                                float right_measured_mps,
                                chassis_can_frame_t *frame);
void chassis_encode_motor_output(int16_t left_pwm,
                                 int16_t right_pwm,
                                 float left_error_mps,
                                 float right_error_mps,
                                 chassis_can_frame_t *frame);
void chassis_encode_encoder(uint32_t id,
                            int32_t total_count,
                            int16_t delta_count,
                            uint8_t sequence,
                            chassis_can_frame_t *frame);
void chassis_encode_status(uint8_t state,
                           uint16_t fault_flags,
                           uint16_t battery_mv,
                           uint8_t last_command_sequence,
                           uint8_t overrun_count,
                           chassis_can_frame_t *frame);
void chassis_encode_parameter_reply(uint8_t parameter_id,
                                    uint8_t reply_status,
                                    float value,
                                    uint8_t sequence,
                                    chassis_can_frame_t *frame);
void chassis_encode_heartbeat(uint32_t uptime_ms,
                              uint8_t state,
                              uint8_t firmware_major,
                              uint8_t firmware_minor,
                              chassis_can_frame_t *frame);
void chassis_encode_imu_vector(uint32_t id,
                               int16_t x,
                               int16_t y,
                               int16_t z,
                               uint8_t sequence,
                               chassis_can_frame_t *frame);
void chassis_encode_imu_status(uint8_t sensor_type,
                               uint8_t calibrated,
                               uint16_t calibration_samples,
                               uint16_t calibration_target,
                               uint8_t sequence,
                               chassis_can_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif
