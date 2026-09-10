#ifndef __IMU_TASK_H
#define __IMU_TASK_H

#include "system.h"

#define IMU_TASK_PRIO 3
#define IMU_STK_SIZE  256
#define IMU_TASK_RATE RATE_100_HZ

typedef struct
{
    short x;
    short y;
    short z;
} IMU_BASE_t;

typedef struct
{
    IMU_BASE_t Deviation_accel;
    IMU_BASE_t Deviation_gyro;
    IMU_BASE_t Original_accel;
    IMU_BASE_t Original_gyro;
    IMU_BASE_t gyro;
    IMU_BASE_t accel;
} IMU_DATA_t;

extern IMU_DATA_t imu;

/* IMU types reported to the upper computer. */
#define IMU_SENSOR_NONE       0U
#define IMU_SENSOR_MPU6050    1U
#define IMU_SENSOR_ICM20948   2U

/* The IMU task runs at 100 Hz. Keep the vehicle still during these samples. */
#define IMU_WARMUP_SAMPLES       50U
#define IMU_CALIBRATION_SAMPLES 500U

typedef struct
{
    IMU_BASE_t accel;
    IMU_BASE_t gyro;
    uint32_t sample_sequence;
    uint16_t calibration_samples;
    uint16_t calibration_target;
    uint8_t sensor_type;
    uint8_t calibrated;
} imu_snapshot_t;

void MPU6050_task(void *pvParameters);
void ICM20948_task(void *pvParameters);

/* Copy the latest complete sample. Returns 0 before the first sample exists. */
uint8_t imu_get_snapshot(imu_snapshot_t *snapshot);

/* Kept for compatibility with the original balance-control source files. */
void ImuData_copy(IMU_BASE_t *destination, const IMU_BASE_t *source);

#endif
