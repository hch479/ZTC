#include "imu_task.h"
#include "MPU6050.h"
#include "ICM20948.h"
#include <string.h>

/*
 * Why calibration is handled here
 * --------------------------------
 * The vendor sensor drivers use SysVal.Time_count to choose between two modes:
 *   Time_count < CONTROL_DELAY  : return uncorrected raw ADC counts
 *   Time_count >= CONTROL_DELAY : subtract imu.Deviation_* from every sample
 *
 * The original program updated Time_count in an old balance task. That task is
 * not used by the current motor project, so calibration could never finish.
 * This module now owns the complete warm-up and calibration sequence.
 */

IMU_DATA_t imu;
static imu_snapshot_t g_imu_snapshot;

static void imu_read_sensor(uint8_t sensor_type)
{
    if (sensor_type == IMU_SENSOR_MPU6050)
    {
        MPU6050_Get_Accelscope();
        MPU6050_Get_Gyroscope();
    }
    else
    {
        ICM20948_Get_Accel();
        ICM20948_Get_Gyroscope();
    }
}

/* Publish one internally consistent sample. */
static void imu_publish_snapshot(uint8_t sensor_type,
                                 uint8_t calibrated,
                                 uint16_t calibration_samples,
                                 uint32_t sample_sequence)
{
    taskENTER_CRITICAL();

    g_imu_snapshot.accel = imu.accel;
    g_imu_snapshot.gyro = imu.gyro;
    g_imu_snapshot.sample_sequence = sample_sequence;
    g_imu_snapshot.calibration_samples = calibration_samples;
    g_imu_snapshot.calibration_target = IMU_CALIBRATION_SAMPLES;
    g_imu_snapshot.sensor_type = sensor_type;
    g_imu_snapshot.calibrated = calibrated;

    taskEXIT_CRITICAL();
}

static void imu_task_run(uint8_t sensor_type)
{
    portTickType last_wake_time;
    int32_t accel_sum_x;
    int32_t accel_sum_y;
    int32_t accel_sum_z;
    int32_t gyro_sum_x;
    int32_t gyro_sum_y;
    int32_t gyro_sum_z;
    uint32_t sample_sequence;
    uint16_t warmup_samples;
    uint16_t calibration_samples;
    uint8_t calibration_complete;
    uint8_t sample_is_corrected;

    last_wake_time = xTaskGetTickCount();
    accel_sum_x = 0;
    accel_sum_y = 0;
    accel_sum_z = 0;
    gyro_sum_x = 0;
    gyro_sum_y = 0;
    gyro_sum_z = 0;
    sample_sequence = 0;
    warmup_samples = 0;
    calibration_samples = 0;
    calibration_complete = 0;

    memset(&imu, 0, sizeof(imu));
    memset(&g_imu_snapshot, 0, sizeof(g_imu_snapshot));

    /* Force the sensor drivers to return uncorrected values first. */
    SysVal.Time_count = 0;
    Flag_Stop = 1;
    Led_Count = 1;

    while (1)
    {
        vTaskDelayUntil(&last_wake_time, F2T(IMU_TASK_RATE));

        /* Remember the mode used for this read. */
        sample_is_corrected = calibration_complete;
        imu_read_sensor(sensor_type);
        sample_sequence++;

        if (!calibration_complete)
        {
            if (warmup_samples < IMU_WARMUP_SAMPLES)
            {
                warmup_samples++;
            }
            else
            {
                accel_sum_x += imu.accel.x;
                accel_sum_y += imu.accel.y;
                accel_sum_z += imu.accel.z;
                gyro_sum_x += imu.gyro.x;
                gyro_sum_y += imu.gyro.y;
                gyro_sum_z += imu.gyro.z;
                calibration_samples++;

                if (calibration_samples >= IMU_CALIBRATION_SAMPLES)
                {
                    /*
                     * Both operands MUST be signed. The sample-count macro is
                     * 500U (unsigned). Dividing a negative signed sum by 500U
                     * first converts the sum to unsigned on this MCU!
                     * Example: -4000 / 500U becomes 8589926, then 4710 after
                     * conversion to int16_t, instead of the correct -8.
                     * Casting AFTER division is too late; cast the divisor.
                     * 500 int16_t samples fit safely in an int32_t sum.
                     */
                    imu.Deviation_accel.x = (int16_t)(accel_sum_x / (int32_t)IMU_CALIBRATION_SAMPLES);
                    imu.Deviation_accel.y = (int16_t)(accel_sum_y / (int32_t)IMU_CALIBRATION_SAMPLES);
                    imu.Deviation_accel.z = (int16_t)(accel_sum_z / (int32_t)IMU_CALIBRATION_SAMPLES);
                    imu.Deviation_gyro.x = (int16_t)(gyro_sum_x / (int32_t)IMU_CALIBRATION_SAMPLES);
                    imu.Deviation_gyro.y = (int16_t)(gyro_sum_y / (int32_t)IMU_CALIBRATION_SAMPLES);
                    imu.Deviation_gyro.z = (int16_t)(gyro_sum_z / (int32_t)IMU_CALIBRATION_SAMPLES);

                    /* The next driver read will subtract these offsets. */
                    SysVal.Time_count = CONTROL_DELAY;
                    calibration_complete = 1;
                    Flag_Stop = 0;
                    Led_Count = 300;
                }
            }
        }

        /*
         * sample_is_corrected becomes 1 one loop after calibration completes.
         * This prevents the final raw calibration sample from being labelled as
         * corrected sensor data.
         */
        imu_publish_snapshot(sensor_type,
                             sample_is_corrected,
                             calibration_samples,
                             sample_sequence);
    }
}

void MPU6050_task(void *pvParameters)
{
    (void)pvParameters;
    imu_task_run(IMU_SENSOR_MPU6050);
}

void ICM20948_task(void *pvParameters)
{
    (void)pvParameters;
    imu_task_run(IMU_SENSOR_ICM20948);
}

uint8_t imu_get_snapshot(imu_snapshot_t *snapshot)
{
    if (snapshot == 0)
    {
        return 0;
    }

    taskENTER_CRITICAL();
    *snapshot = g_imu_snapshot;
    taskEXIT_CRITICAL();

    return (snapshot->sample_sequence != 0U) ? 1U : 0U;
}

void ImuData_copy(IMU_BASE_t *destination, const IMU_BASE_t *source)
{
    if ((destination == 0) || (source == 0))
    {
        return;
    }

    taskENTER_CRITICAL();
    *destination = *source;
    taskEXIT_CRITICAL();
}
