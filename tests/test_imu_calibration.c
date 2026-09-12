/*
 * Run the actual Keil IMU task on a PC with fake sensor/RTOS functions.
 * No hardware is accessed. This catches signed/unsigned division regressions
 * in all six calibration assignments, not just a copied mathematical formula.
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <setjmp.h>

/* Skip hardware-only headers, while keeping the real imu_task.h types. */
#define __SYSTEM_H
#define __MPU6050_H
#define __ICM_20948_H
#define RATE_100_HZ 100
#define CONTROL_DELAY 1000
#define F2T(rate) (1000 / (rate))
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
typedef uint32_t portTickType;
static struct { uint32_t Time_count; } SysVal;
static int Flag_Stop;
static int Led_Count;

#include "../keil_project/R550_C30D_SERIAL_MOTOR/BALANCE/imu_task.h"

static jmp_buf task_exit;
static unsigned int wake_count;
static int16_t input_accel[3];
static int16_t input_gyro[3];
static int fractional_negative;
static int16_t first_corrected_gyro_x;
static uint8_t raw_last_was_calibrated;

static portTickType xTaskGetTickCount(void) { return 0; }
static void vTaskDelayUntil(portTickType *last, int period);

/* Simulate the vendor driver's raw/corrected mode, without I2C. */
static void MPU6050_Get_Accelscope(void)
{
    imu.accel.x = input_accel[0];
    imu.accel.y = input_accel[1];
    imu.accel.z = input_accel[2];
    if (SysVal.Time_count >= CONTROL_DELAY)
    {
        imu.accel.x -= imu.Deviation_accel.x;
        imu.accel.y -= imu.Deviation_accel.y;
        imu.accel.z = (int16_t)((int32_t)imu.accel.z
                              - imu.Deviation_accel.z + 16384);
    }
}

static void MPU6050_Get_Gyroscope(void)
{
    imu.gyro.x = input_gyro[0];
    imu.gyro.y = input_gyro[1];
    imu.gyro.z = input_gyro[2];
    if (fractional_negative && ((wake_count % 2U) == 0U))
    {
        imu.gyro.x--; /* Alternating -8 and -9: average -8.5, truncated to -8. */
    }
    if (SysVal.Time_count >= CONTROL_DELAY)
    {
        imu.gyro.x -= imu.Deviation_gyro.x;
        imu.gyro.y -= imu.Deviation_gyro.y;
        imu.gyro.z -= imu.Deviation_gyro.z;
    }
}

static void ICM20948_Get_Accel(void) { MPU6050_Get_Accelscope(); }
static void ICM20948_Get_Gyroscope(void) { MPU6050_Get_Gyroscope(); }

/* Compile the production task, including its private calibration function. */
#include "../keil_project/R550_C30D_SERIAL_MOTOR/BALANCE/imu_task.c"

static void vTaskDelayUntil(portTickType *last, int period)
{
    *last += (portTickType)period;
    wake_count++;
    if (wake_count == IMU_WARMUP_SAMPLES + IMU_CALIBRATION_SAMPLES + 1U)
    {
        raw_last_was_calibrated = g_imu_snapshot.calibrated;
    }
    if (wake_count == IMU_WARMUP_SAMPLES + IMU_CALIBRATION_SAMPLES + 2U)
    {
        first_corrected_gyro_x = g_imu_snapshot.gyro.x;
        longjmp(task_exit, 1); /* End the infinite embedded task after one corrected sample. */
    }
}

static void run_case(int16_t value, int fractional, uint8_t sensor)
{
    int i;
    for (i = 0; i < 3; i++)
    {
        input_accel[i] = value;
        input_gyro[i] = value;
    }
    fractional_negative = fractional;
    wake_count = 0;
    if (setjmp(task_exit) == 0)
    {
        imu_task_run(sensor);
    }
    assert(imu.Deviation_accel.x == value);
    assert(imu.Deviation_accel.y == value);
    assert(imu.Deviation_accel.z == value);
    assert(imu.Deviation_gyro.x == value);
    assert(imu.Deviation_gyro.y == value);
    assert(imu.Deviation_gyro.z == value);
    assert(raw_last_was_calibrated == 0U);
    assert(g_imu_snapshot.calibrated == 1U);
    assert(g_imu_snapshot.calibration_samples == 500U);
    assert(g_imu_snapshot.accel.z == 16384);
    assert(first_corrected_gyro_x == 0);
    assert(g_imu_snapshot.gyro.y == 0);
    assert(g_imu_snapshot.gyro.z == 0);
}

int main(void)
{
    run_case(-8, 0, IMU_SENSOR_ICM20948);
    run_case(-8, 1, IMU_SENSOR_ICM20948);
    run_case(0, 0, IMU_SENSOR_ICM20948);
    run_case(19, 0, IMU_SENSOR_ICM20948);
    run_case(INT16_MIN, 0, IMU_SENSOR_ICM20948);
    run_case(INT16_MAX, 0, IMU_SENSOR_ICM20948);
    run_case(-8, 0, IMU_SENSOR_MPU6050);
    puts("All 7 production IMU calibration cases passed (six axes and calibration timing).");
    return 0;
}
