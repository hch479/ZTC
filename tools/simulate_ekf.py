"""在同一合成数据上比较纯编码器、固定权重和 EKF；不接触设备。

运行：python tools/simulate_ekf.py --output build/ekf_demo
结果仅验证所设定的仿真条件，不代表实车精度。
"""

import argparse
import csv
import json
import math
from pathlib import Path
import random
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/chassis_can_control'))
from chassis_can_control.planar_ekf import EkfOdometry, wrap_angle  # noqa: E402
from chassis_can_control.wheel_odometry import WheelOdometry  # noqa: E402


def simulate(name, gyro_bias=0.0, stationary=False, imu_gap=False):
    random_source = random.Random(20260910)
    ekf, wheels, weighted = EkfOdometry(), WheelOdometry(), WheelOdometry()
    diameter, counts, track = 0.065, 60000.0, 0.162
    meters_per_count = math.pi * diameter / counts
    left_total = right_total = 0.0
    truth = [0.0, 0.0, 0.0]
    latest_gyro = None
    latest_gyro_time = -1.0
    ekf.update(0, 0, diameter, counts, track, 0.0)
    wheels.update(0, 0, diameter, counts, track)
    weighted.update(0, 0, diameter, counts, track)
    rows = []
    errors = {key: [0.0, 0.0] for key in ('wheel', 'weighted', 'ekf')}
    # 积分时间步 10 ms；IMU 20 ms；编码器 50 ms。
    for step in range(1, 2001):
        timestamp = step * 0.01
        speed = 0.0 if stationary else 0.25 + 0.05 * math.sin(timestamp * 0.5)
        rate = 0.0 if stationary else 0.3 * math.sin(timestamp * 0.4)
        middle = truth[2] + rate * 0.005
        truth[0] += speed * 0.01 * math.cos(middle)
        truth[1] += speed * 0.01 * math.sin(middle)
        truth[2] = wrap_angle(truth[2] + rate * 0.01)
        # 轮速具有尺度误差和独立随机扰动；误差由本脚本人为设定。
        left_speed = (speed - track * rate / 2.0) * 1.01 + random_source.gauss(0, 0.035)
        right_speed = (speed + track * rate / 2.0) * 0.99 + random_source.gauss(0, 0.035)
        left_total += left_speed * 0.01 / meters_per_count
        right_total += right_speed * 0.01 / meters_per_count
        if step % 2 == 0 and not (imu_gap and 8.0 <= timestamp <= 12.0):
            latest_gyro = rate + gyro_bias + random_source.gauss(0, 0.03)
            latest_gyro_time = timestamp
            ekf.add_imu(latest_gyro, timestamp)
        if step % 5 != 0:
            continue
        left, right = round(left_total), round(right_total)
        samples = {
            'wheel': wheels.update(left, right, diameter, counts, track),
            'weighted': weighted.update(
                left, right, diameter, counts, track,
                latest_gyro if timestamp - latest_gyro_time <= 0.2 else None, 0.05, 0.85),
            'ekf': ekf.update(left, right, diameter, counts, track, timestamp),
        }
        row = {'time_s': timestamp, 'truth_x_m': truth[0], 'truth_y_m': truth[1],
               'truth_yaw_rad': truth[2]}
        for key, sample in samples.items():
            position_error = math.hypot(sample.x_m - truth[0], sample.y_m - truth[1])
            angle_error = wrap_angle(sample.yaw_rad - truth[2])
            errors[key][0] += position_error ** 2
            errors[key][1] += angle_error ** 2
            row.update({key + '_x_m': sample.x_m, key + '_y_m': sample.y_m,
                        key + '_yaw_rad': sample.yaw_rad,
                        key + '_position_error_m': position_error,
                        key + '_yaw_error_rad': angle_error})
        rows.append(row)
    summary = {'scenario': name, 'source': 'synthetic', 'samples': len(rows), 'metrics': {}}
    for key, sums in errors.items():
        summary['metrics'][key] = {
            'position_rmse_m': math.sqrt(sums[0] / len(rows)),
            'yaw_rmse_rad': math.sqrt(sums[1] / len(rows)),
        }
    summary['ekf_rejected_wheel'] = ekf.filter.rejected_wheel
    summary['ekf_rejected_imu'] = ekf.filter.rejected_imu
    return rows, summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'build/ekf_demo')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    summaries = []
    for name, options in (
        ('turns', {}), ('gyro_bias', {'gyro_bias': 0.015}),
        ('imu_dropout', {'imu_gap': True}),
        ('stationary_bias', {'stationary': True, 'gyro_bias': 0.015}),
    ):
        rows, summary = simulate(name, **options)
        with (args.output / (name + '.csv')).open('w', newline='', encoding='utf-8') as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        summaries.append(summary)
    text = json.dumps(summaries, ensure_ascii=False, indent=2)
    (args.output / 'summary.json').write_text(text + '\n', encoding='utf-8')
    print(text)


if __name__ == '__main__':
    main()
