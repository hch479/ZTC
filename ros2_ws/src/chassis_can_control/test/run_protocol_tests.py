"""不安装 pytest 时也能执行的轻量协议自测入口。"""

from pathlib import Path
import sys

# 允许从源码包目录或 test 目录直接运行，不要求先执行 colcon build。
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import test_can_protocol  # noqa: E402
import test_wheel_odometry  # noqa: E402


def main() -> None:
    test_can_protocol.test_crc_known_vector()
    test_can_protocol.test_motion_command_layout()
    test_can_protocol.test_parameter_float_round_trip()
    test_can_protocol.test_fault_text()
    test_can_protocol.test_imu_vector_decode_and_crc_rejection()
    test_can_protocol.test_imu_status_decode()
    test_wheel_odometry.test_first_sample_only_sets_baseline()
    test_wheel_odometry.test_equal_counts_move_straight()
    test_wheel_odometry.test_int32_wrap_is_small_delta()
    test_wheel_odometry.test_imu_yaw_rate_is_blended_with_wheel_yaw()
    test_wheel_odometry.test_missing_imu_uses_original_wheel_only_result()
    print("All Python protocol tests passed.")


if __name__ == "__main__":
    main()
