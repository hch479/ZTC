"""不安装 pytest 时也能执行的轻量协议自测入口。"""

from pathlib import Path
import sys

# 允许从源码包目录或 test 目录直接运行，不要求先执行 colcon build。
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import test_can_protocol  # noqa: E402
import test_wheel_odometry  # noqa: E402
import test_planar_ekf  # noqa: E402
import test_serial_transport  # noqa: E402
import test_node_ekf  # noqa: E402


def main() -> None:
    count = 0
    for module in (test_can_protocol, test_wheel_odometry, test_planar_ekf,
                   test_serial_transport, test_node_ekf):
        for name in sorted(vars(module)):
            if name.startswith("test_"):
                getattr(module, name)()
                count += 1
    print(f"All {count} Python tests passed (protocol, odometry, EKF, node wiring).")


if __name__ == "__main__":
    main()
