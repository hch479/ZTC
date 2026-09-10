"""用最小 ROS 消息/传输替身执行真实节点方法；不替代 ROS Humble 运行验收。"""

from contextlib import contextmanager
import importlib.util
from pathlib import Path
import struct
import sys
from types import ModuleType, SimpleNamespace as Namespace
from unittest.mock import patch


def vector():
    return Namespace(x=0.0, y=0.0, z=0.0, w=0.0)


class Message:
    OK, WARN, ERROR = 0, 1, 2

    def __init__(self, **fields):
        self.header = Namespace(stamp=None, frame_id='')
        self.pose = Namespace(pose=Namespace(position=vector(), orientation=vector()),
                              covariance=[0.0] * 36)
        self.twist = Namespace(twist=Namespace(linear=vector(), angular=vector()),
                               covariance=[0.0] * 36)
        self.transform = Namespace(translation=vector(), rotation=vector())
        self.linear, self.angular = vector(), vector()
        self.angular_velocity, self.linear_acceleration = vector(), vector()
        self.orientation_covariance = [0.0] * 9
        self.status, self.values = [], []
        self.__dict__.update(fields)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    overrides = {}

    def __init__(self, name):
        self.parameters = {}
        self.publishers = {}

    def declare_parameter(self, name, default):
        self.parameters[name] = self.overrides.get(name, default)

    def get_parameter(self, name):
        return Namespace(value=self.parameters[name])

    def create_publisher(self, kind, topic, depth):
        self.publishers[topic] = Publisher()
        return self.publishers[topic]

    def create_subscription(self, *args):
        pass

    def create_service(self, *args):
        pass

    def create_timer(self, *args):
        pass

    def add_on_set_parameters_callback(self, *args):
        pass

    def get_logger(self):
        return Namespace(info=lambda *x: None, warning=lambda *x: None, error=lambda *x: None)

    def get_clock(self):
        return Namespace(now=lambda: Namespace(to_msg=lambda: 0))


class FakeTransport:
    def __init__(self, *args):
        self.interface = 'offline'
        self.is_open = True
        self.sent = []

    def send(self, frame):
        self.sent.append(frame)


class Broadcaster:
    def __init__(self, node):
        self.messages = []

    def sendTransform(self, message):
        self.messages.append(message)


@contextmanager
def node_context(**overrides):
    modules = {}
    entries = {
        'diagnostic_msgs.msg': ['DiagnosticArray', 'DiagnosticStatus', 'KeyValue'],
        'geometry_msgs.msg': ['TransformStamped', 'Twist'],
        'nav_msgs.msg': ['Odometry'],
        'rcl_interfaces.msg': ['SetParametersResult'],
        'sensor_msgs.msg': ['Imu', 'JointState'],
        'std_msgs.msg': ['Float32MultiArray'],
        'std_srvs.srv': ['SetBool', 'Trigger'],
    }
    for module_name, names in entries.items():
        parent = module_name.split('.')[0]
        modules[parent] = ModuleType(parent)
        module = modules[module_name] = ModuleType(module_name)
        for name in names:
            if module_name.endswith('.srv'):
                setattr(module, name, Namespace(Request=Message, Response=Message))
            else:
                setattr(module, name, Message)
    modules['rclpy'] = ModuleType('rclpy')
    modules['rclpy.node'] = ModuleType('rclpy.node')
    modules['rclpy.node'].Node = FakeNode
    modules['tf2_ros'] = ModuleType('tf2_ros')
    modules['tf2_ros'].TransformBroadcaster = Broadcaster
    path = Path(__file__).resolve().parents[1] / 'chassis_can_control/chassis_can_node.py'
    name = 'chassis_can_control._offline_node_test'
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    modules[name] = module
    with patch.dict(sys.modules, modules), patch.object(
            FakeNode, 'overrides', {'transport': 'serial', **overrides}):
        spec.loader.exec_module(module)
        with patch.object(module, 'SerialTransport', FakeTransport), patch.object(
                module.time, 'monotonic', return_value=0.0) as clock:
            yield module.ChassisCanNode(), module, clock


def encoder_pair(node, module, left, right, sequence):
    # 使用真实协议 CRC/解码及真实节点配对分支。
    protocol = module.protocol
    for identifier, count in ((protocol.ID_LEFT_ENCODER, left), (protocol.ID_RIGHT_ENCODER, right)):
        body = struct.pack('<ihB', count, 0, sequence)
        node._handle_frame(protocol.CanFrame(identifier, body + bytes([protocol.crc8(body)])))


def imu_pair(node, module, sequence, calibrated=True):
    protocol = module.protocol
    status_body = struct.pack('<BBHHB', 2, int(calibrated), 500, 500, sequence)
    node._handle_frame(protocol.CanFrame(
        protocol.ID_IMU_STATUS, status_body + bytes([protocol.crc8(status_body)])))
    for identifier, z in ((protocol.ID_IMU_ACCELERATION, 16384), (protocol.ID_IMU_GYROSCOPE, 655)):
        body = struct.pack('<hhhB', 0, 0, z, sequence)
        node._handle_frame(protocol.CanFrame(identifier, body + bytes([protocol.crc8(body)])))


def test_node_publishes_ekf_raw_covariance_and_single_tf():
    with node_context() as (node, module, clock):
        assert not node._drive_enabled
        encoder_pair(node, module, 0, 0, 0)
        clock.return_value = 0.02
        imu_pair(node, module, 1)
        assert len(node.publishers['imu/data_raw'].messages) == 1
        before = [row[:] for row in node._ekf_odometry.filter.covariance]
        imu_pair(node, module, 1)
        assert node._ekf_odometry.filter.covariance == before
        clock.return_value = 0.05
        encoder_pair(node, module, 300, 300, 1)
        fused = node.publishers['odom'].messages[-1]
        raw = node.publishers['wheel/odom_raw'].messages[-1]
        assert fused.pose.pose.orientation.z != 0.0
        assert raw.pose.pose.orientation.z == 0.0
        assert fused.pose.covariance[35] > 0.0
        assert fused.twist.twist.angular.z == node._ekf_odometry.filter.state[4]
        assert len(node._tf_broadcaster.messages) == len(node.publishers['odom'].messages) == 2
        assert node._imu_fusion_active
        node._publish_diagnostics()
        values = {v.key: v.value for v in node.publishers['diagnostics'].messages[-1].status[0].values}
        assert values['odometry_mode'] == 'ekf'
        assert values['imu_fusion_active'] == 'True'


def test_node_imu_timeout_reset_and_restart():
    with node_context() as (node, module, clock):
        encoder_pair(node, module, 0, 0, 0)
        clock.return_value = 0.02
        imu_pair(node, module, 1)
        for i in range(1, 9):
            clock.return_value = i * 0.05
            encoder_pair(node, module, i * 300, i * 300, i)
        assert not node._imu_fusion_active
        response = node._on_reset_odometry(Message(), Message())
        assert response.success
        assert node._left_encoder is node._right_encoder is None
        assert node._ekf_odometry.previous is None
        assert node._ekf_odometry.last_accepted_gyro_time is None
        clock.return_value = 0.5
        encoder_pair(node, module, 1200, 1200, 9)
        protocol = module.protocol
        node._previous_stm32_uptime_ms = 2000
        body = struct.pack('<IBBB', 10, 0, 1, 0)
        node._handle_frame(protocol.CanFrame(
            protocol.ID_HEARTBEAT, body + bytes([protocol.crc8(body)])))
        assert node._left_encoder is None
        assert node._ekf_odometry.previous is None


def test_node_mode_parameter_validation_and_weighted_compatibility():
    with node_context(**{'odometry.mode': 'weighted'}) as (node, module, clock):
        encoder_pair(node, module, 0, 0, 0)
        clock.return_value = 0.05
        encoder_pair(node, module, 300, 300, 1)
        assert node.publishers['odom'].messages[-1].pose.pose.position.x > 0.0
        assert not node.publishers['wheel/odom_raw'].messages
    with node_context() as (node, module, clock):
        for name in ('odometry.mode', 'ekf.gyro_stddev', 'geometry.wheel_track_m'):
            result = node._on_parameters_changed([Namespace(name=name, value=0.1)])
            assert not result.successful
        assert node._on_parameters_changed([Namespace(name='imu.fusion_enabled', value=False)]).successful
    try:
        with node_context(**{'odometry.mode': 'unknown'}):
            pass
    except ValueError:
        pass
    else:
        raise AssertionError('invalid mode accepted')


def test_node_uncalibrated_imu_and_disabled_fusion_do_not_enter_ekf():
    with node_context() as (node, module, clock):
        encoder_pair(node, module, 0, 0, 0)
        clock.return_value = 0.02
        imu_pair(node, module, 1, calibrated=False)
        assert node._ekf_odometry.last_accepted_gyro_time is None
        response = node._on_enable(Message(data=True), Message())
        assert not response.success and not node._drive_enabled
        node.parameters['imu.fusion_enabled'] = False
        clock.return_value = 0.04
        imu_pair(node, module, 2)
        assert node._ekf_odometry.last_accepted_gyro_time is None
