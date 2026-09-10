from glob import glob
from setuptools import find_packages, setup


package_name = "chassis_can_control"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="C30D chassis motor control bridge for ROS 2, serial and SocketCAN.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "chassis_can_node = chassis_can_control.chassis_can_node:main",
        ],
    },
)
