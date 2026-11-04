from glob import glob

from setuptools import find_packages, setup

package_name = "dynabarn_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Akumar201",
    maintainer_email="ankitkumar3.1428@gmail.com",
    description="ROS 2 interface for dynabarn-core (MuJoCo and Gazebo)",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mujoco_node = dynabarn_ros.mujoco_node:main",
            "sensor_frames = dynabarn_ros.sensor_frames:main",
            "prepare_gazebo = dynabarn_ros.prepare:main",
        ],
    },
)
