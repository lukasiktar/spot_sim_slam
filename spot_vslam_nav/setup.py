import os
from glob import glob

from setuptools import setup

package_name = "spot_vslam_nav"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Spot Autonomy",
    maintainer_email="user@example.com",
    description="Isaac ROS Visual SLAM + Nav2 for Boston Dynamics Spot on Jetson Orin Nano",
    license="MIT",
    entry_points={
        "console_scripts": [
            "spot_auto_bringup = spot_vslam_nav.spot_auto_bringup:main",
            "goal_cli = spot_vslam_nav.goal_cli:main",
            "cmd_vel_gate = spot_vslam_nav.cmd_vel_gate:main",
            "vslam_health_monitor = spot_vslam_nav.vslam_health_monitor:main",
        ],
    },
)
