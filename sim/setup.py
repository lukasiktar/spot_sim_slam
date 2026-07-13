import os
from glob import glob
from setuptools import setup

package_name = "spot_sim_slam"

setup(
    name=package_name,
    version="1.0.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.py")),
    ],
    zip_safe=True,
    maintainer="Spot Autonomy",
    maintainer_email="user@example.com",
    description="Sim glue for Spot VSLAM testing",
    license="MIT",
)
