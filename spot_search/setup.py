import os
from glob import glob

from setuptools import setup

package_name = "spot_search"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Spot Autonomy",
    maintainer_email="user@example.com",
    description="Autonomous person search for Spot (nvblox + frontier exploration)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "map_slice_to_occupancy = spot_search.map_slice_to_occupancy:main",
            "person_detector = spot_search.person_detector:main",
            "person_map = spot_search.person_map:main",
            "search_mission = spot_search.search_mission:main",
            "search_cli = spot_search.search_cli:main",
            "save_search_map = spot_search.save_search_map:main",
        ],
    },
)
