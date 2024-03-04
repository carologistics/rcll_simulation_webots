import os
from glob import glob

from setuptools import find_packages
from setuptools import setup

package_name = "robotino3_sensors"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="saurabh-borse",
    maintainer_email="saurabh.borse@alumni.fh-aachen.de",
    description="package for robotino sensor interface",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robotino3_joyteleop = robotino3_sensors.robotino3_joyteleop:main",
            "robotino3_laserscan_republisher = robotino3_sensors.robotino3_laserscan_republisher:main",
            "robotino3_laserscanmerger = robotino3_sensors.robotino3_laserscanmerger:main",
            "robotino3_irscanmerger = robotino3_sensors.robotino3_irscanmerger:main",
        ],
    },
)
