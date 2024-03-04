import os
from glob import glob

from setuptools import find_packages
from setuptools import setup

package_name = "robotino_simulation"

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
        (os.path.join("share", package_name, "resource"), glob("resource/*")),
        (os.path.join("share", package_name, "protos"), glob("protos/*")),
        (os.path.join("share", package_name, "protos/machine_base"), glob("protos/machine_base/*")),
        # (os.path.join('share', package_name, 'protos/icons'), glob('protos/icons/*')),
        (os.path.join("share", package_name, "urdf/components"), glob("urdf/components/*")),
        (os.path.join("share", package_name, "urdf/robots"), glob("urdf/robots/*")),
        (os.path.join("share", package_name, "urdf/sensors"), glob("urdf/sensors/*")),
        (os.path.join("share", package_name, "meshes/parts"), glob("meshes/parts/*")),
        (os.path.join("share", package_name, "meshes/sensors"), glob("meshes/sensors/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="saurabh-borse",
    maintainer_email="saurabh.borse@alumni.fh-aachen.de",
    description="package to spawn webots simulation",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robotino_driver = robotino_simulation.robotino_driver:main",
            "robotino_teleopctrl = robotino_simulation.robotino_teleopctrl:main",
            "mps_publisher = robotino_simulation.mps_publisher:main",
            "robotino_joyteleop = robotino_simulation.robotino_joyteleop:main",
            "robotino_laserscan_republisher = robotino_simulation.robotino_laserscan_republisher:main",
        ],
    },
)
