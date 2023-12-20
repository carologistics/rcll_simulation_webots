from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robotino3_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
        (os.path.join('share', package_name, 'protos'), glob('protos/*')),
        (os.path.join('share', package_name, 'protos/machine_base'), glob('protos/machine_base/*')),
        #(os.path.join('share', package_name, 'protos/icons'), glob('protos/icons/*')),
        (os.path.join('share', package_name, 'urdf/components'), glob('urdf/components/*')),
        (os.path.join('share', package_name, 'urdf/robots'), glob('urdf/robots/*')),
        (os.path.join('share', package_name, 'urdf/sensors'), glob('urdf/sensors/*')),
        (os.path.join('share', package_name, 'meshes/parts'), glob('meshes/parts/*')),
        (os.path.join('share', package_name, 'meshes/sensors'), glob('meshes/sensors/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saurabh-borse',
    maintainer_email='saurabh.borse@alumni.fh-aachen.de',
    description='package to spawn webots simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robotino3_driver = robotino3_simulation.robotino3_driver:main',
            'robotino3_teleopctrl = robotino3_simulation.robotino3_teleopctrl:main',
            'robotino3_mpspublisher = robotino3_simulation.robotino3_mpspublisher:main',
            'robotino3_joyteleop = robotino3_simulation.robotino3_joyteleop:main',
            'robotino3_laserscan_republisher = robotino3_simulation.robotino3_laserscan_republisher:main',
        ],
    },
)
