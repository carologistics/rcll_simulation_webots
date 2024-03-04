
# rcll_simulation_webots
This Git repository encompasses a comprehensive collection of code files designed to establish a simulated environment in Webots, replicating the RoboCup Logistics League (RCLL) game field and incorporating Robotino robots from Festo GmbH. Additionally, the repository introduces a representative industrial production scenario. It provides essential packages for constructing detailed robot and machine descriptions, implementing sensor interfaces, developing omnidirectional robot controllers, integrating a SLAM toolbox for mapping, and deploying the Navigation2 (Nav2) stack for autonomous navigation.

Encouraging collaboration and contributions, the repository aims to serve as a reliable resource for researchers, developers, and enthusiasts in the fields of robotics and simulation. Regular updates ensure compatibility with evolving dependencies, making it a sustainable and valuable tool for the robotics community.

## Table of content
- Installation Premise
- Installation
    - Dependencies
    - Installation from source
- Building
- Spawn simulation environment
    - Spawning simulation with one instance of a robot
    - Spawning simulation with multiple instances
    - Testing the simulation
- Launch SLAM toolbox
- Launch NAV2 stack
    - Navigation with a single instance of robotinobase
    - Navigation with multiple instances of robotinobases
- Nodes and Topics to look into
- Contributing to this repo
- Research and References
- Bugs and Issues
- Way-forward

## Installation Premise
This repository has been tested on [ROS2 Humble] and with webots 2023b. It is recommended to use the same versions to avoid any issues;

These instructions assume that you have already installed ROS2 Humble on your machine. If not, please follow the recommended recommended ubuntu installation tutorial;

Create a ros2 workspace, Once you have created the workspace, clone this repository in the source folder of your workspace.

## Installation

### Installing from source:
ATTENTION: These commands assume that you have created a workspace called "simulation_ws" in your home folder. If you used a different directory or name, please adjust the commands accordingly.

After installing ROS2 and creating the workspace, clone this repository in your workspace:

    cd ~/simulation_ws/src
    git clone -b robot_cluster https://github.com/borsesaurabh2022/rcll_simulation_webots.git


    cd ~/simulation_ws/src/rcll_simulation_webots
    git clone -b main https://github.com/borsesaurabh2022/laser_scan_integrator.git

Install the binary dependencies by running the following command in the root of your workspace:

    cd ~/simulation_ws
    rosdep init
    rosdep update

    sudo apt update

    rosdep install --from-paths src/rcll_simulation_webots --ignore-src -r -y --rosdistro humble


If all dependencies are already installed, you should see the message "All required rosdeps installed successfully."

## Building:
To build the workspace, run the following command in the root of your workspace:

    cd ~/simulation_ws
    colcon build --symlink-install

If the build is successful, you should see the message "Finished [build] target(s) in [time]s."

After building the package, open a new terminal and navigate to your workspace. Then, source the overlay by running the following command:

    source /opt/ros/foxy/setup.bash

Then, source the workspace by running the following command:

    cd ~/simulation_ws
    source install/setup.bash

## Spawn simulation environmanet

### Spawning simulation with one instance of robot


    ros2 launch robotino_simulation robotino_simulation.launch.py namespace:=robotinobase1 launch_rviz:=true


- namespace: It's a launch configuration used to spawn the corresponding robotinobase(1/2/3), its controllers, and node parameters
- launch_rviz: It's a launch configuration for starting the Rviz2 with the predefined config file, parse 'false' when using nav2_stack

### Spawn simulation with multiple instances

    ros2 launch robotino_simulation robotinocluster_simulation.launch.py launch_rviz:=true

- launch_rviz: Its a launch configuration for starting the Rviz2 with the predefined config file, parse 'false' when using nav2_stack

### Testing the simulation

- Connect a joystick to your machine and operate the robot using the joystick., by default device_id associated with the joystick is 0 (no need to change for a single instance of robotinobase spawn in simulation), in case of multiple instances of robotinobase spawn in simulation, change the device_id to 0 or 1 or 2 accordingly.
- by default:
    - robotinobase1: device_id = 1
    - robotinobase2: device_id = 2
    - robotinobase3: device_id = 0

## Launch SLAM toolbox

For mapping the environment, first launch the single instance of robotinobase in simulation, ensure the joystick device is connected with correct device ID (by default, device_id=0), then launch the SLAM toolbox by running the following command in the root of your workspace:

    ros2 launch robotino_slamtoolbox robotino_slam.launch.py

Map the environment using the joystick, once the map is ready, save the map by running the following command in the root of your workspace:


    ros2 run nav2_map_server map_saver_cli -f ~/simulation_ws/src/rcll_simulation_webots/robotino_navigation/map/map

## Launch NAV2 stack

### Navigation with a single instance of robotinobase:

For autonomous navigation, first launch the single instance of robotinobase in simulation as described above, then launch the NAV2 stack by running the following command in the root of your workspace:

    ros2 launch robotino_navigation robotino_bringup.launch.py namespace:=robotinobase1

- namespace: It's a launch configuration used to spawn the map server, amcl, nav2_stack, collision monitor, and rviz2 with predefined configs for corresponding robotinobase(1/2/3)
- Initial pose id being set from 'robotinobase(1/2/3)_nav2_params.yaml'param file

Once the robot is localized, use the 2D Nav Goal tool in Rviz2 to send a goal to the robot.

### Navigation with a single instance of robotinobase:

For autonomous navigation, first launch the multiple instances of robotinobase in simulation as described above, then launch the NAV2 stack by running the following commands in different terminal instances from the root of your workspace:

    ros2 launch robotino_navigation robotino_bringup.launch.py namespace:=robotinobase1

    ros2 launch robotino_navigation robotino_bringup.launch.py namespace:=robotinobase2

    ros2 launch robotino_navigation robotino_bringup.launch.py namespace:=robotinobase3

It will launch the map server, amcl, nav2_stack, collision monitor, and rviz2 with predefined configs for the corresponding robotinobase(1/2/3). Use Rviz2 to send a goal to the corresponding robotinobase(1/2/3).

## Nodes and Topics to look into

Important nodes and topics to look into for understanding the simulation environment and its working:

## Contributing to this repo

To contribute to this package, you can either open an issue describing the desired subject or develop the feature yourself and submit a pull request to the main branch (in this case, robot_cluster).

## Research and References
- Omnidirectional robot kinematics and dynamics:
  Moreno, J.; Clotet, E.; Lupiañez, R.; Tresanchez, M.; Martínez, D.; Pallejà, T.; Casanovas, J.; Palacín, J. Design, Implementation and Validation of the Three-Wheel Holonomic Motion System of the Assistant Personal Robot (APR). Sensors 2016, 16, 1658. [Google Scholar]

  Jordi Palacín; Elena Rubies; Eduard Clotet; and David Martínez; Evaluation of the Path-Tracking Accuracy of a Three-Wheeled Omnidirectional Mobile Robot Designed as a personal Assistant https://doi.org/10.3390/s21217216

  https://github.com/mateusmenezes95/omnidirectional_controllers?tab=readme-ov-file

- [Webots](https://cyberbotics.com/)

- [ROS2](https://docs.ros.org/en/foxy/index.html)

- [Navigation2](https://navigation.ros.org/)

## Bugs and Issues

Please report bugs and request features using the Issue Tracker
