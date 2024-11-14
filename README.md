# Mobile Manipulator Gazebo Simulation

Extension to the [mobile manipulator project
code](https://github.com/utiasDSL/mm_diff_ik) for running Gazebo simulations.

**This repository is no longer maintained.**

## Dependencies

The UR packages are not available from apt, so clone from source into the
workspace:
```
git clone https://github.com/ros-industrial/universal_robot
```

Robotiq must also be cloned:
```
git clone https://github.com/ros-industrial/robotiq
```

Ridgeback packages can be installed from apt:
```
sudo apt install ros-<distro>-ridgeback-simulator ros-<distro>-ridgeback-control ros-<distro>-ridgeback-description
```

We also need some generic control infrastructure:
```
sudo apt install ros-<distro>-ros-control ros-<distro>-ros-controllers ros-<distro>-gazebo-ros-control
```

## Usage
Run `roslaunch mm_gazebo simulation.launch`.

## Packages
* `mm_description`: URDF for the robot.
* `mm_gazebo`: Launch files for setting up and running the simulation.
* `mm_gazebo_interface`: Sets up control interfaces for the Gazebo simulation.
