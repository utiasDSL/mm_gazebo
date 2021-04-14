# mm_gazebo_simulation

Extension to the [mm project
code](https://github.com/utiasDSL/dsl__projects__mobile_manipulator) for
running Gazebo simulations, based on work from the original [thing
repository](https://github.com/utiasSTARS/thing).

## Dependencies
Install apt dependencies:
```
sudo apt install ros-indigo-ur-description ros-indigo-ridgeback-gazebo-plugins
```

Clone robotiq (not available as an apt package):
```
git clone https://github.com/ros-industrial/robotiq
cd robotiq
git checkout indigo-devel
```

Clone ridgeback, since we need a specific version. Do not install via apt!
```
git clone https://github.com/ridgeback/ridgeback
cd ridgeback
git checkout 0.1.6
```

## Usage
Run `roslaunch mm_gazebo simulation.launch`.

## Packages
* `mm_description`: URDF for the robot. Adapted from `thing_description`.
* `mm_gazebo`: Launch files for setting up and running the simulation. Adapted
  from `thing_gazebo`.
* `mm_gazebo_interface`: Sets up control interfaces for the Gazebo simulation.
  Partially adapted from `thing_control`.
