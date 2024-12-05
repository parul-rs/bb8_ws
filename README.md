# Control and Simulation of BB8 In Gazebo Using CasADi for Path Optimization
**Authors: Parul Singh, Tiago Gunter, Kelsi St John, Luis Luna**  
This is our final project for ME396. We simulated a robot model for BB8 in Gazebo and controlled him using commands which we optimized using the Python package CasADi. For the optimization demos, BB8 is given a series of Cartesian points to go to and plans a path through those points while optimizing for control effort (minimizing changes in control commands such as motor torque required to move). 

---

## Table of Contents
1. [Installation](#installation)
2. [Usage](#usage)
3. [Notes](#notes)

---

## Installation

### Prerequisites
Requires ROS2 and Gazebo to be installed
Compatible with ROS2 Humble Hawksbill and Gazebo11 (NOT Ignition)
To use gazebo with ROS2: sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

### Steps
1. Clone the repository:
   Using ssh (recommended):
   ```bash
   git clone git@github.com:parul-rs/bb8_ws.git
   ```
   Using HTTPS:  
   ```bash
   git clone https://github.com/parul-rs/bb8_ws.git  
2. In the bb8_ws directory, run
   ```bash
   source /opt/ros/humble/setup.bash
   source /usr/share/gazebo/setup.sh
   colcon build --symlink-install
   source install/setup.bash  
You should be ready to go!

---

## Usage
To run the live demo from our presentation (BB8 optimizing a path through four points that make a square), run the following in the bb8_ws directory:
```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
ros2 launch bb8 spawn_bb8_launch.py
```
This assumes you already ran colcon build as shown in the Installation section.

If you would like to run other demos, go into bb8_ws/src/bb8/launch/spawn_bb8_launch.py, and change the name of the executable in the last node:
```bash
        Node(
            package='bb8',                     
            executable='test_opti_7', # Change to script name you'd like to run
            name='test_command_node', 
            output='screen',          
            parameters=[{'use_sim_time': False}], 
        ),
```
For example, you could replace 'test_opti_7' with 'test_opti_G' and run the launchfile the same way as before to see BB8 optimize a path through points defined to draw a "G". 

---

## Notes
Here is the link to our presentation on this project: https://docs.google.com/presentation/d/1UQcif6uCPB8I5UM3CflaLZQLBjymNcfwrQaePBxO8R8/edit?usp=sharing  
List of executables you can run (as described in the Usage section) and what they do: 
1. test_opti_3: see BB8 simply move forwards and backwards
2. test_opti_7: see BB8 optimize a path through points defined to draw a square 
3. test_opti_G: see BB8 optimize a path through points defined to draw a "G"
4. test_opti_R: see BB8 optimize a path through points defined to draw a "R"
5. test_opti_O: see BB8 optimize a path through points defined to draw a "O"
6. test_opti_T: see BB8 optimize a path through points defined to draw a "T"  

Please note that the mesh files in bb8_ws/src/bb8/meshes were taken from a BB8 simulation in The Construct Sim, and the .xacro files in bb8_ws/src/bb8/urdf as well, though those have been heavily edited to work for ROS2 and Gazebo11 and other edits pertaining to our particular use case and dynamics. Source: https://github.com/parul-rs/bb8 (see README). 
