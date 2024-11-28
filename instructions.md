# Instructions

## Test URDF in RVIZ
### Terminal 1:
```bash
ros2 launch urdf_tutorial display.launch.py model:=/root/projects/bb8_ws/src/bb8/urdf/bb8.urdf
```
### Terminal 2:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_direction_link
```


## Test URDF in Gazebo through ROS
### Terminal 1:
```bash
ros2 launch gazebo_ros gazebo.launch.py
```
### Terminal 2:
```bash
ros2 run gazebo_ros spawn_entity.py -entity bb8 -file /root/projects/bb8_ws/src/bb8/urdf/bb8.urdf -x 0 -y 0 -z 1
```

## Test URDF Directly through Gazebo
### Terminal 1:
```bash
gazebo /root/projects/bb8_ws/src/bb8/urdf/test_bb8.urdf
```