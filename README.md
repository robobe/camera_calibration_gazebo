
# install

```
sudo apt install ros-humble-camera-calibration
```


## move entity

```
ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState "state: {name: landmark, \
pose: {position:{x: 1.0, y: 0.0, z: 2.0}}, \
reference_frame: world}"
```

```
ros2 run camera_calibration cameracalibrator \
--size 7x9 \
--square 0.03 \
--ros-args -r image:=image_raw

```

# Reference
- [calibration_gazebo](https://github.com/oKermorgant/calibration_gazebo/)
- [wiki](https://robobe.github.io/blog/ROS2/demos/camera_calibration/)