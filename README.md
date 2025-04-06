# hackaton_smtu_2024


# mecbot_description
This repository is for mecbot robot related urdf model

This repository is a fully recycled repository https://github.com/santoshbalaji/haruto_description

## Dependencies involved
- [ ] robot_state_publisher
- [ ] joint_state_publisher_gui

## Install dependencies
```
  rosdep install --from-paths src/mecbot_description -y --ignore-src
```

## Build and Execution
- To build the package

```
  colcon build --packages-select mecbot_description
```

- To visualize the robot in rviz
```
  ros2 launch mecbot_description visualize_robot_standalone.launch.py
```

- To visualize the robot in gazebo and rviz (for omnidirectional robot)
```
  ros2 launch mecbot_description visualize_robot_simulation.launch.py robot_type:=omni
```

- To visualize the robot in gazebo and rviz (for differential robot)
```
  ros2 launch mecbot_description visualize_robot_simulation.launch.py robot_type:=diff
```


