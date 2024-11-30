colcon build


source install/setup.bash

ros2 launch mecbot_description visualize_robot_simulation.launch.py world:=$(ros2 pkg prefix mecbot_description)/share/mecbot_description/worlds/close_scene.world

