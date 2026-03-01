colcon build --packages-select aidguide_04_provide_map
source install/setup.bash
ros2 launch aidguide_04_provide_map aidguide_04_provide_map.launch.py

ros2 launch aidguide_04_provide_map waypoints_navigation.launch.py

Luego le damos pose estimated 
