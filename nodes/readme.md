# Flight Control

1. Takeoff
ros2 topic pub /cmd/control std_msgs/msg/String "data: 'takeoff'"

2. Send Move to Goal Intent
ros2 topic pub /cmd/control std_msgs/msg/String "data: 'move_to_goal'"

3. Publish Position Goal
ros2 topic pub /cmd/goal geometry_msgs/msg/PointStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, point: {x: 2, y: 2, z: 2}}"

4. Adjust Altitude (Optional)
ros2 topic pub /cmd/height std_msgs/msg/Float32 "{data: TARGET_Z}"

5. Land
ros2 topic pub /cmd/control std_msgs/msg/String "data: 'land'"

6. Hover (Hold Position)
ros2 topic pub /cmd/control std_msgs/msg/String "data: 'hover'"

# Launching mission/world

cd ~/trailblazer
colcon build --symlink-install
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch trailblazer mission.launch.py rviz:=False nav2:=True world:=simple_trees.sdf