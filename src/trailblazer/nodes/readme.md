1. Takeoff
ros2 topic pub /cmd/control std_msgs/msg/String "data: 'takeoff'"

2. Send Move to Goal Intent
ros2 topic pub /cmd/control std_msgs/msg/String "data: 'move_to_goal'"

3. Publish Position Goal
ros2 topic pub /cmd/goal geometry_msgs/msg/PointStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, point: {x: TARGET_X, y: TARGET_Y, z: TARGET_Z}}"

4. Adjust Altitude (Optional)
ros2 topic pub /cmd/height std_msgs/msg/Float32 "{data: TARGET_Z}"

5. Land
ros2 topic pub /cmd/control std_msgs/msg/String "data: 'land'"

6. Hover (Hold Position)
ros2 topic pub /cmd/control std_msgs/msg/String "data: 'hover'"
