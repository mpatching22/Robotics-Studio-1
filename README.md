Build the workspace replace rs1 with your directory
    cd ~/rs1
    colcon build --symlink-install
    source install/setup.bash

Launch the drone
    ros2 launch trailblazer mission.launch.py

With SLAM + Navigation + RViz + small demo 
    ros2 launch trailblazer mission.launch.py slam:=true nav2:=true rviz:=true world:=simple_trees gui:=true

Simple demo with GUI
    ros2 launch trailblazer mission.launch.py world:=simple_trees gui:=true

With SLAM + Navigation + RViz + Large demo (change to simple_trees for smaller enviroment)
    ros2 launch trailblazer mission.launch.py slam:=true nav2:=true rviz:=true world:=large_demo gui:=true

Lachlan's Enviroment
    ros2 launch trailblazer mission.launch.py world:=test_terrain

Run just the GUI
    python3 gui.node.py

Installs Required

    Install all at once
        pip install -r requirements.txt

    GUI library
        pip install pyside6

Extra's
    To access the file explorer from the current wsl directory
        explorer.exe .

    View all ros topics avaliable
        ros2 topic list

    To kill stray gazebo instances
        ps aux | grep -E "ign|gz" | grep -v grep

        kill 223753 223755   # or:  pkill -f ros_gz_bridge

        pkill -f "__ns:=/rs1"

        ros2 daemon stop
        ros2 daemon start

        ign topic -e -t /world/simple_trees/pose/info -n 1


## Micah's Readme

# Flight Control

1. Takeoff
ros2 topic pub /cmd/control std_msgs/msg/String "data: 'takeoff'"

2. Send Move to Goal Intent
ros2 topic pub /cmd/control std_msgs/msg/String "data: 'move_to_goal'"

3. Publish Position Goal
ros2 topic pub /cmd/goal geometry_msgs/msg/PointStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, point: {x: 4, y: 3, z: 2}}"

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
ros2 launch trailblazer mission.launch.py rviz:=False nav2:=True world:=test_terrain.sdf


rm -rf ~/.ignition ~/.gazebo ~/.gz