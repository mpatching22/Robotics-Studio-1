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


        ign topic -e -t /world/simple_trees/pose/info -n 1