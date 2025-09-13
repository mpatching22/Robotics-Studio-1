Build the workspace replace rs1 with your directory

cd ~/rs1
colcon build --symlink-install
source install/setup.bash

Launch the drone
ros2 launch trailblazer mission.launch.py

With SLAM + Navigation + RViz + Large demo (change to simple_trees for smaller enviroment)


Installs Required

Install all at once
pip install -r requirements.txt

GUI library
pip install pyside6


Extra's
To access the file explorer from the current wsl directory
explorer.exe .

 ros2 topic list

