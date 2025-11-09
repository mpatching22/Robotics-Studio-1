# Trailblazer Robotics Project
Autonomous Drone Navigation using ROS 2 Humble, Nav2, and Custom Terrain-Following Altitude Control

---

## 6.1 Code Access

The final version of the Trailblazer project is publicly available on GitHub:  
**https://github.com/jaydenhazell/trailblazer-final**

If private access is required, permission has been granted to:  
- **Subject Coordinator:** graeme.best@uts.edu.au  
- **Tutor:** [insert tutor email here]

Alternatively, the final code can be uploaded to Canvas as a `.zip` file containing the complete `trailblazer` package (without git history).

---

## 6.2 Installation Instructions

ROS 2 Humble and Gazebo Fortress must be installed prior to running the project.

### Build the Workspace
Replace `rs1` with your chosen directory name:

```bash
cd ~/rs1
colcon build --symlink-install
source install/setup.bash
```

### Install Python Requirements
All required Python dependencies can be installed using:

```bash
pip install -r requirements.txt
```

Or install manually if preferred:

```bash
pip install pyside6 opencv-python matplotlib numpy pandas scipy tqdm requests
```

### Optional: Install GUI Library Only

```bash
pip install pyside6
```

---

## 6.3 Running Instructions

The system can be launched entirely through the `mission.launch.py` file, which starts the drone, SLAM, Nav2, GUI, and optional RViz visualization.

### Default Run (Full Test World)

```bash
ros2 launch trailblazer mission.launch.py
```

### With RViz (Camera, Depth Cloud, and 360° LiDAR Scan)

```bash
ros2 launch trailblazer mission.launch.py rviz:=true
```

### Flat Empty World

```bash
ros2 launch trailblazer mission.launch.py world:=simple_trees
```

### Medium Size Demo World

```bash
ros2 launch trailblazer mission.launch.py world:=large_demo
```

### Full Test World Without Trees

```bash
ros2 launch trailblazer mission.launch.py world:=final_terrain
```

---

### Expected Output

1. **Gazebo** opens with the selected world and drone model.  
2. **RViz2** shows the SLAM map, costmaps, and LiDAR scans if enabled.  
3. **GUI** window appears with command buttons for *Takeoff*, *Move to Goal*, *Hover*, *Land*, and *Emergency Land*.  
4. **Flight Control Node** sends XY goals to Nav2 and manages Z altitude control automatically.  
5. Logs and trace CSV files are created under:
   - `~/.ros/trailblazer_logs/`
   - `data/` for ground traces
   - `maps/` for generated map outputs

---

### Common Issues and Fixes

| Issue | Cause | Fix |
|-------|--------|-----|
| Gazebo spawns empty | Incorrect world argument | Use `world:=simple_trees` |
| Drone does not move | Nav2 not yet active | Wait for status to change to “Landed” |
| GUI does not open | PySide6 missing | Install using `pip install pyside6` |
| Costmap errors | Outdated obstacles | Z-clear watchdog triggers automatically when Δz > 0.4 m |
| RViz missing map | SLAM not started | Ensure `slam:=true` or use default mission command |

---

## 6.4 Key Parameters

### Flight Control Node (`nodes/flight_control.py`)

| Parameter | Default | Description |
|------------|----------|-------------|
| `control_rate_hz` | 35.0 Hz | Main control loop frequency. |
| `max_z_up` | 3.0 m/s | Maximum climb speed. |
| `max_z_down` | 1.0 m/s | Maximum descent speed. |
| `kp_z`, `ki_z` | 0.65 / 0.20 | Z-axis PID controller gains. |
| `pos_tol_xy` | 0.25 m | XY goal tolerance. |
| `pos_tol_z` | 0.15 m | Z altitude tolerance. |
| `z_clear_thresh_m` | 0.4 m | Δz threshold triggering local costmap clear. |
| `z_clear_window_sec` | 0.75 s | Time window for Δz evaluation. |
| `z_clear_cooldown_sec` | 2.0 s | Minimum interval between costmap clears. |
| `z_clear_global_enable` | False | Enables global costmap clearing on large Δz. |
| `z_clear_global_mult` | 2.0 | Multiplier for global clear Δz threshold. |

These parameters are declared in `flight_control.py` and can be edited directly or overridden through ROS launch parameters.

---

### Navigation Parameters (`config/nav2_params.yaml`)

| Parameter | Description | Default / Example |
|------------|--------------|-------------------|
| `controller_server.max_vel_x` / `min_vel_x` | XY speed limits | 1.0 / 0.0 |
| `planner_server.tolerance` | Goal arrival tolerance | 0.25 |
| `controller_frequency` | Nav2 control rate | 20.0 |
| `use_sim_time` | Use simulation clock | true |

---

### SLAM Toolbox Parameters (`config/slam_params.yaml`)

| Parameter | Description | Default / Example |
|------------|--------------|-------------------|
| `map_update_interval` | Time between map updates | 2.0 s |
| `resolution` | Map cell size | 0.05 m |
| `max_laser_range` | Maximum LiDAR range | 10.0 m |

---

## Useful Utilities

```bash
# Open current WSL directory in Windows Explorer
explorer.exe .

# List all available ROS 2 topics
ros2 topic list

# Kill stray Gazebo or Ignition processes
ps aux | grep -E "ign|gz" | grep -v grep
kill <pids>  # or:
pkill -f ros_gz_bridge
pkill -f "__ns:=/rs1"

# Reset ROS 2 daemon
ros2 daemon stop
ros2 daemon start

# Remove Gazebo and Ignition cache
rm -rf ~/.ignition ~/.gazebo ~/.gz
```

---

## Manual Testing Commands (Optional)

| Action | Command |
|---------|----------|
| **Takeoff** | `ros2 topic pub /cmd/control std_msgs/msg/String "data: 'takeoff'"` |
| **Move to Goal** | `ros2 topic pub /cmd/control std_msgs/msg/String "data: 'move_to_goal'"` |
| **Publish Goal** | `ros2 topic pub /cmd/goal geometry_msgs/msg/PointStamped "{header:{frame_id:'map'},point:{x:4,y:3,z:2}}"` |
| **Adjust Height** | `ros2 topic pub /cmd/height std_msgs/msg/Float32 "{data:1.2}"` |
| **Land** | `ros2 topic pub /cmd/control std_msgs/msg/String "data: 'land'"` |
| **Hover** | `ros2 topic pub /cmd/control std_msgs/msg/String "data: 'hover'"` |

---

## Expected Visual Output

| Window | Description |
|---------|-------------|
| **Gazebo** | Displays the simulation environment and drone model. |
| **RViz2** | Shows the SLAM map, costmaps, and 360° LiDAR scan. |
| **GUI** | Provides live status updates, control buttons, and map generation tools. |
| **Terminal** | Displays node logs and flight state transitions (e.g., Pre-Flight → Takeoff → Move to Goal → Hover → Land). |

---

## Notes for Assessors

- The project runs fully from a single command:

```bash
ros2 launch trailblazer mission.launch.py
```

- SLAM, Nav2, and the GUI are included by default.  
- All code is documented using Doxygen-style comments for clarity.  
- The `data/` and `maps/` folders remain empty by default but are automatically populated during simulation and map generation.  
- CSV trace logs are stored in `data/`, and generated maps are stored in `maps/`.
