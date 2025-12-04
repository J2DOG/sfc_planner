## sfc_planner

`sfc_planner` is a ROS package for safe flight corridor (SFC) based global planning and navigation for a MAV. It takes a voxel map, generates SFCs and trajectories, and sends targets to a low‑level tracking controller.

⚠️ **IMPORTANT:** This package is under frequent development and updates. Features and behaviors may change without notice. Use with caution and always check for the latest changes before deploying in production systems.

### 1. Dependencies and environment

- **ROS**: ROS 1 (tested on melodic/noetic style catkin package layout)
- **Core deps** (from `package.xml`):
  - `roscpp`, `std_msgs`, `nav_msgs`, `geometry_msgs`, `sensor_msgs`
  - `mavros_msgs`
  - `tracking_controller`
  - `polys_mapgen` (used by the launch file)
- **Build tools**:
  - `catkin` and a typical catkin workspace, e.g. `~/catkin_ws`

#### 1.1. Create / use a catkin workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

Clone / copy this repo into `src` (if not already there):

```bash
cd ~/catkin_ws/src
git clone <your_sfc_planner_repo_url> sfc_planner
```

Make sure the other required packages (`tracking_controller`, `polys_mapgen` and their dependencies) are also in `~/catkin_ws/src` or installed from binaries.

#### 1.2. Build the workspace

```bash
cd ~/catkin_ws
catkin_make
```

Source the workspace (add this to your shell init if you like):

```bash
source devel/setup.bash   # or setup.zsh
```

### 2. Configuration

#### 2.1 Global planning parameters

The main configuration is in `cfg/global_planning.yaml`:

- **map_topic**: voxel map topic (default `"/voxel_map"`)
- **target_topic**: RViz goal topic (default `"/move_base_simple/goal"`)
- **mavros_state_topic**: MAVROS state topic
- **odom_topic**: odometry topic (default `"/mavros/local_position/odom"`)
- **voxel_width**: resolution of the voxel map
- **map_bound**: `[x_min, x_max, y_min, y_max, z_min, z_max]` (in meters)
- **dilate_radius**: obstacle inflation radius
- **vehicle_mass, grav_acc, horiz_drag, vert_drag, paras_drag, speed_eps**
- **des_height**: desired flight height used for takeoff and goal z‑coordinate

Adjust `map_bound` and `des_height` to match your environment; the `sfc_navigation_node` reads these parameters from the private namespace via this YAML file.

#### 2.2 Controller and map generator

- `tracking_controller` parameters are loaded from:
  - `$(find tracking_controller)/cfg/controller_param.yaml`
- `polys_mapgen` parameters are loaded from:
  - `$(find polys_mapgen)/cfg/mapgen.yaml`

Make sure these YAML files exist and are correctly configured for your vehicle and map source.

### 3. Running the planner

In a sourced terminal (`source ~/catkin_ws/devel/setup.bash`):

```bash
roslaunch sfc_planner sfc_planning.launch
```

This launch file will:

- **Start the tracking controller**:
  - Node: `tracking_controller_node` in package `tracking_controller`
- **Start RViz** with the provided configuration:
  - Uses `rviz/sfc_navigation.rviz`
- **Start the map generator**:
  - Node: `hpolys_mapgen_node` in package `polys_mapgen`
  - Loads `polys_mapgen/cfg/mapgen.yaml`
  - Publishes the voxel map on `/voxel_map` (remapped from `/polys_pc_map`)
- **Start the SFC navigation node**:
  - Node: `sfc_navigation_node` in package `sfc_planner`
  - Loads parameters from `cfg/global_planning.yaml`

Ensure `roscore`, MAVROS, and any simulator or real robot interface are already running so that:

- `/mavros/local_position/odom`
- `/mavros/state`

are being published before takeoff/planning.

### 4. Using the system

1. Launch the system:
   ```bash
   roslaunch sfc_planner sfc_planning.launch
   ```
2. Wait for the console message indicating that odometry, MAVROS state, and voxel map are initialized and that the vehicle has taken off to `des_height`.
3. In RViz, use the **2D Nav Goal** tool on the `map`/`odom` frame to click a goal position (x, y). The planner will:
   - Lift the goal to `des_height` (z)
   - Check collision in the voxel map
   - Plan a path and generate SFCs
   - Send the trajectory/targets to the tracking controller

If the clicked goal is inside an obstacle, the node will warn and ask you to choose another goal.

### 5. Notes

- The planner assumes a quadrotor‑like vehicle controlled through MAVROS (`OFFBOARD` mode, arming through MAVROS services).
- Make sure safety boundaries and failsafes are configured on the flight controller when running on a real robot.
- For development, you can run each node separately using the same parameter files if you prefer finer‑grained control.


