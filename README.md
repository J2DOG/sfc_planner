## sfc_planner

`sfc_planner` is a ROS 1 package for global planning and navigation of a UAV based on Safe Flight Corridors (SFCs).  
It consumes a voxel map, generates SFCs and trajectories, and sends targets to a low‑level `tracking_controller`, which in turn drives PX4.

Below is a **minimal step‑by‑step guide to go from zero to a running demo**.

---

### 0. quick start in 3 mins

```bash
# 1) Put this package into an existing workspace
cd ~/catkin_ws/src
git clone https://github.com/J2DOG/sfc_planner.git

# 2) Make sure dependencies are also in src or installed:
#    - tracking_controller and uav_simulator from CERLAB-UAV-Autonomy:
#        https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy.git
#    - polys_mapgen:
#        https://github.com/J2DOG/polys_mapgen.git
#    - plus all of their required dependencies

# 3) Build and source
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# 4) Launch the full system (controller + map + RViz + sfc_planner)
roslaunch uav_simulator px4_start.launch
roslaunch sfc_planner sfc_planning.launch
```

In RViz, use **2D Nav Goal** to click a target; if topics/params are correct, the UAV will plan and follow a trajectory.

---

### 1. Environment and dependencies

- **ROS**: ROS 1 (tested with a Melodic/Noetic‑style catkin workspace)
- **Core dependencies** (see `package.xml`):
  - `roscpp`, `std_msgs`, `nav_msgs`, `geometry_msgs`, `sensor_msgs`
  - `mavros_msgs`
  - `tracking_controller` (from the CERLAB UAV Autonomy framework)
  - `polys_mapgen` (for voxel map generation)
- **Build tools**:
  - A standard catkin workspace, e.g. `~/catkin_ws`

#### 1.1 Create / use a catkin workspace

If you do not have a workspace yet:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

Clone this repo into `src` (skip if already copied there):

```bash
cd ~/catkin_ws/src
git clone https://github.com/J2DOG/sfc_planner.git
```

Then make sure other required packages (`tracking_controller`, `polys_mapgen` and their dependencies) are also under `~/catkin_ws/src`,  
or installed via system packages or other means.

#### 1.2 Build and setup environment

```bash
cd ~/catkin_ws
catkin_make
```

After a successful build, source the workspace (recommended to add to `~/.bashrc` or your shell init script):

```bash
source ~/catkin_ws/devel/setup.bash   # or setup.zsh
```

---

### 2. Configuration

#### 2.1 Global planning parameters (`global_planning.yaml`)

Main planning parameters are in `cfg/global_planning.yaml`, for example:

- **map_topic**: voxel map topic (default `"/voxel_map"`)
- **target_topic**: RViz goal topic (default `"/move_base_simple/goal"`)
- **mavros_state_topic**: MAVROS state topic
- **odom_topic**: odometry topic (default `"/mavros/local_position/odom"`)
- **voxel_width**: voxel map resolution
- **map_bound**: `[x_min, x_max, y_min, y_max, z_min, z_max]` (meters)
- **dilate_radius**: obstacle inflation radius
- **vehicle_mass, grav_acc, horiz_drag, vert_drag, paras_drag, speed_eps**: vehicle dynamics related parameters
- **des_height**: desired flight height (used for takeoff and goal z coordinate)

When deploying to a new environment, **at minimum check / adjust**:

- `map_bound`: covers your real flight area
- `des_height`: is safe and appropriate

`sfc_navigation_node` reads these parameters from its private namespace.

#### 2.2 Controller and map generator parameters

- `tracking_controller` parameters are loaded from:
  - `$(find tracking_controller)/cfg/controller_param.yaml`
- `polys_mapgen` parameters are loaded from:
  - `$(find polys_mapgen)/cfg/mapgen.yaml`

Ensure these YAML files exist and match your vehicle and map source configuration.

---

### 3. Launching and running (recommended flow)

Launch the px4 SITL gazebo first

```bash
roslaunch uav_simulator px4_start.launch
```

In a terminal where you have already run `source ~/catkin_ws/devel/setup.bash`:

```bash
roslaunch sfc_planner sfc_planning.launch
```

This launch file will:

- **Start the tracking controller**:
  - Node: `tracking_controller_node` in package `tracking_controller`
- **Start RViz**:
  - Using config: `rviz/sfc_navigation.rviz`
- **Start the map generator**:
  - Node: `hpolys_mapgen_node` in package `polys_mapgen`
  - Loads `polys_mapgen/cfg/mapgen.yaml`
  - Publishes a voxel map on `/voxel_map` (remapped from `/polys_pc_map`)
- **Start the SFC navigation node**:
  - Node: `sfc_navigation_node` in package `sfc_planner`
  - Loads parameters from `cfg/global_planning.yaml`

Before launching, make sure the following are already running and publishing:

- `roscore`
- MAVROS (or a simulator) publishing:
  - `/mavros/local_position/odom`
  - `/mavros/state`

---

### 4. Usage steps (interactive flow)

1. **Launch the system**
   ```bash
   roslaunch sfc_planner sfc_planning.launch
   ```
2. Watch the console output and wait until odometry, MAVROS state, and voxel map are initialized and the vehicle has taken off to `des_height`.
3. In RViz, select the **2D Nav Goal** tool and click a target position (x, y) in the `map` / `odom` frame:
   - The node lifts the goal to height `des_height` (z)
   - Checks for collisions in the voxel map
   - Plans a path and generates SFCs
   - Sends the trajectory / targets to the tracking controller
4. If the clicked goal lies **inside an obstacle**, the node will warn in the console and ask you to choose another goal.

---

### 5. Deployment notes / tips

- The package assumes a vehicle controlled via MAVROS (e.g. PX4 in `OFFBOARD` mode, arming via MAVROS services).
- On real hardware, make sure safety boundaries and failsafes are properly configured on the flight controller, and always test in a safe environment.
