# nav_robot_cpp

C++ Nav2 plugins and nodes for [nav_robot](https://github.com/darshmenon/nav_robot). Small on purpose: each one is a readable template for writing the same kind of plugin.

| Piece | Type | What it does |
|---|---|---|
| `scan_watchdog` | node | errors when `/scan` is silent or every beam is `inf` (a blind lidar makes Nav2 report goals as succeeded) |
| `ScanHealthy` | behavior tree condition | fails the navigation tree into its recovery behaviors when the lidar is blind or silent |
| `nav_robot_cpp/PurePursuitController` | `nav2_core::Controller` | pure pursuit for a differential drive, with in-place turns and goal slow-down |
| `nav_robot_cpp/StraightLinePlanner` | `nav2_core::GlobalPlanner` | straight line from start to goal, refused when it crosses an occupied cell |
| `nav_robot_cpp/LogWaypointTask` | `nav2_core::WaypointTaskExecutor` | logs the arrival and pauses at each waypoint |

## Build and use

```bash
cd ~/nav_ws && colcon build --symlink-install --packages-select nav_robot_cpp
colcon test --packages-select nav_robot_cpp          # 4 unit tests of the pure logic

ros2 launch nav_robot nav.launch.py \
  controller:=pure_pursuit planner:=straight_line \
  waypoint_task:=log bt:=health cpp_watchdog:=true
```

Every option defaults to the Python or stock Nav2 behavior, so nothing changes unless you ask.

## Checked in the sim

Office world, Gazebo Harmonic, ROS 2 Humble:
- Pure pursuit controller plus Smac planner drove the 5-waypoint patrol and docked (31.5 m).
- `LogWaypointTask` logged all 5 arrivals.
- `ScanHealthy` tree loaded and ran with no false alarms; the C++ watchdog stayed quiet.
- `StraightLinePlanner` reached a goal on a clear line and refused a line through a wall ("straight line is blocked").

## Limits

- The controller does not avoid obstacles by itself (no costmap check). It relies on the planner and the collision monitor.
- The straight-line planner never goes around anything, so it only suits open rooms.
- Humble loads waypoint task executors from the `nav2_waypoint_follower` export list, so they are exported under that package, not `nav2_core`.
- `ScanHealthy` only failing the tree was not tested with a blind lidar in the sim, only the watchdog was (with a fake all-`inf` scan).
