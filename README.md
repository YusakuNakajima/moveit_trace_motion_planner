# MoveIt Trace Motion Planner

Minimal MoveIt2 planning plugin for the Trace motion planner described in the paper:

Masanori Sekiguchi and Naoyuki Takesue, "Motion Planning for Redundant
  Articulated Robots Based on Geometrical Properties of the Whole Robot Body in
  Initial and Goal Configuration", Journal of the Robotics Society of Japan,
  40(2), 154-161, 2022. https://doi.org/10.7210/jrsj.40.154

## Implemented Scope

- MoveIt2 `planning_interface::PlannerManager` plugin.
- Planner id: `TraceMotion`.
- Direct start-to-goal joint interpolation with collision checking.
- Trace waypoint fallback generated from start and goal robot shapes.
- MoveIt standard full-pose IK for goal and intermediate waypoints.
- Shift motion configuration is present but disabled by default.

The v0 implementation is deliberately not a full reproduction of the paper. In
particular, position-plus-direction IK and prioritized IK are later milestones.

## Package Layout

```text
include/moveit_trace_motion_planner/
src/
config/trace_planner.yaml
examples/ur5e/
examples/fr3/
trace_motion_planner_plugin.xml
CMakeLists.txt
package.xml
```

## Build in a ROS Workspace

From a ROS 2 Humble workspace:

```bash
colcon build --packages-select moveit_trace_motion_planner
```

Expected plugin name:

```text
moveit_trace_motion_planner/TracePlannerManager
```

## Example Launch

UR5e without a real robot uses fake hardware in one terminal and MoveIt in a
second terminal:

```bash
ros2 launch moveit_trace_motion_planner trace_ur5e_fake_hardware.launch.py
```

Then:

```bash
ros2 launch moveit_trace_motion_planner trace_ur5e.launch.py ur_type:=ur5e launch_rviz:=true
```

The UR5e fake hardware launch activates `scaled_joint_trajectory_controller`
by default because the upstream UR MoveIt config sends trajectories to that
controller.

FR3 without a real robot:

```bash
ros2 launch moveit_trace_motion_planner trace_fr3.launch.py robot_ip:=dont-care use_fake_hardware:=true
```

The launch files overlay this package's Trace planner parameters on top of the
upstream robot MoveIt launch files. They have not been executed in this local
environment because ROS 2 is not installed here.

In RViz, use the MotionPlanning panel with planning pipeline
`trace_motion_planner` and planner id `TraceMotion`.

## Example Targets

- UR5e through `ur_moveit_config`
- Franka Research 3 / FR3 through `franka_fr3_moveit_config`

The example configs are not hard dependencies of the planner package.

## References

- Masanori Sekiguchi and Naoyuki Takesue, "Motion Planning for Redundant
  Articulated Robots Based on Geometrical Properties of the Whole Robot Body in
  Initial and Goal Configuration", Journal of the Robotics Society of Japan,
  40(2), 154-161, 2022. https://doi.org/10.7210/jrsj.40.154
