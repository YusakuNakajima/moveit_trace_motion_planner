# MoveIt Trace Motion Planner

Minimal MoveIt2 planning plugin for the Trace motion planner described in
`40_40_154.pdf`.

This repository currently contains implementation files only. It is written for
ROS 2 Humble / MoveIt2 Humble, but this local environment does not include ROS,
so build and runtime verification should be done in a separate ROS workspace.

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

## Example Targets

- UR5e through `ur_moveit_config`
- Franka Research 3 / FR3 through `franka_fr3_moveit_config`

The example configs are not hard dependencies of the planner package.
