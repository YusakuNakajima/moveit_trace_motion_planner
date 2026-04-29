# UR5e Example

This example targets `ur_moveit_config` on ROS 2 Humble.

Use `config/trace_motion_planning.yaml` as an additional planning pipeline
configuration in a normal UR5e MoveIt launch. The planner plugin is:

```yaml
planning_plugin: moveit_trace_motion_planner/TracePlannerManager
planner_id: TraceMotion
```

Shift motion is intentionally disabled for this 6-DOF example.

From an installed ROS 2 Humble workspace:

```bash
ros2 launch moveit_trace_motion_planner trace_ur5e.launch.py ur_type:=ur5e launch_rviz:=true
```

The launch file includes `ur_moveit_config/ur_moveit.launch.py` and overlays
`config/trace_motion_planning.params.yaml`.
