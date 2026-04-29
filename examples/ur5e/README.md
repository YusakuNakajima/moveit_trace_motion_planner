# UR5e Example

This example targets `ur_moveit_config` on ROS 2 Humble.

Use `config/trace_motion_planning.yaml` as an additional planning pipeline
configuration in a normal UR5e MoveIt launch. The planner plugin is:

```yaml
planning_plugin: moveit_trace_motion_planner/TracePlannerManager
planner_id: TraceMotion
```

Shift motion is intentionally disabled for this 6-DOF example.
