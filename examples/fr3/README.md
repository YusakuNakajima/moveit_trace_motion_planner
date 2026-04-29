# Franka Research 3 Example

This example targets `franka_fr3_moveit_config` on ROS 2 Humble.

Use `config/trace_motion_planning.yaml` as an additional planning pipeline
configuration in a normal FR3 MoveIt launch. The planner plugin is:

```yaml
planning_plugin: moveit_trace_motion_planner/TracePlannerManager
planner_id: TraceMotion
```

FR3 has redundancy, but Shift motion remains disabled by default in v0. The
configuration keeps Shift parameters in place so later experiments can turn it
on after the implementation is added.

From an installed ROS 2 Humble workspace:

```bash
ros2 launch moveit_trace_motion_planner trace_fr3.launch.py robot_ip:=dont-care use_fake_hardware:=true
```

The launch file includes `franka_fr3_moveit_config/moveit.launch.py` and
overlays `config/trace_motion_planning.params.yaml`.
