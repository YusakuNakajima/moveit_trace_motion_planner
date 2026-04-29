# Launch Files

The root launch files wrap the example launch files so they can be started with
normal `ros2 launch moveit_trace_motion_planner ...` commands:

```bash
ros2 launch moveit_trace_motion_planner trace_ur5e_fake_hardware.launch.py
ros2 launch moveit_trace_motion_planner trace_ur5e.launch.py ur_type:=ur5e launch_rviz:=true
ros2 launch moveit_trace_motion_planner trace_fr3.launch.py robot_ip:=dont-care use_fake_hardware:=true
```

The local environment does not include ROS 2, so these launch files are not
executed here.
