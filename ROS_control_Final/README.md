# MSD700 ROS 2 Serial Bridge

This package reads odometry JSON from the robot serial port and exposes it to ROS 2.

Expected firmware output:

```json
{"type":"odom","x":-0.0002,"y":0.0000,"theta":-0.0105,"v":-0.0003,"omega":-0.0010,"left_ticks":880,"right_ticks":-985,"avg_ticks_abs":932.5}
```

The right encoder can be negative while the robot moves forward. The bridge keeps that raw sign convention:

- `/encoder_ticks`: `std_msgs/Float64MultiArray`
- data order: `[left_ticks, right_ticks, avg_ticks_abs]`
- `/odom_raw`: `nav_msgs/Odometry`
- pose/twist come from the firmware `x`, `y`, `theta`, `v`, and `omega` fields

Use `ticks_per_meter:=2690.0` for this robot calibration.

## Build

From the workspace root:

```bash
colcon build --packages-select msd700
source install/setup.bash
```

On Windows PowerShell:

```powershell
colcon build --packages-select msd700
.\install\setup.ps1
```

## Run The Serial Bridge

Set the port to your Arduino/ESP32 serial port:

```bash
ros2 run msd700 serial_bridge --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200 -p ticks_per_meter:=2690.0
```

On Windows this is usually a `COM` port:

```powershell
ros2 run msd700 serial_bridge --ros-args -p port:=COM3 -p baud:=115200 -p ticks_per_meter:=2690.0
```

The bridge also subscribes to `/cmd_vel` and writes commands back to serial as JSON lines:

```json
{"type":"cmd_vel","v":0.15,"omega":0.0}
```

If your firmware expects a simple CSV command instead, run:

```bash
ros2 run msd700 serial_bridge --ros-args -p command_format:=csv
```

CSV format is:

```text
0.1500,0.0000
```

## Test Moving 1 Meter

Keep the bridge running in one terminal, then run:

```bash
ros2 run msd700 move_distance 1.0
```

The node publishes `/cmd_vel` until the Euclidean distance from `/odom_raw` reaches the target. For reverse movement:

```bash
ros2 run msd700 move_distance -0.5
```

Useful tuning parameters:

```bash
ros2 run msd700 move_distance 1.0 --ros-args -p speed:=0.10 -p tolerance:=0.01
```
