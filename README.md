# Surgical Simulation Workspace

This workspace is a ROS 2 Jazzy-based surgical manipulation setup centered on KUKA robot descriptions, MoveIt integration, and custom EKI-based bridge nodes for hardware execution. The current tree includes both ROS packages under [src](src) and a small set of root-level helper scripts for direct robot teleoperation.

## Current workspace layout

```text
surgical_sim_ws/
├── README.md
├── build/                  # Colcon build artifacts
├── install/                # Colcon install space
├── keyboard_control.py     # Direct EKI keyboard teleop script
├── keyboard_nudge.py       # Direct EKI single-axis nudge script
├── log/                    # Colcon/log output
├── models/                 # Local model assets
└── src/
    ├── kuka_eki/           # KUKA EKI interface sources
    ├── kuka_eki_bridge/    # ROS 2 bridge nodes for EKI and gripper/vision flows
    ├── kuka-external-control-sdk/  # External SDK sources
    ├── kuka_robot_descriptions/    # URDFs, meshes, MoveIt configs, and KUKA support packages
    ├── kuka_surgical_demo/ # Surgical demo nodes, voice/vision logic, and pick/place orchestration
    ├── kuka_vacuum_gripper/# Vacuum gripper URDF/Xacro definitions
    └── surgical_msgs/      # Custom ROS 2 interfaces
```

## What is in this workspace

### Root-level scripts

- `keyboard_control.py`: connects to the KUKA EKI state/motion servers and provides keyboard teleoperation for basic motion.
- `keyboard_nudge.py`: provides a simple terminal-based single-axis nudge interface.

### ROS 2 packages

| Package | Current purpose |
| --- | --- |
| `kuka_eki_bridge` | Provides bridge executables for EKI motion, gripper control, vision-driven actions, and voice-based bridging. |
| `kuka_surgical_demo` | Contains the surgical pick/place demo, voice AI, vision nodes, coordination nodes, and the current grid-based orchestration logic. |
| `kuka_robot_descriptions` | Supplies the KUKA robot URDFs and MoveIt-related configuration packages, including `kuka_kr_moveit_config`. |
| `kuka_vacuum_gripper` | Holds the vacuum gripper model definitions. |
| `surgical_msgs` | Defines custom ROS 2 message/service interfaces used by the demo stack. |
| `kuka_eki` | Contains the lower-level KUKA EKI interface sources used by the teleop scripts and bridge nodes. |
| `kuka-external-control-sdk` | Includes an additional SDK tree that is present in the workspace. |

## Build and source

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

If you only want to rebuild a subset of packages during development, you can target them explicitly, for example:

```bash
colcon build --packages-select kuka_surgical_demo kuka_eki_bridge kuka_robot_descriptions surgical_msgs
```

## Example launch and run commands

### MoveIt planning simulation

A usable MoveIt launch target is present in the KUKA robot descriptions package:

```bash
ros2 launch kuka_kr_moveit_config moveit_planning_fake_hardware.launch.py \
  robot_model:=kr6_r900_sixx_with_gripper \
  robot_family:=agilus
```

### EKI bridge nodes

These executables are defined in the bridge package:

```bash
ros2 run kuka_eki_bridge bridge_node
ros2 run kuka_eki_bridge gripper_bridge
ros2 run kuka_eki_bridge vision_gripper_bridge
```

### Surgical demo nodes

The surgical demo package exposes several console scripts, including:

```bash
ros2 run kuka_surgical_demo voice_ai_node
ros2 run kuka_surgical_demo voice_bridge_node
ros2 run kuka_surgical_demo vision_node
ros2 run kuka_surgical_demo grid_node
```

### Direct hardware scripts

The root-level scripts communicate directly with the KUKA controller over the EKI interface. They assume the robot is reachable at `192.168.1.147`:

```bash
python3 keyboard_control.py
python3 keyboard_nudge.py
```

## Notes

- The workspace is actively being used for surgical demo development, voice/vision integration, and EKI-based hardware bridging.
- Some package metadata still contains placeholder descriptions, so this README focuses on the currently present structure and executable entry points rather than claiming a fully polished release state.
- The current repository state may evolve as nodes and launch files are added or reorganized.

