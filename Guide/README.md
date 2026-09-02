# RISA-bot Guide

Quick reference documentation for operating and tuning the robot.

![Competition Course Layout](competition_layout_overview.jpeg)

## Contents

| Guide                                          | Description                                                        |
| ---------------------------------------------- | ------------------------------------------------------------------ |
| [Odometry](Odometer_Guide.md)                  | Hardware encoder vs software odometry — equations & reference code |
| [Challenge Breakdown](challenges_breakdown.md) | **Deep dive** into each challenge's code, with course layout       |
| [Main Branch](main_branch.md)                  | How `main` works — individual node testing, original controller    |
| [Test Branch](test_branch.md)                  | How `test` works — competition state machine, all 9 challenges     |
| [Commands Reference](commands_reference.md)    | All ROS topics, launch files, useful commands                      |
| [Tuning Guide](tuning_guide.md)                | Step-by-step parameter tuning on the physical course               |
| [Competition Validation](competition_validation_guide.md) | Hardware test plan, pass/fail checks, regression workflow |
| [Architecture](architecture.md)                | Node graphs, data flow, how it all fits together                   |

## Which Branch?

| I want to...                                  | Branch          | Command                                              |
| --------------------------------------------- | --------------- | ---------------------------------------------------- |
| Test individual nodes                         | `main`          | `run_risabot`                                        |
| Full competition run (original)               | `test`          | `ros2 launch risabot_automode competition.launch.py` |
| Full competition run (refactored + dashboard) | `refactor-test` | `ros2 launch risabot_automode competition.launch.py` |
| Minimal debug bringup (refactored)            | `refactor-test` | `ros2 launch risabot_automode bringup.launch.py`     |
| Drive with joystick only                      | `main`          | `ros2 launch control_servo robot_rc.launch.py`       |
