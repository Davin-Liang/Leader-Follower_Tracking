# Deployment Procedure
1. Input the following code.
```
mkdir -p lft_ws/src
git clone git@github.com:Davin-Liang/Leader-Follower_Tracking.git
cd ..
colcon build
```
2. Start simulation environment.
```
ros2 launch lft_simulation onni_lft_simulation.launch.py
```
3. Control Leader Onni Robot by command line.
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=leader/cmd_vel
```
4. Control slave Onni Robot by command line.
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=slave/cmd_vel
```