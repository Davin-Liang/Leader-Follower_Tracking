# 1 Deployment Procedure
1. Input the following code.
```
mkdir -p lft_ws/src
git clone git@github.com:Davin-Liang/Leader-Follower_Tracking.git
cd Follower_Tracking
git fetch origin
git branch -a
git checkout dev_li
git switch dev_li
cd ..
cd ..
colcon build --packages-select lft_simulation lft_control
```
2. Start simulation environment.
```
ros2 launch lft_simulation onni_lft_simulation.launch.py
```
3. Control Leader Onni Robot by command line.
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=leader/cmd_vel
```
4. Start control algorithm.
``` 
cd ~/lft_ws/src/Leader-Follower_Tracking/lft_control/scripts/
python3 lft_base.py
```

# 2 Git Usage Specification
1. Everyone can't develop on the main branch, only on their own branch, such as dev_liang for Gengming Liang, dev_li for kaiyan li.
2. Usually upload code can only be uploaded to own branch. 
3. Own ranch can only be combined with permission.
