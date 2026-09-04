# 基于[ lidar_align ](https://github.com/ethz-asl/lidar_align)修改的 ROS2 humble 版本

# 使用步骤
```bash
mkdir -p ~/lidar2base/src
cd ~/lidar2base/src
git clone git@github.com:zylyehuo/lidar_align_ros2.git

```

```bash
# 将底盘的 odom 话题数据转换为 .csv 格式
cd ~/lidar2base/src/lidar_align/scripts
python3 ros2_odom_to_csv.py <ros2_bag目录> <odom话题名> <输出的csv名.csv>

```

```bash
cd ~/lidar2base
source install/setup.bash
ros2 launch lidar_align lidar_align.launch.py

```

# 运行结果
```bash
[INFO] [launch]: All log files can be found below /home/zylyehuo/.ros/log/2026-06-19-21-31-49-190258-thinkpad-1799766
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [lidar_align_node-1]: process started with pid [1799767]
[lidar_align_node-1] [INFO] [1781875909.236115440] [lidar_align]: Loading Pointcloud Data...
[lidar_align_node-1] 
[lidar_align_node-1] closing.
[lidar_align_node-1] 
[lidar_align_node-1] closing.
[lidar_align_node-1] [INFO] [1781875909.238146941] [rosbag2_storage]: Opened database '/home/zylyehuo/Documents/ROS/DB/BAG/ROS2/ruige/ruige_office/ruige_office.db3' for READ_ONLY.
 Loading scan: 2380 from ros bag
[lidar_align_node-1] [INFO] [1781875919.528610612] [lidar_align]: Loading Transformation Data...
 Loading transform: 11900 from csv file
[lidar_align_node-1] [INFO] [1781875919.564339407] [lidar_align]: Interpolating Transformation Data...
[lidar_align_node-1] 
[lidar_align_node-1] Performing Global Optimization...
 rx:  -0.00 ry:   0.00 rz:  -0.03 Error:     450.67 Iteration: 1000
[lidar_align_node-1] 
[lidar_align_node-1] Performing Local Optimization...
 x:   0.28 y:   0.01 z:   0.08 rx:  -0.00 ry:   0.00 rz:  -0.03 time:   0.08 Error:     357.99 Iteration: 1114
[lidar_align_node-1] 
[lidar_align_node-1] Saving Aligned Pointcloud...
[lidar_align_node-1] 
[lidar_align_node-1] Saving Calibration File...
[lidar_align_node-1] 
[lidar_align_node-1] Final Calibration:
[lidar_align_node-1] Active Transformation Vector (x,y,z,rx,ry,rz) from the Pose Sensor Frame to  the Lidar Frame:
[lidar_align_node-1] [0.278179, 0.00637575, 0.0802086, -0.00455963, 0.00238118, -0.0310053]
[lidar_align_node-1] 
[lidar_align_node-1] Active Rotation in Degrees (X, Y, Z):
[lidar_align_node-1] [-0.261247°, 0.136431°, -1.77647°]
[lidar_align_node-1] 
[lidar_align_node-1] Active Transformation Matrix from the Pose Sensor Frame to  the Lidar Frame:
[lidar_align_node-1]    0.999517   0.0309947  0.00245147    0.278179
[lidar_align_node-1]  -0.0310056    0.999509  0.00452197  0.00637575
[lidar_align_node-1] -0.00231011 -0.00459579    0.999987   0.0802086
[lidar_align_node-1]           0           0           0           1
[lidar_align_node-1] 
[lidar_align_node-1] Active Translation Vector (x,y,z) from the Pose Sensor Frame to  the Lidar Frame:
[lidar_align_node-1] [0.278179, 0.00637575, 0.0802086]
[lidar_align_node-1] 
[lidar_align_node-1] Active Hamiltonen Quaternion (w,x,y,z) the Pose Sensor Frame to  the Lidar Frame:
[lidar_align_node-1] [0.999877, -0.00227972, 0.00119054, -0.015502]
[lidar_align_node-1] 
[lidar_align_node-1] Time offset that must be added to lidar timestamps in seconds:
[lidar_align_node-1] 0.0803827
[lidar_align_node-1] 
[lidar_align_node-1] ROS 2 Static TF Publisher: ros2 run tf2_ros static_transform_publisher 0.278179 0.00637575 0.0802086 -0.00227972 0.00119054 -0.015502 0.999877 POSE_FRAME LIDAR_FRAME
[INFO] [lidar_align_node-1]: process has finished cleanly [pid 1799767]

```
