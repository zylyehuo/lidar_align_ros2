# 使用步骤
# 雷达时间同步
```bash
sudo ptp4l -m -i enp46s0 -S
```
# 启动雷达驱动
```bash
cd ~/driver_ws
source install/setup.bash
ros2 launch hesai_ros_driver start.py
```
# 录制标定数据集
```bash
cd ~/neu_calib/src/lidar_align/data
ros2 bag record -o calib_bag /lidar_points /wheel_odom
```
# 导出轮式里程计的数据
```bash
# 将底盘的 odom 话题数据转换为 .csv 格式
cd ~/neu_calib/src/lidar_align/scripts
python3 ros2_odom_to_csv.py ~/neu_calib/src/lidar_align/data/calib_bag /wheel_odom ~/neu_calib/src/lidar_align/data/wheel_odom.csv
```
# 进行外参计算
```bash
cd ~/neu_calib
source install/setup.bash
ros2 launch lidar_align lidar_align.launch.py
```
