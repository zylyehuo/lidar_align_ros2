import os
from datetime import datetime
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    current_time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    base_results_dir = '/home/zylyehuo/calibrate/lidar2base/src/lidar_align/results'
    timestamped_dir = os.path.join(base_results_dir, current_time_str)
    os.makedirs(timestamped_dir, exist_ok=True)
    final_pointcloud_path = os.path.join(timestamped_dir, 'lidar_points.ply')
    final_calibration_path = os.path.join(timestamped_dir, 'calibration.txt')

    bag_file_arg = DeclareLaunchArgument(
        'bag_file', 
        default_value='/home/zylyehuo/Documents/ROS/DB/BAG/ROS2/calibration/calib_bag2'
        # default_value='/home/zylyehuo/Documents/ROS/DB/BAG/ROS2/ruige/ruige_meeting/ruige_meeting.db3'
    )
    transforms_from_csv_arg = DeclareLaunchArgument(
        'transforms_from_csv', default_value='true'
    )
    csv_file_arg = DeclareLaunchArgument(
        'csv_file', 
        default_value='/home/zylyehuo/calibrate/lidar2base/src/lidar_align/data/white.csv'
        # default_value='/home/zylyehuo/calibrate/lidar2base/src/lidar_align/data/ruige_meeting.csv'
    )

    lidar_align_node = Node(
        package='lidar_align',
        executable='lidar_align_node',
        name='lidar_align',
        output='screen',
        emulate_tty=True, # 允许实时单行刷新
        parameters=[{
            'input_bag_path': LaunchConfiguration('bag_file'),
            'input_csv_path': LaunchConfiguration('csv_file'),
            'output_pointcloud_path': final_pointcloud_path,
            'output_calibration_path': final_calibration_path,
            'transforms_from_csv': LaunchConfiguration('transforms_from_csv'),
            # 'use_n_scans': 1000,
            'keep_points_ratio': 0.01,    # 默认值 0.01
            'min_point_distance': 2.0,    # 2.0
            'max_point_distance': 30.0,   # 30.0
            
            'local': True,               # True=仅 Local；False=先 Global 再 Local
            # time_cal=True ：优化 6 个变量 [x, y, rx, ry, rz, time_offset]
            # time_cal=False：优化 5 个变量 [x, y, rx, ry, rz]
            'time_cal': True,             # True
            'inital_guess': [-0.50, 0.08, -0.62, 0.0, 0.0, 0.0, 0.0],  # [-0.40, 0.0, 0.6, 0.0, 0.0, 0.0, 0.0]
            'max_time_offset': 0.1,       # 0.1
            
            'max_evals': 50.0,          # 1000 次最大评估
            'xtol': 0.0001,               # 默认值 0.0001
            'local_knn_max_dist': 0.1,    # 0.1
            'knn_batch_size': 2000,       # 仅KNN串行分块大小；不再创建异步线程
            'local_optimization_max_points': 0,   # 0=局部优化使用全部保留点
            'global_optimization_max_points': 0   # 0=全局优化使用全部保留点
        }]
    )

    return LaunchDescription([
        bag_file_arg,
        transforms_from_csv_arg,
        csv_file_arg,
        lidar_align_node
    ])
