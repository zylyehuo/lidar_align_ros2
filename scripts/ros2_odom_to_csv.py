import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def main():
    if len(sys.argv) < 4:
        print("用法: python3 ros2_odom_to_csv.py <ros2_bag目录> <odom话题名> <输出的csv名.csv>")
        sys.exit(1)

    bag_path = sys.argv[1]
    odom_topic = sys.argv[2]
    csv_file = sys.argv[3]

    # 配置 ROS 2 reader
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"打开数据包失败: {e}")
        sys.exit(1)

    # 获取话题和数据类型
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    
    if odom_topic not in type_map:
        print(f"错误: 在包中未找到话题 '{odom_topic}'")
        sys.exit(1)
        
    # 动态获取消息类型 (如 nav_msgs/msg/Odometry)
    msg_type = get_message(type_map[odom_topic])

    count = 0
    print("正在提取并转换，请稍候...")
    with open(csv_file, 'w') as f:
        while reader.has_next():
            (topic, data, t) = reader.read_next()
            if topic == odom_topic:
                msg = deserialize_message(data, msg_type)
                # Maplab CSV 格式: 1.时间戳(纳秒) 2.顶点(填0) 3.px 4.py 5.pz 6.qw 7.qx 8.qy 9.qz
                timestamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                f.write(f"{timestamp_ns},0,{p.x},{p.y},{p.z},{q.w},{q.x},{q.y},{q.z}\n")
                count += 1

    print(f"转换完成，共生成 {count} 条位姿数据，文件位于: {csv_file}")

if __name__ == '__main__':
    main()
