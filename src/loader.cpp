
#include "lidar_align/loader.h"
#include "lidar_align/transform.h"
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <algorithm>

namespace lidar_align {
Loader::Loader(const Config& config) : config_(config) {}
Loader::Config Loader::getConfig(rclcpp::Node* nh) {
  Loader::Config config;
  config.use_n_scans = nh->declare_parameter<int>("use_n_scans", config.use_n_scans);
  return config;
}

void Loader::parsePointcloudMsg(const sensor_msgs::msg::PointCloud2& msg, LoaderPointcloud* pointcloud) {
  bool has_timing = false; bool has_intensity = false;
  for (const sensor_msgs::msg::PointField& field : msg.fields) {
    if (field.name == "time_offset_us") has_timing = true;
    else if (field.name == "intensity") has_intensity = true;
  }
  if (has_timing) {
    pcl::fromROSMsg(msg, *pointcloud); return;
  } else if (has_intensity) {
    Pointcloud raw_pointcloud; pcl::fromROSMsg(msg, raw_pointcloud);
    for (const Point& raw_point : raw_pointcloud) {
      PointAllFields point; 
      point.x = raw_point.x; point.y = raw_point.y; point.z = raw_point.z; 
      point.intensity = raw_point.intensity;
      point.time_offset_us = 0; point.reflectivity = 0; point.ring = 0; // 核心修复：强制赋 0
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) || !std::isfinite(point.intensity)) continue;
      pointcloud->push_back(point);
    }
    pointcloud->header = raw_pointcloud.header;
  } else {
    pcl::PointCloud<pcl::PointXYZ> raw_pointcloud; pcl::fromROSMsg(msg, raw_pointcloud);
    for (const pcl::PointXYZ& raw_point : raw_pointcloud) {
      PointAllFields point; 
      point.x = raw_point.x; point.y = raw_point.y; point.z = raw_point.z;
      point.intensity = 0.0;
      point.time_offset_us = 0; point.reflectivity = 0; point.ring = 0; // 核心修复：强制赋 0
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
      pointcloud->push_back(point);
    }
    pointcloud->header = raw_pointcloud.header;
  }
}

bool Loader::loadPointcloudFromROSBag(const std::string& bag_path, const Scan::Config& scan_config, Lidar* lidar) {
  rosbag2_cpp::Reader reader;
  try { reader.open(bag_path); } catch (const std::exception& e) { std::cerr << "LOADING BAG FAILED: " << e.what() << std::endl; return false; }
  
  rosbag2_storage::StorageFilter filter; auto metadata = reader.get_metadata(); std::vector<std::string> topics;
  for (const auto& topic_info : metadata.topics_with_message_count) {
    if (topic_info.topic_metadata.type == "sensor_msgs/msg/PointCloud2") topics.push_back(topic_info.topic_metadata.name);
  }
  
  if (topics.empty()) {
    std::cerr << "No sensor_msgs/msg/PointCloud2 topics found in the bag!" << std::endl;
    return false;
  }
  
  filter.topics = topics; reader.set_filter(filter);
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization; size_t scan_num = 0;

  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    if (std::find(topics.begin(), topics.end(), bag_message->topic_name) == topics.end()) continue;
    
    rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
    sensor_msgs::msg::PointCloud2 msg; serialization.deserialize_message(&extracted_serialized_msg, &msg);
    std::cout << " Loading scan: \033[1m" << scan_num++ << "\033[0m from ros bag\033[K\r" << std::flush;
    LoaderPointcloud pointcloud; parsePointcloudMsg(msg, &pointcloud); lidar->addPointcloud(pointcloud, scan_config);
    if (lidar->getNumberOfScans() >= (size_t)config_.use_n_scans) break;
  }
  std::cout << std::endl;
  if (lidar->getTotalPoints() == 0) return false;
  return true;
}

bool Loader::loadTformFromROSBag(const std::string& bag_path, Odom* odom) {
  rosbag2_cpp::Reader reader;
  try { reader.open(bag_path); } catch (const std::exception& e) { std::cerr << "LOADING BAG FAILED: " << e.what() << std::endl; return false; }
  
  rosbag2_storage::StorageFilter filter; auto metadata = reader.get_metadata(); std::vector<std::string> topics;
  for (const auto& topic_info : metadata.topics_with_message_count) {
    if (topic_info.topic_metadata.type == "geometry_msgs/msg/TransformStamped" || topic_info.topic_metadata.type == "geometry_msgs/TransformStamped") topics.push_back(topic_info.topic_metadata.name);
  }
  
  if (topics.empty()) {
    std::cerr << "\n[Error] No geometry_msgs/msg/TransformStamped topics found in the bag!" << std::endl;
    return false;
  }

  filter.topics = topics; reader.set_filter(filter);
  rclcpp::Serialization<geometry_msgs::msg::TransformStamped> serialization; size_t tform_num = 0;

  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    if (std::find(topics.begin(), topics.end(), bag_message->topic_name) == topics.end()) continue;
    
    rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
    geometry_msgs::msg::TransformStamped transform_msg; serialization.deserialize_message(&extracted_serialized_msg, &transform_msg);
    std::cout << " Loading transform: \033[1m" << tform_num++ << "\033[0m from ros bag\033[K\r" << std::flush;
    Timestamp stamp = transform_msg.header.stamp.sec * 1000000ll + transform_msg.header.stamp.nanosec / 1000ll;
    Transform T(Transform::Translation(transform_msg.transform.translation.x, transform_msg.transform.translation.y, transform_msg.transform.translation.z),
                Transform::Rotation(transform_msg.transform.rotation.w, transform_msg.transform.rotation.x, transform_msg.transform.rotation.y, transform_msg.transform.rotation.z));
    odom->addTransformData(stamp, T);
  }
  std::cout << std::endl;
  if (odom->empty()) return false;
  return true;
}

bool Loader::loadTformFromMaplabCSV(const std::string& csv_path, Odom* odom) {
  std::ifstream file(csv_path, std::ifstream::in);
  if (!file.is_open()) {
    std::cerr << "\n[Error] 无法打开或找不到 CSV 文件，请检查路径: " << csv_path << std::endl;
    return false;
  }
  size_t tform_num = 0;
  while (file.peek() != EOF) {
    std::cout << " Loading transform: \033[1m" << tform_num++ << "\033[0m from csv file\033[K\r" << std::flush;
    Timestamp stamp; Transform T;
    if (getNextCSVTransform(file, &stamp, &T)) odom->addTransformData(stamp, T);
  }
  std::cout << std::endl;
  if (odom->empty()) {
    std::cerr << "\n[Error] CSV 文件为空或格式不正确，未能加载任何位姿数据！" << std::endl;
    return false;
  }
  return true;
}

bool Loader::getNextCSVTransform(std::istream& str, Timestamp* stamp, Transform* T) {
  std::string line; std::getline(str, line);
  if (line.empty() || line[0] == '#') return false;
  std::stringstream line_stream(line); std::string cell; std::vector<std::string> data;
  while (std::getline(line_stream, cell, ',')) data.push_back(cell);
  if (data.size() < 9) return false;
  *stamp = std::stoll(data[0]) / 1000ll;
  *T = Transform(Transform::Translation(std::stod(data[2]), std::stod(data[3]), std::stod(data[4])),
                 Transform::Rotation(std::stod(data[5]), std::stod(data[6]), std::stod(data[7]), std::stod(data[8])));
  return true;
}
}  // namespace lidar_align
