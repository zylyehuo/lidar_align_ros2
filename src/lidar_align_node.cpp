#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include "lidar_align/aligner.h"
#include "lidar_align/loader.h"
#include "lidar_align/sensors.h"

using namespace lidar_align;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("lidar_align");

  Loader loader(Loader::getConfig(node.get()));
  Lidar lidar;
  Odom odom;

  std::string input_bag_path = node->declare_parameter<std::string>("input_bag_path", "");
  RCLCPP_INFO(node->get_logger(), "Loading Pointcloud Data...");
  if (input_bag_path.empty()) {
    RCLCPP_FATAL(node->get_logger(), "Could not find input_bag_path parameter, exiting");
    return EXIT_FAILURE;
  } else if (!loader.loadPointcloudFromROSBag(input_bag_path, Scan::getConfig(node.get()), &lidar)) {
    RCLCPP_FATAL(node->get_logger(), "Error loading pointclouds from ROS bag.");
    return EXIT_FAILURE;
  }

  bool transforms_from_csv = node->declare_parameter<bool>("transforms_from_csv", false);
  std::string input_csv_path = node->declare_parameter<std::string>("input_csv_path", "");
  
  RCLCPP_INFO(node->get_logger(), "Loading Transformation Data...");
  if (transforms_from_csv) {
    if (input_csv_path.empty()) {
      RCLCPP_FATAL(node->get_logger(), "Could not find input_csv_path parameter, exiting");
      return EXIT_FAILURE;
    } else if (!loader.loadTformFromMaplabCSV(input_csv_path, &odom)) {
      RCLCPP_FATAL(node->get_logger(), "Error loading transforms from CSV.");
      return EXIT_FAILURE;
    }
  } else if (!loader.loadTformFromROSBag(input_bag_path, &odom)) {
    RCLCPP_FATAL(node->get_logger(), "Error loading transforms from ROS bag.");
    return EXIT_FAILURE;
  }

  if (lidar.getNumberOfScans() == 0) {
    RCLCPP_FATAL(node->get_logger(), "No lidar data loaded, exiting");
    return EXIT_FAILURE;
  }
  if (odom.empty()) {
    RCLCPP_FATAL(node->get_logger(), "未读取到任何位姿数据，为防止崩溃程序已安全退出");
    return EXIT_FAILURE;
  }

  RCLCPP_INFO(node->get_logger(), "Interpolating Transformation Data...");
  lidar.setOdomOdomTransforms(odom);

  Aligner aligner(Aligner::getConfig(node.get()));
  aligner.lidarOdomTransform(&lidar, &odom);

  rclcpp::shutdown();
  return 0;
}
