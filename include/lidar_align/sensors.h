#ifndef LIDAR_ALIGN_SENSORS_H_
#define LIDAR_ALIGN_SENSORS_H_

#include <limits>
#include <random>

#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

#include "lidar_align/transform.h"

namespace lidar_align {
typedef std::string LidarId;
typedef long long int Timestamp;

struct EIGEN_ALIGN16 PointAllFields {
  PCL_ADD_POINT4D;
  int32_t time_offset_us;
  uint16_t reflectivity;
  uint16_t intensity;
  uint8_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> Pointcloud;
typedef pcl::PointCloud<PointAllFields> LoaderPointcloud;

class OdomTformData {
 public:
  OdomTformData(Timestamp timestamp_us, Transform T_o0_ot);
  const Transform& getTransform() const;
  const Timestamp& getTimestamp() const;

 private:
  Transform T_o0_ot_;
  Timestamp timestamp_us_;
};

class Odom {
 public:
  void addTransformData(const Timestamp& timestamp_us,
                        const Transform& transform);
  Transform getOdomTransform(const Timestamp timestamp_us,
                             size_t start_idx = 0,
                             size_t* match_idx = nullptr) const;
  bool empty() const { return data_.empty(); }
  size_t size() const { return data_.size(); }

 private:
  std::vector<OdomTformData> data_;
};

class Scan {
 public:
  struct Config {
    float min_point_distance = 0.0;
    float max_point_distance = 100.0;
    float keep_points_ratio = 0.01;
    float min_return_intensity = -1.0;
    bool estimate_point_times = false;
    bool clockwise_lidar = false;
    bool motion_compensation = true;
    float lidar_rpm = 600.0;
  };

  Scan(const LoaderPointcloud& pointcloud, const Config& config);
  static Config getConfig(rclcpp::Node* nh);

  void setOdomTransform(const Odom& odom,
                        double time_offset,
                        size_t start_idx,
                        size_t* match_idx);

  const Transform& getOdomTransform() const;
  const Pointcloud& getRawPointcloud() const;

  void appendTimeAlignedPointcloud(const Transform& T_o_l,
                                   Pointcloud* pointcloud,
                                   size_t stride,
                                   size_t* global_index) const;

 private:
  Timestamp timestamp_us_;
  Pointcloud raw_points_;
  std::vector<Transform> T_o0_ot_;
  bool odom_transform_set_;
};

class Lidar {
 public:
  explicit Lidar(const LidarId& lidar_id = "Lidar");

  size_t getNumberOfScans() const;
  size_t getTotalPoints() const;
  const LidarId& getId() const;

  void addPointcloud(const LoaderPointcloud& pointcloud,
                     const Scan::Config& config = Scan::Config());

  // max_points == 0 means no cap. The sampling is deterministic and evenly
  // spaced over the retained points from all scans.
  void getCombinedPointcloud(Pointcloud* pointcloud,
                             size_t max_points = 0) const;

  void setOdomOdomTransforms(const Odom& odom,
                             double time_offset = 0.0);
  void setOdomLidarTransform(const Transform& T_o_l);
  void saveCombinedPointcloud(const std::string& file_path) const;
  const Transform& getOdomLidarTransform() const;

 private:
  LidarId lidar_id_;
  Transform T_o_l_;
  std::vector<Scan> scans_;
};
}  // namespace lidar_align

#endif  // LIDAR_ALIGN_SENSORS_H_
