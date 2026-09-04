#include "lidar_align/sensors.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lidar_align {
OdomTformData::OdomTformData(Timestamp timestamp_us, Transform T_o0_ot)
    : timestamp_us_(timestamp_us), T_o0_ot_(T_o0_ot) {}

const Transform& OdomTformData::getTransform() const { return T_o0_ot_; }
const Timestamp& OdomTformData::getTimestamp() const { return timestamp_us_; }

void Odom::addTransformData(const Timestamp& timestamp_us, const Transform& T) {
  data_.emplace_back(timestamp_us, T);
}

Transform Odom::getOdomTransform(const Timestamp timestamp_us,
                                 const size_t start_idx,
                                 size_t* match_idx) const {
  if (data_.empty()) {
    throw std::runtime_error("Odom contains no transforms");
  }
  if (data_.size() == 1) {
    if (match_idx != nullptr) *match_idx = 0;
    return data_.front().getTransform();
  }

  size_t idx = std::min(start_idx, data_.size() - 2);
  while ((idx < (data_.size() - 1)) &&
         (timestamp_us > data_[idx].getTimestamp())) {
    ++idx;
  }
  if (idx > 0) --idx;
  idx = std::min(idx, data_.size() - 2);

  if (match_idx != nullptr) *match_idx = idx;

  const Timestamp t0 = data_[idx].getTimestamp();
  const Timestamp t1 = data_[idx + 1].getTimestamp();
  if (t1 == t0) {
    return data_[idx].getTransform();
  }

  const double t_diff_ratio =
      static_cast<double>(timestamp_us - t0) /
      static_cast<double>(t1 - t0);

  const Transform::Vector6 diff_vector =
      (data_[idx].getTransform().inverse() *
       data_[idx + 1].getTransform()).log();

  return data_[idx].getTransform() *
         Transform::exp(t_diff_ratio * diff_vector);
}

Scan::Scan(const LoaderPointcloud& in, const Config& config)
    : timestamp_us_(in.header.stamp), odom_transform_set_(false) {
  std::default_random_engine generator(in.header.stamp);
  std::uniform_real_distribution<float> distribution(0.0f, 1.0f);

  raw_points_.reserve(static_cast<size_t>(
      static_cast<double>(in.size()) *
      std::min(1.0f, std::max(0.0f, config.keep_points_ratio))));

  for (const PointAllFields& point : in) {
    if ((point.intensity > config.min_return_intensity) &&
        distribution(generator) < config.keep_points_ratio) {
      const float sq_dist =
          point.x * point.x + point.y * point.y + point.z * point.z;

      if (std::isfinite(sq_dist) &&
          (sq_dist > (config.min_point_distance * config.min_point_distance)) &&
          (sq_dist < (config.max_point_distance * config.max_point_distance))) {
        Point store_point;
        store_point.x = point.x;
        store_point.y = point.y;
        store_point.z = point.z;
        // Keep the original behavior: intensity temporarily stores the point
        // time offset in microseconds for motion compensation.
        store_point.intensity = static_cast<float>(point.time_offset_us);

        if (config.estimate_point_times) {
          const double timing_factor = 19098593.171 / config.lidar_rpm;
          const double angle = std::atan2(point.x, point.y);
          if (std::abs(angle) > 3.0) continue;
          store_point.intensity = static_cast<float>(angle * timing_factor);
          if (!config.clockwise_lidar) store_point.intensity *= -1.0f;
        }

        raw_points_.push_back(store_point);
      }
    }
  }
  raw_points_.header = in.header;
}

Scan::Config Scan::getConfig(rclcpp::Node* nh) {
  Scan::Config config;
  config.min_point_distance =
      nh->declare_parameter<float>("min_point_distance", config.min_point_distance);
  config.max_point_distance =
      nh->declare_parameter<float>("max_point_distance", config.max_point_distance);
  config.keep_points_ratio =
      nh->declare_parameter<float>("keep_points_ratio", config.keep_points_ratio);
  config.min_return_intensity =
      nh->declare_parameter<float>("min_return_intensity", config.min_return_intensity);
  config.estimate_point_times =
      nh->declare_parameter<bool>("estimate_point_times", config.estimate_point_times);
  config.clockwise_lidar =
      nh->declare_parameter<bool>("clockwise_lidar", config.clockwise_lidar);
  config.motion_compensation =
      nh->declare_parameter<bool>("motion_compensation", config.motion_compensation);
  config.lidar_rpm =
      nh->declare_parameter<float>("lidar_rpm", config.lidar_rpm);
  return config;
}

void Scan::setOdomTransform(const Odom& odom,
                            const double time_offset,
                            const size_t start_idx,
                            size_t* match_idx) {
  T_o0_ot_.clear();
  T_o0_ot_.reserve(raw_points_.size());

  for (const Point& point : raw_points_) {
    const Timestamp point_ts_us =
        timestamp_us_ +
        static_cast<Timestamp>(1000000.0 * time_offset) +
        static_cast<Timestamp>(point.intensity);
    T_o0_ot_.push_back(
        odom.getOdomTransform(point_ts_us, start_idx, match_idx));
  }
  odom_transform_set_ = true;
}

const Transform& Scan::getOdomTransform() const {
  if (!odom_transform_set_ || T_o0_ot_.empty()) {
    throw std::runtime_error(
        "Attempted to get odom transform before it was set");
  }
  return T_o0_ot_.front();
}

void Scan::appendTimeAlignedPointcloud(const Transform& T_o_l,
                                       Pointcloud* pointcloud,
                                       const size_t stride,
                                       size_t* global_index) const {
  if (!odom_transform_set_) {
    throw std::runtime_error(
        "Attempted to build aligned pointcloud before odom transforms were set");
  }
  if (T_o0_ot_.size() != raw_points_.size()) {
    throw std::runtime_error(
        "Point/odometry transform count mismatch inside Scan");
  }

  const size_t safe_stride = std::max<size_t>(1, stride);
  for (size_t i = 0; i < raw_points_.size(); ++i) {
    const size_t current_global_index = (*global_index)++;
    if ((current_global_index % safe_stride) != 0) continue;

    const Transform T_o_lt = T_o0_ot_[i] * T_o_l;
    Eigen::Affine3f pcl_transform;
    pcl_transform.matrix() = T_o_lt.matrix();
    pointcloud->push_back(
        pcl::transformPoint(raw_points_[i], pcl_transform));
  }
}

const Pointcloud& Scan::getRawPointcloud() const { return raw_points_; }

Lidar::Lidar(const LidarId& lidar_id) : lidar_id_(lidar_id) {}

size_t Lidar::getNumberOfScans() const { return scans_.size(); }

size_t Lidar::getTotalPoints() const {
  size_t num_points = 0;
  for (const Scan& scan : scans_) {
    num_points += scan.getRawPointcloud().size();
  }
  return num_points;
}

const LidarId& Lidar::getId() const { return lidar_id_; }

void Lidar::addPointcloud(const LoaderPointcloud& pointcloud,
                          const Scan::Config& config) {
  scans_.emplace_back(pointcloud, config);
}

void Lidar::getCombinedPointcloud(Pointcloud* pointcloud,
                                  const size_t max_points) const {
  pointcloud->clear();

  const size_t total_points = getTotalPoints();
  if (total_points == 0) return;

  size_t stride = 1;
  if (max_points > 0 && total_points > max_points) {
    stride = (total_points + max_points - 1) / max_points;
  }

  const size_t expected_points =
      (total_points + stride - 1) / stride;
  pointcloud->reserve(expected_points);

  size_t global_index = 0;
  for (const Scan& scan : scans_) {
    scan.appendTimeAlignedPointcloud(
        getOdomLidarTransform(), pointcloud, stride, &global_index);
  }
}

void Lidar::saveCombinedPointcloud(const std::string& file_path) const {
  Pointcloud combined;
  // No point cap here: save the complete retained cloud.
  getCombinedPointcloud(&combined, 0);
  pcl::PLYWriter writer;
  writer.write(file_path, combined, true);
}

void Lidar::setOdomOdomTransforms(const Odom& odom,
                                  const double time_offset) {
  size_t idx = 0;
  for (Scan& scan : scans_) {
    scan.setOdomTransform(odom, time_offset, idx, &idx);
  }
}

void Lidar::setOdomLidarTransform(const Transform& T_o_l) {
  T_o_l_ = T_o_l;
}

const Transform& Lidar::getOdomLidarTransform() const { return T_o_l_; }

}  // namespace lidar_align
