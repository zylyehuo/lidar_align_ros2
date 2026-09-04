#include "lidar_align/aligner.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace lidar_align {

Aligner::Aligner(const Config& config) : config_(config) {}

Aligner::Config Aligner::getConfig(rclcpp::Node* nh) {
  Aligner::Config config;

  config.local = nh->declare_parameter<bool>("local", config.local);
  config.inital_guess = nh->declare_parameter<std::vector<double>>(
      "inital_guess", config.inital_guess);
  config.max_time_offset = nh->declare_parameter<double>(
      "max_time_offset", config.max_time_offset);
  config.angular_range = nh->declare_parameter<double>(
      "angular_range", config.angular_range);
  config.translation_range = nh->declare_parameter<double>(
      "translation_range", config.translation_range);
  config.max_evals = nh->declare_parameter<double>(
      "max_evals", config.max_evals);
  config.xtol = nh->declare_parameter<double>("xtol", config.xtol);

  config.knn_batch_size = nh->declare_parameter<int>(
      "knn_batch_size", config.knn_batch_size);
  config.knn_k = nh->declare_parameter<int>("knn_k", config.knn_k);
  config.global_knn_max_dist = nh->declare_parameter<float>(
      "global_knn_max_dist", config.global_knn_max_dist);
  config.local_knn_max_dist = nh->declare_parameter<float>(
      "local_knn_max_dist", config.local_knn_max_dist);

  config.local_optimization_max_points = nh->declare_parameter<int>(
      "local_optimization_max_points",
      config.local_optimization_max_points);
  config.global_optimization_max_points = nh->declare_parameter<int>(
      "global_optimization_max_points",
      config.global_optimization_max_points);

  config.time_cal = nh->declare_parameter<bool>("time_cal", config.time_cal);
  config.output_pointcloud_path = nh->declare_parameter<std::string>(
      "output_pointcloud_path", config.output_pointcloud_path);
  config.output_calibration_path = nh->declare_parameter<std::string>(
      "output_calibration_path", config.output_calibration_path);

  if (config.knn_batch_size <= 0) {
    RCLCPP_WARN(nh->get_logger(),
                "knn_batch_size <= 0 is invalid; forcing it to 2000");
    config.knn_batch_size = 2000;
  }
  if (config.knn_k <= 0) {
    RCLCPP_WARN(nh->get_logger(),
                "knn_k <= 0 is invalid; forcing it to 1");
    config.knn_k = 1;
  }
  if (config.local_optimization_max_points <= 0) {
    RCLCPP_WARN(nh->get_logger(),
                "local_optimization_max_points <= 0: objective will use all retained points");
  }
  if (config.global_optimization_max_points <= 0) {
    RCLCPP_WARN(nh->get_logger(),
                "global_optimization_max_points <= 0: global objective will use all retained points");
  }

  return config;
}

float Aligner::kNNError(const pcl::KdTreeFLANN<Point>& kdtree,
                        const Pointcloud& pointcloud,
                        const size_t k,
                        const float max_dist,
                        const size_t start_idx,
                        const size_t end_idx) {
  std::vector<int> kdtree_idx(k);
  std::vector<float> kdtree_dist(k);
  float error = 0.0f;

  const size_t bounded_end = std::min(pointcloud.size(), end_idx);
  for (size_t idx = start_idx; idx < bounded_end; ++idx) {
    const int found = kdtree.nearestKSearch(
        pointcloud[idx], static_cast<int>(k), kdtree_idx, kdtree_dist);
    for (int j = 0; j < found; ++j) {
      // Preserve the original objective semantics. PCL returns squared
      // distances here; existing tuning values were selected for that behavior.
      error += std::min(kdtree_dist[static_cast<size_t>(j)], max_dist);
    }
  }
  return error;
}

float Aligner::lidarOdomKNNError(const Pointcloud& base_pointcloud,
                                 const Pointcloud& combined_pointcloud) const {
  if (!rclcpp::ok()) {
    throw std::runtime_error("ROS node died, exiting");
  }
  if (base_pointcloud.empty() || combined_pointcloud.empty()) {
    throw std::runtime_error("Cannot evaluate KNN error on an empty point cloud");
  }

  Pointcloud::ConstPtr combined_pointcloud_ptr(
      &combined_pointcloud, [](const Pointcloud*) {});
  pcl::KdTreeFLANN<Point> kdtree;
  kdtree.setInputCloud(combined_pointcloud_ptr);

  const float max_dist =
      config_.local ? config_.local_knn_max_dist
                    : config_.global_knn_max_dist;

  size_t k = static_cast<size_t>(std::max(1, config_.knn_k));
  if (&base_pointcloud == &combined_pointcloud) ++k;
  k = std::min(k, combined_pointcloud.size());

  // CRITICAL FIX:
  // The original implementation created one std::async thread per batch and
  // passed both the KdTree and the complete point cloud by value. On a large
  // bag this produced hundreds/thousands of huge copies and the kernel killed
  // the process with SIGKILL (-9). We intentionally process batches serially;
  // memory usage is now bounded by one cloud + one KdTree.
  const size_t batch_size =
      static_cast<size_t>(std::max(1, config_.knn_batch_size));

  float total_error = 0.0f;
  for (size_t start_idx = 0;
       start_idx < base_pointcloud.size();
       start_idx += batch_size) {
    if (!rclcpp::ok()) {
      throw std::runtime_error("ROS node died during KNN evaluation");
    }
    const size_t end_idx =
        std::min(base_pointcloud.size(), start_idx + batch_size);
    total_error += kNNError(
        kdtree, base_pointcloud, k, max_dist, start_idx, end_idx);
  }

  return total_error;
}

float Aligner::lidarOdomKNNError(const Lidar& lidar) const {
  Pointcloud pointcloud;

  const int configured_max_points =
      config_.local ? config_.local_optimization_max_points
                    : config_.global_optimization_max_points;
  const size_t max_points = configured_max_points > 0
                                ? static_cast<size_t>(configured_max_points)
                                : 0;

  lidar.getCombinedPointcloud(&pointcloud, max_points);
  return lidarOdomKNNError(pointcloud, pointcloud);
}

double Aligner::LidarOdomMinimizer(const std::vector<double>& x,
                                   std::vector<double>& /*grad*/,
                                   void* f_data) {
  OptData* d = static_cast<OptData*>(f_data);

  // Parameterization:
  //   Global rotation stage: [rx, ry, rz]
  //   Local, time_cal=false: [x, y, rx, ry, rz]
  //   Local, time_cal=true : [x, y, rx, ry, rz, time_offset]
  // z is never optimized. It is supplied through OptData::fixed_z.
  if (x.size() == 6) {
    d->lidar->setOdomOdomTransforms(*(d->odom), x[5]);
  }

  Eigen::Matrix<double, 6, 1> vec;
  vec.setZero();
  vec[2] = d->fixed_z;

  if (x.size() == 3) {
    // Global stage only searches rotation. Translation is fixed at
    // x=y=z=0 in the objective; fixed_z is explicitly forced to 0 when
    // local=false by lidarOdomTransform().
    vec[3] = x[0];
    vec[4] = x[1];
    vec[5] = x[2];
  } else if (x.size() == 5 || x.size() == 6) {
    vec[0] = x[0];
    vec[1] = x[1];
    vec[3] = x[2];
    vec[4] = x[3];
    vec[5] = x[4];
  } else {
    std::ostringstream ss;
    ss << "Unexpected optimizer dimension " << x.size()
       << "; expected 3 (global), 5 (local), or 6 (local+time).";
    throw std::runtime_error(ss.str());
  }

  d->lidar->setOdomLidarTransform(Transform::exp(vec.cast<float>()));
  const double error = d->aligner->lidarOdomKNNError(*(d->lidar));

  static size_t iteration = 0;
  static FILE* tty = fopen("/dev/tty", "w");
  if (tty != nullptr) {
    if (x.size() > 3) {
      fprintf(tty,
              " \033[1mx:\033[0m %6.2f \033[1my:\033[0m %6.2f \033[1mz:\033[0m %6.2f",
              vec[0], vec[1], vec[2]);
    }
    fprintf(tty,
            " \033[1mrx:\033[0m %6.2f \033[1mry:\033[0m %6.2f \033[1mrz:\033[0m %6.2f",
            vec[3], vec[4], vec[5]);
    if (x.size() == 6) {
      fprintf(tty, " \033[1mtime:\033[0m %7.4f", x[5]);
    }
    fprintf(tty,
            " \033[1mError:\033[0m %12.2f \033[1mIteration:\033[0m %zu\033[K\r",
            error, iteration++);
    fflush(tty);
  }

  return error;
}

void Aligner::optimize(const std::vector<double>& lb,
                       const std::vector<double>& ub,
                       OptData* opt_data,
                       std::vector<double>* x) {
  if (x == nullptr || x->empty()) {
    throw std::runtime_error("Optimizer received an empty parameter vector");
  }
  if (lb.size() != x->size() || ub.size() != x->size()) {
    std::ostringstream ss;
    ss << "NLOPT dimension mismatch: x=" << x->size()
       << ", lb=" << lb.size() << ", ub=" << ub.size();
    throw std::runtime_error(ss.str());
  }

  nlopt::opt opt = config_.local
                       ? nlopt::opt(nlopt::LN_BOBYQA, x->size())
                       : nlopt::opt(nlopt::GN_DIRECT_L, x->size());

  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);
  opt.set_maxeval(std::max(1, static_cast<int>(std::lround(config_.max_evals))));
  opt.set_xtol_abs(config_.xtol);
  opt.set_min_objective(LidarOdomMinimizer, opt_data);

  double minf = std::numeric_limits<double>::infinity();
  try {
    const nlopt::result result = opt.optimize(*x, minf);
    std::cout << "\nNLOPT result code: " << static_cast<int>(result)
              << ", objective: " << minf << std::endl;
  } catch (const std::exception& e) {
    // NLOPT can throw roundoff_limited after having already produced a useful
    // best-so-far x. Preserve and report that x rather than terminating the node.
    std::cerr << "\n[Warning] NLOPT stopped with exception: " << e.what()
              << ". Keeping best/current parameters." << std::endl;
  }

  std::vector<double> grad;
  LidarOdomMinimizer(*x, grad, opt_data);
}

std::string Aligner::generateCalibrationString(const Transform& T,
                                                const double time_offset) {
  const Transform::Vector6 T_log = T.log();
  std::stringstream ss;

  ss << "Active Transformation Vector (x,y,z,rx,ry,rz) from the Pose Sensor Frame to the Lidar Frame:\n[";
  ss << T_log[0] << ", " << T_log[1] << ", " << T_log[2] << ", "
     << T_log[3] << ", " << T_log[4] << ", " << T_log[5] << "]\n\n";

  ss << "Active Rotation in Degrees (X, Y, Z):\n[";
  ss << (T_log[3] * 180.0 / M_PI) << "°, "
     << (T_log[4] * 180.0 / M_PI) << "°, "
     << (T_log[5] * 180.0 / M_PI) << "°]\n\n";

  ss << "Active Transformation Matrix from the Pose Sensor Frame to the Lidar Frame:\n"
     << T.matrix() << "\n\n";

  ss << "Active Translation Vector (x,y,z) from the Pose Sensor Frame to the Lidar Frame:\n[";
  ss << T.translation().x() << ", " << T.translation().y() << ", "
     << T.translation().z() << "]\n\n";

  ss << "Active Hamiltonen Quaternion (w,x,y,z) the Pose Sensor Frame to the Lidar Frame:\n[";
  ss << T.rotation().w() << ", " << T.rotation().x() << ", "
     << T.rotation().y() << ", " << T.rotation().z() << "]\n\n";

  if (config_.time_cal) {
    ss << "Time offset that must be added to lidar timestamps in seconds:\n"
       << time_offset << "\n\n";
  }

  ss << "ROS 2 Static TF Publisher: ros2 run tf2_ros static_transform_publisher ";
  ss << T.translation().x() << " " << T.translation().y() << " "
     << T.translation().z() << " ";
  ss << T.rotation().x() << " " << T.rotation().y() << " "
     << T.rotation().z() << " " << T.rotation().w()
     << " POSE_FRAME LIDAR_FRAME" << std::endl;

  return ss.str();
}

void Aligner::lidarOdomTransform(Lidar* lidar, Odom* odom) {
  std::cout << "Optimization input: " << lidar->getNumberOfScans()
            << " scans, " << lidar->getTotalPoints() << " retained points"
            << ", serial KNN batch size=" << config_.knn_batch_size
            << std::endl;

  OptData opt_data;
  opt_data.lidar = lidar;
  opt_data.odom = odom;
  opt_data.aligner = this;
  opt_data.time_cal = config_.time_cal;

  // Z translation is intentionally unobservable/disabled in this setup.
  // Preserve the requested semantics exactly:
  //   local=true  -> z = inital_guess[2]
  //   local=false -> z = 0.0, including the subsequent Local refinement.
  const bool requested_local_only = config_.local;
  opt_data.fixed_z = requested_local_only && config_.inital_guess.size() > 2
                         ? config_.inital_guess[2]
                         : 0.0;

  // Local parameterization excludes z completely:
  //   time_cal=false: [x, y, rx, ry, rz]
  //   time_cal=true : [x, y, rx, ry, rz, time_offset]
  const size_t num_params = config_.time_cal ? 6 : 5;
  std::vector<double> x(num_params, 0.0);

  // Map the original 7-element external guess
  // [x, y, z, rx, ry, rz, time] to the reduced optimizer vector.
  if (config_.inital_guess.size() > 0) x[0] = config_.inital_guess[0];
  if (config_.inital_guess.size() > 1) x[1] = config_.inital_guess[1];
  if (config_.inital_guess.size() > 3) x[2] = config_.inital_guess[3];
  if (config_.inital_guess.size() > 4) x[3] = config_.inital_guess[4];
  if (config_.inital_guess.size() > 5) x[4] = config_.inital_guess[5];
  if (config_.time_cal && config_.inital_guess.size() > 6) {
    x[5] = config_.inital_guess[6];
  }

  std::cout << "Fixed Z translation: " << opt_data.fixed_z << " m ("
            << (requested_local_only
                    ? "local=true -> inital_guess[2]"
                    : "local=false -> forced 0.0")
            << ")" << std::endl;

  if (!config_.local) {
    std::cout << "\nPerforming Global Rotation Optimization..." << std::endl;
    std::cout << "Global objective point cap: "
              << config_.global_optimization_max_points << std::endl;

    std::vector<double> lb = {-M_PI, -M_PI, -M_PI};
    std::vector<double> ub = { M_PI,  M_PI,  M_PI};
    std::vector<double> global_x = {x[2], x[3], x[4]};

    optimize(lb, ub, &opt_data, &global_x);

    FILE* tty = fopen("/dev/tty", "w");
    if (tty != nullptr) {
      fprintf(tty, "\n");
      fclose(tty);
    }

    // Switch the objective and optimizer to local mode for refinement.
    // IMPORTANT: fixed_z remains 0.0 because requested_local_only=false.
    config_.local = true;
    x[2] = global_x[0];
    x[3] = global_x[1];
    x[4] = global_x[2];
  }

  std::cout << "\nPerforming Local Optimization..." << std::endl;
  std::cout << "Local objective point cap: "
            << config_.local_optimization_max_points << std::endl;

  // Bounds correspond to reduced vector [x, y, rx, ry, rz, (time)].
  std::vector<double> lb = {
      -config_.translation_range,
      -config_.translation_range,
      -config_.angular_range,
      -config_.angular_range,
      -config_.angular_range};

  std::vector<double> ub = {
      config_.translation_range,
      config_.translation_range,
      config_.angular_range,
      config_.angular_range,
      config_.angular_range};

  for (size_t i = 0; i < 5; ++i) {
    lb[i] += x[i];
    ub[i] += x[i];
  }

  if (config_.time_cal) {
    ub.push_back(config_.max_time_offset);
    lb.push_back(-config_.max_time_offset);
    x[5] = std::max(lb[5], std::min(ub[5], x[5]));
  }

  optimize(lb, ub, &opt_data, &x);

  FILE* tty = fopen("/dev/tty", "w");
  if (tty != nullptr) {
    fprintf(tty, "\n");
    fclose(tty);
  }

  if (!config_.output_pointcloud_path.empty()) {
    std::cout << "\nSaving Aligned Pointcloud (full retained cloud)..."
              << std::endl;
    lidar->saveCombinedPointcloud(config_.output_pointcloud_path);
  }

  const double final_time_offset =
      (config_.time_cal && x.size() == 6) ? x[5] : 0.0;
  const std::string output_calibration = generateCalibrationString(
      lidar->getOdomLidarTransform(), final_time_offset);

  if (!config_.output_calibration_path.empty()) {
    std::cout << "\nSaving Calibration File..." << std::endl;
    std::ofstream file(config_.output_calibration_path, std::ofstream::out);
    file << output_calibration;
  }

  std::cout << "\n\e[1mFinal Calibration:\e[0m\n"
            << output_calibration;
}


}  // namespace lidar_align
