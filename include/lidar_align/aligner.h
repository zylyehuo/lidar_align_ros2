#ifndef LIDAR_ALIGN_ALIGNER_H_
#define LIDAR_ALIGN_ALIGNER_H_

#include <limits>
#include <nlopt.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lidar_align/sensors.h"

namespace lidar_align {
class Aligner {
 public:
  struct Config {
    // local=true  : only run local BOBYQA optimization around inital_guess.
    // local=false : run global rotation search first, then local BOBYQA refinement.
    bool local = false;

    std::vector<double> inital_guess{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double max_time_offset = 0.1;
    double angular_range = 0.5;
    double translation_range = 1.0;
    double max_evals = 200;
    double xtol = 0.0001;

    // IMPORTANT: this is now only a SERIAL processing chunk size. It no longer
    // creates one std::async task per batch. Small values do not increase
    // parallelism and therefore cannot trigger a thread explosion.
    int knn_batch_size = 2000;
    int knn_k = 1;

    float local_knn_max_dist = 0.1;
    float global_knn_max_dist = 1.0;

    // Cap the number of points used by the objective function. The final saved
    // PLY still uses the complete retained point cloud.
    int local_optimization_max_points = 0;   // 0 = use all retained points
    int global_optimization_max_points = 0;  // 0 = use all retained points

    bool time_cal = true;
    std::string output_pointcloud_path = "";
    std::string output_calibration_path = "";
  };

  struct OptData {
    Lidar* lidar;
    Odom* odom;
    Aligner* aligner;
    bool time_cal;

    // Z translation is intentionally not an optimization variable.
    // local=true  -> fixed_z comes from inital_guess[2].
    // local=false -> fixed_z is forced to 0.0 for both Global and Local stages.
    double fixed_z = 0.0;
  };

  explicit Aligner(const Config& config);
  static Config getConfig(rclcpp::Node* nh);
  void lidarOdomTransform(Lidar* lidar, Odom* odom);

 private:
  void optimize(const std::vector<double>& lb,
                const std::vector<double>& ub,
                OptData* opt_data,
                std::vector<double>* x);

  std::string generateCalibrationString(const Transform& T,
                                        double time_offset);

  static float kNNError(const pcl::KdTreeFLANN<Point>& kdtree,
                        const Pointcloud& pointcloud,
                        size_t k,
                        float max_dist,
                        size_t start_idx = 0,
                        size_t end_idx = std::numeric_limits<size_t>::max());

  float lidarOdomKNNError(const Pointcloud& base_pointcloud,
                          const Pointcloud& combined_pointcloud) const;
  float lidarOdomKNNError(const Lidar& lidar) const;

  static double LidarOdomMinimizer(const std::vector<double>& x,
                                   std::vector<double>& grad,
                                   void* f_data);

  Config config_;
};
}  // namespace lidar_align

#endif  // LIDAR_ALIGN_ALIGNER_H_
