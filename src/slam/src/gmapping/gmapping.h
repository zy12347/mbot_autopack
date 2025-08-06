#include "mbot_common.h"
#include "particle.h"
#include "scanmatcher.h"

class GmappingParams {
 public:
  int particle_count = 50;
  float nerf_threshold = 0.5;
  float map_resolution = 0.05;
};

class Gmapping {
 public:
  Gmapping();
  Gmapping(GmappingParams& param) : gmap_param_(param) { particles_.clear(); };

  // 拷贝构造函数
  Gmapping(const Gmapping& other);

  // 赋值运算符
  Gmapping& operator=(const Gmapping& other);

  // 析构函数
  ~Gmapping();

  void Initialize(Pose2D& init_pose);
  void ProcessScan(LaserScan& scan, Odom& odom);
  void UpdateSensorData(LaserScan& scan, Odom& odom) {
    laser_scan_ = scan;
    // RCLCPP_INFO(rclcpp::get_logger("gmapping"),
    //             "laser_scan_.min_angle = %f, laser_scan_.max_angle = %f",
    //             laser_scan_.angle_min, laser_scan_.angle_max);
    // for (int i = 0; i < laser_scan_.ranges.size(); i++) {
    //   RCLCPP_INFO(rclcpp::get_logger("gmapping"), "laser_scan_.ranges[%d] =
    //   %f",
    //               i, laser_scan_.ranges[i]);
    // }
    odo_ = odom;
  };
  void Predict();
  void Resample();
  void UpdateMap();
  void OptimizePose();
  Pose2D GetBestEstimate();
  void computeAndNormalizeWeights();
  float ComputeParticleWeight(Particle& particle);

  static void ICP(std::vector<std::pair<float, float>>& p1,
                  std::vector<std::pair<float, float>>& p2, Pose2D& init_pose);
  GridMap& GetGridMap() { return grid_map_; };

 private:
  std::vector<std::pair<float, float>> Polar2Cartesian(
      std::vector<float>& ranges);

  GridMap grid_map_;
  int particle_num = 0;
  float nerf_threshold_ = 0.5;
  uint64_t last_stamp_time_ = 0;
  std::vector<Particle> particles_;
  GmappingParams gmap_param_;
  LaserScan laser_scan_;
  Odom odo_;
  ScanMatcher scan_matcher_;
};
