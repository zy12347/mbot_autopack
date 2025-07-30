#include "mbot_common.h"
#include "particle.h"
#include "scanmatcher.h"

class GmappingParams {
 public:
  int particle_count = 50;
  float nerf_threshold = 0.5;
  float map_resolution = 0.01;
};

class Gmapping {
 public:
  Gmapping(){};
  Gmapping(GmappingParams& param) : gmap_param_(param) { particles_.clear(); };
  void Initialize(Pose2D& init_pose);
  void ProcessScan(LaserScan& scan, Odom& odom);
  void UpdateSensorData(LaserScan& scan, Odom& odom) {
    laser_scan_ = scan;
    odo_ = odom;
  };
  void Predict();
  void Resample();
  void UpdateMap();
  void OptimizePose(float max_range);
  void ComputeError();
  void GetBestEstimate();

  static void ICP(std::vector<std::pair<float,float>>& p1,std::vector<std::pair<float,float>>& p2, Pose2D& init_pose);

 private:
  std::vector<std::pair<float,float>> Polar2Cartesian(std::vector<float> ranges);
  int particle_num = 0;
  float nerf_threshold = 0;
  uint64_t last_stamp_time_;
  std::vector<Particle> particles_;
  GmappingParams gmap_param_;
  LaserScan laser_scan_;
  Odom odo_;
  ScanMatcher scan_matcher_;
};
