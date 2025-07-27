#include <vector>

#include "particle.h"

#include "mbot_common.h"

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
  void ProcessScan(LaserScan& scan, Pose2D& pose);
  void Predict(Odom& odom);
  void Resample();
  void UpdateMap();
  void OptimizePose();
  void ComputeError();
  void GetBestEstimate();

 private:
  int particle_num = 0;
  float nerf_threshold = 0;
  std::vector<Particle> particles_;
  GmappingParams gmap_param_;
};
