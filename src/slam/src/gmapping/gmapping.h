#include "particle.h"
#include <vector>

class GmappingParams{
public:
    int particle_count = 50;
    float nerf_threshold = 0.5;
    float map_resolution = 0.01;
};

class Gmapping {
 public:
  Gmapping(GmappingParams& param):gmap_param(param){
    particles.clear();
  };
  void Initialize(Pose2D& init_pose);
  void ProcessScan(LaserSacn& scan,Pose2D& pose);
  void Predict(Odom& odom);
  void Resample();
  void UpdateMap();
  void OptimizePose();
  void ComputeError();
  void GetBestEstimate();

 private:
  int particle_num = 0;
  float nerf_threshold = 0;
  std::vector<Particle> particles;
  GmappingParams gmap_param;
}
