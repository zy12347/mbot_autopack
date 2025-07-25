#include "gridmap.h"
class Particle{
public:
    Particle(Pose2D& init_pose):pose(init_pose){
        map = new GridMap();
    };
    void SetWeight(double w){weight = w;};
    Pose2D GetPose(){return pose;};
private:
    GridMap* map;
    double weight;
    Pose2D pose;
}   