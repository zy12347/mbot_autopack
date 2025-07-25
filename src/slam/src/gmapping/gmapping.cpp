#include "gmapping.h"


void Gmapping::Initialize(Pose2D& init_pose){
    particles_.reserve(gmapp_param.particle_count);
    for(int i=0;i<gmapp_param.particle_count;i++){
        Particle p(init_pose);
        p.SetWeight(1.0/double(gmapp_param.particle_count));
        particles_.emplace_back(p);
    }
}

void Gmapping::Predict(Odom& odo){
    double delta_d = odom.twist.linear.x * delta_t;
    double delta_theta = odom.twist.angular.z * delta_t;
    for(auto p:particles){
        Pose2D last_pose = p.GetPose();
        Pose2D cur_pose;
        cur_pose.x = last_pose.x + delta_d * cos(cur_pose.phi + delta_theta / 2);
        cur_pose. = last_pose.y + delta_d * sin(cur_pose.phi + delta_theta / 2);
        cur_pose.phi = cur_pose.phi + DEG2RAD(delta_theta);
    }
}

void Gmapping::ProcessScan(LaserSacn& scan, Pose2D& pose){

}

void Gmapping::Predict(){

}
void Gmapping::Resample(){

}
void Gmapping::UpdateMap(){

}
void Gmapping::OptimizePose(){

}
void Gmapping::ComputeError(){

}
void Gmapping::GetBestEstimate(){

}