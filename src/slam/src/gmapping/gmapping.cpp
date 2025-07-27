#include "gmapping.h"



void Gmapping::Initialize(Pose2D& init_pose){
    particles_.reserve(gmap_param_.particle_count);
    for(int i=0;i<gmap_param_.particle_count;i++){
        Particle p(init_pose);
        p.SetWeight(1.0/double(gmap_param_.particle_count));
        particles_.emplace_back(p);
    }
}

void Gmapping::Predict(Odom& odom){
    double delta_t = 1e-3;
    double delta_d = odom.linear_x * delta_t;
    double delta_theta = odom.angular_z * delta_t;
    for(auto p:particles_){
        Pose2D last_pose = p.GetPose();
        Pose2D cur_pose;
        cur_pose.x = last_pose.x + delta_d * cos(cur_pose.phi + delta_theta / 2);
        cur_pose.y = last_pose.y + delta_d * sin(cur_pose.phi + delta_theta / 2);
        cur_pose.phi = cur_pose.phi + Pose2D::DEG2RAD(delta_theta);
    }
}

void Gmapping::ProcessScan(LaserScan& scan, Pose2D& pose){
    std::cout << scan.range_min << std::endl;

    std::cout << pose.x << std::endl;
    return ;
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