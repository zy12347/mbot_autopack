#pragma once
#include "gmapping.h"

void Gmapping::Initialize(Pose2D& init_pose){
    particles_.reserve(gmap_param_.particle_count);
    for(int i=0;i<gmap_param_.particle_count;i++){
        Particle p(init_pose);
        p.SetWeight(1.0/double(gmap_param_.particle_count));
        particles_.emplace_back(p);
    }
}


void Gmapping::Predict(){
    double delta_t = odo_.stamp - last_stamp_time_;
    last_stamp_time_ = odo_.stamp;
    double delta_d = odo_.linear_x * delta_t;
    double delta_theta = odo_.angular_z * delta_t;
    for(auto p:particles_){
        Pose2D last_pose = p.GetPose();
        Pose2D cur_pose;
        cur_pose.x = last_pose.x + delta_d * cos(cur_pose.GetPhi() + delta_theta / 2);
        cur_pose.y = last_pose.y + delta_d * sin(cur_pose.GetPhi() + delta_theta / 2);
        cur_pose.Phi = cur_pose.GetPhi() + Pose2D::DEG2RAD(delta_theta);
        p.init_pose = cur_pose;
    }
}

void Gmapping::ProcessScan(LaserScan& scan, Odom& odom){
    UpdateSensorData(LaserScan& scan, Odom& odom);
    Predict();
    float max_range = scan.range_max;
    OptimizePose(float max_range);

    std::cout << scan.range_min << std::endl;

    std::cout << pose.x << std::endl;
    return ;
}

void Gmapping::Resample(){

}
void Gmapping::UpdateMap(){
    Pose2D best_pose = GetBestEstimate();
    
}
void Gmapping::OptimizePose(float max_range){
    std::vector<std::pair<float,float>> points = Polar2Cartesian(laser_scan_.ranges);
    for(auto p:particles_){
        std:vector<std::pair<float,float>> points_scan = p.ScanMap(max_range);
        scan_matcher_.ICP(points,points_scan);
    }

}
void Gmapping::ComputeError(){
    raycast();
    diff = 
    prob = Gaussian()
    weight = ;
    normalized_weight = ;
}
void Gmapping::GetBestEstimate(){

}

std::vector<std::pair<float, float>> Polar2Cartesian(std::vector<float>& ranges) {
  std::vector<std::pair<float, float>> points;
  for (int i = 0; i < ranges.size(); i++) {
    float x = cos(Pose2D::DEG2RAD(i)) * range[i];
    float y = sin(Pose2D::DEG2RAD(i)) * range[i];
    std::pair<float, float> point(x, y);
    points.emplace_back(point);
  }
}