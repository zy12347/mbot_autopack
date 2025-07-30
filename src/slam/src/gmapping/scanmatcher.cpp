#include "scanmatcher.h"

void ScanMatcher::ICP(std::vector<std::pair<float,float>>& p_s,std::vector<std::pair<float,float>>& p_d,Pose2D& init_pose){
    int max_iteration = 10;
    float rotation_epsilon = 1e-4;
    float translation_epsilon = 1e-4;
    if(p2.empty() || p1.empty()){
        std::cout<<"error"<<std::endl;
        return ;
    }
    Eigen::Matrix3d T = GetTransForm(Pose2D& init_pose);
    std::vector<std::pair<int,int>> tranformed_pts = TransFormPoints(p_s,T);
    Pose2d optimized_pose;
    for(int i=0;i<max_iteration;i++){
        std::vector<std::pair<int,int>> corresponce_pts;
        FindCorresponcePoints(tranformed_pts,p_d,corresponce_pts);
        delta_T = ComputeOptimalTransform(corresponce_pts);
        T= delta_t * T;
        tranformed_pts = TransFormPoints(p_s,T);
        phi = atan2(T(1,0),T(0,0));
        x = T(0,2);
        y = T(1,2);
        float phi_err = phi - init_pose.GetPhi();
        float x_err = x - init_pose.x;
        float y_err = y - init_pose.y;
        if(theta_err < eplison && x_err < eplison && y_err < eplison){
            return ;
        }
        optimized_pose = Pose2D(x,y,phi);
    }
    init_pose = optimized_pose;
}
// 该函数根据初始位姿生成对应的齐次变换矩阵（3x3）
Eigen::Matrix3d ScanMatcher::GetTransForm(Pose2D& init_pose) {
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    float phi = init_pose.GetPhi();
    float x = init_pose.x;
    float y = init_pose.y;
    T(0,0) = cos(phi);
    T(0,1) = -sin(phi);
    T(1,0) = sin(phi);
    T(1,1) = cos(phi);
    T(0,2) = x;
    T(1,2) = y;
    return T;
}

std::vector<std::pair<int,int>> ScanMatcher::TransFormPoints(std::vector<std::pair<float,float>>& points,Eigen::Matrix3d& T){
    std::vector<std::pair<int,int>> transformed_points;
    for(auto p:points){
        Eigen::Vector3d p_homo(p.first,p.second,1);
        Eigen::Vector3d transformed_p = T * p_homo;
        transformed_points.emplace_back(transformed_p(0),transformed_p(1));
    }
    return transformed_points;
}

void ScanMatcher::FindCorresponcePoints(std::vector<std::pair<int,int>>& transformed_pts,std::vector<std::pair<int,int>>& p_d,std::vector<std::pair<int,int>>& corresponce_pts){
    for(auto p:transformed_pts){
        float min_dist = 1000000;
        int min_index = 0;
        for(int i=0;i<p_d.size();i++){
            float dist = sqrt(pow(p.first - p_d[i].first,2) + pow(p.second - p_d[i].second,2));
        }
    }
}

Eigen::Matrix3d ScanMatcher::ComputeOptimalTransform(std::vector<std::pair<int,int>>& corresponce_pts){
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d B = Eigen::Matrix3d::Zero();
    for(auto p:corresponce_pts){
        A(0,0) = p.first;
        A(0,1) = p.second;
        A(1,0) = p.first;
        A(1,1) = p.second;
    }
    return T;
}