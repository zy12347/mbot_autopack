#include <Eigen/Dense>
#include "mbot_common.h"

class ScanMatcher{
    public:
    // ICP算法：输入源点云、目标点云和初始位姿，输出优化后的位姿
    void ICP(std::vector<std::pair<float, float>>& p_s, std::vector<std::pair<float, float>>& p_d, Pose2D& init_pose);

    // 根据初始位姿生成齐次变换矩阵（3x3）
    Eigen::Matrix3d GetTransForm(Pose2D& init_pose);

    // 对点云进行变换
    std::vector<std::pair<int, int>> TransFormPoints(std::vector<std::pair<float, float>>& points, Eigen::Matrix3d& T);

    // 寻找最近点对应关系
    void FindCorresponcePoints(std::vector<std::pair<int, int>>& transformed_pts, std::vector<std::pair<int, int>>& p_d, std::vector<std::pair<int, int>>& corresponce_pts);

    // 计算最优变换
    Eigen::Matrix3d ComputeOptimalTransform(std::vector<std::pair<int, int>>& corresponce_pts);
};