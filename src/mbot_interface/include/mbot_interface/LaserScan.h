#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"

class LaserScan{
public:
    rclcpp::Time stamp;
    std::string frame_id;
    float angle_min;
    float angle_max;           // 结束角度（弧度）
    float angle_increment;     // 角度增量（弧度）

    float range_min;           // 最小有效距离（米）
    float range_max;           // 最大有效距离（米）

    std::vector<float> ranges;  // 距离数组（米）
    std::vector<float> intensities;  // 强度数组（可选）

    // 计算指定索引对应的角度
    double get_angle(size_t index) const {
        if (index >= ranges.size()) return 0.0;
        return angle_min + index * angle_increment;
    }

    void FromRosMsg(const sensor_msgs::msg::LaserScan::SharedPtr);

    sensor_msgs::msg::LaserScan toRosMsg() const;
};
