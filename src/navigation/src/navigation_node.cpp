#include <cstdio>
#include "Astar.h"
#include "RandomTree.h"
#include "dynamic_simulate.h"
#include "mbot_interface/mbot_common.h"

class NavigationManager : public rclcpp ::Node {
 public:
  NavigationManager() : Node("navigation") {
    map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, [this](const nav_msgs::msg::OccupancyGrid& msg) {
          this->map_event_callback(msg);
        });
    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/map_with_path", 10);
  }

 private:
  void map_event_callback(const nav_msgs::msg::OccupancyGrid& msg) {
    resolution_ = msg.info.resolution;
    width_ = msg.info.width;
    height_ = msg.info.height;
    std::cout << "received map " << "height " << height_ << " width " << width_
              << " resolution " << resolution_ << std::endl;
    if (map_data_ == nullptr) {
      map_data_ = new uint8_t[width_ * height_];
    }
    // std::memcpy(map_data_, msg.data.data(), width_ * height_ *
    // sizeof(int8_t));
    for (size_t i = 0; i < width_ * height_; ++i) {
      int8_t value = msg.data[i];
      map_data_[i] = (value == -1)  ? 127
                     : (value < 0)  ? 0
                     : value == 100 ? 255
                                    : value;
    }
    PlanPath();
  }

  void PlanPath() {
    Astar astar(width_, height_, map_data_);
    std::vector<std::pair<int, int>> path =
        astar.PlanPath(1, 1, width_ - 2, height_ - 2);
    astar.PrintPath(path);
    for (auto& p : path) {
      map_data_[p.second + width_ + p.first] = 127;
    }
    if (map_publisher_) {
      GridMap grid(map_data_, width_, height_, resolution_);
      map_publisher_->publish(grid.ToRosMsg());
    }
  }
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  GridMap map_;
  uint8_t* map_data_;
  int width_;
  int height_;
  float resolution_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  // printf("hello world navigation package\n");
  // cv::Mat img = cv::imread("img/grid_map.bmp", 0);
  // if (img.empty()) {
  //   printf("cannot read image file: grid_map.bmp\n");
  //   return -1;
  // }

  // Robot bot(0, 0, 0, 0, 0, 0.5, 0.3, 0.15);
  // Robot goal(4, 4, 0, 0, 0);
  // DynamicEnvironment<Robot> dyna_env(0.1, img.data, img.cols, img.rows);
  // dyna_env.setRobotState(bot);
  // dyna_env.setGoal(goal);
  // dyna_env.addObstacles(5);
  // dyna_env.run(1000);
  // Astar astar(img.cols, img.rows, img.data);
  // std::vector<std::pair<int, int>> paths =
  //     astar.PlanPath(0, 0, img.cols - 1, 0);
  // astar.PlotMap(paths);

  // RandomTree rrt(img.cols, img.rows, img.data);
  // std::vector<std::pair<int, int>> paths =
  //     rrt.PlanPath(0, 0, img.cols - 1, img.rows - 1, 5, 5000);
  // if (paths.empty()) {
  //   printf("cannot find path\n");
  // }
  // rrt.PlotTree();
  // rrt.PlotMap(paths);
  // cv::imshow("img", img);
  // cv::waitKey(0);
  return 0;
}
