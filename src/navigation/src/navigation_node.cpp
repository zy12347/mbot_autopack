#include <cstdio>
#include "Astar.h"
#include "RandomTree.h"
#include "mbot_interface/mbot_common.h"

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  printf("hello world navigation package\n");
  cv::Mat img = cv::imread("img/grid_map.bmp", 0);
  if (img.empty()) {
    printf("cannot read image file: grid_map.bmp\n");
    return -1;
  }
  // Astar astar(img.cols, img.rows, img.data);
  // std::vector<std::pair<int, int>> paths =
  //     astar.PlanPath(0, 0, img.cols - 1, 0);
  // astar.PlotMap(paths);

  RandomTree rrt(img.cols, img.rows, img.data);
  std::vector<std::pair<int, int>> paths =
      rrt.PlanPath(0, 0, img.cols - 1, img.rows - 1, 5, 5000);
  if (paths.empty()) {
    printf("cannot find path\n");
  }
  rrt.PlotTree();
  rrt.PlotMap(paths);
  // cv::imshow("img", img);
  // cv::waitKey(0);
  return 0;
}
