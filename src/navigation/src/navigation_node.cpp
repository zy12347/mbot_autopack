#include <cstdio>
#include "Astar.h"
#include "mbot_interface/mbot_common.h"

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  printf("hello world navigation package\n");
  cv::Mat img = cv::imread("grid_map.bmp", 0);
  if (img.empty()) {
    printf("cannot read image file: grid_map.bmp\n");
    return -1;
  }
  Astar astar(img.cols, img.rows, img.data);
  std::vector<std::pair<int, int>> paths =
      astar.PlanPath(0, 0, img.cols - 1, 0);
  astar.PlotMap(paths);

  // cv::imshow("img", img);
  // cv::waitKey(0);
  return 0;
}
