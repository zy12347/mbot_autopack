#pragma once
#include <cstdio>
#include "Astar.h"
#include "mbot_interface/mbot_common.h"
int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world navigation package\n");
  cv::Mat img = cv::imread("grid_map.bmp",0);
  Astar astar(img.cols, img.rows, img.data);
  astar.PlanPath(0, 0, img.cols - 1, img.rows - 1);
  
  // cv::imshow("img", img);
  // cv::waitKey(0);
  return 0;
}
