#include "ImageProcess.h"

ImageProcess::ImRead(std::string img_path) {
  cv::imread(img_path);
}