#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

#include "frame_buffer/scan_frame.hpp"

class DepthToScan {
 public:
  DepthToScan(int width, int height, double fx, double fy, double cx, double cy,
              double factor);
  ~DepthToScan() {}

  std::shared_ptr<std::vector<float>> angles() { return angles_; }

  std::shared_ptr<std::vector<float>> convert(const cv::Mat &depth_image);

 private:
  int width_;
  int height_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double factor_;
  std::vector<double> coeff_;

  double lower_height_;
  double upper_height_;

  double lower_range_;
  double upper_range_;

  double optical_axis_pitch_;

  std::shared_ptr<std::vector<float>> angles_;

  void initialize_coefficient();
};
