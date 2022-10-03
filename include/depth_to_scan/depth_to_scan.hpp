#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

#include "frame_buffer/scan_frame.hpp"

class DepthToScan {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DepthToScan(int width, int height, double fx, double fy, double cx, double cy,
              double factor);
  ~DepthToScan() {}

  void set_height(double lower, double upper) {
    lower_height_ = lower;
    upper_height_ = upper;
  }
  void set_range(double lower, double upper) {
    lower_range_ = lower;
    upper_range_ = upper;
  }
  void set_optical_axis_pitch(double optical_axis_pitch) {
    optical_axis_pitch_ = optical_axis_pitch;
  }

  std::shared_ptr<Eigen::VectorXd> angles() { return angles_; }
  void convert(const cv::Mat &depth_image, Eigen::VectorXd &ranges);

 private:
  int width_;
  int height_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double factor_;
  Eigen::VectorXd coeff_;

  double lower_height_;
  double upper_height_;

  double lower_range_;
  double upper_range_;

  double optical_axis_pitch_;

  std::shared_ptr<Eigen::VectorXd> angles_;

  void initialize_coefficient();
};
