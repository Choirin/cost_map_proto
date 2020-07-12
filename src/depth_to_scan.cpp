#include "depth_to_scan/depth_to_scan.hpp"

DepthToScan::DepthToScan(int width, int height, double fx, double fy, double cx,
                         double cy, double factor)
    : width_(width),
      height_(height),
      fx_(fx),
      fy_(fy),
      cx_(cx),
      cy_(cy),
      factor_(factor),
      lower_height_(-0.05),
      upper_height_(0.0),
      lower_range_(0.2),
      upper_range_(2.0),
      optical_axis_pitch_(DEG2RAD(10)) {
  initialize_coefficient();
}

std::shared_ptr<std::vector<float>> DepthToScan::convert(const cv::Mat &depth_image) {
  std::shared_ptr<std::vector<float>> ranges(
      new std::vector<float>(width_, INFINITY));

  auto pdepth = depth_image.ptr<uint16_t>(0);
  double *pcoeff = coeff_.data();
  for (int v = 0; v < height_; ++v) {
    for (int u = 0; u < width_; ++u, ++pdepth) {
      auto range = (*pdepth) * *(pcoeff++);
      auto y = (*pdepth) * *(pcoeff++);
      if (lower_height_ < y && y < upper_height_ &&
          lower_range_ < range && range < upper_range_ &&
          range < (*ranges)[u])
        (*ranges)[u] = range;
    }
  }
  return ranges;
}

void DepthToScan::initialize_coefficient() {
  angles_ = std::make_shared<std::vector<float>>(width_, 0);

  coeff_.resize(2 * width_ * height_);
  double *pcoeff = coeff_.data();
  double s_pitch = sin(optical_axis_pitch_);
  double c_pitch = cos(optical_axis_pitch_);
  for (int v = 0; v < height_; ++v) {
    for (int u = 0; u < width_; ++u) {
      double &range = *(pcoeff++);
      double &y = *(pcoeff++);
      double z;

      // camera coordinate
      auto x_cam = (u - cx_) / fx_ / factor_;
      auto y_cam = (v - cy_) / fy_ / factor_;
      auto z_cam = 1.0 / factor_;

      // TODO: 高さ(y)方向の変換がなんかおかしそう
      // base coordinate
      y = y_cam * c_pitch - z_cam * s_pitch;
      z = y_cam * s_pitch + z_cam * c_pitch;
      range = sqrt(x_cam * x_cam + z * z);
      if (v == int(height_ / 2))
        (*angles_)[u] = atan2(x_cam, z);
    }
  }
}
