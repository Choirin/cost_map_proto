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
      lower_height_(-INFINITY),
      upper_height_(INFINITY),
      lower_range_(0),
      upper_range_(INFINITY),
      optical_axis_pitch_(0) {
  initialize_coefficient();
}

void DepthToScan::convert(const cv::Mat &depth_image,
                          Eigen::VectorXd &ranges) {
  ranges = Eigen::VectorXd::Constant(width_, INFINITY);

  // the image is upside down.
  auto pdepth = depth_image.ptr<uint16_t>(0) + height_ * width_ - 1;
  double *pcoeff = coeff_.data();
  for (int v = 0; v < height_; ++v) {
    for (int u = 0; u < width_; ++u, --pdepth) {
      auto range = (*pdepth) * *(pcoeff++);
      auto y = (*pdepth) * *(pcoeff++);
      // store minimum range with valid conditions
      if (lower_height_ < y && y < upper_height_ && lower_range_ < range &&
          range < upper_range_ && range < ranges[u])
        ranges[u] = range;
    }
  }
}

void DepthToScan::initialize_coefficient() {
  angles_ = std::shared_ptr<Eigen::VectorXd>(new Eigen::VectorXd(width_));

  coeff_.resize(2 * width_ * height_);
  auto *pcoeff = coeff_.data();
  auto s_pitch = sin(optical_axis_pitch_);
  auto c_pitch = cos(optical_axis_pitch_);
  for (int v = 0; v < height_; ++v) {
    for (int u = 0; u < width_; ++u) {
      auto &range = *(pcoeff++);
      auto &z = *(pcoeff++);

      // camera coordinate
      // (x: right, y: down, z: optical)
      auto x_cam = (u - cx_) / fx_ / factor_;
      auto y_cam = (v - cy_) / fy_ / factor_;
      auto z_cam = 1.0 / factor_;

      // base coordinate
      // (x: front, y: left, z: up)
      auto x = y_cam * s_pitch + z_cam * c_pitch;
      auto y = -x_cam;
      z = -(y_cam * c_pitch - z_cam * s_pitch);
      range = sqrt(x * x + y * y);
      if (v == int(height_ / 2)) (*angles_)[u] = atan2(y, x);
    }
  }
}
