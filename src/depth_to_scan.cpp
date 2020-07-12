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
      lower_height_(0),
      upper_height_(height) {
  initialize_matrix();
}

std::shared_ptr<frame_buffer::ScanFrame> DepthToScan::convert(
    const cv::Mat &depth_image) {
  std::shared_ptr<std::vector<float>> ranges(
      new std::vector<float>(width_, INFINITY));

  auto pdepth = depth_image.ptr<unsigned short int>(0);
  double *pcoeff = coeff_.data();
  for (int v = lower_height_; v < upper_height_; ++v) {
    for (int u = 0; u < width_; ++u) {
      auto range = (*pdepth) * *(pcoeff++);
      auto y = (*pdepth) * *(pcoeff++);
      // TODO: if (y in range height)
      if (range < (*ranges)[u]) (*ranges)[u] = range;
    }
  }

  std::shared_ptr<frame_buffer::ScanFrame> scan(
      new frame_buffer::ScanFrame(0.0, angles_, ranges));
  return scan;
}

void DepthToScan::initialize_matrix() {
  angles_ = std::make_shared<std::vector<float>>();
  angles_->reserve(width_);
  for (int u = 0; u < width_; ++u) {
    angles_->emplace_back(tan((u - cx_) / fx_));
  }

  coeff_.resize(2 * width_ * height_);
  double *pcoeff = coeff_.data();
  for (int v = lower_height_; v < upper_height_; ++v) {
    for (int u = 0; u < width_; ++u) {
      // TODO: consider transformation to the baseframe
      *(pcoeff++) = 1.0 / cos((*angles_)[u]);   // range
      *(pcoeff++) = (v - cy_) / fy_ / factor_;  // y axis
    }
  }
}
