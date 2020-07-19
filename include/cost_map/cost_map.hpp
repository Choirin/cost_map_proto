#pragma once
#ifndef __INCLUDE_COST_MAP__
#define __INCLUDE_COST_MAP__
#include <algorithm>
#include <iostream>
#include <iterator>
#include <memory>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <frame_buffer/scan_frame.hpp>

namespace cost_map {

class CostMap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CostMap() : resolution_(0.05) {
    origin_ << 250 * resolution_, 250 * resolution_;
    data_ = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Constant(500, 500, 200.0 / 255);
  }
  ~CostMap() {}

  inline bool world_to_map(const Eigen::Vector2d &position_w, Eigen::Array2i &index_m) const{
    Eigen::Vector2d position_m =
        (position_w + origin_) / resolution_ + Eigen::Vector2d::Constant(0.5);
    auto &mx_f = position_m.data()[0];
    auto &my_f = position_m.data()[1];
    if (0 <= mx_f && mx_f < data_.rows() &&
        0 <= my_f && my_f < data_.cols()) {
      index_m = position_m.array().cast<int>();
      return true;
    }
    return false;
  }

  float &at(const Eigen::Array2i &index) {
    assert(0 <= index(0) && index(0) < data_.rows());
    assert(0 <= index(1) && index(1) < data_.cols());
    return data_(index(0), index(1));
  }

  void save(const std::string &image_path) {
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> data =
        (data_ * 255).cast<uint8_t>();
    cv::Mat img;
    eigen2cv(data, img);
    cv::imwrite(image_path, img);
  }

  void update(const std::shared_ptr<frame_buffer::ScanFrame> &scan) {
    Eigen::Vector2d offset;
    offset << 10, 11;
    const auto &translation = scan->translation();
    Eigen::Array2i center_m, ray_m;
    if (!world_to_map(translation - offset, center_m)) return;
    std::vector<Eigen::Vector2d> points;
    scan->transformed_scan(points);
    for (const auto &point : points) {
      // TODO: check range
      if (!world_to_map(point - offset, ray_m)) continue;
      bresenham(center_m, ray_m,
                [this](Eigen::Array2i index) {
                  at(index) = 1;
                });
      at(ray_m) = 0;
    }
  }

 private:
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> data_;
  Eigen::Vector2d origin_;
  double resolution_;

  template <typename ActionType>
  inline void bresenham(const Eigen::Array2i &a, const Eigen::Array2i &b,
                        ActionType action) const{
    Eigen::Array2i index = a;
    auto &xa = index.data()[0], &ya = index.data()[1];
    auto &xb = b.data()[0], &yb = b.data()[1];
    int dx = abs(xb - xa), sx = xa < xb ? 1 : -1;
    int dy = -abs(yb - ya), sy = ya < yb ? 1 : -1;
    int err = dx + dy, e2; /* error value e_xy */

    for (;;) { /* loop */
      action(index);
      if (xa == xb && ya == yb) break;
      e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        xa += sx;
      } /* e_xy+e_x > 0 */
      if (e2 <= dx) {
        err += dx;
        ya += sy;
      } /* e_xy+e_y < 0 */
    }
  }
};

}  // namespace cost_map

#endif  // __INCLUDE_COST_MAP__
