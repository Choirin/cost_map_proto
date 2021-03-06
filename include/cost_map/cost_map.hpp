#pragma once
#ifndef __INCLUDE_COST_MAP__
#define __INCLUDE_COST_MAP__
#include <algorithm>
#include <iostream>
#include <iterator>
#include <memory>
#include <type_traits>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <frame_buffer/scan_frame.hpp>

namespace cost_map {

class CostMap {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CostMap()
      : origin_(Eigen::Vector2d::Zero()),
        size_(Eigen::Array2i::Zero()),
        resolution_(0.05) {}
  ~CostMap() {}

  void set_origin(const Eigen::Vector2d &origin) { origin_ = origin; }
  void set_size(const Eigen::Array2i &size) { size_ = size; }

  void get_origin(Eigen::Vector2d &origin) { origin = origin_; }
  float get_resolution() { return resolution_; }

  void resize(const Eigen::Array2i &size, const Eigen::Vector2d &origin, const float value) {
    Eigen::Array2i offset_m = ((origin - origin_) / resolution_).cast<int>().array();
    // overlap on the original map coords
    Eigen::Array2i old_lb = offset_m.max(Eigen::Array2i::Zero());
    Eigen::Array2i old_rt = (offset_m + size).min(size_);
    // std::cout << "old_rect: "
    //           << old_lb(0) << ", " << old_lb(1) << ", "
    //           << old_rt(0) << ", " << old_rt(1) << std::endl;

    // overlap on the new map coords
    Eigen::Array2i new_lb = (-offset_m).max(Eigen::Array2i::Zero());
    Eigen::Array2i new_rt = (-offset_m + size_).min(size);
    // std::cout << "new_rect: "
    //           << new_lb(0) << ", " << new_lb(1) << ", "
    //           << new_rt(0) << ", " << new_rt(1) << std::endl;

    // overlap size
    Eigen::Array2i overlap_size = new_rt - new_lb;
    Eigen::Array2i overlap_size_ = old_rt - old_lb;
    // std::cout << "new: " << overlap_size(0) << ", " << overlap_size(1) << std::endl;
    // std::cout << "old: " << overlap_size_(0) << ", " << overlap_size_(1) << std::endl;
    assert((overlap_size == overlap_size_).all());

    // update geometry
    size_ = size;
    origin_ = origin;

    // copy overlapped from the old data
    for (const auto& layer : data_) {
      auto data =
          std::make_shared<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>>(
              Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Constant(
                  size(0), size(1), value));
      if ((overlap_size > Eigen::Array2i::Zero()).all()) {
        data->block(new_lb(0), new_lb(1), overlap_size(0), overlap_size(1)) =
            data_[layer.first]->block(old_lb(0), old_lb(1), overlap_size(0), overlap_size(1));
      }
      data_[layer.first] = data;
    }
  }

  void extend(const Eigen::Vector2d &corner_lb, const Eigen::Vector2d &corner_rt, const float value) {
    Eigen::Array2d new_corner_lb, new_corner_rt;
    if ((size_ == Eigen::Array2i::Zero()).any()) {
      new_corner_lb = (corner_lb.array() / resolution_).floor();
      new_corner_rt = (corner_rt.array() / resolution_).ceil();
    } else {
      Eigen::Array2d old_corner_lb = origin_.array();
      Eigen::Array2d old_corner_rt = origin_.array() + size_.cast<double>() * resolution_;
      new_corner_lb = (corner_lb.array().min(old_corner_lb) / resolution_).floor();
      new_corner_rt = (corner_rt.array().max(old_corner_rt) / resolution_).ceil();
    }
    Eigen::Vector2d origin = (new_corner_lb * resolution_).matrix();
    Eigen::Array2i size = (new_corner_rt - new_corner_lb + 0.5).cast<int>();
    // std::cout << "origin: " << origin(0) << ", " << origin(1) << std::endl;
    // std::cout << "size: " << size(0) << ", " << size(1) << std::endl;
    resize(size, origin, value);
  }

  void add(const std::string &layer, const float value) {
    data_[layer] =
        std::make_shared<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>>(
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Constant(
                size_(0), size_(1), value));
  }

  void add(const std::string &layer,
           const std::shared_ptr<
               Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>>
               data) {
    assert(size_(0) == data->rows());
    assert(size_(1) == data->cols());
    data_[layer] = data;
  }

  inline void world_to_map(const Eigen::Vector2d &position_w, Eigen::Array2i &index_m) const{
    Eigen::Vector2d position_m =
        (position_w - origin_) / resolution_ + Eigen::Vector2d::Constant(0.5);
    index_m = position_m.array().floor().cast<int>();
  }

  inline void map_to_world(const Eigen::Array2i &index_m, Eigen::Vector2d &position_w) const{
    position_w = index_m.matrix().cast<double>() * resolution_ + origin_;
  }

  inline bool is_inside(const std::string &layer, const Eigen::Array2i &index) {
    auto &data = data_[layer];
    return (0 <= index(0) && index(0) < data->rows() &&
            0 <= index(1) && index(1) < data->cols());
  }

  std::shared_ptr<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> data(const std::string &layer) {
    return data_[layer];
  }

  float &at(const std::string &layer, const Eigen::Array2i &index) {
    auto &data = data_[layer];
    assert(0 <= index(0) && index(0) < data->rows());
    assert(0 <= index(1) && index(1) < data->cols());
    return (*data)(index(0), index(1));
  }

  void save(const std::string &layer, const std::string &image_path) {
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> data =
        (*data_[layer] * 255).cast<uint8_t>();
    cv::Mat img;
    eigen2cv(data, img);
    cv::imwrite(image_path, img);
  }

protected:
  template <typename ActionType>
  inline void bresenham(const Eigen::Array2i &a, const Eigen::Array2i &b,
                        ActionType action) const{
    static_assert(
        std::is_same<std::invoke_result_t<ActionType, const Eigen::Array2i &>,
                     bool>::value,
        "Expected functor-type bool(const Eige::Array2i &)");
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

private:
  std::unordered_map<std::string, std::shared_ptr<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>>> data_;
  Eigen::Vector2d origin_;
  Eigen::Array2i size_;
  float resolution_;
};

}  // namespace cost_map

#endif  // __INCLUDE_COST_MAP__
