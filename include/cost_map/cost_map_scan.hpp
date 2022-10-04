#pragma once
#ifndef COST_MAP_COST_MAP_SCAN_HPP
#define COST_MAP_COST_MAP_SCAN_HPP
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cost_map/cost_map.hpp>
#include <frame_buffer/scan_frame.hpp>
#include <iostream>
#include <iterator>
#include <memory>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <type_traits>
#include <vector>

namespace cost_map {

inline float logit(const float raw) { return logf(raw / (1.0 - raw)); }

class CostMapScan : public CostMap<float> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CostMapScan(const float initial = 0.5, const float hit = 0.7,
              const float miss = 0.4, const float min = 0.12,
              const float max = 0.97)
      : sensor_model_initial_(logit(initial)),
        sensor_model_hit_(logit(hit)),
        sensor_model_miss_(logit(miss)),
        sensor_model_min_(logit(min)),
        sensor_model_max_(logit(max)),
        scan_range_max_(INFINITY) {
    add("free", sensor_model_initial_);
    add("occupied", sensor_model_initial_);
    add("cost", sensor_model_initial_);
  }
  CostMapScan(const Eigen::Vector2d &origin, const Eigen::Array2i &size,
              const float resolution, const float initial = 0.5,
              const float hit = 0.7, const float miss = 0.4,
              const float min = 0.12, const float max = 0.97)
      : CostMap(origin, size, resolution),
        sensor_model_initial_(logit(initial)),
        sensor_model_hit_(logit(hit)),
        sensor_model_miss_(logit(miss)),
        sensor_model_min_(logit(min)),
        sensor_model_max_(logit(max)),
        scan_range_max_(INFINITY) {
    add("free", sensor_model_initial_);
    add("occupied", sensor_model_initial_);
    add("cost", sensor_model_initial_);
  }
  ~CostMapScan() {}

  void set_scan_range_max(const float max) { scan_range_max_ = max; }

  void update(frame_buffer::ScanFrame &scan, const bool &expand_map = true) {
    const auto &translation = scan.translation();
    Eigen::Matrix2Xd points;
    scan.transformed_scan(points);

    const Eigen::Array<bool, 1, Eigen::Dynamic> mask =
        scan.ranges().array() < scan_range_max_;
    if (expand_map) {
      expand(points, mask, sensor_model_initial_);
      expand(Eigen::Matrix2Xd(translation),
             Eigen::Array<bool, 1, 1>::Constant(true), sensor_model_initial_);
    }
    project(points, mask, translation);
  }

  void update(frame_buffer::ScanFrame &scan,
              const Eigen::Matrix3d &external_transform,
              const bool &expand_map = true) {
    Eigen::Vector2d translation = scan.translation();
    translation = external_transform.block<2, 2>(0, 0) * translation +
                  external_transform.block<2, 1>(0, 2);
    Eigen::Matrix2Xd points;
    scan.transformed_scan(points, external_transform);

    const Eigen::Array<bool, 1, Eigen::Dynamic> mask =
        scan.ranges().array() < scan_range_max_;
    if (expand_map) {
      expand(points, mask, sensor_model_initial_);
      expand(Eigen::Matrix2Xd(translation),
             Eigen::Array<bool, 1, 1>::Constant(true), sensor_model_initial_);
    }
    project(points, mask, translation);
  }

  // for debug use only
  bool save(const std::string &layer, const std::string &image_path) {
    if (!has(layer)) return false;

    Eigen::Array2i size;
    get_size(size);
    if ((size == 0).any()) return false;

    Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic> array =
        data(layer).array();
    array = array.exp() / (1 + array.exp());
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> data =
        ((array < 0.196).cast<uint8_t>() * 255 +
         (0.65 < array).cast<uint8_t>() * 0 +
         (0.196 <= array && array <= 0.65).cast<uint8_t>() * 200)
            .matrix();
    cv::Mat img;
    eigen2cv(data, img);
    cv::flip(img, img, 0);
    cv::imwrite(image_path, img);
    return true;
  }

 private:
  void hit(const std::string &layer, const Eigen::Array2i &index) {
    auto &cell = at(layer, index);
    cell += sensor_model_hit_;
    if (sensor_model_max_ < cell) cell = sensor_model_max_;
  }
  void miss(const std::string &layer, const Eigen::Array2i &index) {
    auto &cell = at(layer, index);
    cell += sensor_model_miss_;
    if (cell < sensor_model_min_) cell = sensor_model_min_;
  }

  void project(const Eigen::Matrix2Xd &points,
               const Eigen::Array<bool, Eigen::Dynamic, 1> &mask,
               const Eigen::Vector2d &center) {
    assert(points.cols() == mask.size());

    Eigen::Array2i center_m;
    world_to_map(center, center_m);
    if (!is_inside(center_m)) return;

    for (int i = 0; i < points.cols(); ++i) {
      if (!mask[i]) continue;

      Eigen::Array2i ray_m;
      world_to_map(points.col(i), ray_m);
      bresenham(center_m, ray_m, [this, &ray_m](const Eigen::Array2i &index) {
        if (!this->is_inside(index)) return true;
        if ((index == ray_m).all()) {
          hit("occupied", index);
          hit("cost", index);
        } else {
          miss("free", index);
          miss("cost", index);
        }
        return true;
      });
    }
  }

  const float sensor_model_initial_;
  const float sensor_model_hit_;
  const float sensor_model_miss_;
  const float sensor_model_min_;
  const float sensor_model_max_;

  float scan_range_max_;
};

}  // namespace cost_map

#endif  // COST_MAP_COST_MAP_SCAN_HPP
