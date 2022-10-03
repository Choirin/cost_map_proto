#pragma once
#ifndef __INCLUDE_COST_MAP_SCAN__
#define __INCLUDE_COST_MAP_SCAN__
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

class CostMapScan : public CostMap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CostMapScan()
      : sensor_model_initial_(sensor_model(0.5)),
        sensor_model_hit_(sensor_model(0.7)),
        sensor_model_miss_(sensor_model(0.4)),
        sensor_model_min_(sensor_model(0.12)),
        sensor_model_max_(sensor_model(0.97)),
        scan_range_max_(INFINITY) {
    add("free", sensor_model_initial_);
    add("occupied", sensor_model_initial_);
    add("cost", sensor_model_initial_);
  }
  CostMapScan(const Eigen::Vector2d &origin, const Eigen::Array2i &size,
              const float resolution)
      : CostMap(origin, size, resolution),
        sensor_model_initial_(sensor_model(0.5)),
        sensor_model_hit_(sensor_model(0.7)),
        sensor_model_miss_(sensor_model(0.4)),
        sensor_model_min_(sensor_model(0.12)),
        sensor_model_max_(sensor_model(0.97)),
        scan_range_max_(INFINITY) {
    add("free", sensor_model_initial_);
    add("occupied", sensor_model_initial_);
    add("cost", sensor_model_initial_);
  }
  ~CostMapScan() {}

  void set_sensor_model_hit(const float hit) {
    sensor_model_hit_ = sensor_model(hit);
  }
  void set_sensor_model_miss(const float miss) {
    sensor_model_miss_ = sensor_model(miss);
  }
  void set_sensor_model_range(const float min, const float max) {
    sensor_model_min_ = sensor_model(min);
    sensor_model_max_ = sensor_model(max);
  }
  void set_scan_range_max(const float max) { scan_range_max_ = max; }

  void update(frame_buffer::ScanFrame &scan, const bool &extend_map = true) {
    const auto &translation = scan.translation();
    Eigen::Matrix2Xd points;
    scan.transformed_scan(points);

    if (extend_map) {
      Eigen::Vector2d corner_lb = translation, corner_rt = translation;
      for (int i = 0; i < scan.ranges().size(); ++i) {
        auto &range = scan.ranges()[i];
        if (scan_range_max_ < range) continue;
        auto &&point = points.col(i);
        if (point(0) < corner_lb(0)) corner_lb(0) = point(0);
        if (point(1) < corner_lb(1)) corner_lb(1) = point(1);
        if (corner_rt(0) < point(0)) corner_rt(0) = point(0);
        if (corner_rt(1) < point(1)) corner_rt(1) = point(1);
      }
      extend(corner_lb, corner_rt, sensor_model_initial_);
    }

    Eigen::Array2i center_m, ray_m;
    world_to_map(translation, center_m);
    if (!is_inside("free", center_m)) return;
    for (int i = 0; i < points.cols(); ++i) {
      if (scan_range_max_ < scan.ranges()[i]) continue;
      world_to_map(points.col(i), ray_m);
      bresenham(
          center_m, ray_m,
          [this, &ray_m](const Eigen::Array2i &index) {
            if (!this->is_inside("free", index) || (index == ray_m).all()) return true;
            miss("free", index);
            miss("cost", index);
            return true;
          });
      if (!is_inside("free", ray_m)) continue;
      hit("occupied", ray_m);
      hit("cost", ray_m);
    }
  }

  void save(const std::string &layer, const std::string &image_path) {
    Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic> array =
        data(layer).array();
    array = array.exp() / (1 + array.exp());
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> data =
        ((array < 0.5).cast<uint8_t>() * 255 +
         (0.5 < array).cast<uint8_t>() * 0 +
         (0.5 == array).cast<uint8_t>() * 200)
            .matrix();
    cv::Mat img;
    eigen2cv(data, img);
    cv::flip(img, img, 0);
    cv::imwrite(image_path, img);
  }

 private:
  inline float sensor_model(const float raw) { return logf(raw / (1.0 - raw)); }
  void hit(const std::string &layer, const Eigen::Array2i &index) {
    at(layer, index) += sensor_model_hit_;
    if (sensor_model_max_ < at(layer, index))
      at(layer, index) = sensor_model_max_;
  }
  void miss(const std::string &layer, const Eigen::Array2i &index) {
    at(layer, index) += sensor_model_miss_;
    if (at(layer, index) < sensor_model_min_)
      at(layer, index) = sensor_model_min_;
  }

  float sensor_model_initial_;
  float sensor_model_hit_;
  float sensor_model_miss_;
  float sensor_model_min_;
  float sensor_model_max_;

  float scan_range_max_;
};

}  // namespace cost_map

#endif  // __INCLUDE_COST_MAP_SCAN__
