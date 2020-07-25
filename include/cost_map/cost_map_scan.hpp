#pragma once
#ifndef __INCLUDE_COST_MAP_SCAN__
#define __INCLUDE_COST_MAP_SCAN__
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <memory>
#include <type_traits>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <frame_buffer/scan_frame.hpp>
#include <cost_map/cost_map.hpp>

namespace cost_map {

class CostMapScan : public CostMap {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CostMapScan() {
    sensor_model_initial_ = log(0.5 / (1 - 0.5));
    sensor_model_hit_ = log(0.7 / (1 - 0.7));
    sensor_model_miss_ = log(0.4 / (1 - 0.4));

    sensor_model_min_ = log(0.12 / (1 - 0.12));
    sensor_model_max_ = log(0.97 / (1 - 0.97));

    add("free", sensor_model_initial_);
    add("occupied", sensor_model_initial_);
    add("cost", sensor_model_initial_);
  }
  ~CostMapScan() {}

  void update(const std::shared_ptr<frame_buffer::ScanFrame> &scan) {
    const auto &translation = scan->translation();
    std::vector<Eigen::Vector2d> points;
    scan->transformed_scan(points);

    // TODO: update map size
    Eigen::Vector2d corner_lb = translation, corner_rt = translation;
    for (size_t i = 0; i < scan->ranges()->size(); ++i) {
      auto range = (*scan->ranges())[i];
      if (5.0 < range) continue;
      auto &point = points[i];
      if (point(0) < corner_lb(0)) corner_lb(0) = point(0);
      if (point(1) < corner_lb(1)) corner_lb(1) = point(1);
      if (corner_rt(0) < point(0)) corner_rt(0) = point(0);
      if (corner_rt(1) < point(1)) corner_rt(1) = point(1);
    }
    extend(corner_lb, corner_rt, sensor_model_initial_);

    Eigen::Array2i center_m, ray_m;
    world_to_map(translation, center_m);
    if (!is_inside("free", center_m)) return;
    for (const auto &point : points) {
      // TODO: check range
      world_to_map(point, ray_m);
      if (!is_inside("free", ray_m)) continue;
      bresenham(center_m, ray_m,
                [this, &ray_m](const Eigen::Array2i &index) {
                  if ((index == ray_m).all()) return false;
                  miss("free", index);
                  miss("cost", index);
                  return true;
                });
      hit("occupied", ray_m);
      hit("cost", ray_m);
    }
  }

  void save(const std::string &layer, const std::string &image_path) {
    Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic> array = data(layer)->array();
    array = array.exp() / (1 + array.exp());
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> data =
        ((array < 0.5).cast<uint8_t>() * 255 +
         (0.5 < array).cast<uint8_t>() * 0 +
         (0.5 == array).cast<uint8_t>() * 200)
            .matrix();
    cv::Mat img;
    eigen2cv(data, img);
    cv::imwrite(image_path, img);
  }

private:
  void hit(const std::string &layer, const Eigen::Array2i &index) {
    at(layer, index) += sensor_model_hit_;
    if (sensor_model_max_ < at(layer, index)) at(layer, index) = sensor_model_max_;
  }
  void miss(const std::string &layer, const Eigen::Array2i &index) {
    at(layer, index) += sensor_model_miss_;
    if (at(layer, index) < sensor_model_min_) at(layer, index) = sensor_model_min_;
  }

  float sensor_model_initial_;
  float sensor_model_hit_;
  float sensor_model_miss_;
  float sensor_model_min_;
  float sensor_model_max_;
};

}  // namespace cost_map

#endif  // __INCLUDE_COST_MAP_SCAN__
