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
    initial_ = log(0.5 / (1 - 0.5));
    hit_ = log(0.8 / (1 - 0.8));
    miss_ = log(0.2 / (1 - 0.2));
    add("free", initial_);
    add("occupied", initial_);
    add("cost", initial_);
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
    extend(corner_lb, corner_rt, initial_);

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
                  at("free", index) += miss_;
                  at("cost", index) += miss_;
                  return true;
                });
      at("occupied", ray_m) += hit_;
      at("cost", ray_m) += hit_;
    }
  }

private:
  float initial_;
  float hit_;
  float miss_;
};

}  // namespace cost_map

#endif  // __INCLUDE_COST_MAP_SCAN__
