#pragma once
#ifndef __INCLUDE_COST_MAP_MATCH__
#define __INCLUDE_COST_MAP_MATCH__
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

class CostMapMatch : public CostMap {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CostMapMatch(const std::shared_ptr<std::vector<float>> &angles) {
    float MAX_RANGE = 10.0;  // TODO: variable
    max_ranges = std::make_shared<std::vector<float>>(angles->size(), MAX_RANGE);
    max_scan = std::make_shared<frame_buffer::ScanFrame>(
        0.0, Eigen::Vector2d::Zero(), 0.0, angles, max_ranges);
  }
  ~CostMapMatch() {}

  float match(const Eigen::Vector2d &translation,
              const double & rotation,
              const std::shared_ptr<frame_buffer::ScanFrame> &scan) {
    std::vector<Eigen::Vector2d> points;
    max_scan->set_pose(translation, rotation);
    max_scan->transformed_scan(points);

    double score = 0;
    Eigen::Array2i center_m, ray_m;
    world_to_map(translation, center_m);
    if (!is_inside("cost", center_m)) return score;
    // for (const auto &point : points) {
    for (size_t i = 0; i < points.size(); ++i) {
      auto &point = points.at(i);
      auto &range = scan->ranges()->at(i);
      world_to_map(point, ray_m);
      bresenham(center_m, ray_m,
                [&](const Eigen::Array2i &index) {
                  if (!is_inside("cost", index)) return false;
                  if (logf(0.5 / (1 - 0.5)) < at("cost", index)) { // is hit?
                    // TODO: calculate score
                    // distance from center_m
                    double distance = (index - center_m).matrix().norm() * get_resolution();
                    double sigma = 0.2;
                    double error = range - distance;
                    score += exp(-(error * error) / (2 * sigma * sigma));
                    return false;
                  }
                  return true;
                });
    }
    return score;
  }

private:
  std::shared_ptr<std::vector<float>> max_ranges;
  std::shared_ptr<frame_buffer::ScanFrame> max_scan;
};

}  // namespace cost_map

#endif  // __INCLUDE_COST_MAP_MATCH__
