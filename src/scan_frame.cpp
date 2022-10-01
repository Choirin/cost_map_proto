#include "frame_buffer/scan_frame.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace frame_buffer {

ScanFrame::ScanFrame(const double &timestamp, const Eigen::Matrix3d &pose,
                     const std::shared_ptr<std::vector<float>> &angles,
                     const std::vector<float> &ranges)
    : timestamp_(timestamp),
      translation_(pose.block<2, 1>(0, 2)),
      rotation_(acos(pose(0, 0))),
      angles_(angles) {
  ranges_ = std::make_shared<std::vector<float>>();
  std::copy(ranges.begin(), ranges.end(), std::back_inserter(*ranges_));
}

ScanFrame::ScanFrame(const double &timestamp,
                     const Eigen::Vector2d &translation, const double &rotation,
                     const std::shared_ptr<std::vector<float>> &angles,
                     const std::vector<float> &ranges)
    : timestamp_(timestamp),
      translation_(translation),
      rotation_(rotation),
      angles_(angles) {
  ranges_ = std::make_shared<std::vector<float>>();
  std::copy(ranges.begin(), ranges.end(), std::back_inserter(*ranges_));
}

ScanFrame::ScanFrame(const double &timestamp,
                     const Eigen::Vector2d &translation, const double &rotation,
                     const std::shared_ptr<std::vector<float>> &angles,
                     const std::shared_ptr<std::vector<float>> &ranges)
    : timestamp_(timestamp),
      translation_(translation),
      rotation_(rotation),
      angles_(angles),
      ranges_(ranges) {}

void ScanFrame::transformed_scan(std::vector<Eigen::Vector2d> &points) {
  Eigen::Matrix2d rot_matrix;
  rot_matrix << cos(rotation_), -sin(rotation_), sin(rotation_), cos(rotation_);
  size_t size = ranges_->size();
  points.clear();
  points.reserve(size);
  for (size_t i = 0; i < size; ++i) {
    Eigen::Vector2d point;
    point << (*ranges_)[i] * cos((*angles_)[i]),
        (*ranges_)[i] * sin((*angles_)[i]);
    point = rot_matrix * point + translation_;
    points.emplace_back(point);
  }
}

void ScanFrame::transformed_scan(std::vector<Eigen::Vector2d> &points,
                                 const Eigen::Matrix3d &external_transform) {
  Eigen::Matrix2d rot_matrix;
  rot_matrix << cos(rotation_), -sin(rotation_), sin(rotation_), cos(rotation_);
  const size_t size = ranges_->size();
  points.clear();
  points.reserve(size);
  for (size_t i = 0; i < size; ++i) {
    Eigen::Vector2d point;
    point << (*ranges_)[i] * cos((*angles_)[i]),
        (*ranges_)[i] * sin((*angles_)[i]);
    point = rot_matrix * point + translation_;
    point = external_transform.block<2, 2>(0, 0) * point +
            external_transform.block<2, 1>(0, 2);
    points.emplace_back(point);
  }
}

}  // namespace frame_buffer
