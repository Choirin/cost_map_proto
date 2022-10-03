#include "frame_buffer/scan_frame.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace frame_buffer {

ScanFrame::ScanFrame(const double &timestamp, const Eigen::Matrix3d &pose,
                     const std::shared_ptr<Eigen::VectorXd> &angles,
                     const Eigen::VectorXd &ranges)
    : timestamp_(timestamp),
      translation_(pose.block<2, 1>(0, 2)),
      rotation_(acos(pose(0, 0))),
      angles_(angles),
      ranges_(ranges) {}

ScanFrame::ScanFrame(const double &timestamp,
                     const Eigen::Vector2d &translation, const double &rotation,
                     const std::shared_ptr<Eigen::VectorXd> &angles,
                     const Eigen::VectorXd &ranges)
    : timestamp_(timestamp),
      translation_(translation),
      rotation_(rotation),
      angles_(angles),
      ranges_(ranges) {}

void ScanFrame::transformed_scan(Eigen::Matrix2Xd &points) {
  Eigen::Matrix2d rot_matrix;
  rot_matrix << cos(rotation_), -sin(rotation_), sin(rotation_), cos(rotation_);
  const size_t size = ranges_.size();
  points.resize(2, size);
  for (size_t i = 0; i < size; ++i) {
    points.col(i) << ranges_[i] * cos((*angles_)[i]),
        ranges_[i] * sin((*angles_)[i]);
    points.col(i) = rot_matrix * points.col(i) + translation_;
  }
}

void ScanFrame::transformed_scan(Eigen::Matrix2Xd &points,
                                 const Eigen::Matrix3d &external_transform) {
  Eigen::Matrix2d rot_matrix;
  rot_matrix << cos(rotation_), -sin(rotation_), sin(rotation_), cos(rotation_);
  const size_t size = ranges_.size();
  points.resize(2, size);
  for (size_t i = 0; i < size; ++i) {
    points.col(i) << ranges_[i] * cos((*angles_)[i]),
        ranges_[i] * sin((*angles_)[i]);
    points.col(i) = rot_matrix * points.col(i) + translation_;
    points.col(i) = external_transform.block<2, 2>(0, 0) * points.col(i) +
            external_transform.block<2, 1>(0, 2);
  }
}

}  // namespace frame_buffer
