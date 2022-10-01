#pragma once
#ifndef __INCLUDE_SCAN_FRAME__
#define __INCLUDE_SCAN_FRAME__
#include <Eigen/Geometry>
#include <algorithm>
#include <iterator>
#include <memory>
#include <vector>

namespace frame_buffer {

class ScanFrame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ScanFrame(const double &timestamp, const Eigen::Matrix3d &pose,
            const std::shared_ptr<std::vector<float>> &angles,
            const std::vector<float> &ranges);
  ScanFrame(const double &timestamp, const Eigen::Vector2d &translation,
            const double &rotation,
            const std::shared_ptr<std::vector<float>> &angles,
            const std::vector<float> &ranges);
  ScanFrame(const double &timestamp, const Eigen::Vector2d &translation,
            const double &rotation,
            const std::shared_ptr<std::vector<float>> &angles,
            const std::shared_ptr<std::vector<float>> &ranges);
  ~ScanFrame() {}

  double timestamp() { return timestamp_; }
  std::shared_ptr<std::vector<float>> angles() { return angles_; }
  std::shared_ptr<std::vector<float>> ranges() { return ranges_; }

  const Eigen::Vector2d &translation() const { return translation_; }
  const double &rotation() const { return rotation_; }

  double *mutable_translation() { return translation_.data(); }
  double *mutable_rotation() { return &rotation_; }

  // void get_pose(Eigen::Matrix3d &pose);

  void transformed_scan(std::vector<Eigen::Vector2d> &points);
  void transformed_scan(std::vector<Eigen::Vector2d> &points,
                        const Eigen::Matrix3d &external_transform);

 private:
  double timestamp_;
  Eigen::Vector2d translation_;
  double rotation_;

  std::shared_ptr<std::vector<float>> angles_;
  std::shared_ptr<std::vector<float>> ranges_;
};

}  // namespace frame_buffer

#endif  // __INCLUDE_SCAN_FRAME__
