#pragma once
#ifndef __INCLUDE_SCAN_FRAME__
#define __INCLUDE_SCAN_FRAME__
#include <memory>
#include <algorithm>
#include <iterator>
#include <vector>

#include <Eigen/Geometry>

namespace frame_buffer
{

class ScanFrame
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ScanFrame(const double &timestamp, const Eigen::Matrix3d &pose,
            const std::shared_ptr<std::vector<float>> &angles,
            const std::vector<float> &ranges);
  ScanFrame(const double &timestamp, const Eigen::Vector2d &translation, const double &rotation,
            const std::shared_ptr<std::vector<float>> &angles,
            const std::vector<float> &ranges);
  ScanFrame(const double &timestamp,
            const std::shared_ptr<std::vector<float>> &angles,
            const std::shared_ptr<std::vector<float>> &ranges);
  ~ScanFrame() {}

  double timestamp() { return timestamp_; }
  std::shared_ptr<std::vector<float>> angles() { return angles_; }
  std::shared_ptr<std::vector<float>> ranges() { return ranges_; }

  Eigen::Vector2d *translation() { return &translation_; }
  double rotation() { return rotation_; }

  double *mutable_translation() { return translation_.data(); }
  double *mutable_rotation() { return &rotation_; }

  // void get_pose(Eigen::Matrix3d &pose);

  void transformed_scan(std::vector<Eigen::Vector2d> &points);

private:
  double timestamp_;
  Eigen::Vector2d translation_;
  double rotation_;

  std::shared_ptr<std::vector<float>> angles_;
  std::shared_ptr<std::vector<float>> ranges_;

};

} // namespace frame_buffer

#endif // __INCLUDE_SCAN_FRAME__
