#ifndef FRAME_BUFFER_SCAN_FRAME_HPP_
#define FRAME_BUFFER_SCAN_FRAME_HPP_

#include <Eigen/Geometry>
#include <memory>

namespace frame_buffer {

class ScanFrame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ScanFrame(const double &timestamp, const Eigen::Matrix3d &pose,
            const std::shared_ptr<Eigen::VectorXd> &angles,
            const Eigen::VectorXd &ranges);
  ScanFrame(const double &timestamp, const Eigen::Vector2d &translation,
            const double &rotation,
            const std::shared_ptr<Eigen::VectorXd> &angles,
            const Eigen::VectorXd &ranges);
  ~ScanFrame() {}

  double timestamp() { return timestamp_; }
  std::shared_ptr<Eigen::VectorXd> angles() { return angles_; }
  Eigen::VectorXd &ranges() { return ranges_; }

  const Eigen::Vector2d &translation() const { return translation_; }
  const double &rotation() const { return rotation_; }

  double *mutable_translation() { return translation_.data(); }
  double *mutable_rotation() { return &rotation_; }

  // void get_pose(Eigen::Matrix3d &pose);

  void transformed_scan(Eigen::Matrix2Xd &points);
  void transformed_scan(Eigen::Matrix2Xd &points,
                        const Eigen::Matrix3d &external_transform);

 private:
  double timestamp_;
  Eigen::Vector2d translation_;
  double rotation_;

  std::shared_ptr<Eigen::VectorXd> angles_;
  Eigen::VectorXd ranges_;
};

}  // namespace frame_buffer

#endif  // FRAME_BUFFER_SCAN_FRAME_HPP_
