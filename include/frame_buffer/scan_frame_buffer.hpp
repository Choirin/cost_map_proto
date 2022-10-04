#include <cmath>
#include <iostream>
#include <mutex>
#include <queue>

#include "cost_map/cost_map_scan.hpp"
#include "frame_buffer/scan_frame.hpp"

namespace frame_buffer {

class ScanFrameBuffer {
 public:
  ScanFrameBuffer(const std::shared_ptr<Eigen::VectorXd> angles,
                  const int frame_size)
      : angles_(angles), frame_size_(frame_size) {}

  void update(const double &timestamp, const Eigen::VectorXd &ranges,
              const Eigen::Vector2d &translation, const double &yaw) {
    if (frames_.size() != 0) {
      // insert a frame, if there is a large difference in distance or angle
      auto d_translation = frames_.back()->translation() - translation;
      auto d_rotation = fmod(fabs(frames_.back()->rotation() - yaw), M_PI);
      if (d_translation.norm() < 0.3 && d_rotation < 0.5) {
        return;
      }
    }

    std::lock_guard<std::mutex> lock(mtx_);
    frames_.emplace_back(new frame_buffer::ScanFrame(timestamp, translation,
                                                     yaw, angles_, ranges));
    if (frames_.size() > frame_size_) frames_.pop_front();
    std::cout << "new frame inserted. " << frames_.size() << std::endl;
  }

  void project(cost_map::CostMapScan &cost_map, const bool expand_map = true) {
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto frame : frames_) cost_map.update(*frame, expand_map);
  }

  void project(cost_map::CostMapScan &cost_map,
               const Eigen::Matrix3d &external_transform,
               const bool expand_map = true) {
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto frame : frames_)
      cost_map.update(*frame, external_transform, expand_map);
  }

 protected:
  std::shared_ptr<Eigen::VectorXd> angles_;
  const size_t frame_size_;
  std::deque<std::shared_ptr<frame_buffer::ScanFrame>> frames_;

  std::mutex mtx_;
};

}  // namespace frame_buffer
