#include <cmath>
#include <iostream>
#include <mutex>
#include <queue>

#include "cost_map/cost_map_scan.hpp"
#include "frame_buffer/scan_frame.hpp"

class ScanFrameBuffer {
 public:
  ScanFrameBuffer(const std::shared_ptr<Eigen::VectorXd> angles)
      : angles_(angles), frame_size_(32), odom_frame_("odom") {}

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

  void project(void) {
    const Eigen::Vector2d origin(-128 * 0.05, -128 * 0.05);
    const Eigen::Array2i size(256, 256);
    const float resolution = 0.05;

    cost_map::CostMapScan cost_map(origin, size, resolution);
    cost_map.set_scan_range_max(1.9);

    // lock using lock_guard
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto frame : frames_) {
      cost_map.update(frame, false);
    }
    if (frames_.size() != 0) cost_map.save("cost", "./cost_buffer.png");
  }

 protected:
  std::shared_ptr<Eigen::VectorXd> angles_;
  size_t frame_size_;
  std::deque<std::shared_ptr<frame_buffer::ScanFrame>> frames_;

  std::string odom_frame_;

  std::mutex mtx_;
};

int main(int argc, char *argv[]) {
  const int kScanLength = 56;
  const double kFoV = 60.0 * M_PI / 180.0;
  const double kMinAngle = -kFoV / 2.0;
  const double kMaxAngle = kFoV / 2.0;
  const double kAngleIncrement = (kMaxAngle - kMinAngle) / kScanLength;

  std::shared_ptr<Eigen::VectorXd> angles =
      std::make_shared<Eigen::VectorXd>(kScanLength);
  for (int i = 0; i < kScanLength; ++i) {
    (*angles)[i] = kMinAngle + kAngleIncrement * i;
  }

  ScanFrameBuffer frame_buffer(angles);

  const double radius = 3.0;
  const double speed = 0.05;
  const double yaw_rate = -speed / radius;
  double x = 0.0, y = 3.0, yaw = 0.0;
  for (double timestamp = 0; timestamp < 2 * M_PI / abs(yaw_rate); timestamp += 0.1) {
    Eigen::VectorXd ranges(kScanLength);

    for (int i = 0; i < kScanLength; i++) {
      ranges[i] = 1.0 / cos((*angles)[i]);
    }

    frame_buffer.update(0.0, ranges, Eigen::Vector2d(x, y), yaw);

    x += speed * cos(yaw);
    y += speed * sin(yaw);
    yaw += yaw_rate;
  }

  frame_buffer.project();

  return 0;
}
