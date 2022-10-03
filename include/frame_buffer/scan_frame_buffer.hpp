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
                  const int frame_size, const std::string odom_frame)
      : angles_(angles), frame_size_(frame_size), odom_frame_(odom_frame) {}
  ScanFrameBuffer(const std::shared_ptr<Eigen::VectorXd> angles)
      : ScanFrameBuffer(angles, 32, "odom") {}

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
    for (auto frame : frames_) cost_map.update(*frame, false);
    if (frames_.size() != 0) cost_map.save("cost", "./cost_buffer.png");

    const Eigen::Array2i half_size(32, 32);
    Eigen::Array2i center;
    cost_map.world_to_map(Eigen::Vector2d(-3.0, 0.0), center);
    cost_map::CostMapScan::MapType cropped;
    if (cost_map.crop("cost", center - half_size, 2 * half_size, cropped)) {
      Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic> array =
          cropped.array();
      array = array.exp() / (1 + array.exp());
      Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> data =
          ((array < 0.196).cast<uint8_t>() * 255 +
           (0.65 < array).cast<uint8_t>() * 0 +
           (0.196 <= array && array <= 0.65).cast<uint8_t>() * 200)
              .matrix();
      cv::Mat image;
      eigen2cv(data, image);
      cv::flip(image, image, 0);
      cv::imshow("cropped", image);
      cv::waitKey(0);
    }
  }

 protected:
  std::shared_ptr<Eigen::VectorXd> angles_;
  const size_t frame_size_;
  std::deque<std::shared_ptr<frame_buffer::ScanFrame>> frames_;

  const std::string odom_frame_;

  std::mutex mtx_;
};

}  // namespace frame_buffer