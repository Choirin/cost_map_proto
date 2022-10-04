#include <cmath>
#include <iostream>
#include <mutex>
#include <queue>

#include "cost_map/cost_map_scan.hpp"
#include "frame_buffer/scan_frame.hpp"
#include "frame_buffer/scan_frame_buffer.hpp"


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

  const size_t kScanFrameBufferSize = 32;
  frame_buffer::ScanFrameBuffer scan_buffer(angles, 0.5, 30.0 * M_PI / 180.0,
                                            kScanFrameBufferSize);

  const double radius = 3.0;
  const double speed = 0.05;
  const double yaw_rate = -speed / radius;
  double x = 3.0, y = 0.0, yaw = M_PI / 2.0;
  for (double timestamp = 0; timestamp < 2 * M_PI / abs(yaw_rate); timestamp += 0.1) {
  // for (double timestamp = 0; timestamp < 0.5; timestamp += 0.1) {
    Eigen::VectorXd ranges(kScanLength);

    for (int i = 0; i < kScanLength; i++) {
      ranges[i] = 1.0 / cos((*angles)[i]);
    }

    scan_buffer.update(0.0, ranges, Eigen::Vector2d(x, y), yaw);

    x += speed * cos(yaw);
    y += speed * sin(yaw);
    yaw += yaw_rate;
  }

  scan_buffer.update(0.0, (1 / angles->array().cos()).matrix(), Eigen::Vector2d(0.0, 0.0), M_PI / 2.0);
  scan_buffer.update(0.0, (1 / angles->array().cos()).matrix(), Eigen::Vector2d(-5.0, 0.0), 0.0);
  scan_buffer.update(0.0, (1 / angles->array().cos()).matrix(), Eigen::Vector2d(-15.0, 0.0), 0.0);
  scan_buffer.update(0.0, (1 / angles->array().cos()).matrix(), Eigen::Vector2d(-10.0, 0.0), 0.0);
  // scan_buffer.update(0.0, (1 / angles->array().cos()).matrix(), Eigen::Vector2d(-5.0, 0.0), 0.0);

  const Eigen::Vector2d origin(-128 * 0.05, -128 * 0.05);
  const Eigen::Array2i size(256, 256);
  const float resolution = 0.05;

  cost_map::CostMapScan cost_map(origin, size, resolution);
  cost_map.set_scan_range_max(1.9);

  Eigen::Matrix3d external_transform = Eigen::Matrix3d::Identity();
  // external_transform.block<2, 2>(0, 0) = Eigen::Rotation2Dd(M_PI / 2).toRotationMatrix();
  // external_transform(0, 2) = 2.0;
  std::cout << external_transform << std::endl;

  scan_buffer.project(cost_map);
  cost_map.save("cost", "./cost_buffer.png");

  {
    Eigen::Vector2d origin;
    cost_map.get_origin(origin);
    Eigen::Array2i size;
    cost_map.get_size(size);
    std::cout << "origin: " << origin.transpose() << std::endl;
    std::cout << "size: " << size.transpose() << std::endl;
  }


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
    cv::imwrite("./cropped.png", image);
  }

  return 0;
}
