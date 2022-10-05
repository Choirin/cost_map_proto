#include <cmath>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <queue>

#include "cost_map/cost_map_scan.hpp"
#include "cost_map/occupancy_grid.hpp"
#include "frame_buffer/scan_frame.hpp"
#include "frame_buffer/scan_frame_buffer.hpp"

void show_image(
    const std::string &name,
    const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &mat) {
  Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic> array = mat.array();
  array = array.exp() / (1 + array.exp());
  Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> data =
      ((array < 0.196).cast<uint8_t>() * 255 +
       (0.65 < array).cast<uint8_t>() * 0 +
       (0.196 <= array && array <= 0.65).cast<uint8_t>() * 200)
          .matrix();
  cv::Mat image;
  cv::eigen2cv(data, image);
  cv::flip(image, image, 0);
  cv::imshow(name, image);
  cv::waitKey(0);
}

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
  for (double timestamp = 0; timestamp < 2 * M_PI / abs(yaw_rate);
       timestamp += 0.1) {
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

  const Eigen::Array2i kCropMapHalfSize(64, 64);

  const std::filesystem::path map_yaml = "./test_map/map.yaml";
  std::unique_ptr<cost_map::OccupancyGrid> grid_map =
      cost_map::LoadOccupancyGridFromFile(map_yaml);

  grid_map->expand(kCropMapHalfSize, logf((255.0 - 100.0) / 255.0));
  show_image("expanded_grid", grid_map->data("cost"));

  Eigen::Vector2d origin;
  Eigen::Array2i size;
  double resolution = grid_map->get_resolution();
  grid_map->get_origin(origin);
  grid_map->get_size(size);

  cost_map::CostMapScan cost_map(origin, size, resolution);
  cost_map.set_scan_range_max(1.9);

  Eigen::Matrix3d external_transform = Eigen::Matrix3d::Identity();
  // external_transform.block<2, 2>(0, 0) = Eigen::Rotation2Dd(M_PI /
  // 2).toRotationMatrix(); external_transform(0, 2) = 2.0;
  std::cout << external_transform << std::endl;

  scan_buffer.project(cost_map, external_transform, false);
  cost_map.save("cost", "./cost_buffer.png");
  show_image("scan", cost_map.data("cost"));

  Eigen::Array2i center;
  cost_map.world_to_map(Eigen::Vector2d(0.0, 3.0), center);
  cost_map::CostMapScan::MapType cropped;
  if (cost_map.crop("cost", center - kCropMapHalfSize, 2 * kCropMapHalfSize,
                    cropped)) {
    show_image("cropped_scan", cropped);
  }
  if (grid_map->crop("cost", center - kCropMapHalfSize,
                     2 * kCropMapHalfSize, cropped)) {
    show_image("cropped_grid", cropped);
  }

  return 0;
}
