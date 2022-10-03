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

  frame_buffer::ScanFrameBuffer scan_buffer(angles);

  const double radius = 3.0;
  const double speed = 0.05;
  const double yaw_rate = -speed / radius;
  double x = 0.0, y = 3.0, yaw = 0.0;
  for (double timestamp = 0; timestamp < 2 * M_PI / abs(yaw_rate); timestamp += 0.1) {
    Eigen::VectorXd ranges(kScanLength);

    for (int i = 0; i < kScanLength; i++) {
      ranges[i] = 1.0 / cos((*angles)[i]);
    }

    scan_buffer.update(0.0, ranges, Eigen::Vector2d(x, y), yaw);

    x += speed * cos(yaw);
    y += speed * sin(yaw);
    yaw += yaw_rate;
  }

  scan_buffer.project();

  return 0;
}
