#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

#include <cmath>
#include <iostream>
#include <mutex>
#include <queue>

#include "cost_map/cost_map_scan.hpp"
#include "frame_buffer/scan_frame.hpp"
#include "frame_buffer/scan_frame_buffer.hpp"

const int kLaserScanSkip = 4;

class ScanFrameBufferNode {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ScanFrameBufferNode() : frame_size_(32), odom_frame_("odom") {
    laserscan_sub_ =
        nh_.subscribe("/depth/scan", 1, &ScanFrameBufferNode::callback, this);
  }

  void callback(const sensor_msgs::LaserScan &msg) {
    // obatain translation and rotation.
    Eigen::Vector2d translation;
    double yaw;
    try {
      tf::StampedTransform transform;
      std::string frame_id = msg.header.frame_id;
      // std::string frame_id = "laser";
      tf_listener_.lookupTransform(odom_frame_, frame_id, msg.header.stamp,
                                   transform);
      translation << transform.getOrigin().getX(), transform.getOrigin().getY();
      yaw = tf::getYaw(transform.getRotation());
    } catch (const tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }

    if (!buffer_) {
      std::shared_ptr<Eigen::VectorXd> angles_ =
          std::shared_ptr<Eigen::VectorXd>(
              new Eigen::VectorXd(msg.ranges.size() / kLaserScanSkip));
      for (size_t i = 0; i < msg.ranges.size() / kLaserScanSkip; ++i) {
        (*angles_)[i] =
            msg.angle_min + msg.angle_increment * i * kLaserScanSkip;
      }
      buffer_ =
          std::make_unique<frame_buffer::ScanFrameBuffer>(angles_, frame_size_);
    }

    Eigen::VectorXd ranges(msg.ranges.size() / kLaserScanSkip);
    for (size_t i = 0; i < msg.ranges.size() / kLaserScanSkip; ++i) {
      ranges[i] = msg.ranges[msg.ranges.size() - i * kLaserScanSkip - 1];
    }
    buffer_->update(msg.header.stamp.toSec(), ranges, translation, yaw);
  }

  void project(void) {
    if (!buffer_) return;
    cost_map::CostMapScan cost_map;
    cost_map.set_scan_range_max(1.9);
    buffer_->project(cost_map);
    cost_map.save("cost", "./cost_buffer.png");
  }

 private:
  ros::NodeHandle nh_;

  ros::Subscriber laserscan_sub_;
  tf::TransformListener tf_listener_;

  const size_t frame_size_;
  const std::string odom_frame_;
  std::unique_ptr<frame_buffer::ScanFrameBuffer> buffer_;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "scan_frame_node");
  ScanFrameBufferNode frame_buffer_node;

  std::cout << "main" << std::endl;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate loop_rate(0.1);
  while (ros::ok()) {
    frame_buffer_node.project();
    loop_rate.sleep();
  }

  spinner.stop();

  return 0;
}
