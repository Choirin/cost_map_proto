#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

#include <cmath>
#include <iostream>
#include <mutex>
#include <queue>

#include "cost_map/cost_map_scan.hpp"
#include "frame_buffer/scan_frame.hpp"

class ScanFrameBufferNode {
 protected:
  ros::NodeHandle nh_;

  ros::Subscriber laserscan_sub_;
  tf::TransformListener tf_listener_;

  std::shared_ptr<std::vector<float>> angles_;
  size_t frame_size_;
  std::deque<std::shared_ptr<frame_buffer::ScanFrame>> frames_;

  std::string odom_frame_;

  std::mutex mtx_;

 public:
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
      tf_listener_.lookupTransform(odom_frame_, frame_id, msg.header.stamp, transform);
      translation << transform.getOrigin().getX(), transform.getOrigin().getY();
      yaw = tf::getYaw(transform.getRotation());
    } catch (const tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }

    if (frames_.size() != 0) {
      // insert a frame, if there is a large difference in distance or angle
      auto d_translation = frames_.back()->translation() - translation;
      auto d_rotation = fmod(fabs(frames_.back()->rotation() - yaw), M_PI);
      if (d_translation.norm() < 0.3 && d_rotation < 0.5) {
        return;
      }
    }

    {
      std::lock_guard<std::mutex> lock(mtx_);
      std::cout << msg.ranges.size() << std::endl;

      const int skip = 4;

      if (angles_ == nullptr) {
        angles_.reset(new std::vector<float>(msg.ranges.size() / skip));
        for (size_t i = 0; i < msg.ranges.size() / skip; ++i) {
          (*angles_)[i] = msg.angle_min + msg.angle_increment * i * skip;
        }
      }

      std::vector<float> ranges(msg.ranges.size() / skip);
      for (size_t i = 0; i < msg.ranges.size() / skip; ++i) {
        ranges[i] = msg.ranges[msg.ranges.size() - i * skip - 1];
      }
      frames_.emplace_back(new frame_buffer::ScanFrame(
          msg.header.stamp.toSec(), translation, yaw, angles_, ranges));
      if (frames_.size() > frame_size_) frames_.pop_front();
      std::cout << "new frame inserted. " << frames_.size() << std::endl;
    }
  }

  void project(void) {
    cost_map::CostMapScan cost_map;
    cost_map.set_scan_range_max(1.9);

    // lock using lock_guard
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto frame : frames_) {
      cost_map.update(frame);
    }
    if (frames_.size() != 0) cost_map.save("cost", "/tmp/cost_buffer.png");
  }
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
    // ros::spinOnce();
    loop_rate.sleep();
  }

  spinner.stop();

  return 0;
}
