#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>

#include <cmath>
#include <iostream>
#include <mutex>
#include <queue>

#include "cost_map/cost_map_scan.hpp"
#include "depth_to_scan/depth_to_scan.hpp"
#include "frame_buffer/scan_frame.hpp"

class ScanFrameBufferNode {
 protected:
  ros::NodeHandle nh_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber depth_sub_;
  tf::TransformListener tf_;
  ros::Publisher pub;

  std::unique_ptr<DepthToScan> depth_to_scan_;

  size_t frame_size_;
  std::deque<std::shared_ptr<frame_buffer::ScanFrame>> frames_;

  std::unique_ptr<cost_map::CostMapScan> cost_map_;

  std::string odom_frame_;

  std::mutex mtx_;

 public:
  ScanFrameBufferNode() : it_(nh_), frame_size_(20), odom_frame_("odom") {
    depth_to_scan_ = std::make_unique<DepthToScan>(
        224, 172, 195.26491142934, 195.484689318979, 111.31867165296,
        86.8194913656314, 1000.0);
    depth_to_scan_->set_height(0.05, 0.1);
    depth_to_scan_->set_range(0.2, 2.0);
    depth_to_scan_->set_optical_axis_pitch(DEG2RAD(10));

    // auto angles = depth_to_scan_->angles();
    // auto last_angle = angles->at(0);
    // for (auto const &angle : *angles) {
    //   std::cout << RAD2DEG(angle - last_angle) << std::endl;
    //   last_angle = angle;
    // }

    cost_map_ = std::make_unique<cost_map::CostMapScan>();
    cost_map_->set_scan_range_max(1.9);

    depth_sub_ =
        it_.subscribe("/depth/image_raw", 1,
                      boost::bind(&ScanFrameBufferNode::callback, this, _1));
    pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("points", 1);
  }

  void callback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // obatain translation and rotation.
    Eigen::Vector2d translation;
    double yaw;
    try {
      tf::StampedTransform transform;
      // std::string frame_id = msg->header.frame_id;
      std::string frame_id = "laser";
      tf_.lookupTransform(odom_frame_, frame_id, msg->header.stamp, transform);
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
      if (d_translation.norm() < 0.2 && d_rotation < 0.05) {
        return;
      }
    }

    mtx_.lock();
    frames_.emplace_back(new frame_buffer::ScanFrame(
        msg->header.stamp.toSec(), translation, yaw, depth_to_scan_->angles(),
        depth_to_scan_->convert(cv_ptr->image)));
    if (frames_.size() > frame_size_) frames_.pop_front();
    std::cout << "new frame inserted. " << frames_.size() << std::endl;

    cost_map_->update(frames_.back());
    cost_map_->save("occupied", "/tmp/occupied.png");
    cost_map_->save("free", "/tmp/free.png");
    cost_map_->save("cost", "/tmp/cost.png");
    mtx_.unlock();
  }

  void publish_pcl(void) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    point_cloud->width = 0;
    point_cloud->height = 1;

    std::vector<Eigen::Vector2d> points;
    size_t idx = 0;

    cost_map::CostMapScan cost_map;
    cost_map.set_scan_range_max(1.9);

    mtx_.lock();
    for (auto frame : frames_) {
      frame->transformed_scan(points);
      cost_map.update(frame);

      point_cloud->width += points.size();
      point_cloud->points.resize(point_cloud->width);
      for (size_t i = 0; i < points.size(); ++i, ++idx) {
        pcl::PointXYZ &point = point_cloud->points[idx];
        point.x = points[i](0);
        point.y = points[i](1);
        point.z = 0.0;
      }
    }
    mtx_.unlock();

    if (frames_.size() != 0)
      cost_map.save("cost", "/tmp/cost_buffer.png");

    point_cloud->header.frame_id = odom_frame_;
    pcl_conversions::toPCL(ros::Time::now(), point_cloud->header.stamp);
    pub.publish(point_cloud);
    std::cout << "points: " << point_cloud->points.size() << std::endl;
  }
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "frame_buffer_node");
  ScanFrameBufferNode frame_buffer_node;

  std::cout << "main" << std::endl;

  ros::Rate loop_rate(0.5);
  while (ros::ok()) {
    frame_buffer_node.publish_pcl();
    ros::spinOnce();
    loop_rate.sleep();
  }
  // ros::spin();
  return 0;
}
