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
#include "frame_buffer/scan_frame_buffer.hpp"

class DepthFrameBufferNode {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DepthFrameBufferNode() : it_(nh_), frame_size_(32), odom_frame_("odom") {
    depth_to_scan_ = std::make_unique<DepthToScan>(
        224, 172, 195.26491142934, 195.484689318979, 111.31867165296,
        86.8194913656314, 1000.0);
    depth_to_scan_->set_height(0.05, 0.1);
    depth_to_scan_->set_range(0.2, 2.0);
    depth_to_scan_->set_optical_axis_pitch(DEG2RAD(10));

    buffer_ = std::make_unique<frame_buffer::ScanFrameBuffer>(
        depth_to_scan_->angles(), frame_size_, odom_frame_);

    depth_sub_ =
        it_.subscribe("/depth/image_raw", 1,
                      boost::bind(&DepthFrameBufferNode::callback, this, _1));
    // pcl_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("points", 1);
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
      // const std::string &frame_id = msg->header.frame_id;
      const std::string frame_id = "laser";
      tf_listener_.lookupTransform(odom_frame_, frame_id, msg->header.stamp,
                                   transform);
      translation << transform.getOrigin().getX(), transform.getOrigin().getY();
      yaw = tf::getYaw(transform.getRotation());
    } catch (const tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }

    Eigen::VectorXd ranges;
    depth_to_scan_->convert(cv_ptr->image, ranges);
    buffer_->update(msg->header.stamp.toSec(), ranges, translation, yaw);
  }

  void project(void) {
    if (!buffer_) return;
    buffer_->project();
  }

  // void publish_pcl(void) {
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(
  //       new pcl::PointCloud<pcl::PointXYZ>());
  //   point_cloud->width = 0;
  //   point_cloud->height = 1;

  //   size_t idx = 0;

  //   cost_map::CostMapScan cost_map;
  //   cost_map.set_scan_range_max(1.9);

  //   mtx_.lock();
  //   for (auto frame : frames_) {
  //     Eigen::Matrix2Xd points;
  //     frame->transformed_scan(points);
  //     cost_map.update(*frame);

  //     point_cloud->width += points.cols();
  //     point_cloud->points.resize(point_cloud->width);
  //     for (int i = 0; i < points.cols(); ++i, ++idx) {
  //       pcl::PointXYZ &point = point_cloud->points[idx];
  //       point.x = points.col(i)[0];
  //       point.y = points.col(i)[1];
  //       point.z = 0.0;
  //     }
  //   }
  //   mtx_.unlock();

  //   point_cloud->header.frame_id = odom_frame_;
  //   pcl_conversions::toPCL(ros::Time::now(), point_cloud->header.stamp);
  //   pcl_pub_.publish(point_cloud);
  //   std::cout << "points: " << point_cloud->points.size() << std::endl;
  // }

 private:
  ros::NodeHandle nh_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber depth_sub_;
  tf::TransformListener tf_listener_;
  // ros::Publisher pcl_pub_;

  std::unique_ptr<DepthToScan> depth_to_scan_;

  const size_t frame_size_;
  const std::string odom_frame_;
  std::unique_ptr<frame_buffer::ScanFrameBuffer> buffer_;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "frame_buffer_node");
  DepthFrameBufferNode frame_buffer_node;

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
