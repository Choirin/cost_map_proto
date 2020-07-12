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

#include "frame_buffer/scan_frame.hpp"
#include "depth_to_scan/depth_to_scan.hpp"

class ScanFrameBufferNode {
 protected:
  ros::NodeHandle nh_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber depth_sub_;
  tf::TransformListener tf_;
  ros::Publisher pub;

  std::unique_ptr<DepthToScan> depth_to_scan_;

  std::shared_ptr<std::vector<float>> angles_;

  size_t frame_size_;
  std::deque<std::shared_ptr<frame_buffer::ScanFrame>> frames_;

  std::string odom_frame_;

  std::mutex mtx_;

 public:
  ScanFrameBufferNode()
      : it_(nh_),
        angles_(std::make_shared<std::vector<float>>()),
        frame_size_(20),
        odom_frame_("odom") {
    depth_to_scan_ = std::make_unique<DepthToScan>(
        224, 172,
        195.26491142934, 195.484689318979,
        111.31867165296, 86.8194913656314,
        1000.0);
    depth_sub_ =
        it_.subscribe("/depth/image_raw", 1,
                      boost::bind(&ScanFrameBufferNode::callback, this, _1));
    pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("points", 1);
  }

  void callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // obatain translation and rotation.
    Eigen::Vector2d translation;
    double yaw;
    try {
      tf::StampedTransform transform;
      // std::string frame_id = msg->header.frame_id;
      std::string frame_id = "depth_base";
      tf_.lookupTransform(odom_frame_, frame_id, ros::Time(0), transform);
      translation << transform.getOrigin().getX(), transform.getOrigin().getY();
      yaw = tf::getYaw(transform.getRotation());
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }

    if (frames_.size() != 0) {
      // insert a frame, if there is a large difference in distance or angle
      auto d_translation = *frames_.back()->translation() - translation;
      auto d_rotation = fmod(fabs(frames_.back()->rotation() - yaw), M_PI);
      if (d_translation.norm() < 0.3 && d_rotation < 0.1) {
        return;
      }
    }

    mtx_.lock();
    frames_.emplace_back(new frame_buffer::ScanFrame(
        msg->header.stamp.toSec(), translation, yaw,
        depth_to_scan_->angles(), depth_to_scan_->convert(cv_ptr->image)));
    if (frames_.size() > frame_size_) frames_.pop_front();
    std::cout << "new frame inserted. " << frames_.size() << std::endl;
    mtx_.unlock();
  }

  void publish_pcl(void) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    point_cloud->width = 0;
    point_cloud->height = 1;

    std::vector<Eigen::Vector2d> points;
    size_t idx = 0;

    mtx_.lock();
    for (auto frame : frames_) {
      frame->transformed_scan(points);

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

    point_cloud->header.frame_id = "odom";
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
