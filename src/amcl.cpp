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
#include <random>

#include "frame_buffer/scan_frame.hpp"
#include "depth_to_scan/depth_to_scan.hpp"
#include "cost_map/cost_map_scan.hpp"
#include "cost_map/cost_map_match.hpp"
#include "cost_map/cost_map_loader.hpp"
#include "particle_filter/particle_filter.hpp"

void add_noises(Eigen::Vector2d &translation, double &rotation)
{
  double mu = 0.0;
  double sigma = 1.0;
  std::mt19937 rand_src{ std::random_device{}() };

  std::normal_distribution<double> rand_trans_dist(mu, sigma);
  Eigen::Vector2d t_dist(rand_trans_dist(rand_src), rand_trans_dist(rand_src));
  translation += t_dist;

  std::normal_distribution<double> rand_angle_dist(mu, DEG2RAD(45));
  double t_angle = rand_angle_dist(rand_src);
  rotation += t_angle;
}

class AmclNode {
 protected:
  ros::NodeHandle nh_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber depth_sub_;
  tf::TransformListener tf_;
  ros::Publisher pub;

  std::unique_ptr<DepthToScan> depth_to_scan_;

  size_t frame_size_;
  std::deque<std::shared_ptr<frame_buffer::ScanFrame>> frames_;

  std::shared_ptr<cost_map::CostMapMatch> cost_map_;

  std::string odom_frame_;

  std::mutex mtx_;

 public:
  AmclNode()
      : it_(nh_),
        frame_size_(20),
        odom_frame_("odom") {
    depth_to_scan_ = std::make_unique<DepthToScan>(
        224, 172,
        195.26491142934, 195.484689318979,
        111.31867165296, 86.8194913656314,
        1000.0);
    depth_to_scan_->set_height(0.05, 0.1);
    depth_to_scan_->set_range(0.2, 2.0);
    depth_to_scan_->set_optical_axis_pitch(DEG2RAD(10));

    auto angles = depth_to_scan_->angles();
    cost_map_ = std::make_shared<cost_map::CostMapMatch>(angles);
    cost_map::CostMapLoader::load("/workspace/data/data/2d_map/map.yaml", *cost_map_);

    depth_sub_ =
        it_.subscribe("/depth/image_raw", 1,
                      boost::bind(&AmclNode::callback, this, _1));
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
      auto d_translation = frames_.back()->translation() - translation;
      auto d_rotation = fmod(fabs(frames_.back()->rotation() - yaw), M_PI);
      if (d_translation.norm() < 0.3 && d_rotation < 0.1) {
        return;
      }
    }

    std::shared_ptr<frame_buffer::ScanFrame> scan(new frame_buffer::ScanFrame(
        msg->header.stamp.toSec(), translation, yaw, depth_to_scan_->angles(),
        depth_to_scan_->convert(cv_ptr->image)));
    // mtx_.lock();
    // frames_.emplace_back(new frame_buffer::ScanFrame(
    //     msg->header.stamp.toSec(), translation, yaw,
    //     depth_to_scan_->angles(), depth_to_scan_->convert(cv_ptr->image)));
    // if (frames_.size() > frame_size_) frames_.pop_front();
    // std::cout << "new frame inserted. " << frames_.size() << std::endl;

    // cost_map_->update(frames_.back());
    // cost_map_->save("occupied", "/workspace/data/data/occupied.png");
    // cost_map_->save("free", "/workspace/data/data/free.png");
    auto score = cost_map_->match(translation, yaw, scan);
    std::cout << translation(0) << ", " << translation(1) << ", "
              << yaw << ", "
              << score << std::endl;
    for (int i = 0; i < 10; ++i) {
      Eigen::Vector2d translation_r = translation;
      double rotation_r = yaw;
      add_noises(translation_r, rotation_r);
      auto score = cost_map_->match(translation_r, rotation_r, scan);
      std::cout << translation_r(0) << ", " << translation_r(1) << ", "
                << rotation_r << ", "
                << score << std::endl;
    }
      std::cout << std::endl;
    cost_map_->save("cost", "/workspace/data/data/test.png");
    // mtx_.unlock();
  }
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "amcl_node");
  AmclNode amcl_node;

#if 0
  particle_filter::MultivariateNormalDistribution a;
  size_t size = 10000;
  Eigen::Matrix3d covariance;
  covariance << 1.0, 0.8, 0.0,
                0.8, 1.0, 0.0,
                0.0, 0.0, 1.0;
  Eigen::MatrixXd target;
  a.pdf(covariance, size, target);
  std::cout << "random vectors:" << target.rows() << ", " << target.cols() << std::endl;
  std::cout << target << std::endl;

  cost_map::CostMap test_map;
  test_map.add("a", 0.5);
  Eigen::Vector2d corner_lb(-10.0, -10.0);
  Eigen::Vector2d corner_rt(10.0, 10.0);
  test_map.extend(corner_lb, corner_rt, 0.5);
  for (size_t i = 0; i < size; ++i) {
    Eigen::Array2i pos;
    test_map.world_to_map(target.block<2, 1>(0, i), pos);
    if (test_map.is_inside("a", pos)) {
      test_map.at("a", pos) *= 1.1;
      if (test_map.at("a", pos) > 1.0) test_map.at("a", pos) = 1.0;
    }
  }
  test_map.save("a", "/workspace/data/data/random.png");
#endif

  std::cout << "main" << std::endl;

  ros::Rate loop_rate(0.5);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  // ros::spin();
  return 0;
}
