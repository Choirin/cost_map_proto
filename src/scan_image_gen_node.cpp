#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

#include <cmath>
#include <iostream>
#include <mutex>
#include <queue>

#include "cost_map/cost_map.hpp"
#include "cost_map/cost_map_scan.hpp"
#include "cost_map/occupancy_grid.hpp"
#include "frame_buffer/scan_frame.hpp"
#include "frame_buffer/scan_frame_buffer.hpp"

const int kLaserScanSkip = 4;
const Eigen::Array2i kCropMapHalfSize(64, 64);
const double kInitialCellValue = logf((255.0 - 100.0) / 255.0);

const std::string kOdomFrame = "odom";
const std::string kGlobalFrame = "map";
const std::string kLaserTopic = "/depth/scan";

const double kScanUpdateDistThresh = 0.5;
const double kScanUpdateAngleThresh = 30.0 * M_PI / 180.0;

void save_image(
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
  cv::imwrite(name, image);
  // cv::imshow(name, image);
  // cv::waitKey(0);
}

bool crop_map_image(cost_map::CostMap<float> &cost_map,
                    const Eigen::Vector2d &origin,
                    const Eigen::Array2i &half_size,
                    cost_map::CostMap<float>::MapType &cropped) {
  Eigen::Array2i center;
  cost_map.world_to_map(origin, center);
  if (!cost_map.crop("cost", center - half_size, 2 * half_size, cropped))
    return false;
  return true;
}

class ScanFrameBufferNode {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ScanFrameBufferNode(std::shared_ptr<cost_map::OccupancyGrid> occupancy_grid,
                      const std::string &odom_frame,
                      const std::string &global_frame, const size_t &frame_size,
                      const std::string &laser_topic)
      : occupancy_grid_(occupancy_grid),
        odom_frame_(odom_frame),
        global_frame_(global_frame),
        frame_size_(frame_size),
        laserscan_skip_(kLaserScanSkip) {
    laserscan_sub_ =
        nh_.subscribe(laser_topic, 1, &ScanFrameBufferNode::callback, this);
  }

  void callback(const sensor_msgs::LaserScan &msg) {
    // obatain translation and rotation.
    sensor_frame_ = msg.header.frame_id;
    Eigen::Vector2d translation;
    double yaw;
    if (!get_tf(odom_frame_, sensor_frame_, msg.header.stamp, translation, yaw))
      return;

    if (!buffer_) {
      std::shared_ptr<Eigen::VectorXd> angles =
          std::shared_ptr<Eigen::VectorXd>(new Eigen::VectorXd);
      *angles = Eigen::VectorXd::LinSpaced(msg.ranges.size() / laserscan_skip_,
                                           msg.angle_min, msg.angle_max);
      buffer_ = std::make_unique<frame_buffer::ScanFrameBuffer>(
          angles, kScanUpdateDistThresh, kScanUpdateAngleThresh, frame_size_);
    }

    Eigen::VectorXd ranges(msg.ranges.size() / laserscan_skip_);
    for (size_t i = 0; i < msg.ranges.size() / laserscan_skip_; ++i)
      ranges[i] = msg.ranges[msg.ranges.size() - i * laserscan_skip_ - 1];
    buffer_->update(msg.header.stamp.toSec(), ranges, translation, yaw);
  }

  bool generate_cropped_images(cost_map::CostMap<float>::MapType &cropped_scan,
                               cost_map::CostMap<float>::MapType &cropped_grid,
                               const Eigen::Array2i &half_size) {
    if (!buffer_) return false;
    std::unique_ptr<cost_map::CostMapScan> cost_map =
        create_cost_map_from_occupancy_grid();

    // get tfs at the time of the last scan.
    auto last_frame = buffer_->back();
    auto time = ros::Time().fromSec(last_frame->timestamp());
    Eigen::Vector2d global_translation, translation;
    double global_yaw, yaw;
    if (!get_tf(global_frame_, sensor_frame_, time, global_translation,
                global_yaw) ||
        !get_tf(global_frame_, odom_frame_, time, translation, yaw))
      return false;

    // create transform matrix from odom to global frame.
    Eigen::Matrix3d transform = Eigen::Matrix3d::Zero();
    transform.block<2, 2>(0, 0) = Eigen::Rotation2Dd(yaw).toRotationMatrix();
    transform.block<2, 1>(0, 2) = translation;

    // project scans to cost map.
    buffer_->project(*cost_map, transform, false);

    // crop cost maps.
    if (!crop_map_image(*cost_map, global_translation, half_size,
                        cropped_scan) ||
        !crop_map_image(*occupancy_grid_, global_translation, half_size,
                        cropped_grid))
      return false;

    ROS_INFO("scan image is generated from %lu frames.", buffer_->size());
    return true;
  }

 private:
  ros::NodeHandle nh_;
  ros::Subscriber laserscan_sub_;
  tf::TransformListener tf_listener_;

  const std::shared_ptr<cost_map::OccupancyGrid> occupancy_grid_;
  const std::string odom_frame_;
  const std::string global_frame_;
  const size_t frame_size_;

  const int laserscan_skip_;

  std::unique_ptr<frame_buffer::ScanFrameBuffer> buffer_;
  std::string sensor_frame_;

  bool get_tf(const std::string &parent_frame_id, const std::string &frame_id,
              const ros::Time &time, Eigen::Vector2d &translation,
              double &yaw) {
    tf::StampedTransform transform;

    try {
      tf_listener_.lookupTransform(parent_frame_id, frame_id, time, transform);
    } catch (const tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      return false;
    }
    translation << transform.getOrigin().getX(), transform.getOrigin().getY();
    yaw = tf::getYaw(transform.getRotation());
    return true;
  }

  std::unique_ptr<cost_map::CostMapScan> create_cost_map_from_occupancy_grid(
      void) {
    Eigen::Vector2d origin;
    Eigen::Array2i size;
    occupancy_grid_->get_origin(origin);
    occupancy_grid_->get_size(size);
    float resolution = occupancy_grid_->get_resolution();

    std::unique_ptr<cost_map::CostMapScan> cost_map =
        std::make_unique<cost_map::CostMapScan>(origin, size, resolution, 0.5,
                                                0.9, 0.2);
    cost_map->set_scan_range_max(1.9);
    return cost_map;
  }
};

std::unique_ptr<cost_map::OccupancyGrid> load_occupancy_grid_from_rostopic(
    void) {
  ros::NodeHandle nh;
  std::unique_ptr<cost_map::OccupancyGrid> occupancy_grid;

  auto callback = [&](const nav_msgs::OccupancyGridConstPtr &msg) {
    const Eigen::Array<int8_t, Eigen::Dynamic, Eigen::Dynamic> array =
        Eigen::Map<const Eigen::Array<int8_t, Eigen::Dynamic, Eigen::Dynamic,
                                      Eigen::RowMajor>>(
            msg->data.data(), msg->info.height, msg->info.width);
    array.colwise().reverse();
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> data_uint8 =
        ((array == 0).cast<uint8_t>() * 254 +
         (array == -1).cast<uint8_t>() * 205 +
         (array == 100).cast<uint8_t>() * 0)
            .matrix();

    std::unique_ptr<cost_map::OccupancyGrid::MapType> data =
        std::unique_ptr<cost_map::OccupancyGrid::MapType>(
            new cost_map::OccupancyGrid::MapType());
    *data = data_uint8.cast<cost_map::OccupancyGrid::MapType::Scalar>();
    data->array() = (255.0 - data->array()) / 255.0;
    data->array() = (data->array() / (1.0 - data->array())).log();

    const Eigen::Vector2d origin_2d(msg->info.origin.position.x,
                                    msg->info.origin.position.y);
    const Eigen::Array2i size(data->rows(), data->cols());
    occupancy_grid = std::make_unique<cost_map::OccupancyGrid>(
        std::move(data), origin_2d, size, msg->info.resolution);
  };
  auto sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, callback);

  ros::Rate loop_rate(2);
  while (ros::ok()) {
    ros::spinOnce();
    if (occupancy_grid) break;
    loop_rate.sleep();
  }
  sub.shutdown();

  return occupancy_grid;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "scan_image_gen_node");
  ROS_INFO("scan_image_gen_node started.");

  // Load occupancy grid from rostopic, expand it.
  std::unique_ptr<cost_map::OccupancyGrid> occupancy_grid =
      std::move(load_occupancy_grid_from_rostopic());
  occupancy_grid->expand(kCropMapHalfSize, kInitialCellValue);
  ROS_INFO("grid map loaded.");

  ScanFrameBufferNode frame_buffer_node(std::move(occupancy_grid), kOdomFrame,
                                        kGlobalFrame, 32, kLaserTopic);

  // 1 thread to process subscriber.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate loop_rate(0.2);
  while (ros::ok()) {
    cost_map::CostMap<float>::MapType cropped_scan, cropped_grid;
    if (frame_buffer_node.generate_cropped_images(cropped_scan, cropped_grid,
                                                  kCropMapHalfSize)) {
      save_image("scan.png", cropped_scan);
      save_image("grid.png", cropped_grid);
    }
    loop_rate.sleep();
  }

  spinner.stop();

  return 0;
}
