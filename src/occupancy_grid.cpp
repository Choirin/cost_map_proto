#include <cost_map/occupancy_grid.hpp>

#include <yaml-cpp/yaml.h>

namespace cost_map {

std::unique_ptr<OccupancyGrid> LoadOccupancyGridFromFile(
    const std::filesystem::path& map_yaml) {
  std::string image_filename;
  float resolution;
  float occupied_thresh, free_thresh;
  std::vector<double> origin;
  int negate;

  try {
    YAML::Node lconf = YAML::LoadFile(map_yaml.string());
    image_filename = lconf["image"].as<std::string>("map.pgm");
    resolution = lconf["resolution"].as<float>(0.05);
    occupied_thresh = lconf["occupied_thresh"].as<double>(0.65);
    free_thresh = lconf["free_thresh"].as<double>(0.196);
    origin =
        lconf["origin"].as<std::vector<double>>(std::vector<double>(3, 0.0));
    negate = lconf["negate"].as<int>(0);
  } catch (YAML::ParserException& e) {
    throw std::runtime_error(std::string("invalid map yaml:") + e.what());
  }

  cv::Mat image =
      cv::imread(map_yaml.parent_path() / image_filename, cv::IMREAD_ANYDEPTH);
  assert(image.type() == CV_8UC1);
  cv::flip(image, image, 0);

  std::unique_ptr<OccupancyGrid::MapType> data =
      std::unique_ptr<OccupancyGrid::MapType>(new OccupancyGrid::MapType());
  cv::cv2eigen(image, *data);
  if (negate)
    data->array() = data->array() / 255.0;
  else
    data->array() = (255.0 - data->array()) / 255.0;
  data->array() = (data->array() / (1.0 - data->array())).log();

  const Eigen::Vector2d origin_2d(origin[0], origin[1]);
  const Eigen::Array2i size(data->rows(), data->cols());
  return std::make_unique<OccupancyGrid>(std::move(data), origin_2d, size,
                                         resolution, occupied_thresh,
                                         free_thresh, negate);
}

void SaveOccupancyGridAsFile(
    const std::filesystem::path& map_yaml, OccupancyGrid &cost_map) {
  const std::string image_filename = "map.pgm";
  Eigen::Vector2d origin2d;
  cost_map.get_origin(origin2d);
  std::vector<double> origin{origin2d[0], origin2d[1], 0.0};
  const float occupied_thresh = cost_map.get_occupied_thresh();
  const float free_thresh = cost_map.get_free_thresh();
  const int negate = cost_map.get_negate();

  YAML::Node lconf;
  lconf["image"] = image_filename;
  lconf["resolution"] = cost_map.get_resolution();
  lconf["occupied_thresh"] = occupied_thresh;
  lconf["free_thresh"] = free_thresh;
  lconf["origin"] = origin;
  lconf["negate"] = negate;

  if (!std::filesystem::exists(map_yaml.parent_path()))
    std::filesystem::create_directories(map_yaml.parent_path());

  YAML::Emitter out;
  out << lconf;
  std::ofstream file(map_yaml);
  file << out.c_str();
  file.close();

  OccupancyGrid::MapType data = cost_map.data("occupancy");
  data.array() = data.array().exp() / (1 + data.array().exp());
  if (cost_map.get_negate()) data.array() = 1.0 - data.array();
  Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> data_uint8 =
      ((data.array() < free_thresh).cast<uint8_t>() * 255 +
       (occupied_thresh < data.array()).cast<uint8_t>() * 0 +
       (free_thresh <= data.array() && data.array() <= occupied_thresh)
               .cast<uint8_t>() *
           200)
          .matrix();
  cv::Mat image;
  eigen2cv(data_uint8, image);
  cv::flip(image, image, 0);
  cv::imwrite(map_yaml.parent_path() / image_filename, image);
}

}  // namespace cost_map
