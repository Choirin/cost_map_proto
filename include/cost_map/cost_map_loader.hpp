#pragma once
#ifndef __INCLUDE_COST_MAP_LOADER__
#define __INCLUDE_COST_MAP_LOADER__
#include <cost_map/cost_map.hpp>

#include <yaml-cpp/yaml.h>

namespace cost_map {

class CostMapLoader {
public:
  CostMapLoader() {}
  ~CostMapLoader() {}

  static void load(const std::string &yaml_path, CostMap &target_map) {
    YAML::Node lconf = YAML::LoadFile(yaml_path);

    std::cout << lconf << std::endl;

    std::string image = lconf["image"].as<std::string>();
    image = "/workspace/data/data/2d_map/image.png";
    double resolution = lconf["resolution"].as<double>();
    std::vector<double> origin = lconf["origin"].as<std::vector<double>>();
    double occupied_thresh = lconf["occupied_thresh"].as<double>();
    double free_thresh = lconf["free_thresh"].as<double>();
    int negate = lconf["negate"].as<int>();

    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> data_uc;
    cv::Mat img = cv::imread(image, cv::IMREAD_GRAYSCALE);
    cv::cv2eigen(img, data_uc);

    Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic> array;
    if (!negate) {
      array = (255.0 - data_uc.array().cast<float>()) / 255.0;
    } else {
      array = data_uc.array().cast<float>() / 255.0;
    }
    std::cout << array;

    Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic> occupied =
        (occupied_thresh < array).cast<float>();
    Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic> free =
        (array < free_thresh).cast<float>();
    Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic> unknown =
        ((free_thresh <= array) && (array <= occupied_thresh)).cast<float>();

    float cost_initial = logf(0.5 / (1 - 0.5));
    float cost_occupied = logf(0.9 / (1 - 0.9));
    float cost_free = logf(0.1 / (1 - 0.1));
    auto data =
        std::make_shared<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>>(
            occupied * cost_occupied +
            free * cost_free +
            unknown * cost_initial);

    // std::shared_ptr<CostMap> cost_map(new CostMap());
    target_map.set_origin(Eigen::Vector2d(origin.data()));
    target_map.set_resolution(resolution);
    target_map.add("cost", data);
  }

private:
};

}  // namespace cost_map

#endif // __INCLUDE_COST_MAP_LOADER__
