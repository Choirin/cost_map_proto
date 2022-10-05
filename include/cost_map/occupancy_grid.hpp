#ifndef COST_MAP_OCCUPANCY_GRID_HPP_
#define COST_MAP_OCCUPANCY_GRID_HPP_

#include <filesystem>

#include "cost_map/cost_map.hpp"

namespace cost_map {

class OccupancyGrid;

std::unique_ptr<OccupancyGrid> LoadOccupancyGridFromFile(
    const std::filesystem::path &map_yaml);
void SaveOccupancyGridAsFile(const std::filesystem::path &map_yaml,
                             OccupancyGrid &cost_map);

class OccupancyGrid : public CostMap<float> {
 public:
  OccupancyGrid(std::unique_ptr<OccupancyGrid::MapType> data,
                const Eigen::Vector2d &origin, const Eigen::Array2i &size,
                const float resolution)
      : CostMap(origin, size, resolution) {
    add("cost", std::move(data));
  }
  OccupancyGrid(std::unique_ptr<OccupancyGrid::MapType> data,
                const Eigen::Vector2d &origin, const Eigen::Array2i &size,
                const float resolution, const float occupied_thresh,
                const float free_thresh, const int negate)
      : CostMap(origin, size, resolution),
        occupied_thresh_(occupied_thresh),
        free_thresh_(free_thresh),
        negate_(negate) {
    add("cost", std::move(data));
  }

  float get_occupied_thresh() const { return occupied_thresh_; }
  float get_free_thresh() const { return free_thresh_; }
  int get_negate() const { return negate_; }

 private:
  float occupied_thresh_;
  float free_thresh_;
  int negate_;
};

}  // namespace cost_map

#endif  // COST_MAP_OCCUPANCY_GRID_HPP_
