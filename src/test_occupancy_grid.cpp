#include <cost_map/occupancy_grid.hpp>

#include <filesystem>

int main(void) {
  const std::filesystem::path map_yaml = "./test_map/map.yaml";
  std::unique_ptr<cost_map::OccupancyGrid> grid_map =
      cost_map::LoadOccupancyGridFromFile(map_yaml);
  cost_map::SaveOccupancyGridAsFile(std::filesystem::path("./tmp/map.yaml"), *grid_map);
}
