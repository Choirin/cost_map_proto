#ifndef COST_MAP_COST_MAP_SCAN_HPP
#define COST_MAP_COST_MAP_SCAN_HPP
#include <Eigen/Geometry>
#include <cost_map/cost_map.hpp>
#include <frame_buffer/scan_frame.hpp>

namespace cost_map {

inline float logit(const float raw) { return logf(raw / (1.0 - raw)); }

class CostMapScan : public CostMap<float> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CostMapScan(const float initial = 0.5, const float hit = 0.7,
              const float miss = 0.4, const float min = 0.12,
              const float max = 0.97);
  CostMapScan(const Eigen::Vector2d &origin, const Eigen::Array2i &size,
              const float resolution, const float initial = 0.5,
              const float hit = 0.7, const float miss = 0.4,
              const float min = 0.12, const float max = 0.97);
  ~CostMapScan() {}

  void set_scan_range_max(const float max) { scan_range_max_ = max; }

  void update(frame_buffer::ScanFrame &scan, const bool &expand_map = true);
  void update(frame_buffer::ScanFrame &scan,
              const Eigen::Matrix3d &external_transform,
              const bool &expand_map = true);

  // for debug use only
  bool save(const std::string &layer, const std::string &image_path);

 private:
  void hit(const std::string &layer, const Eigen::Array2i &index);
  void miss(const std::string &layer, const Eigen::Array2i &index);

  void project(const Eigen::Matrix2Xd &points,
               const Eigen::Array<bool, Eigen::Dynamic, 1> &mask,
               const Eigen::Vector2d &center);

  const float sensor_model_initial_;
  const float sensor_model_hit_;
  const float sensor_model_miss_;
  const float sensor_model_min_;
  const float sensor_model_max_;

  float scan_range_max_;
};

}  // namespace cost_map

#endif  // COST_MAP_COST_MAP_SCAN_HPP
