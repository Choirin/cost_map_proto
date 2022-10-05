#ifndef COST_MAP_COST_MAP_HPP
#define COST_MAP_COST_MAP_HPP
#include <Eigen/Geometry>
#include <memory>

namespace cost_map {

template <typename T>
class CostMap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using CellType = T;
  using MapType = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

  CostMap()
      : origin_(Eigen::Vector2d::Zero()),
        size_(Eigen::Array2i::Zero()),
        resolution_(0.05) {}
  CostMap(const Eigen::Vector2d &origin, const Eigen::Array2i &size,
          const float resolution)
      : origin_(origin), size_(size), resolution_(resolution) {}
  ~CostMap() {}

  void set_origin(const Eigen::Vector2d &origin) { origin_ = origin; }

  void get_origin(Eigen::Vector2d &origin) const { origin = origin_; }
  void get_size(Eigen::Array2i &size) const { size = size_; }
  float get_resolution() const { return resolution_; }

  void resize(const Eigen::Array2i &size, const Eigen::Vector2d &origin,
              const CellType value) {
    Eigen::Array2i offset_m =
        ((origin - origin_) / resolution_).cast<int>().array().reverse();
    // overlap on the original map coords
    Eigen::Array2i old_lb = offset_m.max(Eigen::Array2i::Zero());
    Eigen::Array2i old_rt = (offset_m + size).min(size_);
    // std::cout << "old_rect: "
    //           << old_lb(0) << ", " << old_lb(1) << ", "
    //           << old_rt(0) << ", " << old_rt(1) << std::endl;

    // overlap on the new map coords
    Eigen::Array2i new_lb = (-offset_m).max(Eigen::Array2i::Zero());
    Eigen::Array2i new_rt = (-offset_m + size_).min(size);
    // std::cout << "new_rect: "
    //           << new_lb(0) << ", " << new_lb(1) << ", "
    //           << new_rt(0) << ", " << new_rt(1) << std::endl;

    // overlap size
    Eigen::Array2i overlap_size = new_rt - new_lb;
    Eigen::Array2i overlap_size_ = old_rt - old_lb;
    // std::cout << "new: " << overlap_size.transpose() << std::endl;
    // std::cout << "old: " << overlap_size_.transpose() << std::endl;
    assert((overlap_size == overlap_size_).all());

    // update geometry
    size_ = size;
    origin_ = origin;

    // copy overlapped from the old data
    for (const auto &layer : data_) {
      auto data = std::unique_ptr<MapType>(new MapType);
      *data = MapType::Constant(size(0), size(1), value);
      if ((overlap_size > Eigen::Array2i::Zero()).all()) {
        data->block(new_lb(0), new_lb(1), overlap_size(0), overlap_size(1)) =
            data_[layer.first]->block(old_lb(0), old_lb(1), overlap_size(0),
                                      overlap_size(1));
      }
      data_[layer.first] = std::move(data);
    }
  }

  void expand(const Eigen::Vector2d &corner_lb,
              const Eigen::Vector2d &corner_rt, const CellType value) {
    Eigen::Array2d new_corner_lb, new_corner_rt;
    if ((size_ == Eigen::Array2i::Zero()).any()) {
      new_corner_lb = (corner_lb.array() / resolution_).floor();
      new_corner_rt = (corner_rt.array() / resolution_).ceil();
    } else {
      Eigen::Array2d old_corner_lb = origin_.array();
      Eigen::Array2d old_corner_rt =
          origin_.array() + size_.cast<double>().reverse() * resolution_;
      new_corner_lb =
          (corner_lb.array().min(old_corner_lb) / resolution_).floor();
      new_corner_rt =
          (corner_rt.array().max(old_corner_rt) / resolution_).ceil();
    }
    Eigen::Vector2d origin = (new_corner_lb * resolution_).matrix();
    Eigen::Array2i size =
        (new_corner_rt - new_corner_lb + 0.5).cast<int>().reverse();
    // std::cout << "origin: " << origin(0) << ", " << origin(1) << std::endl;
    // std::cout << "size: " << size(0) << ", " << size(1) << std::endl;
    resize(size, origin, value);
  }

  // Expand the map to contain the given point
  void expand(const Eigen::Matrix2Xd &points, const Eigen::Vector2d &reference,
              const Eigen::Array<bool, Eigen::Dynamic, 1> &mask,
              const CellType value) {
    assert(points.cols() == mask.size());

    Eigen::Vector2d corner_lb = reference, corner_rt = reference;
    for (int i = 0; i < points.cols(); ++i) {
      if (!mask[i]) continue;
      auto &&point = points.col(i);
      if (point(0) < corner_lb(0)) corner_lb(0) = point(0);
      if (point(1) < corner_lb(1)) corner_lb(1) = point(1);
      if (corner_rt(0) < point(0)) corner_rt(0) = point(0);
      if (corner_rt(1) < point(1)) corner_rt(1) = point(1);
    }
    expand(corner_lb, corner_rt, value);
  }

  // Expand the map with the given margin
  void expand(const Eigen::Array2i &margin, const CellType value) {
    Eigen::Vector2d origin =
        origin_ - margin.cast<double>().matrix() * resolution_;
    Eigen::Array2i size = size_ + margin * 2;
    resize(size, origin, value);
  }

  void add(const std::string &layer, const CellType value) {
    data_[layer] = std::unique_ptr<MapType>(new MapType);
    *data_[layer] = MapType::Constant(size_(0), size_(1), value);
  }

  void add(const std::string &layer, std::unique_ptr<MapType> data) {
    data_[layer] = std::move(data);
  }

  bool has(const std::string &layer) const {
    return data_.find(layer) != data_.end();
  }

  inline void world_to_map(const Eigen::Vector2d &position_w,
                           Eigen::Array2i &index_m) const {
    Eigen::Array2i position_m =
        ((position_w - origin_) / resolution_ + Eigen::Vector2d::Constant(0.5))
            .array()
            .cast<int>();
    index_m[0] = position_m[1];
    index_m[1] = position_m[0];
  }

  inline void map_to_world(const Eigen::Array2i &index_m,
                           Eigen::Vector2d &position_w) const {
    Eigen::Vector2d origin_offset =
        (origin_ / resolution_).array().round().matrix();
    position_w[0] = index_m[1];
    position_w[1] = index_m[0];
    position_w = (position_w + origin_offset) * resolution_;
  }

  inline bool is_inside(const Eigen::Array2i &index) const {
    return (0 <= index).all() && (index < size_).all();
  }

  MapType &data(const std::string &layer) { return *data_[layer]; }

  inline CellType &at(const std::string &layer, const Eigen::Array2i &index) {
    auto &data = data_[layer];
    assert(is_inside(index));
    return (*data)(index(0), index(1));
  }

  bool crop(const std::string &layer, const Eigen::Array2i &left_bottom,
            const Eigen::Array2i size, MapType &value) {
    if (!is_inside(left_bottom) || !is_inside(left_bottom + size)) return false;
    value =
        data_[layer]->block(left_bottom(0), left_bottom(1), size(0), size(1));
    return true;
  }

 protected:
  template <typename ActionType>
  inline bool bresenham(const Eigen::Array2i &a, const Eigen::Array2i &b,
                        ActionType action) const {
    static_assert(
        std::is_same<std::invoke_result_t<ActionType, const Eigen::Array2i &>,
                     bool>::value,
        "Expected functor-type bool(const Eige::Array2i &)");
    Eigen::Array2i index = a;
    auto &xa = index.data()[0], &ya = index.data()[1];
    auto &xb = b.data()[0], &yb = b.data()[1];
    int dx = abs(xb - xa), sx = xa < xb ? 1 : -1;
    int dy = -abs(yb - ya), sy = ya < yb ? 1 : -1;
    int err = dx + dy, e2; /* error value e_xy */

    for (;;) { /* loop */
      if (!action(index)) return false;
      if (xa == xb && ya == yb) break;
      e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        xa += sx;
      } /* e_xy+e_x > 0 */
      if (e2 <= dx) {
        err += dx;
        ya += sy;
      } /* e_xy+e_y < 0 */
    }
    return true;
  }

 private:
  std::unordered_map<std::string, std::unique_ptr<MapType>> data_;
  Eigen::Vector2d origin_;
  Eigen::Array2i size_;
  float resolution_;
};

}  // namespace cost_map

#endif  // COST_MAP_COST_MAP_HPP
