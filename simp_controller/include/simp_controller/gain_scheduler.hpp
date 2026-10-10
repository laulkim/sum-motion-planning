#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <simp_planner_msgs/msg/tracking_trajectory_point.hpp>

namespace simp_controller {
using Point = simp_planner_msgs::msg::TrackingTrajectoryPoint;
struct Gains { double kx{3.0}, ky{4.0}, ktheta{2.0}; };

// Ordered Cartesian CSV: vx outermost, yaw_rate innermost. Axes may be
// nonuniform or singleton. Values are in m/s, m/s, rad/s; gains in 1/s.
class GainLut {
 public:
  GainLut() = default;
  GainLut(std::array<std::vector<double>, 3> axes, std::vector<Gains> values)
      : axes_(std::move(axes)), values_(std::move(values)) {
    std::size_t count = 1;
    for (const auto& axis : axes_) {
      if (axis.empty() || count > std::numeric_limits<std::size_t>::max() / axis.size())
        throw std::invalid_argument("invalid LUT axis size");
      for (std::size_t i = 0; i < axis.size(); ++i)
        if (!std::isfinite(axis[i]) || (i && (axis[i] <= axis[i-1] || !std::isfinite(axis[i]-axis[i-1]))))
          throw std::invalid_argument("LUT axes must be finite and strictly increasing");
      count *= axis.size();
    }
    if (values_.size() != count) throw std::invalid_argument("incomplete LUT grid");
    for (const auto& g : values_)
      if (!std::isfinite(g.kx) || !std::isfinite(g.ky) || !std::isfinite(g.ktheta) ||
          g.kx < 0.0 || g.ky < 0.0 || g.ktheta <= 0.0)
        throw std::invalid_argument("invalid LUT gain");
  }

  static GainLut load_csv(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::invalid_argument("cannot open gain LUT: " + path);
    std::string line;
    std::getline(file, line);
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line != "vx,vy,yaw_rate,kx,ky,ktheta") throw std::invalid_argument("invalid LUT CSV header");
    std::vector<std::array<double, 6>> rows;
    std::array<std::vector<double>, 3> axes;
    while (std::getline(file, line)) {
      if (!line.empty() && line.back() == '\r') line.pop_back();
      std::istringstream stream(line);
      std::array<double, 6> row;
      std::string cell;
      for (auto& value : row) {
        if (!std::getline(stream, cell, ',')) throw std::invalid_argument("short LUT CSV row");
        std::size_t consumed = 0;
        value = std::stod(cell, &consumed);
        if (cell.find_first_not_of(" \t", consumed) != std::string::npos || !std::isfinite(value))
          throw std::invalid_argument("nonfinite or malformed LUT CSV value");
      }
      if (std::getline(stream, cell, ',') || (!line.empty() && line.back() == ','))
        throw std::invalid_argument("extra LUT CSV column");
      rows.push_back(row);
      for (std::size_t i = 0; i < 3; ++i) axes[i].push_back(row[i]);
    }
    for (auto& axis : axes) {
      std::sort(axis.begin(), axis.end());
      axis.erase(std::unique(axis.begin(), axis.end()), axis.end());
    }
    std::vector<Gains> gains;
    for (const auto& row : rows) gains.push_back({row[3], row[4], row[5]});
    GainLut lut(axes, std::move(gains));
    std::size_t n = 0;
    for (double vx : axes[0]) for (double vy : axes[1]) for (double yaw : axes[2]) {
      if (rows[n][0] != vx || rows[n][1] != vy || rows[n][2] != yaw)
        throw std::invalid_argument("LUT CSV grid must be complete, unique and ordered");
      ++n;
    }
    return lut;
  }

  bool empty() const { return values_.empty(); }

  Gains lookup(double vx, double vy, double yaw_rate) const {
    if (empty()) throw std::invalid_argument("lookup of empty LUT");
    std::array<std::size_t, 3> lower{}, upper{};
    std::array<double, 3> fraction{};
    const std::array<double, 3> query{vx, vy, yaw_rate};
    for (std::size_t i = 0; i < 3; ++i) {
      if (!std::isfinite(query[i])) throw std::invalid_argument("nonfinite LUT query");
      const auto& axis = axes_[i];
      const double value = std::clamp(query[i], axis.front(), axis.back());
      const auto it = std::lower_bound(axis.begin(), axis.end(), value);
      upper[i] = static_cast<std::size_t>(it - axis.begin());
      lower[i] = *it == value ? upper[i] : upper[i] - 1;
      if (lower[i] != upper[i])
        fraction[i] = (value - axis[lower[i]]) / (axis[upper[i]] - axis[lower[i]]);
    }
    Gains result{0.0, 0.0, 0.0};
    for (unsigned corner = 0; corner < 8; ++corner) {
      double weight = 1.0;
      std::array<std::size_t, 3> index;
      for (unsigned i = 0; i < 3; ++i) {
        const bool high = corner & (1u << i);
        index[i] = high ? upper[i] : lower[i];
        weight *= high ? fraction[i] : 1.0 - fraction[i];
      }
      const auto& gain = values_[(index[0] * axes_[1].size() + index[1]) * axes_[2].size() + index[2]];
      result.kx += weight * gain.kx;
      result.ky += weight * gain.ky;
      result.ktheta += weight * gain.ktheta;
    }
    return result;
  }

 private:
  std::array<std::vector<double>, 3> axes_;
  std::vector<Gains> values_;
};

// Exact Planner reference semantics, independent of output, odometry or mode.
inline double translational_gain_scale(const Point& reference) {
  return reference.vx == 0.0 && reference.vy == 0.0 && reference.yaw_rate != 0.0 ? 0.0 : 1.0;
}

inline Gains scheduled_gains(const Point& reference, const Gains& nominal,
                             const GainLut* lut = nullptr) {
  auto gain = lut && !lut->empty() ? lut->lookup(reference.vx, reference.vy, reference.yaw_rate) : nominal;
  // Structural constraint AFTER lookup: optimizer cannot override it.
  if (translational_gain_scale(reference) == 0.0) gain.kx = gain.ky = 0.0;
  return gain;
}
}  // namespace simp_controller
