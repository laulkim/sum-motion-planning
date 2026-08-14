#include "simp_planner/core.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace simp_planner {
namespace {

PlanningCallCounts g_planning_call_counts;
PlanningBlockTimings g_planning_block_timings;

// Accumulates elapsed wall-clock time into `bucket_ms` for the lifetime of
// the scope it's declared in, regardless of how that scope is exited.
class ScopedBlockTimer {
 public:
  explicit ScopedBlockTimer(double& bucket_ms)
      : bucket_ms_(bucket_ms), start_(std::chrono::steady_clock::now()) {}
  ~ScopedBlockTimer() {
    bucket_ms_ += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start_).count();
  }
  ScopedBlockTimer(const ScopedBlockTimer&) = delete;
  ScopedBlockTimer& operator=(const ScopedBlockTimer&) = delete;

 private:
  double& bucket_ms_;
  std::chrono::steady_clock::time_point start_;
};

constexpr double kInf = std::numeric_limits<double>::infinity();

template <typename T>
T clamp_value(T value, T low, T high) {
  return std::max(low, std::min(value, high));
}

std::size_t lower_interval(const std::vector<double>& x, double query) {
  if (x.size() < 2) return 0;
  if (query <= x.front()) return 0;
  if (query >= x.back()) return x.size() - 2;
  auto it = std::upper_bound(x.begin(), x.end(), query);
  return static_cast<std::size_t>(std::distance(x.begin(), it) - 1);
}

double interp_scalar(const std::vector<double>& x,
                     const std::vector<double>& y,
                     double query) {
  if (x.empty() || y.empty() || x.size() != y.size()) {
    throw std::invalid_argument("invalid interpolation arrays");
  }
  if (x.size() == 1) return y.front();
  if (query <= x.front()) return y.front();
  if (query >= x.back()) return y.back();
  const std::size_t i = lower_interval(x, query);
  const double dx = std::max(x[i + 1] - x[i], 1.0e-15);
  const double alpha = (query - x[i]) / dx;
  return (1.0 - alpha) * y[i] + alpha * y[i + 1];
}

std::vector<double> gradient(const std::vector<double>& y,
                             const std::vector<double>& x,
                             bool edge_order_two = false) {
  const std::size_t n = y.size();
  if (n != x.size() || n < 2) {
    throw std::invalid_argument("gradient requires equal arrays of length >= 2");
  }
  std::vector<double> out(n, 0.0);
  if (n == 2) {
    const double value = (y[1] - y[0]) / std::max(x[1] - x[0], 1.0e-15);
    out[0] = value;
    out[1] = value;
    return out;
  }
  for (std::size_t i = 1; i + 1 < n; ++i) {
    const double hs = std::max(x[i] - x[i - 1], 1.0e-15);
    const double hd = std::max(x[i + 1] - x[i], 1.0e-15);
    out[i] = (-hd / (hs * (hs + hd))) * y[i - 1]
           + ((hd - hs) / (hs * hd)) * y[i]
           + (hs / (hd * (hs + hd))) * y[i + 1];
  }
  if (edge_order_two) {
    const double h1 = std::max(x[1] - x[0], 1.0e-15);
    const double h2 = std::max(x[2] - x[1], 1.0e-15);
    out[0] = (-(2.0 * h1 + h2) / (h1 * (h1 + h2))) * y[0]
           + ((h1 + h2) / (h1 * h2)) * y[1]
           - (h1 / (h2 * (h1 + h2))) * y[2];
    const double hb = std::max(x[n - 1] - x[n - 2], 1.0e-15);
    const double ha = std::max(x[n - 2] - x[n - 3], 1.0e-15);
    out[n - 1] = (hb / (ha * (ha + hb))) * y[n - 3]
               - ((ha + hb) / (ha * hb)) * y[n - 2]
               + ((2.0 * hb + ha) / (hb * (ha + hb))) * y[n - 1];
  } else {
    out[0] = (y[1] - y[0]) / std::max(x[1] - x[0], 1.0e-15);
    out[n - 1] = (y[n - 1] - y[n - 2]) /
                 std::max(x[n - 1] - x[n - 2], 1.0e-15);
  }
  return out;
}

std::vector<double> cumulative_arc_length(const std::vector<double>& x,
                                          const std::vector<double>& y) {
  if (x.size() != y.size()) throw std::invalid_argument("arc arrays mismatch");
  std::vector<double> out(x.size(), 0.0);
  for (std::size_t i = 1; i < x.size(); ++i) {
    out[i] = out[i - 1] + std::hypot(x[i] - x[i - 1], y[i] - y[i - 1]);
  }
  return out;
}

std::vector<double> make_arange(double stop_plus_step, double step) {
  std::vector<double> values;
  if (step <= 0.0) return values;
  const std::size_t count = static_cast<std::size_t>(
      std::max(0.0, std::ceil(stop_plus_step / step)));
  values.reserve(count + 1);
  for (std::size_t i = 0;; ++i) {
    const double value = static_cast<double>(i) * step;
    if (!(value < stop_plus_step - 1.0e-14)) break;
    values.push_back(value);
  }
  return values;
}

void sort_unique(std::vector<double>& values, double tolerance = 1.0e-12) {
  std::sort(values.begin(), values.end());
  auto it = std::unique(values.begin(), values.end(),
                        [tolerance](double a, double b) {
                          return std::abs(a - b) <= tolerance;
                        });
  values.erase(it, values.end());
}

struct ProfileResult {
  std::vector<double> p;
  std::vector<double> dp;
  std::vector<double> ddp;
  std::vector<double> dddp;
};

double multiply_add(double multiplicand, double multiplier, double addend) {
#if defined(__FMA__)
  return std::fma(multiplicand, multiplier, addend);
#else
  return multiplicand * multiplier + addend;
#endif
}

template <std::size_t N>
double evaluate_polynomial_horner(const std::array<double, N>& coefficients,
                                  double value) {
  static_assert(N > 0, "a polynomial must contain at least one coefficient");
  double result = coefficients.back();
  for (std::size_t index = N - 1; index > 0; --index) {
    result = multiply_add(result, value, coefficients[index - 1]);
  }
  return result;
}

double jerk_integrated_distance(double speed, double acceleration,
                                double jerk, double duration) {
  const double quadratic = multiply_add(
      jerk / 6.0, duration, 0.5 * acceleration);
  return duration * multiply_add(quadratic, duration, speed);
}

ProfileResult septic_boundary_profile(const std::vector<double>& q,
                                      double length,
                                      double p0,
                                      double dp0,
                                      double ddp0,
                                      double dddp0,
                                      double p1) {
  const double L = std::max(length, 1.0e-6);
  const double b0 = p0;
  const double b1 = dp0 * L;
  const double b2 = 0.5 * ddp0 * L * L;
  const double b3 = (1.0 / 6.0) * dddp0 * L * L * L;
  const std::array<double, 4> rhs{{
      p1 - (b0 + b1 + b2 + b3),
      -(b1 + 2.0 * b2 + 3.0 * b3),
      -(2.0 * b2 + 6.0 * b3),
      -6.0 * b3,
  }};
  const double b4 = 35.0 * rhs[0] - 15.0 * rhs[1] + 2.5 * rhs[2] - rhs[3] / 6.0;
  const double b5 = -84.0 * rhs[0] + 39.0 * rhs[1] - 7.0 * rhs[2] + 0.5 * rhs[3];
  const double b6 = 70.0 * rhs[0] - 34.0 * rhs[1] + 6.5 * rhs[2] - 0.5 * rhs[3];
  const double b7 = -20.0 * rhs[0] + 10.0 * rhs[1] - 2.0 * rhs[2] + rhs[3] / 6.0;
  const std::array<double, 8> c{{b0, b1, b2, b3, b4, b5, b6, b7}};
  const std::array<double, 7> dc{{
      b1, 2.0 * b2, 3.0 * b3, 4.0 * b4,
      5.0 * b5, 6.0 * b6, 7.0 * b7}};
  const std::array<double, 6> ddc{{
      2.0 * b2, 6.0 * b3, 12.0 * b4,
      20.0 * b5, 30.0 * b6, 42.0 * b7}};
  const std::array<double, 5> dddc{{
      6.0 * b3, 24.0 * b4, 60.0 * b5,
      120.0 * b6, 210.0 * b7}};
  const double inv_L = 1.0 / L;
  const double inv_L2 = inv_L * inv_L;
  const double inv_L3 = inv_L2 * inv_L;

  ProfileResult result;
  result.p.resize(q.size());
  result.dp.resize(q.size());
  result.ddp.resize(q.size());
  result.dddp.resize(q.size());
  for (std::size_t k = 0; k < q.size(); ++k) {
    const bool active = q[k] <= L;
    if (!active) {
      result.p[k] = p1;
      result.dp[k] = 0.0;
      result.ddp[k] = 0.0;
      result.dddp[k] = 0.0;
      continue;
    }
    const double tau = clamp_value(q[k] / L, 0.0, 1.0);
    result.p[k] = evaluate_polynomial_horner(c, tau);
    result.dp[k] = evaluate_polynomial_horner(dc, tau) * inv_L;
    result.ddp[k] = evaluate_polynomial_horner(ddc, tau) * inv_L2;
    result.dddp[k] = evaluate_polynomial_horner(dddc, tau) * inv_L3;
  }
  return result;
}

ProfileResult hold_then_return_profile(const std::vector<double>& q,
                                       double hold_length,
                                       double return_length,
                                       double p0,
                                       double dp0,
                                       double ddp0,
                                       double dddp0,
                                       double p1) {
  hold_length = std::max(hold_length, 0.0);
  return_length = std::max(return_length, 1.0e-6);
  if (hold_length <= 1.0e-9) {
    return septic_boundary_profile(q, return_length, p0, dp0, ddp0, dddp0, p1);
  }
  const auto first = septic_boundary_profile(q, hold_length, p0, dp0, ddp0, dddp0, p0);
  std::vector<double> shifted(q.size());
  for (std::size_t i = 0; i < q.size(); ++i) shifted[i] = std::max(q[i] - hold_length, 0.0);
  const auto second = septic_boundary_profile(shifted, return_length, p0, 0.0, 0.0, 0.0, p1);
  ProfileResult out;
  out.p.resize(q.size()); out.dp.resize(q.size()); out.ddp.resize(q.size()); out.dddp.resize(q.size());
  for (std::size_t i = 0; i < q.size(); ++i) {
    const bool use_first = q[i] <= hold_length;
    out.p[i] = use_first ? first.p[i] : second.p[i];
    out.dp[i] = use_first ? first.dp[i] : second.dp[i];
    out.ddp[i] = use_first ? first.ddp[i] : second.ddp[i];
    out.dddp[i] = use_first ? first.dddp[i] : second.dddp[i];
  }
  return out;
}

ProfileResult terminal_position_profile(const std::vector<double>& q,
                                        double length,
                                        double p0,
                                        double dp0,
                                        double ddp0,
                                        double dddp0,
                                        double p1,
                                        double blend) {
  const double L = std::max(length, 1.0e-6);
  const double L2 = L * L;
  const double L4 = L2 * L2;
  const double c4 = (p1 - p0 - dp0 * L - 0.5 * ddp0 * L * L
                    - (1.0 / 6.0) * dddp0 * L * L * L) /
                    L4;
  const auto seventh = septic_boundary_profile(q, L, p0, dp0, ddp0, dddp0, p1);
  const double b = clamp_value(blend, 0.0, 1.0);
  ProfileResult out;
  out.p.resize(q.size()); out.dp.resize(q.size()); out.ddp.resize(q.size()); out.dddp.resize(q.size());
  for (std::size_t i = 0; i < q.size(); ++i) {
    const double qc = clamp_value(q[i], 0.0, L);
    double p4 = multiply_add(c4, qc, dddp0 / 6.0);
    p4 = multiply_add(p4, qc, 0.5 * ddp0);
    p4 = multiply_add(p4, qc, dp0);
    p4 = multiply_add(p4, qc, p0);
    double dp4 = multiply_add(4.0 * c4, qc, 0.5 * dddp0);
    dp4 = multiply_add(dp4, qc, ddp0);
    dp4 = multiply_add(dp4, qc, dp0);
    const double ddp4 = multiply_add(
        multiply_add(12.0 * c4, qc, dddp0), qc, ddp0);
    const double dddp4 = multiply_add(24.0 * c4, qc, dddp0);
    out.p[i] = (1.0 - b) * p4 + b * seventh.p[i];
    out.dp[i] = (1.0 - b) * dp4 + b * seventh.dp[i];
    out.ddp[i] = (1.0 - b) * ddp4 + b * seventh.ddp[i];
    out.dddp[i] = (1.0 - b) * dddp4 + b * seventh.dddp[i];
  }
  return out;
}

void distance_transform_1d(const std::vector<double>& f,
                           std::vector<double>& d) {
  const int n = static_cast<int>(f.size());
  d.resize(static_cast<std::size_t>(n));
  std::vector<int> v(static_cast<std::size_t>(n));
  std::vector<double> z(static_cast<std::size_t>(n + 1));
  int k = 0;
  v[0] = 0;
  z[0] = -kInf;
  z[1] = kInf;
  for (int q = 1; q < n; ++q) {
    double s = 0.0;
    while (true) {
      const int vk = v[static_cast<std::size_t>(k)];
      const double fq = f[static_cast<std::size_t>(q)];
      const double fvk = f[static_cast<std::size_t>(vk)];
      if (std::isinf(fq) && std::isinf(fvk)) {
        s = kInf;
      } else {
        s = ((fq + static_cast<double>(q * q))
           - (fvk + static_cast<double>(vk * vk))) /
            (2.0 * static_cast<double>(q - vk));
      }
      if (s > z[static_cast<std::size_t>(k)] || k == 0) break;
      --k;
    }
    ++k;
    v[static_cast<std::size_t>(k)] = q;
    z[static_cast<std::size_t>(k)] = s;
    z[static_cast<std::size_t>(k + 1)] = kInf;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[static_cast<std::size_t>(k + 1)] < static_cast<double>(q)) ++k;
    const double delta = static_cast<double>(q - v[static_cast<std::size_t>(k)]);
    d[static_cast<std::size_t>(q)] = delta * delta + f[static_cast<std::size_t>(v[static_cast<std::size_t>(k)])];
  }
}

std::vector<double> euclidean_distance_transform(const std::vector<std::uint8_t>& occupied,
                                                 int width, int height) {
  const std::size_t total = static_cast<std::size_t>(width * height);
  bool any_occupied = std::any_of(occupied.begin(), occupied.end(), [](std::uint8_t v) { return v != 0; });
  if (!any_occupied) return std::vector<double>(total, 1.0e6);
  std::vector<double> intermediate(total, 0.0);
  std::vector<double> f(static_cast<std::size_t>(std::max(width, height)));
  std::vector<double> d;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      f[static_cast<std::size_t>(x)] = occupied[static_cast<std::size_t>(y * width + x)] ? 0.0 : kInf;
    }
    std::vector<double> row(f.begin(), f.begin() + width);
    distance_transform_1d(row, d);
    for (int x = 0; x < width; ++x) intermediate[static_cast<std::size_t>(y * width + x)] = d[static_cast<std::size_t>(x)];
  }
  std::vector<double> output(total, 0.0);
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) f[static_cast<std::size_t>(y)] = intermediate[static_cast<std::size_t>(y * width + x)];
    std::vector<double> column(f.begin(), f.begin() + height);
    distance_transform_1d(column, d);
    for (int y = 0; y < height; ++y) output[static_cast<std::size_t>(y * width + x)] = std::sqrt(std::max(d[static_cast<std::size_t>(y)], 0.0));
  }
  return output;
}

struct ReferenceEvaluation {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> psi;
  std::vector<double> kappa;
  std::vector<double> kappa_s;
  std::vector<std::uint8_t> is_virtual;
};

ReferenceEvaluation evaluate_reference_with_virtual_extension(
    const ReferencePath& path,
    const std::vector<double>& s_query,
    double real_end_s,
    double blend_length) {
  ReferenceEvaluation out;
  const std::size_t n = s_query.size();
  out.x.resize(n); out.y.resize(n); out.psi.resize(n); out.kappa.resize(n); out.kappa_s.resize(n); out.is_virtual.resize(n);
  real_end_s = clamp_value(real_end_s, path.s_min(), path.s_max());
  double xe, ye, psie, kappae, kappase;
  path.evaluate(real_end_s, xe, ye, psie, kappae, kappase);
  const double B = std::max(blend_length, 1.0e-3);
  double previous_d = 0.0;
  double previous_psi = psie;
  double previous_x = xe;
  double previous_y = ye;
  double previous_kappa = kappae;
  for (std::size_t i = 0; i < n; ++i) {
    if (s_query[i] <= real_end_s + 1.0e-12) {
      path.evaluate(s_query[i], out.x[i], out.y[i], out.psi[i], out.kappa[i], out.kappa_s[i]);
      out.is_virtual[i] = 0;
      continue;
    }
    const double d = s_query[i] - real_end_s;
    const double ks = d <= B ? kappase * (1.0 - d / B) : 0.0;
    const double kap = d <= B
        ? kappae + kappase * (d - 0.5 * d * d / B)
        : kappae + 0.5 * kappase * B;
    const double dd = d - previous_d;
    const double psi = previous_psi + 0.5 * (previous_kappa + kap) * dd;
    const double x = previous_x + 0.5 * (std::cos(previous_psi) + std::cos(psi)) * dd;
    const double y = previous_y + 0.5 * (std::sin(previous_psi) + std::sin(psi)) * dd;
    out.x[i] = x; out.y[i] = y; out.psi[i] = wrap_angle(psi); out.kappa[i] = kap; out.kappa_s[i] = ks; out.is_virtual[i] = 1;
    previous_d = d; previous_psi = psi; previous_x = x; previous_y = y; previous_kappa = kap;
  }
  return out;
}

struct InitialSpatialBoundary {
  double n0{0.0};
  double n1{0.0};
  double n2{0.0};
  double n3{0.0};
  double desired_kappa_l{0.0};
};

std::pair<double, double> offset_path_curvature(double n, double n1, double n2,
                                                 double kappa_ref,
                                                 double kappa_ref_s) {
  const double A = 1.0 - kappa_ref * n;
  const double B = n1;
  const double D = std::max(A * A + B * B, 1.0e-10);
  const double sqrt_D = std::sqrt(D);
  const double A_prime = -kappa_ref_s * n - kappa_ref * B;
  const double numerator = kappa_ref * D + A * n2 - B * A_prime;
  return {numerator / (D * sqrt_D), sqrt_D};
}

InitialSpatialBoundary initial_spatial_boundaries(
    const PlannerState& state,
    const PlannerAction& previous_action,
    const FrenetProjection& fr,
    const EnvConfig& cfg,
    const ReferencePath& path) {
  InitialSpatialBoundary result;
  result.n0 = fr.n;
  const double A = std::max(1.0 - fr.kappa * result.n0, 0.15);
  result.n1 = A * std::tan(fr.heading_error);
  const double speed_abs = std::abs(state.speed);
  const double speed_reference = std::max(cfg.constraints.v_eff_min, 1.0e-3);
  const double motion_weight = speed_abs * speed_abs /
      (speed_abs * speed_abs + speed_reference * speed_reference);
  const double measured_curvature = state.motion_heading_rate /
      std::max(speed_abs, 0.25 * speed_reference);
  const double k_vehicle = (1.0 - motion_weight) * fr.kappa
      + motion_weight * measured_curvature;
  const double B = result.n1;
  const double D = A * A + B * B;
  const double D_to_three_halves = D * std::sqrt(D);
  const double A_prime = -fr.kappa_s * result.n0 - fr.kappa * B;
  result.n2 = clamp_value((k_vehicle * D_to_three_halves - fr.kappa * D + B * A_prime) / A,
                          -0.35, 0.35);
  const double measured_kappa_l = (previous_action.motion_heading_acceleration
      - state.acceleration * k_vehicle) /
      std::max(speed_abs * speed_abs, 0.0625 * speed_reference * speed_reference);
  result.desired_kappa_l = (1.0 - motion_weight) * fr.kappa_s
      + motion_weight * measured_kappa_l;
  result.desired_kappa_l = clamp_value(result.desired_kappa_l, -0.10, 0.10);
  const double eps = 1.0e-3;
  const double s1 = std::min(fr.s + eps, path.s_max());
  const double k1 = interp_scalar(path.s(), path.kappa(), s1);
  const double ks1 = interp_scalar(path.s(), path.kappa_s(), s1);
  auto rate_for = [&](double n3) {
    const auto [kap0, speed0] = offset_path_curvature(result.n0, result.n1, result.n2, fr.kappa, fr.kappa_s);
    const double ne = result.n0 + result.n1 * eps + 0.5 * result.n2 * eps * eps + (1.0 / 6.0) * n3 * eps * eps * eps;
    const double n1e = result.n1 + result.n2 * eps + 0.5 * n3 * eps * eps;
    const double n2e = result.n2 + n3 * eps;
    const auto [kap1, speed1] = offset_path_curvature(ne, n1e, n2e, k1, ks1);
    return (kap1 - kap0) / std::max(0.5 * (speed0 + speed1) * eps, 1.0e-9);
  };
  const double r0 = rate_for(0.0);
  const double r1 = rate_for(1.0);
  result.n3 = std::abs(r1 - r0) < 1.0e-9
      ? 0.0
      : (result.desired_kappa_l - r0) / (r1 - r0);
  result.n3 = clamp_value(result.n3, -0.25, 0.25);
  return result;
}

double minimum_lateral_length(double delta_n, double v_scale, const EnvConfig& cfg) {
  const double d = std::abs(delta_n);
  if (d < 1.0e-6) return cfg.lateral.min_length;
  constexpr double c2 = 7.5131884044;
  constexpr double c3 = 52.5;
  const double l_acc = v_scale * std::sqrt(c2 * d / std::max(cfg.constraints.a_lat_max, 1.0e-6));
  const double l_jerk = v_scale * std::cbrt(c3 * d / std::max(cfg.constraints.lateral_jerk_max, 1.0e-6));
  const double l_curvature = std::sqrt(c2 * d / std::max(cfg.constraints.curvature_max, 1.0e-6));
  return clamp_value(cfg.lateral.dynamic_margin * std::max({cfg.lateral.min_length, l_acc, l_jerk, l_curvature}),
                     cfg.lateral.min_length, cfg.lateral.max_length);
}

std::vector<double> truncate_q_at_path_length(const std::vector<double>& q,
                                              const std::vector<double>& x,
                                              const std::vector<double>& y,
                                              double target_length) {
  const auto l = cumulative_arc_length(x, y);
  const double target = std::min(target_length, l.back());
  auto it = std::upper_bound(l.begin(), l.end(), target);
  std::size_t idx = static_cast<std::size_t>(std::distance(l.begin(), it));
  idx = std::min(std::max<std::size_t>(idx, 3), l.size());
  std::vector<double> out(q.begin(), q.begin() + static_cast<std::ptrdiff_t>(idx));
  if (l[idx - 1] > target && idx >= 2) {
    const double ratio = (target - l[idx - 2]) / std::max(l[idx - 1] - l[idx - 2], 1.0e-9);
    out.back() = q[idx - 2] + ratio * (q[idx - 1] - q[idx - 2]);
  }
  return out;
}

std::optional<SpatialPathCandidate> generate_spatial_path_candidate(
    const PlannerState& state,
    const PlannerAction& previous_action,
    const FrenetProjection& fr,
    double n_target,
    double lateral_length,
    double preview_length,
    const EnvConfig& cfg,
    const ReferencePath& reference,
    int candidate_id,
    double start_delay = 0.0,
    bool terminal_position_only = false,
    const ManeuverProfileState* maneuver_profile = nullptr,
    int curvature_retry_depth = 0,
    bool hard_preview_limit = false) {
  ++g_planning_call_counts.spatial_candidate_generation_attempts;
  const double real_end_s = reference.s_max() - cfg.simulation.path_end_margin;
  const double remaining_real = std::max(real_end_s - fr.s, 0.0);
  const auto boundary = initial_spatial_boundaries(state, previous_action, fr, cfg, reference);
  start_delay = std::max(start_delay, 0.0);
  const double profile_elapsed = maneuver_profile
      ? clamp_value(maneuver_profile->elapsed_length, 0.0,
                    maneuver_profile->start_delay + maneuver_profile->lateral_length)
      : 0.0;
  const double profile_total_length = maneuver_profile
      ? maneuver_profile->start_delay + maneuver_profile->lateral_length
      : start_delay + lateral_length;
  const double total_lateral_length = std::max(profile_total_length - profile_elapsed, 0.0);

  auto evaluate_profile = [&](const std::vector<double>& local_q) {
    if (!maneuver_profile) {
      return (terminal_position_only && start_delay <= 1.0e-9)
          ? terminal_position_profile(local_q, lateral_length, boundary.n0,
                                      boundary.n1, boundary.n2, boundary.n3,
                                      n_target, cfg.terminal.position_profile_blend)
          : hold_then_return_profile(local_q, start_delay, lateral_length,
                                     boundary.n0, boundary.n1, boundary.n2,
                                     boundary.n3, n_target);
    }

    std::vector<double> global_q(local_q.size());
    for (std::size_t i = 0; i < local_q.size(); ++i) {
      global_q[i] = profile_elapsed + local_q[i];
    }
    return hold_then_return_profile(
        global_q,
        maneuver_profile->start_delay,
        maneuver_profile->lateral_length,
        maneuver_profile->n0,
        maneuver_profile->n1,
        maneuver_profile->n2,
        maneuver_profile->n3,
        maneuver_profile->target);
  };
  double base_extent = std::max(preview_length * 1.35 + cfg.lateral.preview_extra,
                                total_lateral_length + 1.0);
  if (remaining_real <= preview_length) {
    base_extent = std::max(base_extent, remaining_real + cfg.simulation.virtual_extension_min);
    base_extent = std::min(base_extent, remaining_real + cfg.simulation.virtual_extension_max);
  }
  const double q_extent = std::max(base_extent, 0.2);
  auto q = make_arange(q_extent + cfg.lateral.spatial_ds, cfg.lateral.spatial_ds);
  q.push_back(remaining_real);
  sort_unique(q);
  if (q.size() < 3) q = {0.0, 0.5 * std::max(q_extent, 0.2), std::max(q_extent, 0.2)};

  ProfileResult profile = evaluate_profile(q);
  std::vector<double> s_query(q.size());
  for (std::size_t i = 0; i < q.size(); ++i) s_query[i] = fr.s + q[i];
  auto ref = evaluate_reference_with_virtual_extension(reference, s_query, real_end_s,
                                                        cfg.simulation.virtual_extension_blend_length);
  std::vector<double> x(q.size()), y(q.size());
  for (std::size_t i = 0; i < q.size(); ++i) {
    x[i] = ref.x[i] - profile.p[i] * std::sin(ref.psi[i]);
    y[i] = ref.y[i] + profile.p[i] * std::cos(ref.psi[i]);
  }
  double target_spatial_length = hard_preview_limit
      ? std::max(preview_length, 0.20)
      : std::max(preview_length, std::min(total_lateral_length + 1.0, q_extent));
  if (remaining_real <= preview_length) {
    target_spatial_length = std::max(target_spatial_length, remaining_real + cfg.simulation.virtual_extension_min);
  }
  q = truncate_q_at_path_length(q, x, y, target_spatial_length);
  q.push_back(std::min(remaining_real, q.back()));
  sort_unique(q);

  profile = evaluate_profile(q);
  s_query.resize(q.size());
  for (std::size_t i = 0; i < q.size(); ++i) s_query[i] = fr.s + q[i];
  ref = evaluate_reference_with_virtual_extension(reference, s_query, real_end_s,
                                                   cfg.simulation.virtual_extension_blend_length);
  x.resize(q.size()); y.resize(q.size());
  for (std::size_t i = 0; i < q.size(); ++i) {
    x[i] = ref.x[i] - profile.p[i] * std::sin(ref.psi[i]);
    y[i] = ref.y[i] + profile.p[i] * std::cos(ref.psi[i]);
  }
  auto arc = cumulative_arc_length(x, y);
  std::vector<std::size_t> keep;
  keep.reserve(q.size());
  keep.push_back(0);
  for (std::size_t i = 1; i < q.size(); ++i) {
    if (arc[i] - arc[keep.back()] > 1.0e-8) keep.push_back(i);
  }
  if (keep.size() != q.size()) {
    auto select_double = [&keep](const std::vector<double>& values) {
      std::vector<double> selected; selected.reserve(keep.size());
      for (auto idx : keep) selected.push_back(values[idx]);
      return selected;
    };
    auto select_byte = [&keep](const std::vector<std::uint8_t>& values) {
      std::vector<std::uint8_t> selected; selected.reserve(keep.size());
      for (auto idx : keep) selected.push_back(values[idx]);
      return selected;
    };
    q = select_double(q); x = select_double(x); y = select_double(y);
    profile.p = select_double(profile.p); profile.dp = select_double(profile.dp); profile.ddp = select_double(profile.ddp);
    ref.psi = select_double(ref.psi); ref.kappa = select_double(ref.kappa); ref.kappa_s = select_double(ref.kappa_s);
    ref.is_virtual = select_byte(ref.is_virtual);
    arc = cumulative_arc_length(x, y);
  }
  if (arc.size() < 3 || arc.back() < 0.1) return std::nullopt;

  std::vector<double> psi(arc.size()), kappa(arc.size());
  for (std::size_t i = 0; i < arc.size(); ++i) {
    const double A = 1.0 - ref.kappa[i] * profile.p[i];
    psi[i] = ref.psi[i] + std::atan2(profile.dp[i], A);
  }
  psi = unwrap_angles(psi);
  const double branch_shift = 2.0 * kPi * std::round((state.chi - psi.front()) / (2.0 * kPi));
  for (double& value : psi) value += branch_shift;
  for (std::size_t i = 0; i < arc.size(); ++i) {
    const double A = 1.0 - ref.kappa[i] * profile.p[i];
    const double D = std::max(A * A + profile.dp[i] * profile.dp[i], 1.0e-10);
    const double A_prime = -ref.kappa_s[i] * profile.p[i] - ref.kappa[i] * profile.dp[i];
    kappa[i] = (ref.kappa[i] * D + A * profile.ddp[i] - profile.dp[i] * A_prime) /
               (D * std::sqrt(D));
  }
  auto kappa_l = gradient(kappa, arc, false);
  if (maneuver_profile == nullptr) {
    // A newly generated profile must begin exactly at the measured motion
    // state.  For a latched profile, however, overriding its front tangent and
    // curvature on every 10 Hz replan would repeatedly flatten the first
    // spatial interval and recreate the low-speed procrastination problem.
    psi.front() = state.chi;
    const double speed_abs = std::abs(state.speed);
    const double speed_reference = std::max(cfg.constraints.v_eff_min, 1.0e-3);
    const double motion_weight = speed_abs * speed_abs /
        (speed_abs * speed_abs + speed_reference * speed_reference);
    const double measured_curvature = state.motion_heading_rate /
        std::max(speed_abs, 0.25 * speed_reference);
    kappa.front() = (1.0 - motion_weight) * fr.kappa
        + motion_weight * measured_curvature;
    kappa_l.front() = boundary.desired_kappa_l;
  }
  const double real_end_q = std::min(remaining_real, q.back());
  const double real_end_l = interp_scalar(q, arc, real_end_q);
  const double real_end_n = interp_scalar(q, profile.p, real_end_q);
  const double real_end_psi = interp_scalar(q, psi, real_end_q);
  double end_x, end_y, ref_end_psi, end_kappa, end_kappa_s;
  reference.evaluate(real_end_s, end_x, end_y, ref_end_psi, end_kappa, end_kappa_s);
  (void)end_kappa_s;
  const double real_end_heading_error = wrap_angle(real_end_psi - ref_end_psi);
  const double virtual_extension_length = std::max(0.0, q.back() - remaining_real);
  std::vector<std::size_t> actual;
  for (std::size_t i = 0; i < q.size(); ++i) if (q[i] <= real_end_q + 1.0e-9) actual.push_back(i);
  if (actual.size() < 3) {
    actual.clear();
    for (std::size_t i = 0; i < std::min<std::size_t>(3, q.size()); ++i) actual.push_back(i);
  }
  double max_curvature = 0.0;
  for (auto idx : actual) max_curvature = std::max(max_curvature, std::abs(kappa[idx]));
  if (max_curvature > cfg.constraints.curvature_max + 1.0e-9
      && lateral_length < cfg.lateral.max_length - 1.0e-9
      && curvature_retry_depth < 8) {
    const double scale = std::max(1.15, 1.05 * std::sqrt(max_curvature /
        std::max(cfg.constraints.curvature_max, 1.0e-6)));
    return generate_spatial_path_candidate(state, previous_action, fr, n_target,
        std::min(lateral_length * scale, cfg.lateral.max_length), preview_length,
        cfg, reference, candidate_id, start_delay, terminal_position_only,
        maneuver_profile, curvature_retry_depth + 1, hard_preview_limit);
  }
  const double bound = std::max(std::abs(boundary.n0), std::abs(n_target)) + 0.4;
  double max_abs_n = 0.0;
  double mean_n2 = 0.0;
  double mean_k2 = 0.0;
  double mean_ks2 = 0.0;
  for (auto idx : actual) {
    max_abs_n = std::max(max_abs_n, std::abs(profile.p[idx]));
    mean_n2 += profile.p[idx] * profile.p[idx];
    mean_k2 += kappa[idx] * kappa[idx];
    mean_ks2 += kappa_l[idx] * kappa_l[idx];
  }
  const double denom = static_cast<double>(actual.size());
  mean_n2 /= denom; mean_k2 /= denom; mean_ks2 /= denom;
  const double overshoot = std::max(0.0, max_abs_n - bound);
  const double reference_offset_cost = cfg.cost.w_lateral_offset * mean_n2
      + cfg.cost.w_terminal_offset * real_end_n * real_end_n
      + cfg.cost.w_offset_overshoot * overshoot * overshoot;
  const double path_cost = reference_offset_cost
      + cfg.cost.w_curvature * mean_k2
      + cfg.cost.w_curvature_rate * mean_ks2;

  SpatialPathCandidate candidate;
  candidate.candidate_id = candidate_id;
  candidate.n_target = n_target;
  candidate.lateral_length = lateral_length;
  candidate.start_delay = start_delay;
  candidate.terminal_position_only = terminal_position_only;
  candidate.q_ref = std::move(q);
  candidate.arc_length = std::move(arc);
  candidate.x = std::move(x);
  candidate.y = std::move(y);
  candidate.psi = std::move(psi);
  candidate.kappa = std::move(kappa);
  candidate.kappa_l = std::move(kappa_l);
  candidate.lateral_offset = std::move(profile.p);
  candidate.reference_x = std::move(ref.x);
  candidate.reference_y = std::move(ref.y);
  candidate.virtual_mask = std::move(ref.is_virtual);
  candidate.path_cost = path_cost;
  candidate.reference_offset_cost = reference_offset_cost;
  candidate.real_end_q = real_end_q;
  candidate.real_end_l = real_end_l;
  candidate.real_end_n = real_end_n;
  candidate.real_end_heading_error = real_end_heading_error;
  candidate.virtual_extension_length = virtual_extension_length;
  return candidate;
}

struct PathSample {
  double x{0.0}; double y{0.0}; double psi{0.0}; double kappa{0.0}; double kappa_l{0.0};
};

PathSample interpolate_path(const SpatialPathCandidate& path, double progress) {
  const double query = clamp_value(progress, 0.0, path.arc_length.back());
  return {interp_scalar(path.arc_length, path.x, query),
          interp_scalar(path.arc_length, path.y, query),
          interp_scalar(path.arc_length, path.psi, query),
          interp_scalar(path.arc_length, path.kappa, query),
          interp_scalar(path.arc_length, path.kappa_l, query)};
}

double interpolate_reference_progress(const SpatialPathCandidate& path, double progress) {
  return interp_scalar(path.arc_length, path.q_ref,
                       clamp_value(progress, 0.0, path.arc_length.back()));
}

double reference_progress_rate(const SpatialPathCandidate& path, double progress) {
  if (path.arc_length.size() < 2) return 1.0;
  const auto rate = gradient(path.q_ref, path.arc_length, false);
  return clamp_value(interp_scalar(path.arc_length, rate,
      clamp_value(progress, 0.0, path.arc_length.back())), 0.20, 1.20);
}

struct TerminalControlCommand {
  double jerk{0.0};
  double raw_jerk{0.0};
  double acceleration_command{0.0};
  double speed_reference{0.0};
  bool braking_active{false};
};

struct TerminalRollout {
  bool reached{false};
  double position{0.0};
  double speed{0.0};
  double acceleration{0.0};
  bool overshoot{false};
  int steps{0};
  double elapsed_time{0.0};
};

class TerminalFeedbackController {
 public:
  explicit TerminalFeedbackController(const EnvConfig& cfg) : cfg_(cfg) {}

  double required_stopping_distance(double speed, double acceleration, bool emergency = false) const {
    const auto& c = cfg_.constraints;
    const auto& lc = cfg_.longitudinal;
    const double v = std::max(speed, 0.0);
    const double a = acceleration;
    const double deceleration_limit = emergency ? std::abs(c.a_min)
        : std::min(std::abs(c.a_min), lc.terminal_braking_deceleration);
    const double curve_deceleration = std::max(
        std::min(deceleration_limit, lc.terminal_braking_curve_deceleration), 1.0e-6);
    const double jerk_limit = std::max(emergency ? c.jerk_max
        : std::min(c.jerk_max, lc.terminal_jerk_limit), 1.0e-6);
    double ramp_time = std::max((a + curve_deceleration) / jerk_limit, 0.0);
    ramp_time = std::min(ramp_time, 2.0);
    const double ramp_distance = std::max(
        jerk_integrated_distance(v, a, -jerk_limit, ramp_time), 0.0);
    const double speed_after = std::max(v + a * ramp_time - 0.5 * jerk_limit * ramp_time * ramp_time, 0.0);
    return ramp_distance + speed_after * speed_after / (2.0 * curve_deceleration)
        + lc.terminal_braking_distance_buffer + cfg_.terminal.activation_margin;
  }

  TerminalControlCommand command(double position_error, double speed,
                                 double acceleration, double requested_target_speed,
                                 bool emergency = false) const {
    const auto& c = cfg_.constraints;
    const auto& lc = cfg_.longitudinal;
    const double e = position_error;
    const double v = std::max(speed, 0.0);
    const double a = acceleration;
    // A terminal braking fallback can stop slightly before the exact target.
    // At capture speed, continuing to apply the emergency hold would deadlock
    // the planner a few centimetres outside the terminal tolerance.  Once the
    // vehicle is already quasi-stationary and the remaining target is ahead,
    // switch to the nominal low-speed terminal controller on the already
    // collision-screened path.
    const double recovery_speed_threshold =
        2.0 * std::max(lc.stop_speed_threshold, lc.terminal_capture_speed);
    const bool terminal_recovery = emergency
        && v <= recovery_speed_threshold + 1.0e-9
        && e > lc.stop_position_tolerance;
    const bool emergency_braking = emergency && !terminal_recovery;
    const double deceleration_limit = emergency_braking ? std::abs(c.a_min)
        : std::min(std::abs(c.a_min), lc.terminal_braking_deceleration);
    const double jerk_limit = emergency_braking ? c.jerk_max
        : std::min(c.jerk_max, lc.terminal_jerk_limit);
    if (emergency_braking) {
      const double acceleration_command = v > lc.stop_speed_threshold ? -deceleration_limit : 0.0;
      const double response_time = std::max(lc.dt, 1.0e-6);
      const double raw = (acceleration_command - a) / response_time;
      return {clamp_value(raw, -jerk_limit, jerk_limit), raw,
              acceleration_command, 0.0, true};
    }
    const double curve_deceleration = std::min(deceleration_limit,
                                               lc.terminal_braking_curve_deceleration);
    double ramp_time = std::max((a + curve_deceleration) /
                                std::max(jerk_limit, 1.0e-6), 0.0);
    ramp_time = std::min(ramp_time, 2.0);
    const double ramp_distance = std::max(
        jerk_integrated_distance(v, a, -jerk_limit, ramp_time), 0.0);
    const double effective_error = std::max(e - lc.terminal_braking_distance_buffer - ramp_distance, 0.0);
    const double braking_speed = std::sqrt(2.0 * curve_deceleration * effective_error);
    // The requested speed is a required planner input.  The braking envelope
    // may be above the current speed early in a short phase, so acceleration
    // is allowed until the vehicle actually reaches the braking boundary.
    const double speed_reference = std::min(
        std::max(requested_target_speed, 0.0), braking_speed);
    const double response_time = std::max(lc.terminal_acceleration_response_time, 1.0e-6);
    const double speed_feedback = (speed_reference - v) / response_time;
    const bool braking_active =
        e <= lc.stop_position_tolerance
        || v >= braking_speed - 0.02
        || a < -0.02;
    double acceleration_command = 0.0;
    if (braking_active) {
      if (e > lc.stop_position_tolerance) {
        const double stopping_feedforward = -1.00 * v * v /
            std::max(2.0 * effective_error, 0.10);
        // Combine feedforward with envelope tracking feedback.  Taking the
        // minimum of the two terms kept braking saturated even when the
        // vehicle had already fallen below the stopping-speed envelope, which
        // caused an early zero-speed point followed by a creep restart.
        acceleration_command = stopping_feedforward + speed_feedback;
      } else {
        acceleration_command = std::min(
            speed_feedback, -v / std::max(0.30, response_time));
      }
      // Once actual braking has begun, positive acceleration is forbidden.
      // Positive jerk remains available to release a negative acceleration
      // smoothly back toward zero.
      acceleration_command = clamp_value(
          acceleration_command, -deceleration_limit, 0.0);
    } else {
      acceleration_command = clamp_value(
          speed_feedback, -deceleration_limit, lc.cruise_acceleration);
    }
    const double raw = (acceleration_command - a) / response_time;
    return {clamp_value(raw, -jerk_limit, jerk_limit), raw,
            acceleration_command, speed_reference, braking_active};
  }

  TerminalRollout rollout(double position, double speed, double acceleration,
                          double stop_target, double requested_target_speed,
                          bool emergency, double max_time) const;

 private:
  const EnvConfig& cfg_;
};

std::optional<double> positive_velocity_root(double v, double a,
                                             double jerk, double dt) {
  std::vector<double> roots;
  if (std::abs(jerk) < 1.0e-12) {
    if (a < -1.0e-12) roots.push_back(-v / a);
  } else {
    const double discriminant = a * a - 2.0 * jerk * v;
    if (discriminant >= 0.0) {
      const double root = std::sqrt(discriminant);
      roots.push_back((-a + root) / jerk);
      roots.push_back((-a - root) / jerk);
    }
  }
  std::optional<double> best;
  for (double value : roots) {
    if (value > 1.0e-10 && value <= dt + 1.0e-10
        && (!best || value < *best)) best = value;
  }
  return best;
}

TerminalRollout TerminalFeedbackController::rollout(
    double position, double speed, double acceleration, double stop_target,
    double requested_target_speed, bool emergency, double max_time) const {
  const auto& c = cfg_.constraints;
  const auto& lc = cfg_.longitudinal;
  const auto& tc = cfg_.terminal;
  const double dt = lc.dt;
  const double deceleration_limit = emergency ? std::abs(c.a_min)
      : std::min(std::abs(c.a_min), lc.service_deceleration);
  double s = position;
  double v = std::max(speed, 0.0);
  double a = acceleration;
  const int maximum_steps = static_cast<int>(std::ceil(max_time / dt));
  for (int step = 0; step < maximum_steps; ++step) {
    const double error = stop_target - s;
    if (std::abs(error) <= tc.longitudinal_tolerance
        && v <= std::max(tc.speed_tolerance, lc.terminal_capture_speed)) {
      return {true, s, 0.0, 0.0, false, step, step * dt};
    }
    const auto control = command(error, v, a, requested_target_speed, emergency);
    // Preserve jerk continuity if terminal entry occurs with positive
    // acceleration, but never allow acceleration to increase above its current
    // positive value or cross above zero after becoming non-positive.
    const double acceleration_upper = control.braking_active
        ? std::max(a, 0.0)
        : std::min(c.a_max, lc.cruise_acceleration);
    double a_next = clamp_value(
        a + control.jerk * dt, -deceleration_limit, acceleration_upper);
    const double effective_jerk = (a_next - a) / dt;
    const double v_trial = v + a * dt + 0.5 * effective_jerk * dt * dt;
    const auto stop_time = positive_velocity_root(v, a, effective_jerk, dt);
    double distance = 0.0;
    double v_next = 0.0;
    if (stop_time || v_trial <= 0.0) {
      const double use = stop_time ? *stop_time : dt;
      distance = std::max(
          jerk_integrated_distance(v, a, effective_jerk, use), 0.0);
      v_next = 0.0; a_next = 0.0;
    } else {
      v_next = clamp_value(v_trial, c.v_min, c.v_max);
      distance = std::max(
          jerk_integrated_distance(v, a, effective_jerk, dt), 0.0);
    }
    const double s_next = s + distance;
    if (s_next > stop_target + tc.longitudinal_tolerance) {
      return {false, s_next, v_next, a_next, true, step + 1, (step + 1) * dt};
    }
    s = s_next; v = v_next; a = a_next;
  }
  const bool reached = std::abs(stop_target - s) <= tc.longitudinal_tolerance
      && v <= tc.speed_tolerance;
  return {reached, s, v, a, false, maximum_steps, maximum_steps * dt};
}

}  // namespace

// Public utilities and basic classes follow.

void reset_planning_call_counts() {
  g_planning_call_counts = PlanningCallCounts{};
  g_planning_block_timings = PlanningBlockTimings{};
}

PlanningCallCounts planning_call_counts() { return g_planning_call_counts; }

PlanningBlockTimings planning_block_timings() { return g_planning_block_timings; }

double wrap_angle(double angle) {
  double wrapped = std::fmod(angle + kPi, 2.0 * kPi);
  if (wrapped < 0.0) wrapped += 2.0 * kPi;
  return wrapped - kPi;
}

double drive_mode_heading_offset(DriveMode mode) {
  switch (mode) {
    case DriveMode::Forward: return 0.0;
    case DriveMode::Reverse: return kPi;
    case DriveMode::Left: return 0.5 * kPi;
    case DriveMode::Right: return -0.5 * kPi;
  }
  return 0.0;
}

double motion_heading_from_body_yaw(double body_yaw, DriveMode mode) {
  return wrap_angle(body_yaw + drive_mode_heading_offset(mode));
}

std::vector<double> unwrap_angles(const std::vector<double>& angles) {
  if (angles.empty()) return {};
  std::vector<double> out = angles;
  double correction = 0.0;
  for (std::size_t i = 1; i < out.size(); ++i) {
    const double delta = angles[i] - angles[i - 1];
    if (delta > kPi) correction -= 2.0 * kPi;
    else if (delta < -kPi) correction += 2.0 * kPi;
    out[i] = angles[i] + correction;
  }
  return out;
}

double effective_hard_clearance_margin(double speed, const CostConfig& config) {
  (void)speed;
  // Static-obstacle geometry uses one deterministic hard margin.  Speed is
  // handled by the soft-clearance preference and stopping-distance logic, not
  // by changing whether the same footprint geometrically collides.
  return std::max(config.hard_clearance_margin, 0.0);
}

double obstacle_soft_clearance(double speed, const CostConfig& config) {
  const double preferred = effective_hard_clearance_margin(speed, config)
      + config.obstacle_soft_margin
      + config.obstacle_time_headway * std::max(speed, 0.0);
  return std::min(preferred, config.obstacle_soft_margin_max);
}

double circumscribed_radius(double length, double width, double margin) {
  return std::hypot(0.5 * length, 0.5 * width) + margin;
}

ReferencePath::ReferencePath(std::vector<double> s, std::vector<double> x,
                             std::vector<double> y, std::vector<double> psi,
                             std::vector<double> kappa)
    : s_(std::move(s)), x_(std::move(x)), y_(std::move(y)),
      psi_(unwrap_angles(psi)), kappa_(std::move(kappa)) {
  if (s_.size() < 3 || s_.size() != x_.size() || s_.size() != y_.size()
      || s_.size() != psi_.size() || s_.size() != kappa_.size()) {
    throw std::invalid_argument("ReferencePath arrays must have equal length >= 3");
  }
  for (std::size_t i = 1; i < s_.size(); ++i) {
    if (!(s_[i] > s_[i - 1])) throw std::invalid_argument("ReferencePath s must increase");
  }
  kappa_s_ = gradient(kappa_, s_, true);
}

double ReferencePath::s_min() const { return s_.front(); }
double ReferencePath::s_max() const { return s_.back(); }
std::size_t ReferencePath::size() const { return s_.size(); }

void ReferencePath::evaluate(double query, double& x, double& y,
                             double& psi, double& kappa, double& kappa_s) const {
  query = clamp_value(query, s_min(), s_max());
  if (s_.size() == 1 || query <= s_.front()) {
    x = x_.front(); y = y_.front(); psi = wrap_angle(psi_.front());
    kappa = kappa_.front(); kappa_s = kappa_s_.front();
    return;
  }
  if (query >= s_.back()) {
    x = x_.back(); y = y_.back(); psi = wrap_angle(psi_.back());
    kappa = kappa_.back(); kappa_s = kappa_s_.back();
    return;
  }
  const std::size_t i = lower_interval(s_, query);
  const double dx = std::max(s_[i + 1] - s_[i], 1.0e-15);
  const double alpha = (query - s_[i]) / dx;
  x = (1.0 - alpha) * x_[i] + alpha * x_[i + 1];
  y = (1.0 - alpha) * y_[i] + alpha * y_[i + 1];
  psi = wrap_angle((1.0 - alpha) * psi_[i] + alpha * psi_[i + 1]);
  kappa = (1.0 - alpha) * kappa_[i] + alpha * kappa_[i + 1];
  kappa_s = (1.0 - alpha) * kappa_s_[i] + alpha * kappa_s_[i + 1];
}

FrenetProjection ReferencePath::project(double px, double py, double heading) const {
  std::size_t nearest = 0;
  double nearest_distance = kInf;
  for (std::size_t i = 0; i < x_.size(); ++i) {
    const double d2 = (x_[i] - px) * (x_[i] - px) + (y_[i] - py) * (y_[i] - py);
    if (d2 < nearest_distance) { nearest_distance = d2; nearest = i; }
  }
  double best_d2 = kInf;
  std::size_t best_segment = 0;
  double best_t = 0.0;
  double best_x = x_.front();
  double best_y = y_.front();
  const std::size_t begin = nearest > 0 ? nearest - 1 : 0;
  const std::size_t end = std::min(nearest, x_.size() - 2);
  for (std::size_t i = begin; i <= end; ++i) {
    const double vx = x_[i + 1] - x_[i];
    const double vy = y_[i + 1] - y_[i];
    const double denominator = vx * vx + vy * vy;
    const double t = denominator <= 1.0e-14 ? 0.0
        : clamp_value(((px - x_[i]) * vx + (py - y_[i]) * vy) / denominator, 0.0, 1.0);
    const double qx = x_[i] + t * vx;
    const double qy = y_[i] + t * vy;
    const double d2 = (px - qx) * (px - qx) + (py - qy) * (py - qy);
    if (d2 < best_d2) { best_d2 = d2; best_segment = i; best_t = t; best_x = qx; best_y = qy; }
  }
  const std::size_t i = best_segment;
  const double s = s_[i] + best_t * (s_[i + 1] - s_[i]);
  const double psi = psi_[i] + best_t * (psi_[i + 1] - psi_[i]);
  const double kappa = kappa_[i] + best_t * (kappa_[i + 1] - kappa_[i]);
  const double ks = kappa_s_[i] + best_t * (kappa_s_[i + 1] - kappa_s_[i]);
  const double ex = px - best_x;
  const double ey = py - best_y;
  return {s, -std::sin(psi) * ex + std::cos(psi) * ey,
          wrap_angle(heading - psi), kappa, ks, wrap_angle(psi), i};
}

FrenetProjection ReferencePath::project_local(double px, double py, double heading,
                                               double s_hint, double search_back,
                                               double search_forward) const {
  const double lo_s = std::max(s_min(), s_hint - search_back);
  const double hi_s = std::min(s_max(), s_hint + search_forward);
  std::size_t lo = static_cast<std::size_t>(std::lower_bound(s_.begin(), s_.end(), lo_s) - s_.begin());
  std::size_t hi = static_cast<std::size_t>(std::upper_bound(s_.begin(), s_.end(), hi_s) - s_.begin());
  hi = std::max(hi, lo + 1);
  hi = std::min(hi, s_.size());
  std::size_t nearest = lo;
  double best = kInf;
  for (std::size_t i = lo; i < hi; ++i) {
    const double d2 = (x_[i] - px) * (x_[i] - px) + (y_[i] - py) * (y_[i] - py);
    if (d2 < best) { best = d2; nearest = i; }
  }
  const double ex = px - x_[nearest];
  const double ey = py - y_[nearest];
  return {s_[nearest], -std::sin(psi_[nearest]) * ex + std::cos(psi_[nearest]) * ey,
          wrap_angle(heading - psi_[nearest]), kappa_[nearest], kappa_s_[nearest],
          psi_[nearest], nearest};
}

Costmap2D::Costmap2D(std::vector<std::int8_t> data, int width, int height,
                     double resolution, double origin_x, double origin_y,
                     int occupied_threshold, int unknown_value,
                     bool unknown_is_occupied,
                     bool conservative_cell_correction)
    : width_(width), height_(height), resolution_(resolution),
      origin_x_(origin_x), origin_y_(origin_y), data_(std::move(data)) {
  if (width_ < 2 || height_ < 2 || data_.size() != static_cast<std::size_t>(width_ * height_)
      || !(resolution_ > 0.0)) {
    throw std::invalid_argument("invalid costmap");
  }
  std::vector<std::uint8_t> occupied(data_.size(), 0);
  for (std::size_t i = 0; i < data_.size(); ++i) {
    occupied[i] = static_cast<std::uint8_t>(data_[i] >= occupied_threshold
        || (unknown_is_occupied && data_[i] == unknown_value));
  }
  distance_field_ = euclidean_distance_transform(occupied, width_, height_);
  const double correction = conservative_cell_correction
      ? 0.5 * std::sqrt(2.0) * resolution_ : 0.0;
  for (double& value : distance_field_) value = std::max(value * resolution_ - correction, 0.0);
}

double Costmap2D::distance_at_world(double x, double y) const {
  if (distance_field_.empty()) return 1.0e6;
  const double gx = (x - origin_x_) / resolution_ - 0.5;
  const double gy = (y - origin_y_) / resolution_ - 0.5;
  const int x0 = static_cast<int>(std::floor(gx));
  const int y0 = static_cast<int>(std::floor(gy));
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;
  if (x0 < 0 || y0 < 0 || x1 >= width_ || y1 >= height_) return 0.0;
  const double wx = gx - x0;
  const double wy = gy - y0;
  auto at = [&](int ix, int iy) { return distance_field_[static_cast<std::size_t>(iy * width_ + ix)]; };
  return (1.0 - wx) * (1.0 - wy) * at(x0, y0)
       + wx * (1.0 - wy) * at(x1, y0)
       + (1.0 - wx) * wy * at(x0, y1)
       + wx * wy * at(x1, y1);
}

double Costmap2D::clearance_single_circle(double x, double y, double radius) const {
  return distance_at_world(x, y) - radius;
}

namespace {

struct SpatialScreen {
  double screening_cost{0.0};
  double obstacle_cost{0.0};
  bool preview_collision_free{false};
  bool ranking_collision_free{false};
  bool coarse_collision_free{false};
  double ranking_min_clearance{kInf};
  double coarse_min_clearance{kInf};
  int checked_poses{0};
  bool curvature_valid{false};
  double maximum_abs_curvature{0.0};
  double soft_clearance_margin{0.0};
  double hard_clearance_margin{0.0};
  double continuity_cost{0.0};
  bool bottleneck_limited{false};
  double decision_horizon_length{0.0};
  double first_coarse_collision_length{kInf};
  bool terminal_goal_region_safe{true};
  double terminal_goal_offset{0.0};
  double terminal_goal_clearance{kInf};
};

std::size_t bottleneck_decision_count(const SpatialPathCandidate& path,
                                      const PlannerState& state,
                                      const EnvConfig& cfg,
                                      const Costmap2D* costmap,
                                      double target_speed,
                                      bool& limited,
                                      double& decision_length) {
  limited = false;
  decision_length = path.arc_length.empty() ? 0.0 : path.arc_length.back();
  if (!cfg.adaptive_replan.enabled || costmap == nullptr
      || path.reference_x.size() != path.x.size()
      || path.reference_y.size() != path.y.size()
      || path.q_ref.size() != path.x.size() || path.x.size() < 2) {
    return path.x.size();
  }

  const double screening_speed = std::max(std::abs(state.speed), target_speed);
  const double hard_margin = effective_hard_clearance_margin(screening_speed, cfg.cost);
  const double base_radius = 0.5 * cfg.vehicle.width + cfg.vehicle.footprint_margin;
  const double trigger = base_radius + hard_margin
      + cfg.adaptive_replan.bottleneck_trigger_extra;
  const double release = base_radius + hard_margin
      + cfg.adaptive_replan.bottleneck_release_extra;

  std::optional<std::size_t> start;
  std::size_t end = path.x.size() - 1;
  int release_run = 0;
  for (std::size_t i = 0; i < path.x.size(); ++i) {
    if (path.virtual_mask.size() == path.x.size() && path.virtual_mask[i] != 0) break;
    const double distance = costmap->distance_at_world(
        path.reference_x[i], path.reference_y[i]);
    if (!start) {
      if (distance <= trigger) start = i;
      continue;
    }
    if (distance > release) {
      ++release_run;
      if (release_run >= std::max(1, cfg.adaptive_replan.bottleneck_release_samples)) {
        end = i;
        break;
      }
    } else {
      release_run = 0;
    }
  }
  if (!start) return path.x.size();

  const double bottleneck_end_q = path.q_ref[std::min(end, path.q_ref.size() - 1)];
  const double desired_q = std::max(
      cfg.adaptive_replan.bottleneck_minimum_decision_length,
      bottleneck_end_q + cfg.adaptive_replan.bottleneck_post_buffer);
  auto it = std::upper_bound(path.q_ref.begin(), path.q_ref.end(), desired_q + 1.0e-9);
  std::size_t count = static_cast<std::size_t>(std::distance(path.q_ref.begin(), it));
  count = std::min(std::max<std::size_t>(count, 2), path.x.size());
  decision_length = path.arc_length[std::min(count - 1, path.arc_length.size() - 1)];
  limited = count < path.x.size();
  return count;
}

std::vector<double> path_clearance(const std::vector<double>& x,
                                   const std::vector<double>& y,
                                   const Costmap2D* costmap,
                                   double radius) {
  std::vector<double> clearance(x.size(), 1.0e6);
  if (costmap == nullptr) return clearance;
  for (std::size_t i = 0; i < x.size(); ++i) {
    clearance[i] = costmap->clearance_single_circle(x[i], y[i], radius);
  }
  return clearance;
}

struct TerminalGoalRegionEvaluation {
  bool safe{true};
  double minimum_clearance{kInf};
  double absolute_lateral_offset{0.0};
};

TerminalGoalRegionEvaluation evaluate_terminal_goal_region(
    const SpatialPathCandidate& path,
    const EnvConfig& cfg,
    const Costmap2D* costmap,
    double screening_speed) {
  TerminalGoalRegionEvaluation result;
  const double terminal_goal_q = std::max(
      path.real_end_q - cfg.longitudinal.stop_target_offset, 0.0);
  result.absolute_lateral_offset = std::abs(interp_scalar(
      path.q_ref, path.lateral_offset,
      std::min(terminal_goal_q, path.q_ref.back())));
  if (costmap == nullptr || path.arc_length.empty()) {
    result.minimum_clearance = 1.0e6;
    return result;
  }

  const double goal_l = interp_scalar(
      path.q_ref, path.arc_length,
      std::min(terminal_goal_q, path.q_ref.back()));
  const double half_length = 0.5 * cfg.vehicle.length;
  const double radius = 0.5 * cfg.vehicle.width + cfg.vehicle.footprint_margin;
  const double sample_step = std::max(
      0.05, std::min(cfg.lateral.spatial_ds, 0.20));
  const int sample_count = std::max(
      3, static_cast<int>(std::ceil(2.0 * half_length / sample_step)) + 1);
  result.minimum_clearance = kInf;
  for (int i = 0; i < sample_count; ++i) {
    const double alpha = sample_count <= 1
        ? 0.5 : static_cast<double>(i) / static_cast<double>(sample_count - 1);
    const double query_l = clamp_value(
        goal_l - half_length + 2.0 * half_length * alpha,
        0.0, path.arc_length.back());
    const auto sample = interpolate_path(path, query_l);
    result.minimum_clearance = std::min(
        result.minimum_clearance,
        costmap->clearance_single_circle(sample.x, sample.y, radius));
  }
  const double hard_margin = effective_hard_clearance_margin(
      screening_speed, cfg.cost);
  result.safe = result.minimum_clearance > hard_margin;
  return result;
}

SpatialScreen screen_spatial_candidate(const SpatialPathCandidate& path,
                                       const PlannerState& state,
                                       const EnvConfig& cfg,
                                       const Costmap2D* costmap,
                                       double target_speed,
                                       bool terminal_constraint_active) {
  bool bottleneck_limited = false;
  double decision_horizon_length = path.arc_length.empty() ? 0.0 : path.arc_length.back();
  const std::size_t decision_count = bottleneck_decision_count(
      path, state, cfg, costmap, target_speed,
      bottleneck_limited, decision_horizon_length);
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> kappa;
  std::vector<double> arc;
  for (std::size_t i = 0; i < std::min(decision_count, path.q_ref.size()); ++i) {
    if (path.q_ref[i] <= path.real_end_q + 1.0e-9) {
      x.push_back(path.x[i]);
      y.push_back(path.y[i]);
      kappa.push_back(path.kappa[i]);
      arc.push_back(path.arc_length[i]);
    }
  }
  if (x.size() < 2) {
    const std::size_t count = std::min<std::size_t>(2, path.x.size());
    x.assign(path.x.begin(), path.x.begin() + static_cast<std::ptrdiff_t>(count));
    y.assign(path.y.begin(), path.y.begin() + static_cast<std::ptrdiff_t>(count));
    kappa.assign(path.kappa.begin(), path.kappa.begin() + static_cast<std::ptrdiff_t>(count));
    arc.assign(path.arc_length.begin(), path.arc_length.begin() + static_cast<std::ptrdiff_t>(count));
  }
  const double screening_speed = std::max(state.speed, target_speed);
  const double hard_margin = effective_hard_clearance_margin(screening_speed, cfg.cost);
  // Spatial generation does not yet know the allocated body yaw.  Retain the
  // circumscribed radius only for conservative obstacle ranking, but use the
  // vehicle half-width for the hard coarse screen.  Final safety is guaranteed
  // by the post-allocation oriented multi-circle swept check.
  const double ranking_radius = circumscribed_radius(
      cfg.vehicle.length, cfg.vehicle.width, cfg.vehicle.footprint_margin);
  const double coarse_radius = 0.5 * cfg.vehicle.width
                             + cfg.vehicle.footprint_margin;
  const auto ranking_clearance = path_clearance(x, y, costmap, ranking_radius);
  const auto coarse_clearance = path_clearance(x, y, costmap, coarse_radius);
  const double ranking_min = *std::min_element(ranking_clearance.begin(), ranking_clearance.end());
  const double coarse_min = *std::min_element(coarse_clearance.begin(), coarse_clearance.end());
  const bool ranking_free = std::all_of(ranking_clearance.begin(), ranking_clearance.end(),
                                        [hard_margin](double value) { return value > hard_margin; });
  const bool coarse_free = std::all_of(coarse_clearance.begin(), coarse_clearance.end(),
                                       [hard_margin](double value) { return value > hard_margin; });
  double first_coarse_collision_length = kInf;
  for (std::size_t i = 0; i < coarse_clearance.size(); ++i) {
    if (coarse_clearance[i] <= hard_margin) {
      first_coarse_collision_length = i < arc.size() ? arc[i] : 0.0;
      break;
    }
  }
  double max_curvature = 0.0;
  for (double value : kappa) max_curvature = std::max(max_curvature, std::abs(value));
  const bool curvature_valid = max_curvature <= cfg.constraints.curvature_max + 1.0e-9;
  const double soft_margin = obstacle_soft_clearance(screening_speed, cfg.cost);
  double sum_deficit_squared = 0.0;
  double max_deficit = 0.0;
  int collision_count = 0;
  for (double value : ranking_clearance) {
    const double deficit = std::max(0.0, soft_margin - value);
    sum_deficit_squared += deficit * deficit;
    max_deficit = std::max(max_deficit, deficit);
    collision_count += value <= hard_margin ? 1 : 0;
  }
  const double mean_deficit_squared = ranking_clearance.empty()
      ? 0.0 : sum_deficit_squared / static_cast<double>(ranking_clearance.size());
  const auto terminal_goal = evaluate_terminal_goal_region(
      path, cfg, costmap, screening_speed);
  const double terminal_soft_deficit = terminal_constraint_active
      ? std::max(0.0, soft_margin - terminal_goal.minimum_clearance)
      : 0.0;
  const double obstacle_cost = cfg.cost.w_obstacle * mean_deficit_squared
      + cfg.cost.w_min_clearance * max_deficit * max_deficit
      + cfg.cost.w_collision * static_cast<double>(collision_count)
      + cfg.cost.w_min_clearance * terminal_soft_deficit * terminal_soft_deficit;
  SpatialScreen result;
  result.screening_cost = path.path_cost + obstacle_cost;
  result.obstacle_cost = obstacle_cost;
  result.preview_collision_free = coarse_free && curvature_valid
      && (!terminal_constraint_active || terminal_goal.safe);
  result.ranking_collision_free = ranking_free;
  result.coarse_collision_free = coarse_free;
  result.ranking_min_clearance = ranking_min;
  result.coarse_min_clearance = coarse_min;
  result.checked_poses = static_cast<int>(x.size());
  result.curvature_valid = curvature_valid;
  result.maximum_abs_curvature = max_curvature;
  result.soft_clearance_margin = soft_margin;
  result.hard_clearance_margin = hard_margin;
  result.bottleneck_limited = bottleneck_limited;
  result.decision_horizon_length = decision_horizon_length;
  result.first_coarse_collision_length = first_coarse_collision_length;
  result.terminal_goal_region_safe = terminal_goal.safe;
  result.terminal_goal_offset = terminal_goal.absolute_lateral_offset;
  result.terminal_goal_clearance = terminal_goal.minimum_clearance;
  return result;
}

std::vector<std::tuple<std::string, double, double, bool>> lateral_length_options(
    const PlannerState& state, const FrenetProjection& fr, double n_target,
    const EnvConfig& cfg) {
  const double v_scale = std::max(std::abs(state.speed), cfg.constraints.v_eff_min);
  const double delta_n = n_target - fr.n;
  const double nominal = minimum_lateral_length(delta_n, v_scale, cfg);
  std::vector<std::tuple<std::string, double, double, bool>> options;
  options.emplace_back("speed_based", nominal, 0.0, false);
  // Narrow gates on curved references can require a longer geometric
  // transition than the minimum dynamic bound, especially at low speed where
  // the minimum bound becomes short.  Evaluate two deterministic longer
  // alternatives rather than forcing a full stop and replanning from rest.
  if (std::abs(delta_n) >= 0.50) {
    const double medium = clamp_value(
        std::max(1.35 * nominal, nominal + 3.0),
        cfg.lateral.min_length, cfg.lateral.max_length);
    const double longer = clamp_value(
        std::max(1.70 * nominal, nominal + 6.0),
        cfg.lateral.min_length, cfg.lateral.max_length);
    if (medium > nominal + 0.05) {
      options.emplace_back("extended_transition", medium, 0.0, false);
    }
    if (longer > medium + 0.05) {
      options.emplace_back("long_transition", longer, 0.0, false);
    }
  }
  if (std::abs(delta_n) < 0.50 && v_scale > cfg.constraints.v_eff_min) {
    const double omega = std::abs(state.motion_heading_rate);
    const double acceleration = std::abs(state.acceleration);
    const double jerk_available = std::max(
        cfg.constraints.lateral_jerk_max - acceleration * omega,
        0.25 * cfg.constraints.lateral_jerk_max);
    const double unwind_distance = omega * v_scale * v_scale / jerk_available;
    const double settling_length = clamp_value(
        cfg.lateral.min_length + 1.35 * unwind_distance,
        cfg.lateral.min_length, cfg.lateral.max_length);
    if (settling_length > nominal + 0.05) {
      options.emplace_back("heading_rate_settle", settling_length, 0.0, false);
    }
  }
  return options;
}

double reachable_spatial_preview_length(double current_speed, const EnvConfig& cfg) {
  const double horizon = std::max(cfg.longitudinal.horizon, 0.0);
  const double speed = clamp_value(std::abs(current_speed), cfg.constraints.v_min,
                                   cfg.constraints.v_max);
  const double acceleration = std::max(cfg.longitudinal.cruise_acceleration, 0.0);
  const double maximum_speed = cfg.constraints.v_max;
  double distance = 0.0;
  if (horizon <= 0.0) {
    distance = 0.0;
  } else if (acceleration <= 1.0e-9 || speed >= maximum_speed - 1.0e-9) {
    distance = speed * horizon;
  } else {
    const double time_to_max = std::max((maximum_speed - speed) / acceleration, 0.0);
    const double accelerating_time = std::min(horizon, time_to_max);
    distance = speed * accelerating_time + 0.5 * acceleration * accelerating_time * accelerating_time;
    distance += maximum_speed * std::max(horizon - accelerating_time, 0.0);
  }
  const double minimum_preview = cfg.adaptive_replan.enabled
      ? cfg.adaptive_replan.minimum_spatial_preview : 0.0;
  return std::max({0.20, 1.10 * distance, cfg.lateral.min_length,
                   minimum_preview});
}

double curve_speed_limit(const SpatialPathCandidate& path, double progress,
                         double speed, const EnvConfig& cfg,
                         double target_speed) {
  const double lookahead = std::max(cfg.longitudinal.curvature_lookahead_min,
      cfg.longitudinal.curvature_lookahead_time * std::max(speed, 0.0));
  const double s1 = std::min(progress + lookahead, path.real_end_l);
  double limit = std::min(cfg.constraints.v_max, target_speed);
  // Dense sampling is required for short curvature-derivative peaks in
  // S-curves and obstacle-avoidance offset profiles.  The former 15-point
  // scan could miss a narrow peak and postpone deceleration until the next
  // replan.
  for (int i = 0; i < 61; ++i) {
    const double alpha = static_cast<double>(i) / 60.0;
    const double sample_s = progress + alpha * (std::max(progress, s1) - progress);
    const auto sample = interpolate_path(path, sample_s);
    const double v_kappa = std::sqrt(cfg.constraints.a_lat_max /
                                     (std::abs(sample.kappa) + 1.0e-5));
    const double v_kappa_rate = std::cbrt(
        cfg.longitudinal.lateral_jerk_speed_margin * cfg.constraints.lateral_jerk_max /
        (std::abs(sample.kappa_l) + 1.0e-5));
    limit = std::min({limit, v_kappa, v_kappa_rate});
  }
  return limit;
}

TimeTrajectory generate_open_loop_trajectory(
    const SpatialPathCandidate& path,
    const PlannerState& initial_state,
    const PlannerAction& previous_action,
    bool map_end_mode,
    const EnvConfig& cfg,
    const Costmap2D* costmap,
    bool emergency,
    bool terminal_constraint_active,
    bool terminal_mode_active,
    double target_speed,
    bool enforce_obstacle_collision = true) {
  const auto& c = cfg.constraints;
  const auto& lc = cfg.longitudinal;
  const auto& cc = cfg.cost;
  const auto& tc = cfg.terminal;
  const double dt = lc.dt;
  const int N = static_cast<int>(std::llround(lc.horizon / dt));
  const double progress_limit = map_end_mode ? path.real_end_l : path.arc_length.back();
  const double terminal_goal_q = std::max(path.real_end_q - lc.stop_target_offset, 0.0);

  TimeTrajectory trajectory;
  trajectory.states.assign(static_cast<std::size_t>(N + 1), PlannerState{});
  trajectory.actions.assign(static_cast<std::size_t>(N), PlannerAction{});
  trajectory.progress.assign(static_cast<std::size_t>(N + 1), 0.0);
  trajectory.speed_reference.assign(static_cast<std::size_t>(N + 1),
                                    std::numeric_limits<double>::quiet_NaN());
  trajectory.states.front() = initial_state;
  TerminalFeedbackController terminal_controller(cfg);

  for (int k = 0; k < N; ++k) {
    const std::size_t index = static_cast<std::size_t>(k);
    const double l_progress = trajectory.progress[index];
    const double q_progress = interpolate_reference_progress(path, l_progress);
    const double v = std::max(trajectory.states[index].speed, 0.0);
    const double a = trajectory.states[index].acceleration;
    const double remaining_s = terminal_goal_q - q_progress;
    if (terminal_mode_active
        && std::abs(remaining_s) <= tc.longitudinal_tolerance
        && v <= std::max(lc.stop_speed_threshold, lc.terminal_capture_speed)
        && std::abs(a) <= tc.acceleration_tolerance) {
      for (int j = k; j <= N; ++j) {
        trajectory.states[static_cast<std::size_t>(j)] = trajectory.states[index];
        trajectory.states[static_cast<std::size_t>(j)].speed = 0.0;
        trajectory.states[static_cast<std::size_t>(j)].acceleration = 0.0;
        trajectory.states[static_cast<std::size_t>(j)].motion_heading_rate = 0.0;
        trajectory.progress[static_cast<std::size_t>(j)] = l_progress;
      }
      trajectory.predicted_stop_progress = q_progress;
      trajectory.predicted_stop_error = std::abs(terminal_goal_q - q_progress);
      break;
    }
    const double accel_limit = emergency ? c.a_max : std::min(c.a_max, lc.cruise_acceleration);
    const double decel_limit = emergency ? std::abs(c.a_min)
        : std::min(std::abs(c.a_min), lc.service_deceleration);
    const double jerk_limit = emergency ? c.jerk_max
        : std::min(c.jerk_max, lc.comfort_jerk);
    double jerk_raw = 0.0;
    double jerk = 0.0;
    bool braking_active_this_step = false;
    if (terminal_mode_active) {
      const double dq_dl = reference_progress_rate(path, l_progress);
      const auto control = terminal_controller.command(
          remaining_s, v * dq_dl, a * dq_dl, target_speed, emergency);
      braking_active_this_step = control.braking_active;
      const double desired_path_acceleration = control.acceleration_command / dq_dl;
      const double response_time = std::max(lc.terminal_acceleration_response_time, 1.0e-6);
      jerk_raw = (desired_path_acceleration - a) / response_time;
      if (k == 0 && std::isfinite(previous_action.longitudinal_jerk)) {
        const double weight = std::max(lc.terminal_initial_jerk_continuity_weight, 0.0);
        jerk_raw = (jerk_raw + weight * previous_action.longitudinal_jerk) / (1.0 + weight);
      }
      const double terminal_jerk_limit = emergency ? c.jerk_max
          : std::min(c.jerk_max, lc.terminal_jerk_limit);
      jerk = clamp_value(jerk_raw, -terminal_jerk_limit, terminal_jerk_limit);
      trajectory.speed_reference[index] = control.speed_reference / dq_dl;
    } else if (emergency) {
      jerk = a > -decel_limit + 1.0e-9 ? -jerk_limit : 0.0;
      jerk_raw = jerk;
      trajectory.speed_reference[index] = 0.0;
    } else {
      const double desired_speed = curve_speed_limit(path, l_progress, v, cfg, target_speed);
      trajectory.speed_reference[index] = desired_speed;
      const double target_acceleration = clamp_value(
          (desired_speed - v) / std::max(lc.speed_time_constant, 1.0e-6),
          -decel_limit, accel_limit);
      jerk_raw = (target_acceleration - a) /
                 std::max(lc.acceleration_response_time, 1.0e-6);
      jerk = clamp_value(jerk_raw, -jerk_limit, jerk_limit);
    }
    const double acceleration_upper =
        (terminal_mode_active && braking_active_this_step)
        ? std::max(a, 0.0)
        : accel_limit;
    double a_next = clamp_value(
        a + jerk * dt, -decel_limit, acceleration_upper);
    jerk = (a_next - a) / dt;
    const double v_trial = v + a * dt + 0.5 * jerk * dt * dt;
    const auto stop_time = positive_velocity_root(v, a, jerk, dt);
    double distance = 0.0;
    double v_next = 0.0;
    if (stop_time || v_trial <= 0.0) {
      const double use = stop_time ? *stop_time : dt;
      distance = std::max(jerk_integrated_distance(v, a, jerk, use), 0.0);
      v_next = 0.0;
      a_next = 0.0;
    } else {
      v_next = clamp_value(v_trial, c.v_min, c.v_max);
      distance = std::max(jerk_integrated_distance(v, a, jerk, dt), 0.0);
    }
    const double l_next = l_progress + distance;
    const double q_next = interpolate_reference_progress(path, l_next);
    const bool terminal_tolerance_extension = terminal_mode_active
        && q_next <= terminal_goal_q + tc.longitudinal_tolerance + 1.0e-9;
    // Near the terminal point, the current state can already lie a few
    // centimetres beyond the nominal stop target while still remaining well
    // inside the allowed longitudinal tolerance.  In that case the exact
    // jerk-limited stopping distance may extend a few millimetres past the
    // authored path endpoint.  The generated path contains a virtual straight
    // extension for precisely this purpose, so do not reject that physically
    // valid capture merely because it crosses real_end_l.  The terminal
    // tolerance and the virtual-path bound remain hard limits.
    if ((map_end_mode && l_next > progress_limit + 1.0e-9
            && !terminal_tolerance_extension)
        || (terminal_mode_active
            && q_next > terminal_goal_q + tc.longitudinal_tolerance + 1.0e-9)
        || l_next > path.arc_length.back() + 1.0e-9) {
      trajectory.endpoint_overshoot_attempt = true;
      for (int j = k + 1; j <= N; ++j) {
        trajectory.states[static_cast<std::size_t>(j)] = trajectory.states[index];
        trajectory.progress[static_cast<std::size_t>(j)] = l_progress;
      }
      break;
    }
    const auto sample = interpolate_path(path, l_next);
    PlannerState next;
    next.x = sample.x;
    next.y = sample.y;
    next.chi = wrap_angle(sample.psi);
    next.speed = v_next;
    next.acceleration = a_next;
    next.motion_heading_rate = v_next * sample.kappa;
    trajectory.states[index + 1] = next;
    trajectory.actions[index] = {jerk, a_next * sample.kappa
        + v_next * v_next * sample.kappa_l};
    trajectory.progress[index + 1] = l_next;
    trajectory.speed_reference[index + 1] = trajectory.speed_reference[index];
    if (terminal_mode_active
        && v_next <= std::max(lc.stop_speed_threshold, lc.terminal_capture_speed)
        && std::abs(terminal_goal_q - q_next) <= tc.longitudinal_tolerance
        && std::abs(a_next) <= tc.acceleration_tolerance) {
      trajectory.states[index + 1].speed = 0.0;
      trajectory.states[index + 1].acceleration = 0.0;
      trajectory.states[index + 1].motion_heading_rate = 0.0;
      for (int j = k + 2; j <= N; ++j) {
        trajectory.states[static_cast<std::size_t>(j)] = trajectory.states[index + 1];
        trajectory.progress[static_cast<std::size_t>(j)] = l_next;
      }
      trajectory.predicted_stop_progress = q_next;
      trajectory.predicted_stop_error = std::abs(terminal_goal_q - q_next);
      break;
    }
  }

  const std::size_t state_count = trajectory.states.size();
  trajectory.lateral_acceleration.resize(state_count, 0.0);
  trajectory.lateral_jerk.resize(state_count, 0.0);
  std::vector<double> reference_progress(state_count, 0.0);
  std::vector<double> trajectory_kappa(state_count, 0.0);
  std::vector<double> trajectory_kappa_l(state_count, 0.0);
  for (std::size_t i = 0; i < state_count; ++i) {
    reference_progress[i] = interpolate_reference_progress(path, trajectory.progress[i]);
    const auto sample = interpolate_path(path, trajectory.progress[i]);
    trajectory_kappa[i] = sample.kappa;
    trajectory_kappa_l[i] = sample.kappa_l;
    const double v = trajectory.states[i].speed;
    const double a = trajectory.states[i].acceleration;
    trajectory.lateral_acceleration[i] = v * v * sample.kappa;
    trajectory.lateral_jerk[i] = 2.0 * v * a * sample.kappa
        + v * v * v * sample.kappa_l;
  }

  bool speed_min_ok = true, speed_max_ok = true, acceleration_ok = true;
  bool heading_rate_ok = true, heading_accel_ok = true;
  bool lateral_accel_ok = true, lateral_jerk_ok = true, curvature_ok = true;
  const double execution_dt = std::min(std::max(lc.execution_dt, 1.0e-4), dt);
  const int substep_count = std::max(1, static_cast<int>(std::ceil(dt / execution_dt)));
  for (int k = 0; k < N; ++k) {
    const auto& state = trajectory.states[static_cast<std::size_t>(k)];
    const auto& action = trajectory.actions[static_cast<std::size_t>(k)];
    const auto stop_time = positive_velocity_root(state.speed, state.acceleration,
                                                   action.longitudinal_jerk, dt);
    for (int sub = 1; sub <= substep_count; ++sub) {
      const double tau = execution_dt + (dt - execution_dt) *
          static_cast<double>(sub - 1) / std::max(substep_count - 1, 1);
      double speed = std::max(state.speed + state.acceleration * tau
          + 0.5 * action.longitudinal_jerk * tau * tau, 0.0);
      double acceleration = state.acceleration + action.longitudinal_jerk * tau;
      double distance = std::max(jerk_integrated_distance(
          state.speed, state.acceleration, action.longitudinal_jerk, tau), 0.0);
      double progress = std::min(trajectory.progress[static_cast<std::size_t>(k)] + distance,
                                 trajectory.progress[static_cast<std::size_t>(k + 1)]);
      if (stop_time && tau >= *stop_time - 1.0e-12) {
        speed = 0.0; acceleration = 0.0;
        progress = trajectory.progress[static_cast<std::size_t>(k + 1)];
      }
      const auto sample = interpolate_path(path, progress);
      const double heading_rate = speed * sample.kappa;
      const double heading_acceleration = acceleration * sample.kappa
          + speed * speed * sample.kappa_l;
      const double lateral_acceleration = speed * heading_rate;
      const double lateral_jerk = acceleration * heading_rate
          + speed * heading_acceleration;
      constexpr double tolerance = 3.0e-2;
      speed_min_ok &= speed >= c.v_min - tolerance;
      speed_max_ok &= speed <= c.v_max + tolerance;
      acceleration_ok &= acceleration >= c.a_min - tolerance && acceleration <= c.a_max + tolerance;
      heading_rate_ok &= std::abs(heading_rate) <= c.heading_rate_max + 0.5 * kPi / 180.0;
      heading_accel_ok &= std::abs(heading_acceleration) <= c.heading_accel_max + 0.5 * kPi / 180.0;
      lateral_accel_ok &= std::abs(lateral_acceleration) <= c.a_lat_max + tolerance;
      lateral_jerk_ok &= std::abs(lateral_jerk) <= c.lateral_jerk_max + tolerance;
      curvature_ok &= std::abs(sample.kappa) <= c.curvature_max + 1.0e-9;
    }
  }
  bool jerk_ok = true;
  for (const auto& action : trajectory.actions) jerk_ok &= std::abs(action.longitudinal_jerk) <= c.jerk_max + 3.0e-2;
  bool progress_ok = true;
  for (std::size_t i = 0; i < trajectory.progress.size(); ++i) {
    if (trajectory.progress[i] <= progress_limit + 1.0e-9) continue;
    const bool allowed_terminal_extension = terminal_mode_active
        && trajectory.progress[i] <= path.arc_length.back() + 1.0e-9
        && reference_progress[i]
            <= terminal_goal_q + tc.longitudinal_tolerance + 1.0e-9;
    if (!allowed_terminal_extension) {
      progress_ok = false;
      break;
    }
  }
  trajectory.valid_dynamic = speed_min_ok && speed_max_ok && acceleration_ok && jerk_ok
      && heading_rate_ok && heading_accel_ok && lateral_accel_ok && lateral_jerk_ok
      && curvature_ok && !trajectory.endpoint_overshoot_attempt && progress_ok;

  const double maximum_speed = std::max_element(trajectory.states.begin(), trajectory.states.end(),
      [](const PlannerState& lhs, const PlannerState& rhs) { return lhs.speed < rhs.speed; })->speed;
  const double hard_margin = effective_hard_clearance_margin(maximum_speed, cc);
  if (enforce_obstacle_collision) {
    std::vector<double> state_x(state_count), state_y(state_count);
    for (std::size_t i = 0; i < state_count; ++i) {
      state_x[i] = trajectory.states[i].x;
      state_y[i] = trajectory.states[i].y;
    }
    const double ranking_radius = circumscribed_radius(
        cfg.vehicle.length, cfg.vehicle.width, cfg.vehicle.footprint_margin);
    const double coarse_radius = 0.5 * cfg.vehicle.width
                               + cfg.vehicle.footprint_margin;
    trajectory.clearance = path_clearance(state_x, state_y, costmap, ranking_radius);
    const auto coarse = path_clearance(state_x, state_y, costmap, coarse_radius);
    trajectory.coarse_min_clearance = *std::min_element(coarse.begin(), coarse.end());
    trajectory.precise_min_clearance = *std::min_element(
        trajectory.clearance.begin(), trajectory.clearance.end());
    trajectory.collision_free = std::all_of(coarse.begin(), coarse.end(),
        [hard_margin](double value) { return value > hard_margin; });
  } else {
    // Static-obstacle geometry was already screened on the complete spatial
    // candidate.  The time stage checks only dynamic/terminal feasibility;
    // final hard collision safety is evaluated after body-yaw allocation.
    trajectory.clearance.assign(state_count, 1.0e6);
    trajectory.coarse_min_clearance = 1.0e6;
    trajectory.precise_min_clearance = 1.0e6;
    trajectory.collision_free = true;
  }

  double speed_cost_sum = 0.0;
  int speed_cost_count = 0;
  double low_speed_cost_sum = 0.0;
  double acceleration_cost = 0.0;
  double lateral_jerk_cost = 0.0;
  double clearance_cost = 0.0;
  double maximum_clearance_deficit = 0.0;
  int collision_count = 0;
  for (std::size_t i = 0; i < state_count; ++i) {
    if (std::isfinite(trajectory.speed_reference[i])) {
      const double error = trajectory.speed_reference[i] - trajectory.states[i].speed;
      speed_cost_sum += error * error;
      ++speed_cost_count;
    }
    if (!terminal_mode_active) {
      const double deficit = std::max(0.0, 1.0 - trajectory.states[i].speed);
      low_speed_cost_sum += deficit * deficit;
    }
    acceleration_cost += trajectory.states[i].acceleration * trajectory.states[i].acceleration;
    lateral_jerk_cost += trajectory.lateral_jerk[i] * trajectory.lateral_jerk[i];
    const double soft = obstacle_soft_clearance(trajectory.states[i].speed, cc);
    const double deficit = std::max(0.0, soft - trajectory.clearance[i]);
    clearance_cost += deficit * deficit;
    maximum_clearance_deficit = std::max(maximum_clearance_deficit, deficit);
    collision_count += trajectory.clearance[i] <= hard_margin ? 1 : 0;
  }
  double jerk_cost = 0.0;
  double heading_accel_cost = 0.0;
  for (const auto& action : trajectory.actions) {
    jerk_cost += action.longitudinal_jerk * action.longitudinal_jerk;
    heading_accel_cost += action.motion_heading_acceleration * action.motion_heading_acceleration;
  }
  const double speed_cost = speed_cost_count > 0 ? speed_cost_sum / speed_cost_count : 0.0;
  const double low_speed_cost = terminal_mode_active ? 0.0 : low_speed_cost_sum / state_count;
  const double progress_error = terminal_mode_active
      ? terminal_goal_q - reference_progress.back()
      : std::min(progress_limit, target_speed * lc.horizon) - trajectory.progress.back();
  trajectory.trajectory_cost = cc.w_speed * speed_cost
      + cc.w_low_speed * low_speed_cost
      + cc.w_progress * progress_error * progress_error
      + cc.w_accel * acceleration_cost / state_count
      + cc.w_jerk * jerk_cost / std::max<std::size_t>(trajectory.actions.size(), 1)
      + cc.w_heading_accel * heading_accel_cost / std::max<std::size_t>(trajectory.actions.size(), 1)
      + cc.w_lateral_jerk * lateral_jerk_cost / state_count
      + cc.w_obstacle * clearance_cost / state_count
      + cc.w_min_clearance * maximum_clearance_deficit * maximum_clearance_deficit
      + cc.w_collision * collision_count;

  trajectory.terminal_constraint_active = terminal_constraint_active;
  const auto terminal_goal_region = enforce_obstacle_collision
      ? evaluate_terminal_goal_region(path, cfg, costmap, maximum_speed)
      : TerminalGoalRegionEvaluation{true, 1.0e6,
          std::abs(interp_scalar(path.q_ref, path.lateral_offset,
              std::min(std::max(path.real_end_q - cfg.longitudinal.stop_target_offset, 0.0),
                       path.q_ref.back())))};
  trajectory.terminal_spatial_valid = !terminal_constraint_active
      || terminal_goal_region.safe;
  trajectory.terminal_braking_active = terminal_mode_active;
  trajectory.terminal_lateral_error =
      terminal_goal_region.absolute_lateral_offset;
  std::optional<std::size_t> stop_index;
  for (std::size_t i = 1; i < state_count; ++i) {
    if (trajectory.states[i].speed <= tc.speed_tolerance + 1.0e-9
        && std::abs(trajectory.states[i].acceleration) <= tc.acceleration_tolerance + 1.0e-9
        && std::abs(terminal_goal_q - reference_progress[i]) <= tc.longitudinal_tolerance + 1.0e-9) {
      stop_index = i;
      break;
    }
  }
  trajectory.terminal_stop_reached = stop_index.has_value();
  if (stop_index) {
    trajectory.terminal_longitudinal_error = std::abs(terminal_goal_q - reference_progress[*stop_index]);
    trajectory.terminal_speed_error = std::abs(trajectory.states[*stop_index].speed);
    trajectory.terminal_stop_feasible = true;
    trajectory.terminal_stop_valid = trajectory.terminal_spatial_valid;
    trajectory.predicted_stop_progress = reference_progress[*stop_index];
    trajectory.predicted_stop_error = trajectory.terminal_longitudinal_error;
  } else if (terminal_mode_active) {
    const double dq_dl = reference_progress_rate(path, trajectory.progress.back());
    // An emergency/braking fallback may intentionally stop before the exact
    // terminal point.  Once the fixed 4 s fallback has reached zero speed,
    // the next planning cycle can resume with the nominal terminal feedback
    // controller.  Keeping the auxiliary rollout in emergency mode would
    // hold zero speed forever and incorrectly label every recoverable early
    // stop as terminal-infeasible.
    const bool auxiliary_emergency = emergency
        && trajectory.states.back().speed
            > std::max(tc.speed_tolerance, lc.terminal_capture_speed);
    const auto auxiliary = terminal_controller.rollout(reference_progress.back(),
        trajectory.states.back().speed * dq_dl,
        trajectory.states.back().acceleration * dq_dl,
        terminal_goal_q, target_speed, auxiliary_emergency,
        tc.feedback_feasibility_horizon);
    trajectory.terminal_feedback_rollout_steps = auxiliary.steps;
    trajectory.terminal_feedback_rollout_time = auxiliary.elapsed_time;
    trajectory.terminal_stop_feasible = auxiliary.reached
        && !trajectory.endpoint_overshoot_attempt && !auxiliary.overshoot;
    trajectory.terminal_stop_valid = trajectory.terminal_stop_feasible
        && trajectory.terminal_spatial_valid;
    trajectory.terminal_longitudinal_error = std::numeric_limits<double>::quiet_NaN();
    trajectory.terminal_speed_error = trajectory.states.back().speed;
    trajectory.predicted_stop_progress = auxiliary.position;
    trajectory.predicted_stop_error = std::abs(terminal_goal_q - auxiliary.position);
  } else {
    trajectory.terminal_stop_feasible = true;
    trajectory.terminal_stop_valid = true;
    trajectory.terminal_longitudinal_error = std::numeric_limits<double>::quiet_NaN();
    trajectory.terminal_speed_error = trajectory.states.back().speed;
  }
  return trajectory;
}

PlannerMotionTrajectory build_allocator_trajectory(
    const std::optional<SpatialPathCandidate>& selected_path,
    const TimeTrajectory& trajectory,
    DriveMode drive_mode,
    double dt,
    const std::string& name,
    const std::string& description) {
  PlannerMotionTrajectory motion;
  motion.name = name;
  motion.description = description;
  const std::size_t n = trajectory.states.size();
  motion.t.resize(n); motion.x.resize(n); motion.y.resize(n); motion.chi.resize(n);
  motion.kappa.resize(n); motion.kappa_s.resize(n); motion.speed.resize(n);
  motion.acceleration.resize(n); motion.longitudinal_jerk.resize(n);
  motion.motion_heading_rate.resize(n); motion.motion_heading_acceleration.resize(n);
  motion.drive_mode.assign(n, drive_mode);
  for (std::size_t i = 0; i < n; ++i) {
    motion.t[i] = static_cast<double>(i) * dt;
    const auto& state = trajectory.states[i];
    if (selected_path) {
      const auto sample = interpolate_path(*selected_path, trajectory.progress[i]);
      motion.x[i] = sample.x; motion.y[i] = sample.y; motion.chi[i] = sample.psi;
      motion.kappa[i] = sample.kappa; motion.kappa_s[i] = sample.kappa_l;
    } else {
      motion.x[i] = state.x; motion.y[i] = state.y; motion.chi[i] = state.chi;
      motion.kappa[i] = state.speed > 0.10 ? state.motion_heading_rate / state.speed : 0.0;
      motion.kappa_s[i] = 0.0;
    }
    motion.speed[i] = std::max(state.speed, 0.0);
    motion.acceleration[i] = state.acceleration;
    motion.motion_heading_rate[i] = state.motion_heading_rate;
    if (!trajectory.actions.empty()) {
      const std::size_t action_index = std::min(i, trajectory.actions.size() - 1);
      motion.longitudinal_jerk[i] = trajectory.actions[action_index].longitudinal_jerk;
      motion.motion_heading_acceleration[i] = trajectory.actions[action_index].motion_heading_acceleration;
    }
  }
  return motion;
}

TimeTrajectory stationary_time_trajectory(const PlannerState& state,
                                          const EnvConfig& cfg,
                                          const Costmap2D* costmap) {
  const int N = static_cast<int>(std::llround(cfg.longitudinal.horizon / cfg.longitudinal.dt));
  TimeTrajectory tr;
  tr.states.assign(static_cast<std::size_t>(N + 1), state);
  for (auto& item : tr.states) {
    item.speed = 0.0; item.acceleration = 0.0; item.motion_heading_rate = 0.0;
  }
  tr.actions.assign(static_cast<std::size_t>(N), PlannerAction{});
  tr.progress.assign(static_cast<std::size_t>(N + 1), 0.0);
  tr.speed_reference.assign(static_cast<std::size_t>(N + 1), 0.0);
  tr.lateral_acceleration.assign(static_cast<std::size_t>(N + 1), 0.0);
  tr.lateral_jerk.assign(static_cast<std::size_t>(N + 1), 0.0);
  std::vector<double> x(tr.states.size()), y(tr.states.size());
  for (std::size_t i = 0; i < tr.states.size(); ++i) { x[i] = tr.states[i].x; y[i] = tr.states[i].y; }
  tr.clearance = path_clearance(x, y, costmap,
      circumscribed_radius(cfg.vehicle.length, cfg.vehicle.width, cfg.vehicle.footprint_margin));
  const double margin = effective_hard_clearance_margin(0.0, cfg.cost);
  tr.valid_dynamic = true;
  tr.collision_free = std::all_of(tr.clearance.begin(), tr.clearance.end(),
                                  [margin](double value) { return value > margin; });
  tr.trajectory_cost = 0.0;
  tr.terminal_spatial_valid = true;
  tr.terminal_stop_reached = true;
  tr.terminal_stop_feasible = true;
  tr.terminal_braking_active = true;
  tr.terminal_stop_valid = true;
  return tr;
}

}  // namespace
PathVelocityPlanner::PathVelocityPlanner(EnvConfig config, ReferencePath path,
                                         std::optional<Costmap2D> costmap)
    : config_(std::move(config)), path_(std::move(path)), costmap_(std::move(costmap)) {
  speed_cap_probe_interval_ = std::max(
      1, static_cast<int>(std::llround(1.8 / std::max(config_.longitudinal.dt, 1.0e-6))));
}

void PathVelocityPlanner::set_lateral_target_hint(std::optional<double> hint) {
  if (!hint || !std::isfinite(*hint) || std::abs(*hint) < 0.5) {
    lateral_target_hint_.reset();
  } else {
    lateral_target_hint_ = *hint;
  }
}

void PathVelocityPlanner::set_feasible_speed_cap_hint(std::optional<double> cap,
                                                       int success_count) {
  if (!cap || !std::isfinite(*cap)) {
    last_feasible_speed_cap_.reset();
    speed_cap_success_count_ = 0;
  } else {
    last_feasible_speed_cap_ = clamp_value(*cap, config_.constraints.v_min,
                                           config_.constraints.v_max);
    speed_cap_success_count_ = std::max(success_count, 0);
  }
}

void PathVelocityPlanner::record_allocation_failure(
    double minimum_clearance, double speed,
    std::optional<double> failed_lateral_target) {
  (void)minimum_clearance;
  (void)speed;
  failed_lateral_target_ = failed_lateral_target;
}

void PathVelocityPlanner::record_allocation_success() {
  allocation_failure_severity_ = 0.0;
  allocation_failure_memory_ = 0.0;
  failed_lateral_target_.reset();
}

void PathVelocityPlanner::set_one_shot_excluded_candidate(
    std::optional<int> candidate_id,
    std::optional<double> lateral_target) {
  std::vector<int> ids;
  std::vector<double> targets;
  if (candidate_id && *candidate_id >= 0) ids.push_back(*candidate_id);
  if (lateral_target && std::isfinite(*lateral_target)) targets.push_back(*lateral_target);
  set_one_shot_excluded_paths(std::move(ids), std::move(targets));
}

void PathVelocityPlanner::set_one_shot_excluded_paths(
    std::vector<int> candidate_ids,
    std::vector<double> lateral_targets) {
  candidate_ids.erase(std::remove_if(candidate_ids.begin(), candidate_ids.end(),
      [](int value) { return value < 0; }), candidate_ids.end());
  lateral_targets.erase(std::remove_if(lateral_targets.begin(), lateral_targets.end(),
      [](double value) { return !std::isfinite(value); }), lateral_targets.end());
  one_shot_excluded_candidate_ids_ = std::move(candidate_ids);
  one_shot_excluded_lateral_targets_ = std::move(lateral_targets);
}

PlannerContinuityState PathVelocityPlanner::export_continuity_state(
    const PlannerState& state) const {
  PlannerContinuityState result;
  result.lateral_target_hint = lateral_target_hint_;
  if (maneuver_profile_ && maneuver_start_s_) {
    const auto projection = path_.project(state.x, state.y, state.chi);
    auto profile = *maneuver_profile_;
    const double total_length = profile.start_delay + profile.lateral_length;
    profile.elapsed_length = clamp_value(
        projection.s - *maneuver_start_s_, 0.0, total_length);
    result.maneuver_profile = profile;
    // Retain the scalar fields for backwards-compatible diagnostics and tests.
    result.maneuver_target = profile.target;
    result.maneuver_remaining_length = std::max(
        total_length - profile.elapsed_length, 0.0);
  }
  result.terminal_safe_region_active_last = terminal_safe_region_active_last_;
  result.allocation_failure_severity = allocation_failure_severity_;
  result.allocation_failure_memory = allocation_failure_memory_;
  result.failed_lateral_target = failed_lateral_target_;
  return result;
}

void PathVelocityPlanner::import_continuity_state(
    const PlannerContinuityState& continuity, const PlannerState& state) {
  lateral_target_hint_ = continuity.lateral_target_hint;
  if (continuity.maneuver_profile) {
    const auto projection = path_.project(state.x, state.y, state.chi);
    maneuver_profile_ = continuity.maneuver_profile;
    const double total_length = maneuver_profile_->start_delay
        + maneuver_profile_->lateral_length;
    maneuver_profile_->elapsed_length = clamp_value(
        maneuver_profile_->elapsed_length, 0.0, total_length);
    maneuver_start_s_ = projection.s - maneuver_profile_->elapsed_length;
  } else {
    maneuver_profile_.reset();
    maneuver_start_s_.reset();
  }
  terminal_safe_region_active_last_ =
      continuity.terminal_safe_region_active_last;
  allocation_failure_severity_ = clamp_value(
      continuity.allocation_failure_severity, 0.0, 1.0);
  allocation_failure_memory_ = clamp_value(
      continuity.allocation_failure_memory, 0.0, 1.0);
  failed_lateral_target_ = continuity.failed_lateral_target;
}

std::vector<double> PathVelocityPlanner::speed_trials(double requested_speed) const {
  const auto& lc = config_.longitudinal;
  const double minimum = std::min(requested_speed,
      std::max(lc.obstacle_speed_minimum, config_.constraints.v_min));
  const int maximum_attempts = std::max(1, lc.obstacle_speed_max_attempts);
  const double factor = clamp_value(lc.obstacle_speed_backoff_factor, 0.2, 0.95);
  const double recovery = std::max(lc.obstacle_speed_recovery_step, 0.05);
  std::vector<double> values;
  if (!last_feasible_speed_cap_) {
    values.push_back(requested_speed);
  } else {
    const double cap = *last_feasible_speed_cap_;
    const bool should_probe = cap < requested_speed - 1.0e-6
        && speed_cap_success_count_ >= speed_cap_probe_interval_;
    const double first = should_probe ? std::min(requested_speed, cap + recovery) : cap;
    values.push_back(first);
    if (cap < first - 1.0e-6) values.push_back(cap);
  }
  while (static_cast<int>(values.size()) < maximum_attempts
         && values.back() > minimum + 1.0e-6) {
    const double next = std::max(minimum, values.back() * factor);
    if (std::abs(next - values.back()) < 1.0e-6) break;
    values.push_back(next);
  }
  if (values.back() > minimum + 1.0e-6
      && static_cast<int>(values.size()) < maximum_attempts) {
    values.push_back(minimum);
  }
  std::vector<double> unique;
  for (double value : values) {
    value = clamp_value(value, config_.constraints.v_min, requested_speed);
    if (unique.empty() || std::abs(value - unique.back()) > 1.0e-6) unique.push_back(value);
  }
  return unique;
}

TimeTrajectory PathVelocityPlanner::emergency_stop(
    const PlannerState& state, const PlannerAction& previous_action,
    double target_speed, bool terminal_stop_required) {
  const auto fr = path_.project(state.x, state.y, state.chi);
  const double remaining = std::max(path_.s_max() - config_.simulation.path_end_margin - fr.s, 0.2);
  const double preview = std::min(config_.constraints.v_max * config_.longitudinal.horizon,
                                  remaining);
  auto path = generate_spatial_path_candidate(state, previous_action, fr, fr.n,
      config_.lateral.min_length, std::max(preview, 0.2), config_, path_, -1);
  if (!path) {
    auto tr = stationary_time_trajectory(state, config_, costmap_ ? &*costmap_ : nullptr);
    tr.valid_dynamic = false;
    tr.collision_free = false;
    tr.trajectory_cost = 1.0e9;
    tr.terminal_constraint_active = true;
    tr.terminal_spatial_valid = false;
    tr.terminal_stop_reached = false;
    tr.terminal_stop_feasible = false;
    tr.terminal_braking_active = true;
    tr.terminal_stop_valid = false;
    tr.terminal_longitudinal_error = kInf;
    tr.terminal_lateral_error = std::abs(fr.n);
    tr.terminal_speed_error = std::abs(state.speed);
    tr.predicted_stop_error = kInf;
    return tr;
  }
  return generate_open_loop_trajectory(
      *path, state, previous_action, terminal_stop_required,
      config_, costmap_ ? &*costmap_ : nullptr, true,
      terminal_stop_required, terminal_stop_required, target_speed);
}

PlanResult PathVelocityPlanner::plan_at_speed(
    const PlannerState& state, const PlannerAction& previous_action,
    double target_speed, DriveMode drive_mode,
    const std::vector<int>& excluded_candidate_ids,
    const std::vector<double>& excluded_lateral_targets) {
  const auto start_time = std::chrono::steady_clock::now();
  const auto fr = path_.project(state.x, state.y, state.chi);
  const double real_end_s = path_.s_max() - config_.simulation.path_end_margin;
  const double terminal_goal_s = std::max(real_end_s - config_.longitudinal.stop_target_offset, 0.0);
  const double remaining_to_goal = std::max(terminal_goal_s - fr.s, 0.0);
  const double preview_length = reachable_spatial_preview_length(state.speed, config_);
  TerminalFeedbackController terminal_controller(config_);
  const double activation_speed = std::max(std::abs(state.speed),
      std::min(target_speed, config_.constraints.v_max));
  const double activation_acceleration = std::max(state.acceleration, 0.0);
  const double required_stop_distance = terminal_controller.required_stopping_distance(
      activation_speed, activation_acceleration, false);
  const double activation = std::max(config_.terminal.minimum_activation_distance,
                                     required_stop_distance);
  const bool terminal_constraint_active = remaining_to_goal <= activation + 1.0e-9;
  // Select a collision-free terminal lateral region before longitudinal braking
  // becomes mandatory.  This prevents high-speed approaches from discovering
  // an endpoint obstacle only after too little lateral transition distance
  // remains, while keeping the velocity planner's stopping activation unchanged.
  const double safe_region_activation = std::max(
      activation + std::max(config_.terminal.safe_region_planning_buffer, 0.0),
      preview_length);
  bool terminal_center_region_safe = true;
  if (costmap_) {
    const double radius = 0.5 * config_.vehicle.width
        + config_.vehicle.footprint_margin;
    const double half_length = 0.5 * config_.vehicle.length;
    const double sample_step = std::max(
        0.05, std::min(config_.lateral.spatial_ds, 0.20));
    const int sample_count = std::max(
        3, static_cast<int>(std::ceil(2.0 * half_length / sample_step)) + 1);
    double minimum_clearance = kInf;
    for (int i = 0; i < sample_count; ++i) {
      const double alpha = sample_count <= 1
          ? 0.5 : static_cast<double>(i) / static_cast<double>(sample_count - 1);
      const double query_s = clamp_value(
          terminal_goal_s - half_length + 2.0 * half_length * alpha,
          path_.s_min(), real_end_s);
      double x = 0.0, y = 0.0, psi = 0.0, kappa = 0.0, kappa_s = 0.0;
      path_.evaluate(query_s, x, y, psi, kappa, kappa_s);
      (void)psi;
      (void)kappa;
      (void)kappa_s;
      minimum_clearance = std::min(
          minimum_clearance,
          costmap_->clearance_single_circle(x, y, radius));
    }
    const double screening_speed = std::max(std::abs(state.speed), target_speed);
    terminal_center_region_safe = minimum_clearance
        > effective_hard_clearance_margin(screening_speed, config_.cost);
  }
  const bool terminal_safe_region_active = !terminal_center_region_safe
      && remaining_to_goal <= safe_region_activation + 1.0e-9;
  if (terminal_safe_region_active != terminal_safe_region_active_last_) {
    // A terminal safe-region maneuver has a different endpoint objective from
    // an ordinary obstacle-avoidance maneuver.  Reset the old spatial latch on
    // entry/exit, then latch the newly selected terminal profile so receding-
    // horizon replans advance along one C3-continuous path instead of repeatedly
    // restarting a boundary polynomial and accumulating lateral overshoot.
    maneuver_profile_.reset();
    maneuver_start_s_.reset();
    lateral_target_hint_.reset();
  }
  terminal_safe_region_active_last_ = terminal_safe_region_active;
  const bool terminal_mode_active = terminal_constraint_active;
  const bool map_end_mode = terminal_constraint_active;

  if (maneuver_profile_ && maneuver_start_s_) {
    const double total_length = maneuver_profile_->start_delay
        + maneuver_profile_->lateral_length;
    const double elapsed = fr.s - *maneuver_start_s_;
    const bool target_reached = std::abs(fr.n - maneuver_profile_->target)
        <= config_.adaptive_replan.maneuver_target_tolerance;
    const bool progress_reached = elapsed >= total_length
        - config_.adaptive_replan.maneuver_release_progress_margin;
    if ((target_reached && progress_reached)
        || elapsed > total_length + config_.lateral.min_length) {
      maneuver_profile_.reset();
      maneuver_start_s_.reset();
    }
  }

  // v7 keeps bottleneck preview and maneuver latching, but removes the
  // allocation-failure-driven weight scheduler.  A failed allocated path is
  // excluded exactly once and the deterministic next-best path is selected.
  const bool adaptive_replan_active = false;
  const double clearance_scale = 1.0;
  const double continuity_scale = 1.0;
  const double reference_scale = 1.0;

  struct Entry {
    SpatialPathCandidate path;
    std::string path_mode;
    SpatialScreen screen;
    bool short_path_stop{false};
  };
  struct FullCandidate {
    SpatialPathCandidate path;
    TimeTrajectory trajectory;
    std::string path_mode;
    SpatialScreen screen;
    bool short_path_stop{false};
    double lateral_selection_cost{0.0};
    double total_cost{0.0};
  };
  std::vector<Entry> entries;
  std::vector<Entry> all_spatial;
  int candidate_id = 0;
  bool short_path_fallback_active = false;
  double short_path_length = 0.0;
  {
  ScopedBlockTimer spatial_block_timer(terminal_safe_region_active
      ? g_planning_block_timings.spatial_terminal_ms
      : g_planning_block_timings.spatial_normal_ms);
  // Phase 1: generate every spatial-path candidate across all lateral
  // targets/length options first. Screening (Phase 2 below) only starts once
  // this full generation pass is complete.
  struct GeneratedCandidate {
    SpatialPathCandidate path;
    std::string path_mode;
    int candidate_id{0};
    double n_target{0.0};
    bool active_target{false};
  };
  std::vector<GeneratedCandidate> generated;
  ++g_planning_call_counts.spatial_path_generation;
  for (double n_target : config_.lateral.n_targets) {
    auto options = lateral_length_options(state, fr, n_target, config_);
    if (terminal_safe_region_active) {
      // Complete the lateral transition before the terminal vehicle footprint
      // enters the blocked goal zone.  Reaching the offset only at the stop
      // coordinate can leave the front half of the vehicle on a colliding
      // approach even though the final center pose itself is safe.
      const double terminal_transition_length = clamp_value(
          std::max(remaining_to_goal
                       - std::max(config_.terminal.safe_region_settle_distance, 0.0),
                   config_.lateral.min_length),
          config_.lateral.min_length, config_.lateral.max_length);
      options.clear();
      const bool terminal_position_only = terminal_constraint_active
          && remaining_to_goal < config_.lateral.min_length;
      options.emplace_back(
          terminal_position_only ? "terminal_safe_region_position_only"
                                 : "terminal_safe_region",
          terminal_transition_length, 0.0, terminal_position_only);
    }
    const bool active_target = maneuver_profile_ && maneuver_start_s_
        && std::abs(n_target - maneuver_profile_->target) <= 1.0e-9;
    ManeuverProfileState shifted_profile;
    const ManeuverProfileState* profile_override = nullptr;
    if (active_target) {
      shifted_profile = *maneuver_profile_;
      const double total_length = shifted_profile.start_delay
          + shifted_profile.lateral_length;
      shifted_profile.elapsed_length = clamp_value(
          fr.s - *maneuver_start_s_, 0.0, total_length);
      const double remaining_length = std::max(
          total_length - shifted_profile.elapsed_length, 0.0);
      // For the active target, retain only the remaining segment of the
      // original maneuver.  This is also required in terminal safe-region
      // mode: repeatedly rebuilding a terminal polynomial from the current
      // derivatives can push the vehicle farther outward at every snapshot.
      options.clear();
      options.emplace_back(
          terminal_safe_region_active ? "latched_terminal_transition"
                                      : "latched_transition",
          clamp_value(remaining_length, 0.10, config_.lateral.max_length),
          0.0, false);
      profile_override = &shifted_profile;
    }
    for (const auto& option : options) {
      const auto& path_mode = std::get<0>(option);
      const double lateral_length = std::get<1>(option);
      const double start_delay = std::get<2>(option);
      const bool terminal_position_only = std::get<3>(option);
      auto candidate = generate_spatial_path_candidate(state, previous_action, fr,
          n_target, lateral_length, preview_length, config_, path_, candidate_id,
          start_delay, terminal_position_only, profile_override);
      const int generated_candidate_id = candidate_id;
      ++candidate_id;
      if (!candidate) continue;
      generated.push_back(GeneratedCandidate{std::move(*candidate), path_mode,
          generated_candidate_id, n_target, active_target});
    }
  }

  // Phase 2: screen every generated candidate now that generation is done.
  for (auto& gen : generated) {
    const bool excluded_id = std::find(
        excluded_candidate_ids.begin(), excluded_candidate_ids.end(),
        gen.candidate_id) != excluded_candidate_ids.end();
    const bool excluded_target = std::any_of(
        excluded_lateral_targets.begin(), excluded_lateral_targets.end(),
        [&gen](double value) {
          return std::abs(gen.n_target - value) <= 1.0e-9;
        });
    if (excluded_id || excluded_target) continue;
    auto screen = screen_spatial_candidate(gen.path, state, config_,
        costmap_ ? &*costmap_ : nullptr, target_speed,
        terminal_safe_region_active);
    if (adaptive_replan_active) {
      screen.screening_cost += (clearance_scale - 1.0) * screen.obstacle_cost;
      screen.screening_cost -= (1.0 - reference_scale)
          * gen.path.reference_offset_cost;
      if (failed_lateral_target_) {
        const double sigma = std::max(
            config_.adaptive_replan.failed_target_sigma, 1.0e-3);
        const double normalized = (gen.n_target - *failed_lateral_target_) / sigma;
        screen.screening_cost += config_.adaptive_replan.failed_target_weight
            * allocation_failure_severity_
            * std::exp(-0.5 * normalized * normalized);
      }
    }
    if (gen.active_target && !adaptive_replan_active) {
      // A safe active maneuver is evaluated first and retained.  Alternative
      // targets remain available as fallbacks when its temporal rollout is
      // infeasible, while Allocation-triggered adaptive replanning disables
      // this lock and may choose a different path.
      screen.screening_cost -= 1.0e6;
    }
    Entry entry{std::move(gen.path), gen.path_mode, screen, false};
    all_spatial.push_back(entry);
    if (screen.preview_collision_free) entries.push_back(std::move(entry));
  }

  if (entries.empty() && config_.lateral.short_path_fallback_enabled && costmap_) {
    const double minimum_stop_length = std::max(
        config_.lateral.min_length,
        required_stop_distance + config_.lateral.short_path_stop_reserve
            + config_.longitudinal.stop_target_offset);
    std::vector<Entry> short_entries;
    ++g_planning_call_counts.spatial_path_generation;
    for (const auto& source : all_spatial) {
      if (!source.screen.curvature_valid
          || !std::isfinite(source.screen.first_coarse_collision_length)) {
        continue;
      }
      const double available = source.screen.first_coarse_collision_length
          - config_.lateral.short_path_collision_buffer;
      const double usable = std::min(
          available, config_.lateral.short_path_max_length);
      if (usable + 1.0e-9 < minimum_stop_length) continue;
      auto candidate = generate_spatial_path_candidate(
          state, previous_action, fr, source.path.n_target,
          source.path.lateral_length, usable, config_, path_, candidate_id,
          source.path.start_delay, source.path.terminal_position_only,
          nullptr, 0, true);
      ++candidate_id;
      if (!candidate) continue;
      auto screen = screen_spatial_candidate(
          *candidate, state, config_, costmap_ ? &*costmap_ : nullptr,
          target_speed, true);
      if (!screen.preview_collision_free) continue;
      short_path_length = std::max(short_path_length, candidate->real_end_l);
      short_entries.push_back(Entry{
          std::move(*candidate), source.path_mode + "_short_stop",
          screen, true});
    }
    if (!short_entries.empty()) {
      entries = std::move(short_entries);
      short_path_fallback_active = true;
    }
  }
  }  // end spatial_block_timer scope

  std::vector<std::size_t> center_indices;
  for (std::size_t i = 0; i < entries.size(); ++i) {
    if (std::abs(entries[i].path.n_target) < 1.0e-9) center_indices.push_back(i);
  }
  bool reference_center_lock_active = false;
  if (config_.lateral.reference_center_lock_enabled && !center_indices.empty()) {
    const auto& center = entries[center_indices.front()];
    reference_center_lock_active = center.screen.preview_collision_free
        && center.screen.obstacle_cost <=
           config_.lateral.reference_center_lock_obstacle_cost_tolerance;
  }
  if (reference_center_lock_active) {
    std::vector<Entry> centers;
    for (auto index : center_indices) centers.push_back(entries[index]);
    entries = std::move(centers);
    lateral_target_hint_.reset();
  }
  if (lateral_target_hint_ && !center_indices.empty() && !reference_center_lock_active) {
    const auto& center = entries[center_indices.front()];
    const bool center_clear = center.screen.preview_collision_free
        && center.screen.ranking_min_clearance > center.screen.hard_clearance_margin + 0.05;
    if (center_clear) lateral_target_hint_.reset();
  }
  if (lateral_target_hint_) {
    for (auto& entry : entries) {
      const double target_error =
          entry.path.n_target - *lateral_target_hint_;
      entry.screen.continuity_cost = continuity_scale
          * config_.lateral.target_continuity_weight
          * target_error * target_error;
      entry.screen.screening_cost += entry.screen.continuity_cost;
    }
  }
  const auto terminal_candidate_key = [&](double n_target,
                                          double terminal_goal_offset,
                                          double cost,
                                          int candidate_id) {
    const double exclusion_radius = std::max(
        0.25, 0.5 * config_.adaptive_replan.failed_target_sigma);
    const bool near_failed_target = adaptive_replan_active
        && failed_lateral_target_
        && std::abs(n_target - *failed_lateral_target_) <= exclusion_radius;
    const bool switches_failed_side = adaptive_replan_active
        && failed_lateral_target_
        && std::abs(*failed_lateral_target_) > 1.0e-6
        && n_target * *failed_lateral_target_ < 0.0;
    // Nominally minimize the terminal offset.  After an oriented-footprint
    // allocation failure, first exclude the failed neighborhood and expand on
    // the same side before considering a cross-center switch.  This preserves
    // the minimum-offset objective while allowing allocator feedback to find
    // the minimum *body-feasible* terminal region.
    return std::tuple<int, int, double, double, double, int>{
        near_failed_target ? 1 : 0,
        switches_failed_side ? 1 : 0,
        terminal_goal_offset,
        std::abs(n_target - fr.n),
        cost,
        candidate_id};
  };
  std::sort(entries.begin(), entries.end(),
      [&](const Entry& a, const Entry& b) {
    if (terminal_safe_region_active) {
      return terminal_candidate_key(
                 a.path.n_target, a.screen.terminal_goal_offset,
                 a.screen.screening_cost, a.path.candidate_id)
           < terminal_candidate_key(
                 b.path.n_target, b.screen.terminal_goal_offset,
                 b.screen.screening_cost, b.path.candidate_id);
    }
    return std::tuple<double, int>{a.screen.screening_cost, a.path.candidate_id}
         < std::tuple<double, int>{b.screen.screening_cost, b.path.candidate_id};
  });

  std::vector<FullCandidate> full;
  std::size_t cursor = 0;
  bool first_batch = true;
  if (!entries.empty()) ++g_planning_call_counts.trajectory_planning;
  {
  ScopedBlockTimer trajectory_block_timer(terminal_mode_active
      ? g_planning_block_timings.trajectory_terminal_ms
      : g_planning_block_timings.trajectory_normal_ms);
  while (cursor < entries.size()) {
    const int batch_size = first_batch
        ? std::max(1, terminal_safe_region_active ? config_.lateral.terminal_shortlist_size
                                           : config_.lateral.normal_shortlist_size)
        : std::max(1, config_.lateral.normal_fallback_batch_size);
    const std::size_t end = std::min(entries.size(), cursor + static_cast<std::size_t>(batch_size));
    bool batch_safe = false;
    for (std::size_t i = cursor; i < end; ++i) {
      const bool local_stop = entries[i].short_path_stop;
      auto tr = generate_open_loop_trajectory(entries[i].path, state, previous_action,
          map_end_mode || local_stop, config_, costmap_ ? &*costmap_ : nullptr, false,
          terminal_constraint_active || local_stop,
          terminal_mode_active || local_stop, target_speed, false);
      FullCandidate item{entries[i].path, std::move(tr), entries[i].path_mode,
                         entries[i].screen, local_stop,
                         entries[i].screen.screening_cost,
                         0.0};
      item.total_cost = item.path.path_cost + item.trajectory.trajectory_cost
          + item.screen.continuity_cost;
      batch_safe = batch_safe || item.trajectory.safe();
      full.push_back(std::move(item));
    }
    cursor = end;
    first_batch = false;
    if (batch_safe) break;
  }
  }  // end trajectory_block_timer scope

  int selected = -1;
  for (std::size_t i = 0; i < full.size(); ++i) {
    if (!full[i].trajectory.safe()) continue;
    if (selected < 0) {
      selected = static_cast<int>(i);
      continue;
    }
    const auto& current = full[static_cast<std::size_t>(selected)];
    if (terminal_safe_region_active) {
      const auto key = terminal_candidate_key(
          full[i].path.n_target, full[i].screen.terminal_goal_offset,
          full[i].total_cost, full[i].path.candidate_id);
      const auto best_key = terminal_candidate_key(
          current.path.n_target, current.screen.terminal_goal_offset,
          current.total_cost, current.path.candidate_id);
      if (key < best_key) selected = static_cast<int>(i);
    } else {
      const auto key = std::tuple<double, double, int>{
          full[i].lateral_selection_cost,
          std::abs(full[i].path.n_target),
          full[i].path.candidate_id};
      const auto best_key = std::tuple<double, double, int>{
          current.lateral_selection_cost,
          std::abs(current.path.n_target),
          current.path.candidate_id};
      if (key < best_key) selected = static_cast<int>(i);
    }
  }

  PlanResult result;
  int number_of_safe_paths = 0;
  for (const auto& item : full) number_of_safe_paths += item.trajectory.safe() ? 1 : 0;
  if (selected >= 0) {
    auto& item = full[static_cast<std::size_t>(selected)];
    result.trajectory = item.trajectory;
    result.selected_path = item.path;
    set_lateral_target_hint(item.path.n_target);
    const bool same_maneuver = maneuver_profile_ && maneuver_start_s_
        && std::abs(maneuver_profile_->target - item.path.n_target) <= 1.0e-9;
    const bool transition_required = !item.short_path_stop && (same_maneuver
        || std::abs(item.path.n_target - fr.n) >= 0.5);
    if (transition_required) {
      if (!same_maneuver) {
        const auto boundary = initial_spatial_boundaries(
            state, previous_action, fr, config_, path_);
        maneuver_profile_ = ManeuverProfileState{
            item.path.n_target,
            0.0,
            item.path.start_delay,
            item.path.lateral_length,
            boundary.n0,
            boundary.n1,
            boundary.n2,
            boundary.n3};
        maneuver_start_s_ = fr.s;
      }
    } else {
      maneuver_profile_.reset();
      maneuver_start_s_.reset();
    }
    result.diagnostics.status = item.short_path_stop
        ? "FEASIBLE_SHORT_PATH_STOP"
        : (terminal_mode_active
            ? "FEASIBLE_S_TERMINAL_CONTROL" : "FEASIBLE_OPEN_LOOP");
    result.diagnostics.selected_candidate_id = item.path.candidate_id;
    result.diagnostics.selected_path_mode = item.path_mode;
    result.diagnostics.selected_n_target = item.path.n_target;
    result.diagnostics.selected_lateral_length = item.path.lateral_length;
    result.diagnostics.selected_start_delay = item.path.start_delay;
    result.diagnostics.selected_min_clearance = item.screen.coarse_min_clearance;
    result.diagnostics.selected_total_cost = item.total_cost;
    result.diagnostics.bottleneck_limited = item.screen.bottleneck_limited;
    result.diagnostics.decision_horizon_length = item.screen.decision_horizon_length;
    result.diagnostics.terminal_goal_region_safe =
        item.screen.terminal_goal_region_safe;
    result.diagnostics.terminal_goal_offset = item.screen.terminal_goal_offset;
    result.diagnostics.terminal_goal_clearance =
        item.screen.terminal_goal_clearance;
  } else {
    auto best = full.end();
    if (!full.empty()) {
      best = std::min_element(full.begin(), full.end(),
          [&](const FullCandidate& a, const FullCandidate& b) {
        if (terminal_safe_region_active) {
          return terminal_candidate_key(
                     a.path.n_target, a.screen.terminal_goal_offset,
                     a.total_cost, a.path.candidate_id)
               < terminal_candidate_key(
                     b.path.n_target, b.screen.terminal_goal_offset,
                     b.total_cost, b.path.candidate_id);
        }
        return std::tuple<double, double, int>{
                   a.lateral_selection_cost, std::abs(a.path.n_target),
                   a.path.candidate_id}
             < std::tuple<double, double, int>{
                   b.lateral_selection_cost, std::abs(b.path.n_target),
                   b.path.candidate_id};
      });
      result.selected_path = best->path;
      result.diagnostics.selected_candidate_id = best->path.candidate_id;
      result.diagnostics.selected_path_mode = best->path_mode;
      result.diagnostics.selected_n_target = best->path.n_target;
      result.diagnostics.selected_lateral_length = best->path.lateral_length;
      result.diagnostics.selected_start_delay = best->path.start_delay;
      result.diagnostics.bottleneck_limited = best->screen.bottleneck_limited;
      result.diagnostics.decision_horizon_length = best->screen.decision_horizon_length;
      result.diagnostics.terminal_goal_region_safe =
          best->screen.terminal_goal_region_safe;
      result.diagnostics.terminal_goal_offset = best->screen.terminal_goal_offset;
      result.diagnostics.terminal_goal_clearance =
          best->screen.terminal_goal_clearance;
    }

    // If the nominal cruise rollout becomes dynamically infeasible, use a
    // jerk-limited braking rollout on the same collision-screened spatial
    // path.  When that rollout itself satisfies every dynamic, collision, and
    // terminal check it is a controlled feasibility response, not an
    // unvalidated emergency trajectory.  The lower applied speed is retained
    // by the speed-cap supervisor for subsequent replans.
    bool braking_fallback_safe = false;
    if (best != full.end()) {
      const bool local_stop = best->short_path_stop;
      ++g_planning_call_counts.trajectory_planning;
      TimeTrajectory braking;
      {
      ScopedBlockTimer braking_block_timer(terminal_mode_active
          ? g_planning_block_timings.trajectory_terminal_ms
          : g_planning_block_timings.trajectory_normal_ms);
      braking = generate_open_loop_trajectory(
          best->path, state, previous_action, map_end_mode || local_stop, config_,
          costmap_ ? &*costmap_ : nullptr, true,
          terminal_constraint_active || local_stop,
          terminal_mode_active || local_stop, target_speed, false);
      }  // end braking_block_timer scope
      if (braking.safe()) {
        result.trajectory = std::move(braking);
        braking_fallback_safe = true;
        number_of_safe_paths = std::max(number_of_safe_paths, 1);
        result.diagnostics.status = terminal_mode_active
            ? "FEASIBLE_TERMINAL_BRAKING_FALLBACK"
            : "FEASIBLE_DYNAMIC_BRAKING_FALLBACK";
      }
    }
    if (!braking_fallback_safe) {
      auto braking = emergency_stop(state, previous_action, target_speed, terminal_mode_active);
      if (braking.safe()) {
        result.selected_path.reset();
        result.trajectory = std::move(braking);
        braking_fallback_safe = true;
        number_of_safe_paths = std::max(number_of_safe_paths, 1);
        result.diagnostics.status = terminal_mode_active
            ? "FEASIBLE_TERMINAL_CENTERLINE_BRAKING_FALLBACK"
            : "FEASIBLE_CENTERLINE_BRAKING_FALLBACK";
      } else {
        result.trajectory = std::move(braking);
        std::string suffix;
        if (!result.trajectory.valid_dynamic) suffix += "_DYNAMIC";
        if (!result.trajectory.collision_free) suffix += "_COLLISION";
        if (!result.trajectory.terminal_valid()) suffix += "_TERMINAL";
        if (result.trajectory.endpoint_overshoot_attempt) suffix += "_OVERSHOOT";
        result.diagnostics.status = terminal_mode_active
            ? "S_TERMINAL_CONTROL_INFEASIBLE" + suffix
            : "EMERGENCY_STOP_NO_SAFE_PATH" + suffix;
      }
    }
  }
  result.diagnostics.number_of_lateral_paths = static_cast<int>(full.size());
  result.diagnostics.number_of_safe_paths = number_of_safe_paths;
  result.diagnostics.requested_target_speed = target_speed;
  result.diagnostics.applied_target_speed = target_speed;
  result.diagnostics.terminal_mode_active = terminal_mode_active;
  result.diagnostics.terminal_constraint_active = terminal_constraint_active;
  result.diagnostics.terminal_safe_region_active = terminal_safe_region_active;
  result.diagnostics.lateral_selection_basis = terminal_safe_region_active
      ? "TERMINAL_SAFE_REGION_MIN_OFFSET" : "SPATIAL_ONLY";
  result.diagnostics.reference_center_lock_active = reference_center_lock_active;
  result.diagnostics.adaptive_replan_active = adaptive_replan_active;
  result.diagnostics.allocation_failure_severity = allocation_failure_severity_;
  result.diagnostics.allocation_failure_memory = allocation_failure_memory_;
  result.diagnostics.short_path_fallback_active = short_path_fallback_active;
  result.diagnostics.short_path_length = short_path_length;
  result.diagnostics.excluded_candidate_id = excluded_candidate_ids.empty()
      ? -1 : excluded_candidate_ids.back();
  result.motion = build_allocator_trajectory(result.selected_path, result.trajectory,
      drive_mode, config_.longitudinal.dt, "planner_trajectory",
      "C++ Path-Velocity Decomposed Planner trajectory");
  result.diagnostics.compute_time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start_time).count();
  return result;
}

PlanResult PathVelocityPlanner::plan(const PlannerState& state,
                                     const PlannerAction& previous_action,
                                     const PlanningCommand& command) {
  const auto excluded_candidate_ids = one_shot_excluded_candidate_ids_;
  const auto excluded_lateral_targets = one_shot_excluded_lateral_targets_;
  one_shot_excluded_candidate_ids_.clear();
  one_shot_excluded_lateral_targets_.clear();
  const double requested_speed = clamp_value(command.target_speed,
      config_.constraints.v_min, config_.constraints.v_max);
  if (requested_speed <= config_.longitudinal.stop_speed_threshold
      && state.speed <= config_.longitudinal.stop_speed_threshold) {
    PlanResult result;
    result.trajectory = stationary_time_trajectory(state, config_, costmap_ ? &*costmap_ : nullptr);
    result.motion = build_allocator_trajectory(std::nullopt, result.trajectory,
        command.drive_mode, config_.longitudinal.dt, "planner_stationary_hold",
        "Zero-speed hold supplied by supervisory layer");
    result.diagnostics.status = "STATIONARY_COMMAND_HOLD";
    result.diagnostics.requested_target_speed = requested_speed;
    result.diagnostics.applied_target_speed = 0.0;
    return result;
  }

  // Short phases and high requested speeds may be position-feasible only at a
  // lower cruise cap.  Keep the deterministic speed backoff active in the
  // terminal region instead of falling directly to an emergency stop.
  const auto trials = speed_trials(requested_speed);
  PlanResult chosen;
  bool chosen_set = false;
  bool feasible = false;
  double applied_speed = requested_speed;
  int attempts = 0;
  for (double trial : trials) {
    ++attempts;
    auto result = plan_at_speed(
        state, previous_action, trial, command.drive_mode,
        excluded_candidate_ids, excluded_lateral_targets);
    feasible = result.diagnostics.number_of_safe_paths > 0
        && result.diagnostics.status.find("INFEASIBLE") == std::string::npos
        && result.diagnostics.status.find("EMERGENCY") == std::string::npos;
    chosen = std::move(result);
    chosen_set = true;
    applied_speed = trial;
    if (feasible) break;
    const bool geometric_failure =
        result.diagnostics.status.find("NO_SAFE_PATH_COLLISION") != std::string::npos;
    if (geometric_failure) break;
  }
  if (!chosen_set) throw std::runtime_error("speed search produced no result");
  if (feasible) {
    if (!last_feasible_speed_cap_ || std::abs(applied_speed - *last_feasible_speed_cap_) > 1.0e-6) {
      last_feasible_speed_cap_ = applied_speed;
      speed_cap_success_count_ = 0;
    } else {
      ++speed_cap_success_count_;
    }
  } else {
    // Preserve the most conservative attempted cap.  The runtime may reject
    // this infeasible replan and continue the already validated active
    // trajectory; the next planning attempt must then start from the lower
    // speed request instead of immediately probing the original high speed.
    last_feasible_speed_cap_ = applied_speed;
    speed_cap_success_count_ = 0;
  }
  const bool speed_cap_active = feasible && applied_speed < requested_speed - 1.0e-6;
  if (speed_cap_active) {
    if (chosen.diagnostics.status.find("BRAKING_FALLBACK") != std::string::npos) {
      chosen.diagnostics.status += "_WITH_SPEED_CAP";
    } else {
      chosen.diagnostics.status = "FEASIBLE_WITH_OBSTACLE_SPEED_CAP";
    }
  }
  chosen.diagnostics.requested_target_speed = requested_speed;
  chosen.diagnostics.applied_target_speed = applied_speed;
  chosen.diagnostics.obstacle_speed_cap_active = speed_cap_active;
  chosen.diagnostics.speed_search_attempts = attempts;
  chosen.motion.name = "planner_" + std::to_string(static_cast<int>(command.drive_mode));
  return chosen;
}

AllocatorInitialState AllocationResult::state_at(std::size_t index) const {
  if (beta.empty()) return {};
  index = std::min(index, beta.size() - 1);
  const std::size_t command_index = index > 0 ? index - 1 : 0;
  return {beta[index], beta_rate[command_index], yaw_rate[command_index],
          yaw_acceleration[command_index]};
}

AllocationResult allocate_trajectory(
    const PlannerMotionTrajectory& trajectory,
    const AllocationLimits& limits,
    std::optional<AllocatorInitialState> initial_state) {
  const std::size_t n = trajectory.t.size();
  if (n < 2 || trajectory.x.size() != n || trajectory.y.size() != n
      || trajectory.chi.size() != n || trajectory.speed.size() != n
      || trajectory.acceleration.size() != n || trajectory.drive_mode.size() != n) {
    throw std::invalid_argument("invalid PlannerMotionTrajectory");
  }
  AllocationResult result;
  result.trajectory = trajectory;
  result.beta_center.resize(n);
  result.beta_desired.assign(n, 0.0);
  result.beta.assign(n, 0.0);
  result.beta_rate.assign(n, 0.0);
  result.beta_acceleration.assign(n, 0.0);
  result.psi.assign(n, 0.0);
  result.vx.assign(n, 0.0);
  result.vy.assign(n, 0.0);
  result.yaw_rate.assign(n, 0.0);
  result.yaw_acceleration.assign(n, 0.0);
  result.yaw_jerk.assign(n, 0.0);
  result.vx_acceleration.assign(n, 0.0);
  result.vy_acceleration.assign(n, 0.0);
  result.vx_jerk.assign(n, 0.0);
  result.vy_jerk.assign(n, 0.0);
  result.fallback_used.assign(n, 0);
  result.speed_reconstruction_error.assign(n, 0.0);
  result.heading_relation_error.assign(n, 0.0);
  result.world_velocity_identity_error.assign(n, 0.0);

  std::vector<double> raw_center(n);
  for (std::size_t i = 0; i < n; ++i) raw_center[i] = drive_mode_heading_offset(trajectory.drive_mode[i]);
  result.beta_center = unwrap_angles(raw_center);
  const auto chi = unwrap_angles(trajectory.chi);
  result.beta[0] = initial_state ? initial_state->beta : result.beta_center[0];
  double previous_beta_rate = initial_state ? initial_state->beta_rate : 0.0;
  double previous_yaw_rate = initial_state ? initial_state->yaw_rate : 0.0;
  double previous_yaw_acceleration = initial_state ? initial_state->yaw_acceleration : 0.0;
  std::vector<std::size_t> mode_change_indices;
  for (std::size_t i = 1; i < n; ++i) {
    if (trajectory.drive_mode[i] != trajectory.drive_mode[i - 1]) mode_change_indices.push_back(i);
  }
  for (std::size_t k = 0; k + 1 < n; ++k) {
    const double dt = trajectory.t[k + 1] - trajectory.t[k];
    const auto mode = trajectory.drive_mode[k];
    const auto next_mode = trajectory.drive_mode[k + 1];
    const std::size_t mode_index = static_cast<std::size_t>(mode);
    const std::size_t next_index = static_cast<std::size_t>(next_mode);
    const double chi_dot = trajectory.motion_heading_rate[k];
    const double chi_ddot = trajectory.motion_heading_acceleration[k];
    const double allocation_speed = std::max(
        std::abs(trajectory.speed[k]), std::abs(trajectory.speed[k + 1]));
    const double activation_reference = std::max(limits.allocation_active_speed, 1.0e-3);
    const double speed_activation = allocation_speed * allocation_speed /
        (allocation_speed * allocation_speed
         + activation_reference * activation_reference);
    const double alpha = limits.body_yaw_fraction[mode_index];
    const double beta_gain = limits.beta_return_gain[mode_index];
    const double equilibrium_deviation = clamp_value(
        (1.0 - alpha) * chi_dot / std::max(beta_gain, 1.0e-6),
        -limits.beta_max_deviation[mode_index],
        limits.beta_max_deviation[mode_index]);
    result.beta_desired[k] = result.beta_center[k] + equilibrium_deviation;
    double low = -limits.yaw_rate_max;
    double high = limits.yaw_rate_max;
    const double acceleration_low = std::max(-limits.yaw_accel_max,
        previous_yaw_acceleration - limits.yaw_jerk_max * dt);
    const double acceleration_high = std::min(limits.yaw_accel_max,
        previous_yaw_acceleration + limits.yaw_jerk_max * dt);
    low = std::max(low, previous_yaw_rate + acceleration_low * dt);
    high = std::min(high, previous_yaw_rate + acceleration_high * dt);
    double yaw_rate = 0.0;
    if (next_mode != mode) {
      yaw_rate = clamp_value(0.0, low, high);
    } else {
      const double beta_error = result.beta[k] - result.beta_center[k];
      const double desired_yaw_rate = speed_activation
          * (alpha * chi_dot + beta_gain * beta_error);
      double core_low = std::max(low, chi_dot - limits.beta_rate_max);
      double core_high = std::min(high, chi_dot + limits.beta_rate_max);
      const double maximum_beta_rate_change = limits.beta_accel_max * dt;
      core_low = std::max(core_low, chi_dot - (previous_beta_rate + maximum_beta_rate_change));
      core_high = std::min(core_high, chi_dot - (previous_beta_rate - maximum_beta_rate_change));
      if (core_low > core_high) {
        result.fallback_used[k] = 1;
        yaw_rate = clamp_value(desired_yaw_rate, low, high);
      } else {
        const double beta_low = result.beta_center[k + 1] - limits.beta_max_deviation[next_index];
        const double beta_high = result.beta_center[k + 1] + limits.beta_max_deviation[next_index];
        const double envelope_low = std::max(core_low,
            (result.beta[k] + chi_dot * dt - beta_high) / dt);
        const double envelope_high = std::min(core_high,
            (result.beta[k] + chi_dot * dt - beta_low) / dt);
        yaw_rate = envelope_low <= envelope_high
            ? clamp_value(desired_yaw_rate, envelope_low, envelope_high)
            : clamp_value(desired_yaw_rate, core_low, core_high);
      }
    }
    const double beta_rate = chi_dot - yaw_rate;
    result.beta_rate[k] = beta_rate;
    result.yaw_rate[k] = yaw_rate;
    result.yaw_acceleration[k] = (yaw_rate - previous_yaw_rate) / dt;
    result.yaw_jerk[k] = (result.yaw_acceleration[k] - previous_yaw_acceleration) / dt;
    result.beta_acceleration[k] = chi_ddot - result.yaw_acceleration[k];
    result.beta[k + 1] = result.beta[k] + beta_rate * dt;
    previous_beta_rate = beta_rate;
    previous_yaw_rate = yaw_rate;
    previous_yaw_acceleration = result.yaw_acceleration[k];
  }
  result.beta_rate.back() = result.beta_rate[n - 2];
  result.beta_acceleration.back() = result.beta_acceleration[n - 2];
  result.yaw_rate.back() = result.yaw_rate[n - 2];
  result.yaw_acceleration.back() = result.yaw_acceleration[n - 2];
  result.yaw_jerk.back() = result.yaw_jerk[n - 2];
  result.beta_desired.back() = result.beta_desired[n - 2];
  for (std::size_t i = 0; i < n; ++i) {
    result.psi[i] = chi[i] - result.beta[i];
    const double cos_beta = std::cos(result.beta[i]);
    const double sin_beta = std::sin(result.beta[i]);
    result.vx[i] = trajectory.speed[i] * cos_beta;
    result.vy[i] = trajectory.speed[i] * sin_beta;
    result.vx_acceleration[i] = trajectory.acceleration[i] * cos_beta
        - trajectory.speed[i] * result.beta_rate[i] * sin_beta;
    result.vy_acceleration[i] = trajectory.acceleration[i] * sin_beta
        + trajectory.speed[i] * result.beta_rate[i] * cos_beta;
    result.vx_jerk[i] = trajectory.longitudinal_jerk[i] * cos_beta
        - 2.0 * trajectory.acceleration[i] * result.beta_rate[i] * sin_beta
        - trajectory.speed[i] * result.beta_acceleration[i] * sin_beta
        - trajectory.speed[i] * result.beta_rate[i] * result.beta_rate[i] * cos_beta;
    result.vy_jerk[i] = trajectory.longitudinal_jerk[i] * sin_beta
        + 2.0 * trajectory.acceleration[i] * result.beta_rate[i] * cos_beta
        + trajectory.speed[i] * result.beta_acceleration[i] * cos_beta
        - trajectory.speed[i] * result.beta_rate[i] * result.beta_rate[i] * sin_beta;
    const double c = std::cos(result.psi[i]);
    const double s = std::sin(result.psi[i]);
    const double world_vx = c * result.vx[i] - s * result.vy[i];
    const double world_vy = s * result.vx[i] + c * result.vy[i];
    const double desired_world_vx = trajectory.speed[i] * std::cos(chi[i]);
    const double desired_world_vy = trajectory.speed[i] * std::sin(chi[i]);
    result.speed_reconstruction_error[i] = std::hypot(result.vx[i], result.vy[i])
        - trajectory.speed[i];
    result.heading_relation_error[i] = wrap_angle(chi[i] - result.psi[i] - result.beta[i]);
    result.world_velocity_identity_error[i] = std::hypot(world_vx - desired_world_vx,
                                                          world_vy - desired_world_vy);
  }
  for (auto index : mode_change_indices) {
    const std::size_t lo = index > 0 ? index - 1 : 0;
    const std::size_t hi = std::min(n, index + 2);
    double maximum_speed = 0.0;
    for (std::size_t i = lo; i < hi; ++i) maximum_speed = std::max(maximum_speed, trajectory.speed[i]);
    result.mode_change_while_moving.push_back(
        static_cast<std::uint8_t>(maximum_speed > limits.stop_speed_threshold));
  }
  return result;
}


const char* allocation_profile_name(AllocationProfile profile) {
  switch (profile) {
    case AllocationProfile::LateralPriority: return "LATERAL_PRIORITY";
    case AllocationProfile::Balanced: return "BALANCED";
    case AllocationProfile::YawPriority: return "YAW_PRIORITY";
    case AllocationProfile::MinimumVy: return "MINIMUM_VY";
  }
  return "UNKNOWN";
}

AllocationLimits allocation_limits_for_profile(AllocationProfile profile) {
  AllocationLimits limits;
  switch (profile) {
    case AllocationProfile::LateralPriority:
      break;
    case AllocationProfile::Balanced:
      limits.body_yaw_fraction[0] = 0.50;
      limits.body_yaw_fraction[1] = 0.55;
      limits.beta_return_gain[0] = 0.65;
      limits.beta_return_gain[1] = 0.70;
      limits.beta_max_deviation[0] = 35.0 * kPi / 180.0;
      limits.beta_max_deviation[1] = 30.0 * kPi / 180.0;
      break;
    case AllocationProfile::YawPriority:
      limits.body_yaw_fraction[0] = 0.75;
      limits.body_yaw_fraction[1] = 0.80;
      limits.beta_return_gain[0] = 0.90;
      limits.beta_return_gain[1] = 0.95;
      limits.beta_max_deviation[0] = 25.0 * kPi / 180.0;
      limits.beta_max_deviation[1] = 20.0 * kPi / 180.0;
      break;
    case AllocationProfile::MinimumVy:
      limits.body_yaw_fraction[0] = 0.92;
      limits.body_yaw_fraction[1] = 0.92;
      limits.beta_return_gain[0] = 1.20;
      limits.beta_return_gain[1] = 1.20;
      limits.beta_max_deviation[0] = 12.0 * kPi / 180.0;
      limits.beta_max_deviation[1] = 12.0 * kPi / 180.0;
      break;
  }
  return limits;
}

OrientedCollisionResult check_oriented_allocation_collision(
    const AllocationResult& allocation,
    const Costmap2D& costmap,
    const VehicleConfig& vehicle,
    const CostConfig& cost,
    const OrientedFootprintConfig& footprint) {
  if (footprint.circle_count < 1 || footprint.maximum_translation_step <= 0.0
      || footprint.maximum_yaw_step <= 0.0) {
    throw std::invalid_argument("invalid oriented footprint configuration");
  }
  const std::size_t n = allocation.trajectory.x.size();
  if (n < 1 || allocation.trajectory.y.size() != n || allocation.psi.size() != n
      || allocation.trajectory.speed.size() != n) {
    throw std::invalid_argument("invalid allocation for oriented collision check");
  }
  const int circle_count = footprint.circle_count;
  const double segment_length = vehicle.length / static_cast<double>(circle_count);
  const double radius = std::hypot(0.5 * segment_length, 0.5 * vehicle.width)
      + vehicle.footprint_margin;
  OrientedCollisionResult result;
  auto check_pose = [&](double x, double y, double psi, double speed) {
    const double c = std::cos(psi);
    const double s = std::sin(psi);
    const double hard_margin = effective_hard_clearance_margin(speed, cost);
    ++result.checked_poses;
    for (int k = 0; k < circle_count; ++k) {
      const double longitudinal = -0.5 * vehicle.length
          + (static_cast<double>(k) + 0.5) * segment_length;
      const double cx = x + c * longitudinal;
      const double cy = y + s * longitudinal;
      const double clearance = costmap.clearance_single_circle(cx, cy, radius);
      result.minimum_clearance = std::min(result.minimum_clearance, clearance);
      ++result.checked_circles;
      if (clearance <= hard_margin + 1.0e-9) result.collision_free = false;
    }
  };
  if (n == 1) {
    check_pose(allocation.trajectory.x[0], allocation.trajectory.y[0],
               allocation.psi[0], allocation.trajectory.speed[0]);
    return result;
  }
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const double dx = allocation.trajectory.x[i + 1] - allocation.trajectory.x[i];
    const double dy = allocation.trajectory.y[i + 1] - allocation.trajectory.y[i];
    const double dpsi = wrap_angle(allocation.psi[i + 1] - allocation.psi[i]);
    const int translation_steps = static_cast<int>(std::ceil(
        std::hypot(dx, dy) / footprint.maximum_translation_step));
    const int yaw_steps = static_cast<int>(std::ceil(
        std::abs(dpsi) / footprint.maximum_yaw_step));
    const int subdivisions = std::max(1, std::max(translation_steps, yaw_steps));
    for (int j = 0; j < subdivisions; ++j) {
      const double u = static_cast<double>(j) / static_cast<double>(subdivisions);
      check_pose(allocation.trajectory.x[i] + u * dx,
                 allocation.trajectory.y[i] + u * dy,
                 allocation.psi[i] + u * dpsi,
                 allocation.trajectory.speed[i]
                     + u * (allocation.trajectory.speed[i + 1]
                            - allocation.trajectory.speed[i]));
    }
  }
  check_pose(allocation.trajectory.x.back(), allocation.trajectory.y.back(),
             allocation.psi.back(), allocation.trajectory.speed.back());
  return result;
}

AllocationSelectionResult allocate_with_oriented_collision_search(
    const PlannerMotionTrajectory& trajectory,
    const Costmap2D& costmap,
    const VehicleConfig& vehicle,
    const CostConfig& cost,
    std::optional<AllocatorInitialState> initial_state,
    const OrientedFootprintConfig& footprint) {
  ++g_planning_call_counts.allocation;
  ScopedBlockTimer allocation_block_timer(g_planning_block_timings.allocation_ms);
  const bool crab_mode = std::any_of(
      trajectory.drive_mode.begin(), trajectory.drive_mode.end(),
      [](DriveMode mode) { return mode == DriveMode::Left || mode == DriveMode::Right; });
  // Evaluate the nominal lateral-priority allocation first.  Only when its
  // complete four-second trajectory collides, retry the same motion trajectory
  // once with the minimum-vy profile.  Spatial replanning is intentionally
  // left to the caller after both profiles fail.
  const std::array<AllocationProfile, 2> profiles{{
      AllocationProfile::LateralPriority,
      AllocationProfile::MinimumVy}};
  std::optional<AllocationSelectionResult> best_failed;
  int evaluated = 0;
  for (const auto profile : profiles) {
    if (crab_mode && profile != AllocationProfile::LateralPriority) break;
    auto allocation = allocate_trajectory(
        trajectory, allocation_limits_for_profile(profile), initial_state);
    if (std::any_of(allocation.fallback_used.begin(), allocation.fallback_used.end(),
                    [](std::uint8_t value) { return value != 0; })) {
      continue;
    }
    ++evaluated;
    auto collision = check_oriented_allocation_collision(
        allocation, costmap, vehicle, cost, footprint);
    AllocationSelectionResult candidate{std::move(allocation), collision, profile, evaluated};
    if (collision.collision_free) {
      return candidate;
    } else if (!best_failed
               || collision.minimum_clearance > best_failed->collision.minimum_clearance) {
      best_failed = std::move(candidate);
    }
  }
  if (best_failed) {
    best_failed->candidates_evaluated = evaluated;
    return std::move(*best_failed);
  }
  throw std::runtime_error("all allocation profiles violated dynamic constraints");
}

}  // namespace simp_planner
