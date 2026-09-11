#include "simp_planner/core.hpp"
#include "simp_planner/runtime.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

using namespace simp_planner;

namespace {

template <class T>
T read_value(std::ifstream& input) {
  T value{};
  input.read(reinterpret_cast<char*>(&value), sizeof(T));
  if (!input) throw std::runtime_error("unexpected end of validation input");
  return value;
}

std::string read_string(std::ifstream& input) {
  const auto size = read_value<std::uint32_t>(input);
  std::string value(size, '\0');
  input.read(value.data(), static_cast<std::streamsize>(size));
  if (!input) throw std::runtime_error("unexpected end of validation string");
  return value;
}

std::vector<double> read_doubles(std::ifstream& input, std::size_t count) {
  std::vector<double> values(count);
  input.read(reinterpret_cast<char*>(values.data()),
             static_cast<std::streamsize>(count * sizeof(double)));
  if (!input) throw std::runtime_error("unexpected end of validation array");
  return values;
}

struct PhaseData {
  std::string name;
  DriveMode mode{DriveMode::Forward};
  double cruise_speed{0.0};
  double switch_s{-1.0};
  bool closed_loop{false};
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> kappa;
};

struct GateData {
  double s{0.0};
  double center{0.0};
  double gap{0.0};
};

struct ScenarioData {
  std::string name;
  double terminal_margin{3.0};
  double stop_request_distance{3.0};
  bool repeat{false};
  int footprint_circle_count{3};
  std::vector<PhaseData> phases;
  int width{0};
  int height{0};
  double resolution{0.2};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<std::int8_t> costmap;
  std::vector<GateData> gates;
};

ScenarioData load_scenario(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot open validation input: " + path);
  char magic[8]{};
  input.read(magic, 8);
  const bool version2 = input && std::memcmp(magic, "SIMPVAL2", 8) == 0;
  const bool version3 = input && std::memcmp(magic, "SIMPVAL3", 8) == 0;
  if (!version2 && !version3) {
    throw std::runtime_error("invalid validation input magic");
  }
  ScenarioData scenario;
  scenario.name = read_string(input);
  scenario.terminal_margin = read_value<double>(input);
  scenario.stop_request_distance = read_value<double>(input);
  scenario.repeat = read_value<std::uint8_t>(input) != 0;
  if (version3) scenario.footprint_circle_count = read_value<std::int32_t>(input);
  const auto phase_count = read_value<std::uint32_t>(input);
  scenario.phases.reserve(phase_count);
  for (std::uint32_t i = 0; i < phase_count; ++i) {
    PhaseData phase;
    phase.name = read_string(input);
    phase.mode = static_cast<DriveMode>(read_value<std::int32_t>(input));
    phase.cruise_speed = read_value<double>(input);
    phase.switch_s = read_value<double>(input);
    phase.closed_loop = read_value<std::uint8_t>(input) != 0;
    const auto count = read_value<std::uint32_t>(input);
    phase.x = read_doubles(input, count);
    phase.y = read_doubles(input, count);
    phase.yaw = read_doubles(input, count);
    phase.kappa = read_doubles(input, count);
    scenario.phases.push_back(std::move(phase));
  }
  scenario.width = read_value<std::int32_t>(input);
  scenario.height = read_value<std::int32_t>(input);
  scenario.resolution = read_value<double>(input);
  scenario.origin_x = read_value<double>(input);
  scenario.origin_y = read_value<double>(input);
  const std::size_t cells = static_cast<std::size_t>(scenario.width)
      * static_cast<std::size_t>(scenario.height);
  scenario.costmap.resize(cells);
  input.read(reinterpret_cast<char*>(scenario.costmap.data()),
             static_cast<std::streamsize>(cells));
  if (!input) throw std::runtime_error("unexpected end of costmap");
  const auto gate_count = read_value<std::uint32_t>(input);
  scenario.gates.resize(gate_count);
  for (auto& gate : scenario.gates) {
    gate.s = read_value<double>(input);
    gate.center = read_value<double>(input);
    gate.gap = read_value<double>(input);
  }
  return scenario;
}

std::vector<double> cumulative_s(const std::vector<double>& x,
                                 const std::vector<double>& y) {
  std::vector<double> s(x.size(), 0.0);
  for (std::size_t i = 1; i < x.size(); ++i) {
    s[i] = s[i - 1] + std::hypot(x[i] - x[i - 1], y[i] - y[i - 1]);
  }
  return s;
}

PhaseData clip_phase(const PhaseData& source, double stop_s) {
  const auto s = cumulative_s(source.x, source.y);
  stop_s = std::clamp(stop_s, 0.0, s.back());
  auto upper = std::upper_bound(s.begin(), s.end(), stop_s + 1.0e-10);
  std::size_t count = static_cast<std::size_t>(std::distance(s.begin(), upper));
  count = std::max<std::size_t>(count, 2);
  PhaseData result = source;
  result.closed_loop = false;
  result.x.assign(source.x.begin(), source.x.begin() + static_cast<std::ptrdiff_t>(count));
  result.y.assign(source.y.begin(), source.y.begin() + static_cast<std::ptrdiff_t>(count));
  result.yaw.assign(source.yaw.begin(), source.yaw.begin() + static_cast<std::ptrdiff_t>(count));
  result.kappa.assign(source.kappa.begin(), source.kappa.begin() + static_cast<std::ptrdiff_t>(count));
  if (std::abs(s[count - 1] - stop_s) > 1.0e-8 && count < s.size()) {
    const std::size_t lo = count - 1;
    const std::size_t hi = count;
    const double u = (stop_s - s[lo]) / std::max(s[hi] - s[lo], 1.0e-9);
    result.x.push_back(source.x[lo] + u * (source.x[hi] - source.x[lo]));
    result.y.push_back(source.y[lo] + u * (source.y[hi] - source.y[lo]));
    result.yaw.push_back(source.yaw[lo] + u * (source.yaw[hi] - source.yaw[lo]));
    result.kappa.push_back(source.kappa[lo] + u * (source.kappa[hi] - source.kappa[lo]));
  }
  while (result.x.size() < 4) {
    const double dx = result.x.back() - result.x[result.x.size() - 2];
    const double dy = result.y.back() - result.y[result.y.size() - 2];
    result.x.push_back(result.x.back() + std::max(0.05, std::hypot(dx, dy))
        * std::cos(result.yaw.back()));
    result.y.push_back(result.y.back() + std::max(0.05, std::hypot(dx, dy))
        * std::sin(result.yaw.back()));
    result.yaw.push_back(result.yaw.back());
    result.kappa.push_back(result.kappa.back());
  }
  return result;
}

ReferencePath make_reference(const PhaseData& phase) {
  return ReferencePath(cumulative_s(phase.x, phase.y), phase.x, phase.y,
                       phase.yaw, phase.kappa);
}

double mode_offset(DriveMode mode) {
  return drive_mode_heading_offset(mode);
}

struct Metrics {
  bool pass{false};
  std::string reason{"UNKNOWN"};
  double simulated_time{0.0};
  int plans{0};
  int planning_failures{0};
  int primary_allocation_collisions{0};
  int minimum_vy_successes{0};
  int adaptive_path_replans{0};
  int adaptive_path_replan_successes{0};
  int bottleneck_limited_plans{0};
  int safety_collisions{0};
  int gate_passes{0};
  int gate_count{0};
  double minimum_clearance{std::numeric_limits<double>::infinity()};
  double maximum_plan_ms{0.0};
  double maximum_plan_time_s{0.0};
  double maximum_plan_progress_m{0.0};
  int maximum_plan_replans{0};
  double maximum_lateral_execution_error{0.0};
  double maximum_direction_execution_error_deg{0.0};
  double final_progress{0.0};
  double final_speed{0.0};
};

struct PlanBundle {
  PlanResult result;
  AllocationSelectionResult selection;
  double trial_speed{0.0};
  double elapsed_ms{0.0};
  int replans_this_call{0};
};

std::optional<PlanBundle> plan_with_pipeline(
    PathVelocityPlanner& planner, const PlannerState& state,
    const PlannerAction& previous_action, DriveMode mode, double target_speed,
    const Costmap2D& costmap, const EnvConfig& config,
    std::optional<AllocatorInitialState> allocator_state, Metrics& metrics,
    const OrientedFootprintConfig& footprint) {
  const auto wall_start = std::chrono::steady_clock::now();
  const int replans_before = metrics.adaptive_path_replans;
  auto result = planner.plan(state, previous_action, {target_speed, mode});
  if (std::getenv("SIMP_VALIDATION_VERBOSE") != nullptr && metrics.plans <= 3) {
    std::cerr << "  target_speed=" << target_speed
              << " status=" << result.diagnostics.status
              << " safe_paths=" << result.diagnostics.number_of_safe_paths
              << " candidate=" << result.diagnostics.selected_candidate_id
              << " n_target=" << result.diagnostics.selected_n_target
              << " short_path=" << result.diagnostics.short_path_fallback_active
              << " min_clearance=" << result.diagnostics.selected_min_clearance
              << "\n";
  }
  const bool infeasible = result.diagnostics.status.find("EMERGENCY") != std::string::npos
      || result.diagnostics.status.find("INFEASIBLE") != std::string::npos;
  if (infeasible) {
    ++metrics.planning_failures;
    return std::nullopt;
  }

  auto selection = allocate_with_oriented_collision_search(
      result.motion, costmap, config.vehicle, config.cost,
      allocator_state, footprint);
  if (selection.candidates_evaluated >= 2) {
    ++metrics.primary_allocation_collisions;
  }
  std::vector<int> excluded_candidate_ids;
  std::vector<double> excluded_lateral_targets;
  const int maximum_path_replans = std::max(
      0, config.lateral.allocation_path_replan_max_attempts);
  for (int attempt = 0;
       !selection.collision.collision_free
           && result.selected_path
           && attempt < maximum_path_replans;
       ++attempt) {
    excluded_candidate_ids.push_back(result.selected_path->candidate_id);
    excluded_lateral_targets.push_back(result.selected_path->n_target);
    planner.set_one_shot_excluded_paths(
        excluded_candidate_ids, excluded_lateral_targets);
    ++metrics.adaptive_path_replans;
    auto replanned = planner.plan(state, previous_action, {target_speed, mode});
    const bool replanned_infeasible =
        replanned.diagnostics.status.find("EMERGENCY") != std::string::npos
        || replanned.diagnostics.status.find("INFEASIBLE") != std::string::npos;
    if (replanned_infeasible) break;
    auto replanned_selection = allocate_with_oriented_collision_search(
        replanned.motion, costmap, config.vehicle, config.cost,
        allocator_state, footprint);
    result = std::move(replanned);
    selection = std::move(replanned_selection);
    if (selection.collision.collision_free) {
      result.diagnostics.status += "_EXCLUDED_PATH_REPLAN";
      ++metrics.adaptive_path_replan_successes;
      break;
    }
  }


  const auto wall_end = std::chrono::steady_clock::now();
  const double elapsed_ms = 1000.0 * std::chrono::duration<double>(wall_end - wall_start).count();
  if (!selection.collision.collision_free) {
    if (std::getenv("SIMP_VALIDATION_VERBOSE") != nullptr) {
      std::cerr << "  allocation failure: final_candidate="
                << (result.selected_path ? result.selected_path->candidate_id : -1)
                << " target="
                << (result.selected_path ? result.selected_path->n_target : 0.0)
                << " clearance=" << selection.collision.minimum_clearance
                << " status=" << result.diagnostics.status << "\n";
    }
    return std::nullopt;
  }
  if (selection.profile == AllocationProfile::MinimumVy) {
    ++metrics.minimum_vy_successes;
  }
  if (result.diagnostics.bottleneck_limited) {
    ++metrics.bottleneck_limited_plans;
  }
  planner.record_allocation_success();
  return PlanBundle{std::move(result), std::move(selection), target_speed, elapsed_ms, metrics.adaptive_path_replans - replans_before};
}

void integrate(double& x, double& y, double& body_yaw,
               const BodyCommand& command, double dt) {
  const double yaw_mid = body_yaw + 0.5 * command.yaw_rate * dt;
  x += (std::cos(yaw_mid) * command.vx - std::sin(yaw_mid) * command.vy) * dt;
  y += (std::sin(yaw_mid) * command.vx + std::cos(yaw_mid) * command.vy) * dt;
  body_yaw = wrap_angle(body_yaw + command.yaw_rate * dt);
}

AllocationResult pose_allocation(double x, double y, double body_yaw, double speed) {
  AllocationResult pose;
  pose.trajectory.x = {x};
  pose.trajectory.y = {y};
  pose.trajectory.speed = {speed};
  pose.psi = {body_yaw};
  return pose;
}

PhaseData transformed_next_phase(const ScenarioData& scenario,
                                 std::size_t next_index,
                                 double stop_x, double stop_y,
                                 double body_yaw) {
  PhaseData result = scenario.phases[next_index];
  if (scenario.name == "crab_switch" && next_index == 1) {
    const auto source_s = cumulative_s(result.x, result.y);
    const double chi = body_yaw + 0.5 * kPi;
    for (std::size_t i = 0; i < result.x.size(); ++i) {
      result.x[i] = stop_x + source_s[i] * std::cos(chi);
      result.y[i] = stop_y + source_s[i] * std::sin(chi);
      result.yaw[i] = chi;
      result.kappa[i] = 0.0;
    }
  } else if (scenario.name == "reverse_switch" && next_index == 1) {
    const double dx = stop_x - result.x.front();
    const double dy = stop_y - result.y.front();
    for (std::size_t i = 0; i < result.x.size(); ++i) {
      result.x[i] += dx;
      result.y[i] += dy;
    }
  }
  return result;
}

std::optional<double> environment_double(const char* name) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') return std::nullopt;
  return std::stod(value);
}

Metrics run_scenario(const ScenarioData& scenario, double target_speed,
                     int external_max_plans = 200000) {
  Metrics metrics;
  EnvConfig config;
  Costmap2D costmap(scenario.costmap, scenario.width, scenario.height,
                    scenario.resolution, scenario.origin_x, scenario.origin_y);
  std::size_t phase_index = 0;
  PhaseData phase = scenario.phases.front();
  auto phase_total_s = cumulative_s(phase.x, phase.y).back();
  double stop_s = scenario.repeat ? phase_total_s
      : (phase.switch_s >= 0.0 ? phase.switch_s
                               : std::max(phase_total_s - scenario.terminal_margin, 0.0));
  phase = clip_phase(phase, stop_s);
  auto reference = make_reference(phase);
  auto planner = std::make_unique<PathVelocityPlanner>(config, reference, costmap);

  const auto validation_start_s_env = environment_double("SIMP_VALIDATION_START_S");
  const auto validation_end_s_env = environment_double("SIMP_VALIDATION_END_S");
  const double validation_start_s = std::clamp(
      validation_start_s_env.value_or(0.0), 0.0, reference.s_max());
  const double validation_end_s = validation_end_s_env
      ? std::clamp(*validation_end_s_env, validation_start_s, reference.s_max())
      : reference.s_max();
  const bool window_mode = validation_start_s_env.has_value()
      || validation_end_s_env.has_value();

  double x = 0.0;
  double y = 0.0;
  double motion_heading = 0.0;
  double initial_kappa = 0.0;
  double initial_kappa_s = 0.0;
  reference.evaluate(validation_start_s, x, y, motion_heading, initial_kappa, initial_kappa_s);
  (void)initial_kappa;
  (void)initial_kappa_s;
  double body_yaw = wrap_angle(motion_heading - mode_offset(phase.mode));
  const double initial_speed = std::clamp(
      environment_double("SIMP_VALIDATION_INITIAL_SPEED").value_or(0.0),
      0.0, config.constraints.v_max);
  PlannerState state{x, y, motion_heading, initial_speed, 0.0,
                     initial_speed * initial_kappa};
  PlannerAction previous_action{};
  std::optional<AllocatorInitialState> allocator_state = AllocatorInitialState{
      mode_offset(phase.mode), 0.0, 0.0, 0.0};
  std::optional<double> previous_projection_s = validation_start_s;
  std::vector<bool> gate_seen(scenario.gates.size(), false);
  for (std::size_t i = 0; i < scenario.gates.size(); ++i) {
    const bool in_window = scenario.gates[i].s > validation_start_s + 1.0e-9
        && scenario.gates[i].s <= validation_end_s + 1.0e-9;
    gate_seen[i] = !in_window;
    if (in_window) ++metrics.gate_count;
  }
  const double validation_distance = window_mode
      ? std::max(validation_end_s - validation_start_s, 0.2)
      : stop_s;
  const double nominal_duration = std::max(
      validation_distance / std::max(target_speed, 0.2), 1.0);
  const double maximum_time = scenario.repeat
      ? 2.0 * nominal_duration + 60.0
      : 2.5 * nominal_duration + 120.0;
  const double plan_dt = std::clamp(
      environment_double("SIMP_VALIDATION_PLAN_DT").value_or(0.10),
      0.05, config.longitudinal.horizon);
  constexpr double execution_dt = 0.01;
  std::ofstream trajectory_output;
  if (const char* trajectory_path = std::getenv("SIMP_VALIDATION_TRAJECTORY_CSV")) {
    trajectory_output.open(trajectory_path);
    if (!trajectory_output) {
      throw std::runtime_error(std::string("cannot open trajectory output: ")
                               + trajectory_path);
    }
    trajectory_output
        << "time,x,y,body_yaw,motion_heading,beta,speed,acceleration,"
           "minimum_clearance,allocation_profile\n";
    trajectory_output << std::fixed << std::setprecision(9);
  }

  for (int plan_index = 0;
       metrics.simulated_time <= maximum_time && plan_index < external_max_plans;
       ++plan_index) {
    ++metrics.plans;
    const auto projection = reference.project(state.x, state.y, state.chi);
    if (std::getenv("SIMP_VALIDATION_VERBOSE") != nullptr && plan_index % 25 == 0) {
      std::cerr << "plan=" << plan_index << " t=" << metrics.simulated_time
                << " s=" << projection.s << " n=" << projection.n
                << " v=" << state.speed << " max_ms=" << metrics.maximum_plan_ms
                << "\n";
    }
    metrics.final_progress = projection.s;
    metrics.final_speed = state.speed;
    if (phase_index == 0 && !scenario.gates.empty() && previous_projection_s) {
      for (std::size_t i = 0; i < scenario.gates.size(); ++i) {
        if (gate_seen[i]) continue;
        const auto& gate = scenario.gates[i];
        if (*previous_projection_s <= gate.s && projection.s >= gate.s) {
          gate_seen[i] = true;
          const double allowance = 0.5 * gate.gap - 0.5 * config.vehicle.width
              - effective_hard_clearance_margin(state.speed, config.cost);
          if (std::abs(projection.n - gate.center) <= allowance + 0.10) {
            ++metrics.gate_passes;
          } else {
            metrics.reason = "GATE_ALIGNMENT_FAILURE";
            return metrics;
          }
        }
      }
    }
    previous_projection_s = projection.s;

    if (window_mode && projection.s >= validation_end_s - 0.05) {
      const bool gates_ok = metrics.gate_passes == metrics.gate_count;
      metrics.pass = metrics.safety_collisions == 0 && gates_ok;
      metrics.reason = metrics.pass ? "PASS_WINDOW" :
          (metrics.safety_collisions > 0 ? "EXECUTION_COLLISION"
                                         : "GATE_COUNT_FAILURE");
      return metrics;
    }

    if (scenario.repeat && projection.s >= reference.s_max() - 0.50) {
      metrics.pass = metrics.safety_collisions == 0;
      metrics.reason = metrics.pass ? "PASS_ONE_LAP" : "EXECUTION_COLLISION";
      return metrics;
    }
    const double terminal_goal_s = std::max(
        reference.s_max() - config.longitudinal.stop_target_offset, 0.0);
    const double terminal_goal_error = terminal_goal_s - projection.s;
    if (!scenario.repeat
        && std::abs(terminal_goal_error) <= config.terminal.longitudinal_tolerance
        && state.speed <= config.longitudinal.stop_speed_threshold) {
      if (phase_index + 1 < scenario.phases.size()) {
        ++phase_index;
        phase = transformed_next_phase(scenario, phase_index, x, y, body_yaw);
        phase_total_s = cumulative_s(phase.x, phase.y).back();
        stop_s = phase.switch_s >= 0.0 ? phase.switch_s
            : std::max(phase_total_s - scenario.terminal_margin, 0.0);
        phase = clip_phase(phase, stop_s);
        reference = make_reference(phase);
        planner = std::make_unique<PathVelocityPlanner>(config, reference, costmap);
        state.chi = motion_heading_from_body_yaw(body_yaw, phase.mode);
        state.speed = 0.0;
        state.acceleration = 0.0;
        state.motion_heading_rate = 0.0;
        previous_action = {};
        allocator_state = AllocatorInitialState{mode_offset(phase.mode), 0.0, 0.0, 0.0};
        previous_projection_s.reset();
        continue;
      }
      const bool gates_ok = metrics.gate_passes == metrics.gate_count;
      metrics.pass = metrics.safety_collisions == 0 && gates_ok;
      metrics.reason = metrics.pass ? "PASS" :
          (metrics.safety_collisions > 0 ? "EXECUTION_COLLISION" : "GATE_COUNT_FAILURE");
      return metrics;
    }

    const OrientedFootprintConfig footprint{
        scenario.footprint_circle_count, 0.20, 2.0 * kPi / 180.0};
    const auto planned = plan_with_pipeline(
        *planner, state, previous_action, phase.mode, target_speed,
        costmap, config, allocator_state, metrics, footprint);
    if (!planned) {
      metrics.reason = "NO_COLLISION_FREE_PLAN";
      return metrics;
    }
    if (planned->elapsed_ms > metrics.maximum_plan_ms) {
      metrics.maximum_plan_ms = planned->elapsed_ms;
      metrics.maximum_plan_time_s = metrics.simulated_time;
      metrics.maximum_plan_progress_m = projection.s;
      metrics.maximum_plan_replans = planned->replans_this_call;
    }

    const auto& allocation = planned->selection.allocation;
    const auto& actions = planned->result.trajectory.actions;
    const double start_x = x;
    const double start_y = y;
    BodyCommand final_command;
    for (int step = 0; step < static_cast<int>(std::llround(plan_dt / execution_dt)); ++step) {
      const double sample_time = (static_cast<double>(step) + 0.5) * execution_dt;
      const auto command = sample_body_command(allocation, actions, sample_time, execution_dt);
      integrate(x, y, body_yaw, command, execution_dt);
      const auto collision = check_oriented_allocation_collision(
          pose_allocation(x, y, body_yaw, command.planned_speed),
          costmap, config.vehicle, config.cost, footprint);
      metrics.minimum_clearance = std::min(metrics.minimum_clearance,
                                           collision.minimum_clearance);
      if (!collision.collision_free) {
        ++metrics.safety_collisions;
        metrics.reason = "EXECUTION_COLLISION";
        return metrics;
      }
      if (trajectory_output) {
        trajectory_output << metrics.simulated_time + execution_dt << ','
                          << x << ',' << y << ',' << body_yaw << ','
                          << command.motion_heading << ',' << command.beta << ','
                          << command.planned_speed << ','
                          << command.planned_acceleration << ','
                          << collision.minimum_clearance << ','
                          << static_cast<int>(planned->selection.profile) << '\n';
      }
      final_command = command;
      metrics.simulated_time += execution_dt;
    }

    const double traveled = std::hypot(x - start_x, y - start_y);
    if (traveled > 1.0e-8) {
      const double actual_chi = std::atan2(y - start_y, x - start_x);
      metrics.maximum_direction_execution_error_deg = std::max(
          metrics.maximum_direction_execution_error_deg,
          std::abs(wrap_angle(actual_chi - final_command.motion_heading))
              * 180.0 / kPi);
    }
    const double segment_dx = final_command.segment_end_x - final_command.segment_start_x;
    const double segment_dy = final_command.segment_end_y - final_command.segment_start_y;
    const double segment_norm = std::hypot(segment_dx, segment_dy);
    if (segment_norm > 1.0e-9 && std::isfinite(final_command.segment_start_x)) {
      const double cross = std::abs((x - final_command.segment_start_x) * segment_dy
          - (y - final_command.segment_start_y) * segment_dx) / segment_norm;
      metrics.maximum_lateral_execution_error = std::max(
          metrics.maximum_lateral_execution_error, cross);
    }

    state.x = x;
    state.y = y;
    state.chi = wrap_angle(body_yaw + final_command.beta);
    state.speed = final_command.planned_speed;
    state.acceleration = final_command.planned_acceleration;
    state.motion_heading_rate = final_command.motion_heading_rate;
    previous_action = {final_command.planned_jerk,
                       final_command.planned_heading_acceleration};
    allocator_state = AllocatorInitialState{
        final_command.beta, final_command.beta_rate,
        final_command.yaw_rate, final_command.yaw_acceleration};
  }
  metrics.reason = "TIMEOUT";
  return metrics;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc < 3 || argc > 4) {
      std::cerr << "usage: closed_loop_validator <scenario.svd> <target_speed> [max_plans]\n";
      return 2;
    }
    const auto scenario = load_scenario(argv[1]);
    const double speed = std::stod(argv[2]);
    const int max_plans = argc == 4 ? std::stoi(argv[3]) : 200000;
    const auto metrics = run_scenario(scenario, speed, max_plans);
    std::cout << std::fixed << std::setprecision(9)
              << scenario.name << ',' << speed << ','
              << (metrics.pass ? 1 : 0) << ',' << metrics.reason << ','
              << metrics.simulated_time << ',' << metrics.plans << ','
              << metrics.planning_failures << ','
              << metrics.primary_allocation_collisions << ','
              << metrics.minimum_vy_successes << ','
              << metrics.adaptive_path_replans << ','
              << metrics.adaptive_path_replan_successes << ','
              << metrics.bottleneck_limited_plans << ','
              << metrics.safety_collisions << ','
              << metrics.gate_passes << ',' << metrics.gate_count << ','
              << metrics.minimum_clearance << ','
              << metrics.maximum_plan_ms << ','
              << metrics.maximum_plan_time_s << ','
              << metrics.maximum_plan_progress_m << ','
              << metrics.maximum_plan_replans << ','
              << metrics.maximum_lateral_execution_error << ','
              << metrics.maximum_direction_execution_error_deg << ','
              << metrics.final_progress << ',' << metrics.final_speed << '\n';
    return metrics.pass ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "validator error: " << error.what() << '\n';
    return 3;
  }
}
