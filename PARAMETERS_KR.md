# SIMP Planner 현재 파라미터 기준서

이 문서는 현재 소스 코드에 설정된 Planner, Motion Allocation, ROS2 runtime,
planar simulator, scenario 도구의 기본 파라미터를 한곳에 정리한다.

- 기준 소스 revision: `7b48c30`
- 기준일: 2026-08-11
- 단위계: 별도 표기가 없으면 SI 단위, 내부 각도는 rad
- 이 문서는 현재 동작을 기록할 뿐 파라미터를 변경하지 않는다.
- test fixture와 validation 실행별 override는 현재 runtime 기본값이 아니므로 제외한다.
- `활성`은 현재 실행 경로에서 읽히는 값, `조건부`는 특정 상태에서만 읽히는 값,
  `미사용`은 선언되어 있지만 현재 production 코드의 판단에 사용되지 않는 값이다.

## 1. 핵심 파생값

| 항목 | 현재값 | 산출 근거 |
|---|---:|---|
| 시간 Trajectory horizon | 4.0 s | `LongitudinalConfig::horizon` |
| Trajectory knot 간격 | 0.10 s | ROS `trajectory_knot_dt_sec`가 내부 `dt`를 덮어씀 |
| Trajectory 구성 | 40 interval / 41 sample | `round(4.0 / 0.1) + 1` |
| 실행 명령 간격 | 0.01 s | `command_frequency_hz=100 Hz` |
| knot 사이 동적 검사 | 기본 10 substep | `0.10 / 0.01` |
| 횡방향 목표 | -8.0~+8.0 m, 0.25 m 간격 | 총 65개 |
| 신규 횡 transition 길이 | 3.0~50.0 m | 속도·offset·곡률에 따라 동적 결정; 기존 maneuver 재계획 시 남은 길이는 0.10 m까지 가능 |
| 공간 경로 sampling | 0.25 m | `spatial_ds` |
| 기본 최소 spatial preview | 32.0 m | adaptive bottleneck preview |
| 기본 preview 후보 preliminary extent | 45.2 m 이상 | `1.35 × 32.0 + 2.0` |
| local reference window | 뒤 5.0 m / 앞 45.0 m | Scenario/Track provider 기본값 |
| 진행 속도 범위 | 0.0~5.8 m/s | Body 축별 속도 제한이 아님 |
| 진행 가속도 범위 | -2.0~+1.2 m/s² | nominal 운용은 -1.0~+0.8 m/s² |
| 진행 jerk 상한 | 3.0 m/s³ | nominal comfort 상한은 0.8 m/s³ |
| 차량 크기 | 3.0 m × 2.0 m | 길이 × 폭 |
| final oriented footprint | 3 circles | translation 0.20 m, yaw 2° 이하로 보간 |
| planning 시작 상한 | 10 Hz | scheduler polling은 별도 100 Hz |

`path_ahead_length=45 m`는 local reference의 전방 길이이다. 뒤쪽 5 m도 함께
전달되므로 nominal 전체 window는 약 50 m이며, 경로 끝과 closed-loop slicing에
따라 실제 길이는 달라질 수 있다. `horizon=4 s`는 시간 Trajectory 길이이므로
reference window와는 다른 개념이다.

## 2. 설정 계층과 적용 우선순위

현재 별도 YAML 파일은 없다. 설정값은 다음 순서로 결정된다.

1. launch argument가 해당 ROS parameter를 덮어쓴다.
2. launch에서 전달하지 않은 ROS parameter는 각 노드의 선언 기본값을 사용한다.
3. C++ `EnvConfig`는 대부분 `core.hpp`의 컴파일된 기본값을 사용한다.
4. Planner ROS node가 `EnvConfig`에서 바꾸는 값은 현재 다음 두 개뿐이다.
   - `longitudinal.dt = trajectory_knot_dt_sec`
   - `longitudinal.execution_dt = 1 / command_frequency_hz`
5. Oriented footprint의 circle 수와 보간 간격은 별도 ROS parameter로 전달된다.

주요 원본 파일:

- `simp_planner/simp_planner_cpp/include/simp_planner/core.hpp`
- `simp_planner/simp_planner_cpp/include/simp_planner/runtime.hpp`
- `simp_planner/simp_planner_cpp/src/core.cpp`
- `simp_planner/simp_planner_cpp/src/planner_node.cpp`
- `planar_velocity_sim/planar_velocity_sim/planar_velocity_sim_node.py`
- `simp_planner_tools/launch/simulation.launch.py`
- `simp_planner_tools/simp_planner_tools/scenario_definition.py`

## 3. C++ Planner 내부 기본값

### 3.1 VehicleConfig

| 파라미터 | 기본값 | 단위 | 상태 및 의미 |
|---|---:|---:|---|
| `length` | 3.0 | m | 활성, 차량 길이 |
| `width` | 2.0 | m | 활성, 차량 폭 |
| `footprint_margin` | 0.0 | m | 활성, collision footprint 추가 팽창량 |

현재 파생 footprint 값:

- 공간 ranking용 circumscribed radius: `sqrt(1.5² + 1.0²) = 1.8028 m`
- 공간 hard coarse screen 반경: `width / 2 = 1.0 m`
- final 3-circle의 circle 간 segment 길이: `3.0 / 3 = 1.0 m`
- final circle 반경: `sqrt(0.5² + 1.0²) = 1.1180 m`

### 3.2 ConstraintConfig

이 제약은 기존 `(V, a, chi)`와 공간 경로 기준 제약이다. Motion Allocation 이후
Body `vx`, `vy` 제약이 아니다.

| 파라미터 | 기본값 | 단위 | 상태 및 의미 |
|---|---:|---:|---|
| `v_max` | 5.8 | m/s | 활성, 진행 속도 상한 |
| `v_min` | 0.0 | m/s | 활성, 진행 속도 하한 |
| `a_max` | 1.2 | m/s² | 활성, 진행 가속도 상한 |
| `a_min` | -2.0 | m/s² | 활성, 진행 감속도 하한 |
| `jerk_max` | 3.0 | m/s³ | 활성, 진행 jerk 절댓값 상한 |
| `heading_rate_max` | 50°/s (`0.872665`) | rad/s | 활성, 이동방향 `chi_dot` 상한 |
| `heading_accel_max` | 90°/s² (`1.570796`) | rad/s² | 활성, `chi_ddot` 상한 |
| `a_lat_max` | 2.8 | m/s² | 활성, 경로 기준 횡가속도 상한 |
| `lateral_jerk_max` | 3.0 | m/s³ | 활성, 경로 기준 횡 jerk 상한 |
| `curvature_max` | 0.20 | 1/m | 활성, 곡률 상한; 대응 반경 5.0 m |
| `v_eff_min` | 0.20 | m/s | 활성, 저속 곡률·transition 계산 regularization |

동적 validity 비교에는 현재 다음 tolerance가 내부 하드코딩되어 있다.

- 속도, 가속도, 횡가속도, 횡 jerk, 종 jerk: `0.03`의 해당 단위
- heading rate/acceleration: `0.5°`의 해당 단위
- curvature: `1e-9 1/m`

### 3.3 LateralPathConfig

| 파라미터 | 기본값 | 단위 | 상태 및 의미 |
|---|---:|---:|---|
| `n_targets` | -8.0~+8.0, step 0.25 | m | 활성, Frenet lateral target 65개 |
| `min_length` | 3.0 | m | 활성, 새로 생성하는 nominal 횡 transition 최소 길이 |
| `max_length` | 50.0 | m | 활성, 횡 transition 최대 길이; 기존 maneuver 재계획 시 남은 segment는 0.10 m까지 허용 |
| `spatial_ds` | 0.25 | m | 활성, 공간 후보 sampling 간격 |
| `dynamic_margin` | 1.08 | ratio | 활성, 동적 최소 transition 길이 배율 |
| `preview_extra` | 2.0 | m | 활성, 후보 preliminary extent 추가량 |
| `normal_shortlist_size` | 9 | count | 활성, 일반 첫 time-rollout batch |
| `terminal_shortlist_size` | 3 | count | 활성, terminal 첫 time-rollout batch |
| `normal_fallback_batch_size` | 4 | count | 활성, 첫 batch 실패 후 batch 크기 |
| `target_continuity_weight` | 3.0 | cost | 조건부, 이전 lateral target 유지 비용 |
| `reference_center_lock_enabled` | true | bool | 활성, 완전히 안전한 center path 고정 |
| `reference_center_lock_obstacle_cost_tolerance` | 1e-12 | cost | 활성, center-lock obstacle cost 허용치 |
| `short_path_fallback_enabled` | true | bool | 활성, 정지 가능한 짧은 path fallback |
| `short_path_collision_buffer` | 1.0 | m | 활성, 첫 collision 전 buffer |
| `short_path_stop_reserve` | 0.75 | m | 활성, stopping distance 이후 reserve |
| `short_path_max_length` | 20.0 | m | 활성, short path 길이 상한 |
| `allocation_path_replan_max_attempts` | 16 | count | 활성, Allocation 충돌 뒤 path 재선택 상한 |

공간 preview 입력은 현재 다음 식으로 결정된다.

```text
max(0.20 m,
    1.10 × horizon 안의 도달가능거리,
    lateral.min_length,
    adaptive_replan.minimum_spatial_preview)
```

현재 `V <= 5.8 m/s`, `horizon=4 s`에서 `1.10 × 23.2 = 25.52 m`이므로
`minimum_spatial_preview=32 m`가 기본적으로 지배한다.

새 후보의 preliminary extent는 다음과 같다.

```text
max(1.35 × preview + preview_extra,
    남은 lateral transition 길이 + 1.0 m)
```

### 3.4 LongitudinalConfig

| 파라미터 | 기본값 | 단위 | 상태 및 의미 |
|---|---:|---:|---|
| `horizon` | 4.0 | s | 활성, 시간 Trajectory 길이 |
| `dt` | 0.10 | s | 활성/ROS override, knot 간격 |
| `execution_dt` | 0.01 | s | 활성/ROS derived, knot 내부 검사·실행 간격 |
| `speed_time_constant` | 1.6 | s | 활성, 속도 오차→목표가속도 응답 |
| `acceleration_response_time` | 0.9 | s | 활성, 목표가속도→jerk 응답 |
| `cruise_acceleration` | 0.8 | m/s² | 활성, nominal 가속도 상한 |
| `service_deceleration` | 1.0 | m/s² | 활성, nominal 감속도 크기 |
| `comfort_jerk` | 0.8 | m/s³ | 활성, nominal jerk 상한 |
| `terminal_position_gain_far` | 0.25 | - | 미사용 |
| `terminal_speed_gain` | 1.20 | - | 미사용 |
| `terminal_acceleration_response_time` | 1.00 | s | 활성, terminal jerk feedback 응답 |
| `terminal_braking_deceleration` | 1.60 | m/s² | 활성, terminal 감속 명령 상한 |
| `terminal_braking_curve_deceleration` | 0.85 | m/s² | 활성, terminal braking speed envelope |
| `terminal_jerk_limit` | 1.50 | m/s³ | 활성, terminal nominal jerk 상한 |
| `terminal_initial_jerk_continuity_weight` | 2.0 | cost | 활성, 첫 terminal jerk 연속성 가중치 |
| `terminal_braking_distance_buffer` | 0.0 | m | 활성, stopping distance 추가 buffer |
| `terminal_capture_speed` | 0.03 | m/s | 활성, terminal 정지 포착 속도 |
| `curvature_lookahead_time` | 2.0 | s | 활성, curve-speed lookahead 시간 |
| `curvature_lookahead_min` | 8.0 | m | 활성, curve-speed 최소 lookahead 거리 |
| `lateral_jerk_speed_margin` | 0.30 | ratio | 활성, 횡 jerk 기반 speed limit 안전 배율 |
| `stop_preview_time` | 0.30 | s | 미사용 |
| `stop_target_offset` | 0.06 | m | 활성, 실제 path end 전 정지 목표 offset |
| `stop_distance_margin` | 0.05 | m | 미사용 |
| `stop_position_tolerance` | 0.05 | m | 활성, terminal capture/회복 위치 허용치 |
| `stop_speed_threshold` | 0.03 | m/s | 활성, stationary command 판단 |
| `motion_started_speed` | 0.20 | m/s | 미사용 |
| `motion_started_progress` | 0.50 | m | 미사용 |
| `obstacle_speed_backoff_factor` | 0.50 | ratio | 활성, 실패 후 다음 target speed 배율 |
| `obstacle_speed_recovery_step` | 0.25 | m/s | 활성, feasible speed cap 회복 step |
| `obstacle_speed_minimum` | 0.50 | m/s | 활성, speed trial 최저값 |
| `obstacle_speed_max_attempts` | 3 | count | 활성, 한 plan의 최대 speed trial 수 |

현재 정상 주행에서 실제 제어 상한은 다음과 같이 더 낮다.

```text
acceleration: min(a_max=1.2, cruise_acceleration=0.8) = +0.8 m/s²
deceleration: min(|a_min|=2.0, service_deceleration=1.0) = -1.0 m/s²
jerk:         min(jerk_max=3.0, comfort_jerk=0.8) = ±0.8 m/s³
```

### 3.5 CostConfig

`w_*`는 서로 정규화되지 않은 tuning weight다.

| 파라미터 | 기본값 | 상태 및 의미 |
|---|---:|---|
| `w_speed` | 5.0 | 활성, speed tracking cost |
| `w_low_speed` | 10.0 | 활성, 비-terminal에서 1 m/s 미만 penalty |
| `w_progress` | 7.0 | 활성, horizon progress 부족 penalty |
| `w_accel` | 0.35 | 활성, acceleration cost |
| `w_jerk` | 0.50 | 활성, longitudinal jerk cost |
| `w_heading_accel` | 0.05 | 활성, motion-heading acceleration cost |
| `w_lateral_jerk` | 0.10 | 활성, path-frame lateral jerk cost |
| `w_lateral_offset` | 0.08 | 활성, 전체 lateral offset cost |
| `w_terminal_offset` | 0.12 | 활성, 실제 path end lateral offset cost |
| `w_offset_overshoot` | 2.0 | 활성, lateral target 범위 overshoot cost |
| `w_curvature` | 8.0 | 활성, curvature cost |
| `w_curvature_rate` | 1.2 | 활성, curvature 공간미분 cost |
| `w_real_end_offset` | 90.0 | 미사용 |
| `w_real_end_heading` | 45.0 | 미사용 |
| `w_obstacle` | 110.0 | 활성, 평균 soft-clearance 부족 cost |
| `w_min_clearance` | 380.0 | 활성, 최대 clearance 부족 cost |
| `w_collision` | 200000 | 활성, collision sample penalty |
| `obstacle_soft_margin` | 0.10 m | 활성, soft-clearance 기본 추가량 |
| `obstacle_time_headway` | 0.20 s | 활성, 속도 비례 soft-clearance |
| `obstacle_soft_margin_max` | 1.25 m | 활성, soft-clearance 상한 |
| `hard_clearance_margin_low_speed` | 0.10 m | 미사용 |
| `hard_clearance_margin` | 0.10 m | 활성, 모든 속도에서 동일한 hard margin |
| `hard_clearance_full_speed` | 2.0 m/s | 미사용 |

현재 soft-clearance 목표는 다음 식이다.

```text
min(hard_clearance_margin
      + obstacle_soft_margin
      + obstacle_time_headway × speed,
    obstacle_soft_margin_max)
```

`effective_hard_clearance_margin()`은 속도를 사용하지 않으므로 low-speed/full-speed
두 필드는 현재 효과가 없다.

### 3.6 TerminalConstraintConfig

| 파라미터 | 기본값 | 단위 | 상태 및 의미 |
|---|---:|---:|---|
| `activation_margin` | 3.0 | m | 활성, stopping distance 추가량 |
| `minimum_activation_distance` | 1.0 | m | 활성, terminal 제약 최소 활성 거리 |
| `safe_region_planning_buffer` | 12.0 | m | 활성, terminal safe-region 조기 탐색 buffer |
| `safe_region_settle_distance` | 4.0 | m | 활성, endpoint 전 lateral settle 거리 |
| `longitudinal_tolerance` | 0.20 | m | 활성, terminal 위치 허용치 |
| `speed_tolerance` | 0.03 | m/s | 활성, auxiliary terminal stop 판정 |
| `acceleration_tolerance` | 0.10 | m/s² | 활성, terminal 가속도 허용치 |
| `feedback_feasibility_horizon` | 15.0 | s | 활성, 4 s 이후 auxiliary stop rollout |
| `lateral_tolerance` | 0.20 | m | 미사용 |
| `activation_distance` | 0.0 | m | 미사용 |
| `position_profile_blend` | 0.0 | ratio | 조건부, terminal position-only profile blend |
| `hold_return_enabled` | false | bool | 미사용 |
| `hold_clearance_margin` | 0.0 | m | 미사용 |

Terminal constraint 활성거리는 다음 값 중 큰 값이다.

```text
max(minimum_activation_distance,
    jerk-limited stopping distance
    + terminal_braking_distance_buffer
    + activation_margin)
```

### 3.7 SimulationConfig

이 구조체 이름은 Planner 내부 path-end 처리 설정이며 planar simulator 설정과 다르다.

| 파라미터 | 기본값 | 단위 | 상태 및 의미 |
|---|---:|---:|---|
| `path_end_margin` | 0.0 | m | 활성, authored path 끝에서 제외할 거리 |
| `virtual_extension_min` | 12.0 | m | 활성, path-end 가상 연장 최소 길이 |
| `virtual_extension_max` | 55.0 | m | 활성, preliminary 가상 연장 상한 |
| `virtual_extension_blend_length` | 8.0 | m | 활성, curvature 연장 blend 길이 |
| `local_projection_back` | 2.0 | m | 미사용 Config 필드 |
| `local_projection_forward` | 15.0 | m | 미사용 Config 필드 |

### 3.8 AdaptiveReplanConfig

| 파라미터 | 기본값 | 단위 | 상태 및 의미 |
|---|---:|---:|---|
| `enabled` | true | bool | 활성, bottleneck 탐색 및 최소 preview |
| `minimum_spatial_preview` | 32.0 | m | 활성 |
| `bottleneck_trigger_extra` | 1.50 | m | 활성, bottleneck 진입 여유 |
| `bottleneck_release_extra` | 2.00 | m | 활성, bottleneck 해제 여유 |
| `bottleneck_post_buffer` | 8.0 | m | 활성, bottleneck 뒤 decision buffer |
| `bottleneck_minimum_decision_length` | 8.0 | m | 활성, 최소 decision 길이 |
| `bottleneck_release_samples` | 3 | count | 활성, 해제에 필요한 연속 sample |
| `failure_soft_clearance_extra` | 0.20 | m | 미사용 |
| `failure_sigmoid_slope` | 5.0 | - | 미사용 |
| `speed_reference` | 1.0 | m/s | 미사용 |
| `failure_memory_decay` | 0.75 | ratio | 미사용 |
| `clearance_weight_gain` | 2.50 | ratio | 미사용 |
| `memory_weight_gain` | 1.50 | ratio | 미사용 |
| `clearance_speed_interaction_gain` | 1.00 | ratio | 미사용 |
| `continuity_decay_gain` | 2.00 | ratio | 미사용 |
| `reference_decay_gain` | 1.20 | ratio | 미사용 |
| `minimum_continuity_scale` | 0.20 | ratio | 미사용 |
| `minimum_reference_scale` | 0.35 | ratio | 미사용 |
| `failed_target_weight` | 80.0 | cost | dormant allocation-failure branch |
| `failed_target_sigma` | 0.75 | m | dormant allocation-failure branch |
| `maneuver_target_tolerance` | 0.30 | m | 활성, latched maneuver 완료 판단 |
| `maneuver_release_progress_margin` | 0.50 | m | 활성, maneuver release 판단 |

Allocation-failure 기반 weight scheduler는 현재 production 경로에서
`adaptive_replan_active=false`로 고정되어 있다. 따라서 failure/memory 계열 값은
`enabled=true`여도 적용되지 않는다.

## 4. Motion Allocation 설정

배열 순서는 항상 `Forward, Reverse, Left, Right`다.

### 4.1 Drive mode와 beta 중심

| 값 | Mode | `beta_center` | 일반적인 Body 속도 방향 |
|---:|---|---:|---|
| 0 | Forward | 0° | `vx > 0` |
| 1 | Reverse | 180° | `vx < 0` |
| 2 | Left | +90° | `vy > 0` |
| 3 | Right | -90° | `vy < 0` |

### 4.2 AllocationLimits 기본값

| 파라미터 | 기본값 | 단위 | 상태 및 의미 |
|---|---|---:|---|
| `beta_max_deviation` | [45°, 40°, 25°, 25°] | rad | 활성, mode 중심 대비 envelope |
| `body_yaw_fraction` | [0.20, 0.25, 0.15, 0.15] | ratio | 활성, `chi_dot`의 Body yaw 분담 |
| `beta_return_gain` | [0.35, 0.40, 0.45, 0.45] | 1/s | 활성, beta 중심 복귀 gain |
| `beta_rate_max` | 40°/s | rad/s | 활성 |
| `beta_accel_max` | 180°/s² | rad/s² | 활성 |
| `beta_filter_tau` | 0.80 | s | 미사용 |
| `yaw_rate_max` | 50°/s | rad/s | 활성, Body yaw rate 상한 |
| `yaw_accel_max` | 90°/s² | rad/s² | 활성 |
| `yaw_jerk_max` | 400°/s³ | rad/s³ | 활성 |
| `vx_accel_max` | 3.2 | m/s² | 선언됨, 현재 제약 검사 미사용 |
| `vy_accel_max` | 3.2 | m/s² | 선언됨, 현재 제약 검사 미사용 |
| `vx_jerk_max` | 9.0 | m/s³ | 선언됨, 현재 제약 검사 미사용 |
| `vy_jerk_max` | 9.0 | m/s³ | 선언됨, 현재 제약 검사 미사용 |
| `stop_speed_threshold` | 0.03 | m/s | mode-change flag 기록에만 사용 |
| `allocation_active_speed` | 0.30 | m/s | 활성, 저속 allocation smoothing 기준 |
| `path_replay_tolerance` | 0.30 | m | 미사용 |
| `psi_replay_tolerance` | 1° | rad | 미사용 |

Allocation은 현재 다음 결과를 이미 계산한다.

```text
beta, beta_rate, beta_acceleration, psi,
vx, vy, yaw_rate, yaw_acceleration, yaw_jerk,
vx_acceleration, vy_acceleration, vx_jerk, vy_jerk
```

Body acceleration/jerk 배열은 계산되지만 위의 `3.2`, `9.0` 값과 비교하여
후보를 reject하지 않는다.

### 4.3 Allocation profile override

| Profile | Forward/Reverse `body_yaw_fraction` | Forward/Reverse `beta_return_gain` | Forward/Reverse `beta_max_deviation` | 자동 사용 |
|---|---|---|---|---|
| `LateralPriority` | 0.20 / 0.25 | 0.35 / 0.40 | 45° / 40° | 1순위 |
| `Balanced` | 0.50 / 0.55 | 0.65 / 0.70 | 35° / 30° | 사용 안 함 |
| `YawPriority` | 0.75 / 0.80 | 0.90 / 0.95 | 25° / 20° | 사용 안 함 |
| `MinimumVy` | 0.92 / 0.92 | 1.20 / 1.20 | 12° / 12° | Forward/Reverse 2순위 |

Left/Right 값은 모든 profile에서 기본값을 유지한다. Left/Right crab trajectory는
`MinimumVy`를 재시도하지 않고 `LateralPriority`만 평가한다.

### 4.4 OrientedFootprintConfig

| 파라미터 | 기본값 | 단위 | 상태 |
|---|---:|---:|---|
| `circle_count` | 3 | count | 활성/ROS override |
| `maximum_translation_step` | 0.20 | m | 활성/ROS override |
| `maximum_yaw_step` | 2° (`0.0349066`) | rad | 활성/ROS override |

## 5. 주요 비노출 알고리즘 상수

다음 값은 ROS parameter나 Config 필드가 아니지만 현재 동작에 영향을 주는 주요
하드코딩 tuning 값이다. 단순 floating-point guard는 제외한다.

| 항목 | 현재값 |
|---|---|
| 최소 lateral length profile 계수 | `c2=7.5131884044`, `c3=52.5` |
| 초기 spatial boundary clamp | `n2 ±0.35`, `desired_kappa_l ±0.10`, `n3 ±0.25` |
| 곡률 초과 시 경로 길이 재생성 | 최대 8회, 길이 scale 최소 1.15 |
| extended transition 조건 | `abs(delta_n) >= 0.5 m`, `max(1.35×nominal, nominal+3 m)` |
| long transition | `max(1.70×nominal, nominal+6 m)` |
| curve-speed lookahead scan | 61 points |
| terminal jerk ramp time cap | 2.0 s |
| terminal goal footprint sampling | 0.05~0.20 m, 최소 3개 |
| low-speed cost 기준 | 1.0 m/s 미만 |
| lateral target hint 최소 절댓값 | 0.5 m |
| speed backoff factor clamp | 0.20~0.95 |
| speed recovery 최소 step | 0.05 m/s |
| active latched maneuver 우선 비용 | -1e6 |
| feasible speed-cap probe 간격 | `round(1.8 / dt)`; 현재 18회 |

## 6. Planner ROS runtime 파라미터

`planner_node_cpp`가 ROS에 노출하는 파라미터는 아래 16개뿐이다.

| ROS parameter | 기본값 | 단위 | 적용 |
|---|---:|---:|---|
| `max_planning_frequency_hz` | 10.0 | Hz | planning 시작 빈도 상한; 0.100 s 간격 |
| `maximum_plan_age_sec` | 0.50 | s | active plan watchdog age |
| `input_coalescing_sec` | 0.005 | s | non-urgent 입력 병합 시간 |
| `planning_handover_min_lead_sec` | 0.12 | s | handover lead 하한 |
| `planning_handover_initial_lead_sec` | 0.15 | s | 초기 handover lead |
| `planning_handover_max_lead_sec` | 0.60 | s | handover lead 상한 |
| `planning_handover_margin_sec` | 0.03 | s | compute estimate margin |
| `planning_handover_scale` | 1.20 | ratio | P95 compute time 배율 |
| `trajectory_knot_dt_sec` | 0.10 | s | `longitudinal.dt` override |
| `command_frequency_hz` | 100.0 | Hz | command timer; `command_dt=0.01 s` |
| `planning_scheduler_frequency_hz` | 100.0 | Hz | scheduler polling; 0.01 s |
| `mode_change_stop_speed_mps` | 0.03 | m/s | Planner mode 전환 정지 기준 |
| `mode_command_period_sec` | 0.25 | s | mode command 재송신 간격; 4 Hz |
| `oriented_footprint_circle_count` | 3 | count | final footprint circle 수 |
| `oriented_footprint_translation_step_m` | 0.20 | m | swept translation step |
| `oriented_footprint_yaw_step_deg` | 2.0 | deg | swept yaw step |

Timing 관련 hidden runtime 값:

| 항목 | 현재값 |
|---|---:|
| handover compute quantile | 0.95 |
| handover history size | 64 samples |
| recent peak window | 최근 최대 8 samples |
| late compute 추가 scale | 1.35 |
| lead 하향 변화 상한 | record당 0.01 s |
| status JSON `deadline_ms` | 100 ms, 진단 표시만 하며 강제 중단 아님 |
| terminal active-plan finish 억제 | remaining ≤0.20 m 및 speed ≤0.25 m/s |
| worker executor thread 수 | 3 |

Runtime safety stop의 현재 유효값:

```text
deceleration = min(abs(a_min=−2.0), service_deceleration=1.0) = 1.0 m/s²
jerk         = min(jerk_max=3.0, comfort_jerk=0.8) = 0.8 m/s³
```

Terminal hold latch의 default constructor 값:

| 항목 | 값 |
|---|---:|
| longitudinal capture | 0.20 m |
| speed capture | 0.08 m/s |
| acceleration capture | 1.00 m/s² |
| goal change/release distance | 0.30 m |

Runtime의 motion direction 판단 threshold는 별도 하드코딩 `0.03 m/s`다.
Rolling local-reference를 soft update로 판단하는 기준은 위치 0.50 m, heading 15°,
curvature 0.05 1/m 이하다.

## 7. Planar velocity simulator

| ROS parameter | 기본값 | 단위/값 | 의미 |
|---|---:|---|---|
| `update_rate_hz` | 100.0 | Hz | 적분 및 odom publish |
| `mode_state_rate_hz` | 20.0 | Hz | mode-state 주기 publish |
| `initial_x` | 0.0 | m | 초기 위치 |
| `initial_y` | 0.0 | m | 초기 위치 |
| `initial_yaw` | 0.0 | rad | 초기 Body yaw |
| `initial_drive_mode` | 0 | Forward | 초기 mode |
| `mode_transition_duration_sec` | 2.0 | s | 차량측 mode dwell |
| `mode_change_speed_threshold` | 0.03 | m/s | simulator mode command 허용 속도 |
| `cmd_topic` | `/cmd_vel` | topic | Body velocity 명령 입력 |
| `odom_topic` | `/odom` | topic | odometry 출력 |
| `mode_command_topic` | `/vehicle/drive_mode_command` | topic | mode 명령 입력 |
| `mode_state_topic` | `/vehicle/drive_mode_state` | topic | mode 상태 출력 |
| `odom_frame` | `odom` | frame | odom header frame |
| `base_frame` | `base_link` | frame | child/body frame |

Simulator 특성:

- 실제 clock `dt`를 사용하는 midpoint kinematic integration이다.
- transition 중에는 `(vx, vy, yaw_rate)=(0,0,0)`을 강제한다.
- transition 밖에서는 명령을 그대로 적용한다.
- velocity, acceleration, jerk, yaw-rate saturation과 actuator lag가 없다.
- command timeout이 없어 새 명령이 없으면 마지막 명령을 유지한다.
- mode 전환 정지 판정은 `hypot(vx,vy)`만 사용하며 yaw-rate는 검사하지 않는다.

Planner의 `mode_change_stop_speed_mps`와 simulator의
`mode_change_speed_threshold`는 서로 다른 파라미터다. launch가 두 값을 연결하지
않으며 현재 기본값만 우연히 모두 `0.03 m/s`다.

## 8. 메인 simulation launch의 유효 기본값

`simp_planner_tools/launch/simulation.launch.py` 기준이다.

| Launch argument | 기본값 | 유효 동작 |
|---|---:|---|
| `scenario` | `stadium` | 시나리오 선택 |
| `target_speed` | -1.0 | 음수 sentinel, scenario 기본 cruise speed 사용 |
| `path_update_distance` | 1.0 m | local reference 재발행 거리 |
| `mode_transition_duration_sec` | 2.0 s | simulator mode transition |
| `command_frequency_hz` | 100.0 Hz | Planner command frequency만 override |
| `oriented_footprint_circle_count` | 0 | sentinel, scenario 권장값 사용 |
| `costmap_resolution` | -1.0 | sentinel, scenario 권장값 사용 |
| `oriented_footprint_translation_step_m` | 0.20 m | Planner final swept check |
| `oriented_footprint_yaw_step_deg` | 2.0° | Planner final swept check |
| `save_period` | 10.0 s | debug snapshot 저장 주기 |
| `debug_output_dir` | `/home/sum/Desktop/simp_planner/simp_planner_debug` | debug root |

기본 `stadium` 실행에서 sentinel을 해석한 최종 주요값:

| 항목 | 유효값 |
|---|---:|
| target speed | 2.0 m/s |
| costmap resolution | 0.20 m/cell |
| footprint circles | 3 |
| command rate | 100 Hz |
| simulator update rate | 100 Hz; launch에서 override하지 않음 |
| mode transition | 2.0 s |

초기 simulator pose와 mode는 첫 phase의 첫 path sample에서 정한다.

```text
initial_x          = first_path.x[0]
initial_y          = first_path.y[0]
initial_drive_mode = first_path.mode[0]
initial_body_yaw   = first_path.yaw[0] - beta_center(mode)
```

## 9. Scenario Manager 파라미터

| ROS parameter | 기본값 | 단위/값 | 상태 및 의미 |
|---|---:|---|---|
| `scenario` | `stadium` | name | launch에서 선택 |
| `target_speed` | -1.0 | m/s | 음수면 phase별 기본 속도 |
| `frame_id` | `odom` | frame | path/costmap frame |
| `path_back_length` | 5.0 | m | local reference 뒤쪽 길이 |
| `path_ahead_length` | 45.0 | m | local reference 앞쪽 길이 |
| `path_update_distance` | 1.0 | m | reference 재발행 이동거리 |
| `costmap_resolution` | -1.0 | m/cell | sentinel, scenario 값 사용 |
| `costmap_margin` | 10.0 | m | path/obstacle bounds 외곽 여유 |
| `stop_speed_threshold` | 0.03 | m/s | phase 종료 정지 판정 |
| `terminal_capture_distance` | 0.20 | m | phase stop 위치 허용치 |
| `projection_search_back` | 20 | segment | local projection 뒤 검색 |
| `projection_search_forward` | 80 | segment | local projection 앞 검색 |
| `projection_fallback_distance` | 3.0 | m | full search 전환 거리 |

추가 하드코딩 동작:

- heartbeat: 0.5 s
- odom 수신 전 static input 재발행: 최대 10회
- 일반 phase 실행 끝: `path.total_length - terminal_margin`
- mode-change phase 실행 끝: 지정된 `switch_s`
- phase 전환 포착: 위치오차 ≤0.20 m이고 speed ≤0.03 m/s

## 10. 시나리오별 현재 설정

공통 scenario 기본값:

| 항목 | 값 |
|---|---:|
| `terminal_margin` | 3.0 m |
| `stop_request_distance` | 3.0 m |
| `repeat` | false |
| `costmap_resolution` | 0.20 m/cell |
| `footprint_circle_count` | 3 |

아래 길이는 CSV 좌표와 현재 `ScenarioPath` segment 합산으로 얻은 파생값이다.
`실행 stop s`는 반복 경로가 아닌 경우 terminal margin 또는 mode switch를 적용한 값이다.

| Scenario | Authored 길이 / 실행 stop s | 기본 속도 | Mode | 장애물/게이트 | Costmap / circles |
|---|---|---:|---|---:|---|
| `stadium` | 154.832 m / closed-loop | 2.0 | Forward | 0 / 0 | 0.20 / 3 |
| `crab_switch` | 60→18 m; 43→40 m | 1.5 / 1.0 | Forward→Left | 0 / 0 | 0.20 / 3 |
| `reverse_switch` | 60→18 m; 43→40 m | 1.5 / 1.0 | Forward→Reverse | 0 / 0 | 0.20 / 3 |
| `s_curve` | 64→61 m | 2.0 | Forward | 0 / 0 | 0.20 / 3 |
| `s_curve_obstacles` | 104→101 m | 1.2 | Forward | 3 / 0 | 0.20 / 3 |
| `obstacle_avoidance` | 80→77 m | 1.5 | Forward | 1 / 0 | 0.20 / 3 |
| `terminal_safe_region` | 80→77 m | 1.0 | Forward | 1 / 0 | 0.20 / 3 |
| `alternating_gate_corridor` | 190→187 m | 1.5 | Forward | 10 / 5 | 0.20 / 3 |
| `curved_gate_maze` | 192.949→189.949 m | 1.6 | Forward | 10 / 5 | 0.20 / 3 |
| `narrow_22m_stop_corridor` | 50→47 m | 0.8 | Forward | 2 / 1 | 0.20 / 3 |
| `narrow_28m_corridor` | 130.007→127.007 m | 0.6 | Forward | 2 / 1 | 0.20 / 3 |
| `narrow_offset_corridor` | 130.007→127.007 m | 1.0 | Forward | 2 / 1 | 0.05 / 16 |
| `winding_obstacle_course` | 349.928→346.928 m | 1.8 | Forward | 14 / 5 | 0.20 / 3 |

Scenario alias:

- `narrow_10pct_corridor` → `narrow_22m_stop_corridor`
- `narrow_28m_coarse_corridor` → `narrow_28m_corridor`

### 10.1 시나리오 장애물·게이트 설정

| Scenario | 현재 geometry 설정 |
|---|---|
| `s_curve_obstacles` | s=16,38,58 m; lateral=+0.25,-0.25,+0.25 m; 기본 obstacle 3.2×2.0 m |
| `obstacle_avoidance` | (x,y)=(35,0) m; 4.0×2.5 m; yaw=0 |
| `terminal_safe_region` | (76.95,0) m; 3.0×2.0 m; yaw=0 |
| `alternating_gate_corridor` | s/center=(30,+3),(65,-3),(100,+3),(135,-3),(170,+2.75) m; gap 4.8 m; extent 14 m; wall length 5 m |
| `curved_gate_maze` | s/center=(32,-2.5),(66,+2.5),(100,-2.5),(134,+2.5),(168,-2.25) m; gap 5.0 m; extent 14 m; wall length 5 m |
| `narrow_22m_stop_corridor` | s=25 m; gap 2.20 m; extent 8 m; length 20 m |
| `narrow_28m_corridor` | center=(65,0.10) m; gap 2.80 m; extent 8 m; length 70 m |
| `narrow_offset_corridor` | center x=65 m, global y=0; gap 2.40 m; extent 8 m; length 70 m |
| `winding_obstacle_course` | gate 5개와 별도 회전 obstacle 4개; 상세값은 `scenario_definition.py` 참조 |

### 10.2 CSV path 입력 검증 기본값

| 항목 | 현재값 |
|---|---:|
| CSV schema | `x,y,yaw,kappa,mode` |
| 최소 point 수 | 4 |
| tangent tolerance | 5° |
| curvature limit | 0.20 1/m |
| adjacent yaw-step limit | 15° |
| 최소 segment 길이 | `>1e-4 m` |
| 허용 mode | 0,1,2,3 |
| 제공 CSV nominal spacing | 약 0.20 m |

## 11. Costmap 기본 동작

### 11.1 C++ Costmap2D

| 항목 | 기본값 |
|---|---:|
| occupied threshold | 50 |
| unknown value | -1 |
| unknown is occupied | true |
| conservative cell correction | true |

Conservative correction은 distance field에서 `0.5 × sqrt(2) × resolution`을
차감한다.

### 11.2 Scenario rasterization

| 항목 | 기본값 |
|---|---:|
| resolution | 0.20 m/cell |
| margin | 10.0 m |
| free value | 0 |
| occupied value | 100 |
| unknown cell | 생성하지 않음 |
| 별도 inflation | 없음 |

회전 rectangle과 grid cell이 조금이라도 교차하면 occupied로 만드는 보수적
separating-axis 검사를 사용한다. `narrow_offset_corridor`만 scenario 기본
resolution이 0.05 m/cell이다.

## 12. 대체 Track Map 실행 경로

`simp_planner_tools/launch/track_map.launch.py`는 main scenario launch와 별개다.

Launch 기본값:

| 항목 | 값 |
|---|---:|
| map | `simp_planner_tools/maps/stadium_track.csv` |
| target speed | 2.0 m/s |
| debug save period | 10.0 s |
| debug root | `/home/sum/Desktop/simp_planner/simp_planner_debug` |

`track_map_provider_node` 기본값:

| ROS parameter | 기본값 | 의미 |
|---|---:|---|
| `map_file` | `stadium_track.csv` | authored track |
| `frame_id` | `odom` | frame |
| `path_back_length` | 5.0 m | local path 뒤 길이 |
| `path_ahead_length` | 45.0 m | local path 앞 길이 |
| `path_update_distance` | 1.0 m | 재발행 거리 |
| `target_speed` | 2.0 m/s | 목표속도 |
| `costmap_resolution` | 0.5 m/cell | free costmap 해상도 |
| `costmap_margin` | 10.0 m | track bounds 여유 |
| `projection_search_back` | 20 segments | projection 검색 |
| `projection_search_forward` | 80 segments | projection 검색 |
| `projection_fallback_distance` | 3.0 m | full-search 전환 |

Provider heartbeat는 1.0 s이고 costmap은 전체 free cell이다. 이 launch는 simulator
초기 pose와 Planner parameter를 override하지 않는다.

## 13. Debug node 파라미터

이 값들은 Planner feasibility 제약이 아니라 기록·진단 기준이다.

| ROS parameter | 기본값 | 의미 |
|---|---:|---|
| `scenario` | `stadium` | 진단 scenario 이름 |
| `output_dir` | `/home/sum/Desktop/simp_planner/simp_planner_debug` | 출력 root |
| `save_period` | 10.0 s | snapshot 주기 |
| `frame_id` | `odom` | frame |
| `vehicle_length` | 3.0 m | 시각화 차량 길이 |
| `vehicle_width` | 2.0 m | 시각화 차량 폭 |
| `movement_speed_threshold` | 0.03 m/s | motion direction 진단 |
| `global_lateral_limit` | 1.0 m | global deviation 진단 |
| `global_motion_limit_deg` | 45° | global heading 진단 |
| `tracking_lateral_limit` | 0.20 m | tracking 진단 |
| `tracking_motion_limit_deg` | 3° | tracking heading 진단 |
| `planning_deadline_ms` | 100 ms | 진단 threshold |
| `dynamic_topic_timeout` | 2.0 s | stale topic 진단 |

출력 위치는 `<output_dir>/<scenario>/<timestamp>/`이며 CSV history, status JSON,
PNG snapshot을 생성한다.

## 14. Planner 고정 topic과 frame

Planner topic은 현재 ROS parameter가 아니라 코드에 고정되어 있다.

| 방향 | Topic | Message |
|---|---|---|
| Subscribe | `/odom` | `nav_msgs/Odometry` |
| Subscribe | `/reference_path_data` | `simp_planner_msgs/ReferencePath` |
| Subscribe | `/costmap` | `nav_msgs/OccupancyGrid` |
| Subscribe | `/target_speed` | `std_msgs/Float64` |
| Subscribe | `/requested_drive_mode` | `std_msgs/UInt8` |
| Subscribe | `/vehicle/drive_mode_state` | `simp_planner_msgs/DriveModeState` |
| Publish | `/cmd_vel` | `geometry_msgs/Twist` |
| Publish | `/planner/cmd_vel_stamped` | `geometry_msgs/TwistStamped` |
| Publish | `/planner/executed_command` | `simp_planner_msgs/ExecutedCommand` |
| Publish | `/planner/status` | `std_msgs/String` |
| Publish | `/planner/execution_state` | `std_msgs/String` |
| Publish | `/planner/selected_trajectory` | `nav_msgs/Path` |
| Publish | `/planner/selected_trajectory_data` | `simp_planner_msgs/ReferencePath` |
| Publish | `/vehicle/drive_mode_command` | `std_msgs/UInt8` |

Stamped Body command frame은 `base_link`로 고정되어 있다.

## 15. 현재 미구현 Body feasibility 파라미터

다음 항목은 현재 코드에 승인된 설정값이나 candidate reject 로직이 없다.

| 필요한 항목 | 현재 상태 |
|---|---|
| mode별 `vx_min`, `vx_max` | 없음 |
| mode별 `vy_min`, `vy_max` | 없음 |
| `[vx,vy,r] in F_mode` | 없음 |
| Allocation profile과 독립된 물리적 beta 제한 | 없음 |
| Body `vx/vy` acceleration reject | 결과 배열과 3.2 값은 있으나 비교 안 함 |
| Body `vx/vy` jerk reject | 결과 배열과 9.0 값은 있으나 비교 안 함 |
| mode transition의 `epsilon_r` | 없음 |
| mode-change validity reject | flag만 기록하고 reject하지 않음 |

따라서 `vx_accel_max=3.2`, `vy_accel_max=3.2`, `vx_jerk_max=9.0`,
`vy_jerk_max=9.0`은 현재 차량 물리 제약으로 확정된 값이 아니며 동작에도 영향을
주지 않는다. 새로운 Body feasibility 수치와 로직은 별도 승인 전까지 이 문서의
현재 설정에 포함하지 않는다.

또한 현재 mode transition 정지 판정은 Planner와 simulator 모두 병진 scalar
speed만 사용한다. 논의된 `abs(vx)<epsilon_v`, `abs(vy)<epsilon_v`,
`abs(r)<epsilon_r`의 세 조건은 아직 구현되어 있지 않다.

## 16. 유지보수 시 확인할 소스 위치

| 설정 그룹 | Source of truth |
|---|---|
| C++ Planner 기본값 | `simp_planner/simp_planner_cpp/include/simp_planner/core.hpp` |
| Planner 수식·사용 여부 | `simp_planner/simp_planner_cpp/src/core.cpp` |
| Planner ROS 파라미터 | `simp_planner/simp_planner_cpp/src/planner_node.cpp` |
| Runtime hidden 기본값 | `simp_planner/simp_planner_cpp/include/simp_planner/runtime.hpp` |
| Runtime state machine | `simp_planner/simp_planner_cpp/src/runtime.cpp` |
| Simulator | `planar_velocity_sim/planar_velocity_sim/planar_velocity_sim_node.py` |
| Mode transition model | `planar_velocity_sim/planar_velocity_sim/mode_transition.py` |
| Main launch | `simp_planner_tools/launch/simulation.launch.py` |
| Track launch | `simp_planner_tools/launch/track_map.launch.py` |
| Scenario defaults | `simp_planner_tools/simp_planner_tools/scenario_definition.py` |
| Scenario node | `simp_planner_tools/simp_planner_tools/scenario_manager_node.py` |
| CSV path validation | `simp_planner_tools/simp_planner_tools/scenario_path.py` |
| Debug thresholds | `simp_planner_tools/simp_planner_tools/debug_plot_node.py` |

파라미터를 변경할 때는 선언값만 바꾸지 말고 이 문서의 `상태 및 의미`, 실제 사용
코드, launch override, scenario별 override를 함께 대조해야 한다.
