# Fixed / 3D LUT gain 자동 최적화

두 브랜치의 공통 도구는 `simp_planner_tools.gain_experiment`이다. 실제 ROS
`simulation.launch.py`를 반복 실행하고 `/tracker/control`을 기존 기록기로
저장한다. Planner reference selection, odometry prediction, body-frame error,
vehicle response, 초기 pose를 변경하지 않는다. 추가 초기 오차도 없다.

자동 실행은 **sensor_delay_sec=0.0, AR(2) enabled, scale=1.0**을 강제한다.
manifest에 다른 sensor delay를 쓰면 오류로 거부한다. FOPDT actuator delay는
유지한다. 일반 수동 launch의 sensor delay 기본값(0.100 s)은 유지하므로,
수동 비교에는 반드시 `sensor_delay_sec:=0.0`을 전달한다.

## 설정과 case

- `simp_planner_tools/experiments/gain_cases.json`: 실제 연구용 초기 설정.
- `simp_planner_tools/experiments/gain_smoke.json`: 구현 확인용 3초 straight case.
- `settings`: 공통 launch 설정. gain/초기 pose/차량 식별 파라미터를 넣을 수 없다.
- `objective.scales`, `objective.weights`: 단위 정규화와 목적함수 가중치.
- `optimizer.bounds`, `initial_gains`, `random_seed`, `maxiter`, `popsize`, `tol`.
- LUT: `max_sweeps`, `sweep_tolerance`, `smoothness_lambda`, `block_de`.
- `recorder.quantiles`, `histogram_bins`: 분포 출력 설정. knot 수와 무관하다.
- 각 case: 고유 `id`, `scenario`, `target_speed`, `sensor_noise_seed`, `role`,
  `weight`, `end_condition`, `timeout_sec`.
- 종료 조건: `complete`는 scenario COMPLETE까지, `duration`은 첫 control
  timestamp부터 지정 `duration_sec`까지, `safe_stop`은 안전 정지 확인까지.

기본 구성은 tuning 13개, validation 7개, safety 4개다.

| 역할 | 시나리오 / 속도(m/s) |
|---|---|
| tuning | straight_long 1/3/5 (각 90초), s_curve 1/3/5 |
| tuning | crab_switch 1, reverse_switch 1, spot_turn_course 4.2 |
| tuning | parking_ramp_loop 1.2, winding_obstacle_course 1/3/5 |
| validation | stadium 3 (180초), hdmap_crab1_switch 1.5, fmtc_demo 1.5 |
| validation | hdmap_lap_switch 1.5, s_curve_obstacles 1.2, curved_gate_maze 1.5, warehouse_logistics 1 |
| safety | terminal_safe_region 1, narrow_22m_stop_corridor 1 (정지), narrow_28m_corridor 1, narrow_offset_corridor 1 |

기본 noise seed는 모두 42다. 여러 seed를 사용하려면 case를 복사하여 고유 id와
seed를 지정한다. candidate마다 seed를 바꾸지 않는다. 같은 seed는 같은 AR(2)
sample-index sequence를 생성한다. ROS wall-time 스케줄링은 실행마다 다를 수
있으므로 전체 trajectory의 bitwise 동일성까지 보장하는 것은 아니다.
명시적 target_speed는 기존 scenario manager에 의해 모든 phase의 cruise speed를
override한다. 특수 case의 속도는 manifest에서 검토/수정한다.

정규화 scale과 bounds는 초기 실험 설정이며 최종 튜닝 결과가 아니다. 설정을
바꾼 최적화는 다른 output 디렉토리에 저장해야 한다.

## 목적함수와 실패

유효 controller snapshot으로 각 case의 sample RMSE와 max_abs를 계산한다.
`position_rmse = hypot(rmse_ex, rmse_ey)`다. yaw는 rad, command yaw-rate는 rad/s.
`RMS(delta_command)`는 인접 control sample의 차이이며 시간 미분이 아니다.
command 변화량에는 유효 구간 내부의 invalid zero command도 포함한다.
startup 및 마지막 valid 이후의 invalid sample은 제외한다.

```text
J_position = ((RMSE(ex)/s_ex)^2 + (RMSE(ey)/s_ey)^2)/2
J_yaw      = (RMSE(etheta)/s_etheta)^2
J_control  = ((RMS(Δvx)/s_dvx)^2 + (RMS(Δvy)/s_dvy)^2
              + (RMS(Δyaw_rate)/s_dr)^2)/3
J_case     = w_position*J_position + w_yaw*J_yaw + w_control*J_control
J_total    = Σ(case_weight * J_case) / Σ(case_weight)
J_LUT      = J_total + smoothness_lambda * Σ_adjacent ||K_i-K_j||²
```

optimizer에는 tuning case만 전달한다. validation/safety 결과는 최적화에
영향을 주지 않는다. 실패/timeout/부족한 valid sample/valid fraction 미달에는
설정된 failure_penalty를 반환한다. best 결과에는 성공한 candidate만 저장한다.
오류가 난 candidate를 최적 gain이라고 보고하지 않는다.

원본 metric은 `case_metrics.csv`와 각 case `result.json`에 보존한다. 추가로
정확한 pure-rotation reference에서 command vx/vy 최대 절댓값, yaw RMSE를
저장한다. LUT 실행에서 병진 command가 0이 아니면 constraint 위반으로 실패한다.

## 알고리즘과 재개

Fixed는 SciPy differential_evolution으로 세 gain을 bounded search한다.
LUT는 sample에 가중치가 있는 interpolation neighborhood의 knot를 active로
분류한다. inactive knot는 그대로 둔다. active knot 하나의 세 gain을 block으로
DE search한 뒤 개선된 결과를 채택한다. 한 순회를 sweep으로 보고 설정된
개선 tolerance 또는 최대 sweep 수에서 종료한다. 단조성은 강제하지 않는다.

동일 명령/동일 output으로 재실행하면 자동 resume한다. 설정, 입력 LUT/sample,
설치된 controller/planner binary와 공통 Python/launch source fingerprint가
다르면 재개를 거부한다. 각 완료 trial과 case 결과를 cache한다. Fixed의
resume는 best를 x0로 넣는 seeded DE 재시작이며 SciPy 내부 population의 완전
복구는 아니다. LUT는 sweep/block/current LUT를 복구한다. 실행 중이던 block은
cached trial을 재사용하면서 다시 평가한다. 중단된 case의 로그는 보존하고 새로
실행한다. 대규모 실행은 이 문서의 smoke 설정으로 먼저 검증한다.

## LUT 파일

grid JSON의 `axes.vx`, `axes.vy`, `axes.yaw_rate`가 knot 목록이다. 수나 위치는
도구에 고정되지 않는다. `grid` 명령은 사용자가 고른 quantile과 zero로 초안을
만든다. zero 포함 후 사용자가 grid JSON을 확인/수정한다.

CSV header는 `vx,vy,yaw_rate,kx,ky,ktheta`, row는 vx 바깥 / yaw_rate 안쪽 순서의
완전 Cartesian product다. 비균일 축과 singleton 축을 지원한다. 누락/중복/
순서 오류, NaN/Inf, 음수 kx/ky, 양수가 아닌 ktheta는 load 시 거부한다.
query를 축 boundary에 clamp한 뒤 최대 8개 corner의 gain을 trilinear 보간한다.
그 후 **vx==0 && vy==0 && yaw_rate!=0**일 때만 kx/ky를 0으로 강제한다.
ktheta는 보간값을 유지한다. 완전 정지는 강제하지 않는다.

`gain_lut_file`은 controller startup에서 한 번 load한다. 빈 파일 경로로 LUT
브랜치를 실행하면 기존 nominal gain scheduling으로 동작한다. 연구용
evaluate/optimize-lut는 파일을 명시적으로 전달한다. fixed branch는 LUT 파일을
거부한다. controller 로그의 gain_policy를 runner가 검사해 잘못 빌드된 브랜치
사용을 방지한다.

## 결과 파일

- 최적화: `best.json`, `history.jsonl`, `history.csv`, `checkpoint.json`,
  `configuration.json`, `optimizer_result.json`.
- 각 trial: `trials/<candidate-hash>/summary.json`, `case_metrics.csv`,
  `<case-id>/command.json`, `launch.log`, `result.json`, `tracking_*/tracking.csv`,
  `odometry.csv`, `drive_mode.csv`, 상태/진행 JSON.
- operating: `operating/operating_points.csv`, `operating_statistics.json`,
  `operating_histograms.png`.
- LUT: 사용자가 지정한 `axes.json`, `initial_lut.csv` 및 초기값 출처 `.json`,
  optimizer output의 `active_points.json`, `optimized_lut.csv`.
- 비교: `comparison.csv`, `comparison.json`.

각 실행 명령/설정/summary/best/checkpoint에 sensor_delay_sec=0.0을 기록한다.
비교 도구는 manifest와 공통 planner/vehicle/scenario/tooling fingerprint가
다르면 거부한다. 같은 scenario/속도/seed의 metric을 나란히 출력한다.

## 사용자가 실제로 할 일

아래는 저장소 경로와 실제 CLI를 사용하는 전체 순서다. `OUT`은 두 브랜치에서
동일 경로를 사용한다. push는 필요 없다. branch를 바꿀 때마다 controller를
반드시 다시 build한다.

```bash
cd /home/a/ros2_ws
sudo apt-get install python3-scipy
source /opt/ros/jazzy/setup.bash
git -C src/sum-motion-planning switch feature/realistic-vehicle-simulation
colcon build --symlink-install --base-paths src/sum-motion-planning \
  --packages-up-to simp_controller simp_planner_tools
source install/setup.bash
cd src/sum-motion-planning
export OUT=/home/a/ros2_ws/simp_planner_debug/gain_optimization
export MANIFEST=$PWD/simp_planner_tools/experiments/gain_cases.json

# 1. Fixed gain optimization. 모든 내부 launch에 sensor_delay_sec:=0.0.
python3 -m simp_planner_tools.gain_experiment optimize-fixed \
  --manifest "$MANIFEST" --output "$OUT/fixed" --domain 173
# best.json의 gains=[kx,ky,ktheta], objective를 확인한다.
cat "$OUT/fixed/best.json"

# 2. Operating point 수집. tuning case만, delay=0.0, 동일 seed.
python3 -m simp_planner_tools.gain_experiment collect \
  --manifest "$MANIFEST" --fixed-result "$OUT/fixed/best.json" \
  --output "$OUT/operating_run" --domain 173

# 3. LUT grid 초안. quantile 수/위치는 사용자가 명시한다.
python3 -m simp_planner_tools.gain_experiment grid \
  --statistics "$OUT/operating_run/operating/operating_statistics.json" \
  --quantiles 0.0 0.5 1.0 --output "$OUT/axes.json"
# 반드시 axes.vx/vy/yaw_rate를 확인하고 필요한 nonuniform knot로 편집한다.
cat "$OUT/axes.json"

# 4. Fixed optimum으로 모든 knot 초기화.
python3 -m simp_planner_tools.gain_experiment init-lut \
  --axes "$OUT/axes.json" --fixed-result "$OUT/fixed/best.json" \
  --output "$OUT/initial_lut.csv"

# 5. LUT branch build + optimization. 모든 내부 launch에 delay=0.0.
git switch feature/lut-gain-scheduling
cd /home/a/ros2_ws
colcon build --symlink-install --base-paths src/sum-motion-planning \
  --packages-up-to simp_controller simp_planner_tools
source install/setup.bash
cd src/sum-motion-planning
python3 -m simp_planner_tools.gain_experiment optimize-lut \
  --manifest "$MANIFEST" --initial-lut "$OUT/initial_lut.csv" \
  --samples "$OUT/operating_run/operating/operating_points.csv" \
  --output "$OUT/lut" --domain 173

# 6a. LUT validation (delay=0.0).
python3 -m simp_planner_tools.gain_experiment evaluate \
  --manifest "$MANIFEST" --role validation --policy lut \
  --fixed-result "$OUT/fixed/best.json" --lut "$OUT/lut/optimized_lut.csv" \
  --output "$OUT/validation_lut" --domain 173

# 7a. LUT ROS safety regression (delay=0.0).
python3 -m simp_planner_tools.gain_experiment evaluate \
  --manifest "$MANIFEST" --role safety --policy lut \
  --fixed-result "$OUT/fixed/best.json" --lut "$OUT/lut/optimized_lut.csv" \
  --output "$OUT/safety_lut" --domain 173

# 6b/7b. Fixed branch를 다시 build한 후 같은 validation/safety case 실행.
git switch feature/realistic-vehicle-simulation
cd /home/a/ros2_ws
colcon build --symlink-install --base-paths src/sum-motion-planning \
  --packages-up-to simp_controller simp_planner_tools
source install/setup.bash
cd src/sum-motion-planning
python3 -m simp_planner_tools.gain_experiment evaluate \
  --manifest "$MANIFEST" --role validation --policy fixed \
  --fixed-result "$OUT/fixed/best.json" --output "$OUT/validation_fixed" --domain 173
python3 -m simp_planner_tools.gain_experiment evaluate \
  --manifest "$MANIFEST" --role safety --policy fixed \
  --fixed-result "$OUT/fixed/best.json" --output "$OUT/safety_fixed" --domain 173
python3 -m simp_planner_tools.gain_experiment compare \
  --fixed "$OUT/validation_fixed/summary.json" --lut "$OUT/validation_lut/summary.json" \
  --output "$OUT/comparison"

# 별도 기존 Planner swept-collision regression (ROS gain 비교가 아님).
python3 validation/run_all_scenarios.py --jobs 2

# 8a. Fixed 최종 simulation. generated args에도 delay=0.0이 포함된다.
read -r -a FIXED_ARGS <<< "$(python3 -m simp_planner_tools.gain_experiment launch-args \
  --fixed-result "$OUT/fixed/best.json")"
ros2 launch simp_planner_tools simulation.launch.py \
  scenario:=spot_turn_course use_rviz:=true target_speed:=4.2 \
  costmap_resolution:=0.1 debug_output_dir:="$OUT/final_fixed" \
  "${FIXED_ARGS[@]}"

# 8b. LUT branch를 위와 같이 다시 build/source한 후:
ros2 launch simp_planner_tools simulation.launch.py \
  scenario:=spot_turn_course use_rviz:=true target_speed:=4.2 \
  costmap_resolution:=0.1 debug_output_dir:="$OUT/final_lut" \
  sensor_delay_sec:=0.0 gain_lut_file:="$OUT/lut/optimized_lut.csv"
```

`gain_smoke.json`으로 `optimize-fixed`부터 먼저 실행하면 작은 실제 실행을
확인할 수 있다. 연구용 전체 최적화는 이 작업에서 완료하지 않는다. smoke의
best gain은 최종 연구 optimum으로 사용하지 않는다.

Safety ROS 평가의 stop-only case는 Planner safety-stop 상태, noise 없는 odom
pose로 계산한 정지 속도, 최대 이동거리를 검사한다. 다른 safety case는 COMPLETE
및 manifest metric limit를 검사한다. 이는 swept collision 인증을 대체하지 않는다.
기존 `validation/run_all_scenarios.py`는 별도의 Planner-only collision/expected-stop
검증이며 실제 ROS controller gain 효과를 평가하지 않는다. 이를 두 결과에
명확히 구분하여 사용한다.

## 테스트

```bash
cd /home/a/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select simp_controller planar_velocity_sim
colcon test-result --verbose
cd src/sum-motion-planning
python3 -m pytest -q simp_planner_tools/test/test_gain_experiment.py \
  simp_planner_tools/test/test_tracking_shutdown_plot.py \
  simp_planner_tools/test/test_tracking_shutdown_ros.py
```
