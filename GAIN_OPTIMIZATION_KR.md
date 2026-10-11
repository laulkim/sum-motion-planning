# ROS reference 수집 → offline gain search → full ROS 검증

병목은 DE 자체가 아니라 `optimize.trial → evaluate → run_case → ros2 launch`였다.
이제 `trial`은 `evaluate_offline`만 호출한다. ROS `run_case/evaluate`는 collect,
최종 validation, compare를 위해 유지한다. 두 optimization backend를 두지 않는다.

기존 `/tracker/control` 기록을 사용한다. `tracking.csv`의 `xr/yr/yaw_r`,
`vx_ref/vy_ref/omega_ref`, `control_stamp_ns/reference_stamp_ns`, `valid`가
controller가 실제 선택한 reference와 시각이다. 추가한 열은 `requested_mode` 하나다.
예전 CSV는 이 열이 없으므로 새 collection이 필요하다. 별도 recorder/ROS node는 없다.

## Replay 한 step

1. 기록된 control 시각까지 100 Hz plant tick을 숫자로 진행한다. mode 전환 완료 시
   최신 command를 queue하고 `FirstOrderResponse.update`로 actual 속도를 얻는다.
2. `integrate_body_velocity`로 actual pose를 적분한다. `AR2Noise`는 published
   odometry tick마다 측정 속도에만 더한다. pose/plant에는 noise를 넣지 않는다.
3. 기록된 `reference_stamp_ns`까지 최신 측정 속도로 pose를 예측한다. C++의
   midpoint prediction과 같은 `integrate_body_velocity`를 사용한다.
4. 해당 pose와 기록된 reference로 body error와 기존 Lyapunov 식을 계산한다.
   fixed는 입력 gain, LUT는 기존 axis weights의 clamped trilinear interpolation을 쓴다.
5. `DriveModeTransitionModel.command`는 actual 병진 속도로 정지 여부를 검사한다.
   ALIGNING 동안 입력은 zero지만 actual response/pose 및 in-flight delay는 유지한다.
   READY이면 세 command를 기존 FOPDT에 넣는다.

초기 pose는 첫 유효 reference pose, 초기 actual velocity는 기존 ROS plant처럼 zero다.
인위적인 초기 오차가 없다. startup/마지막 유효 reference 이후는 제외하지만 내부
invalid control은 zero command로 replay하고 기존 metric window에 포함한다.
plant tick 위상은 첫 reference에 맞춰 시작하며 ROS callback jitter는 재현하지 않는다.
AR(2) sequence는 case seed + channel index로 매 candidate마다 동일하게 시작한다.

`RESPONSE_PARAMETER_DEFAULTS`의 identified gain/tau/actuator delay를 그대로 import한다.
`sensor_delay_sec=0.0`만 강제한다. 수집 summary에 model parameter를 저장하고
replay에서 현재 defaults와 일치하는지 검사한다. AR(2)는 연구 manifest에서 enabled,
scale=1.0이다. noise와 measured velocity는 기존 prediction에 사용된다.

LUT pure rotation (`vx_ref==0 && vy_ref==0 && omega_ref!=0`)에서는 interpolation
**이후** kx/ky를 zero로 강제한다. ktheta는 scheduled 값을 쓴다. 완전 정지에는
이 constraint를 적용하지 않는다. Fixed replay는 fixed 브랜치의 원래 제어식을 쓴다.
LUT 브랜치 C++는 fixed fallback에도 spot constraint를 적용하므로 fixed ROS 비교는
fixed 브랜치 설치를 사용한다.

## 기존 search/metric 재사용

`gain_evaluation.metrics/case_objective/aggregate`, manifest의 bounds/scales/weights,
fixed SciPy differential_evolution, LUT active blocks/block DE/smoothness/sweeps,
history/cache/checkpoint를 유지한다. LUT CSV load/save와 axis weights도 재사용한다.
Python replay lookup은 기존 weights를 합산하며 active_points도 같은 helper를 쓴다.
추가 package, controller hierarchy, backend interface는 없다.

```text
J_position = ((RMSE(ex)/s_ex)^2 + (RMSE(ey)/s_ey)^2)/2
J_yaw      = (RMSE(etheta)/s_etheta)^2
J_control  = Σ(RMS(Δcommand)/s_Δcommand)^2 / 3
J_case     = w_position*J_position + w_yaw*J_yaw + w_control*J_control
J_total    = Σ(case_weight*J_case) / Σ(case_weight)
J_LUT      = J_total + smoothness_lambda*Σ_adjacent ||K_i-K_j||²
```

최적화에는 tuning만 사용한다. 부족한 valid samples/fraction, metric limits 위반,
nonfinite evolution에는 기존 failure penalty를 쓴다. 성공한 candidate만 best로 저장한다.
수집은 각 manifest case를 한 번 실행하고 동일 output의 완료 case는 재사용한다.
collection에서 실패한 case가 있으면 먼저 로그/manifest를 수정해 수집을 완료해야 한다.

새 offline output은 비어 있어야 한다. 구형 ROS optimization 결과를 삭제하거나
checkpoint로 변환하지 않는다. `backend=offline_replay`, reference CSV/summary hash,
model/optimizer source, manifest, LUT/sample hash가 동일할 때만 offline resume한다.
Fixed resume는 best를 x0로 쓰는 seeded DE 재시작 + exact candidate cache다.
LUT는 기존 sweep/block checkpoint를 복구한다.

`best.json`에는 fixed `gains=[kx,ky,ktheta]`, `objective`, `success`가 저장된다.
LUT는 `best.json`의 objective와 `optimized_lut.csv`를 사용한다. `optimizer_result.json`,
`history.jsonl/history.csv`, `checkpoint.json`, trial별 `summary.json`도 유지한다.
Candidate마다 console/history에 실제 evaluation wall time을 기록한다. `benchmark`는
전체 수집 tuning set의 J_total과 wall/reference duration을 출력하고 time history도 저장한다.
CSV actual pose는 `x/y/yaw`, actual velocity는 `vx_actual/vy_actual/omega_actual`,
measured velocity는 `vx/vy/omega`, command는 `vx_cmd/vy_cmd/omega_cmd`다.

## 실제 실행 명령

이 작업에서는 두 브랜치를 별도 worktree로 수정했다. 현재 LUT worktree는
`/home/a/ros2_ws/src/sum-motion-planning`, fixed worktree는 `/home/a/sum-motion-planning-fixed`다.
각 설치를 해당 단계에서 source한다. 아래 `research_offline_v1`은 구형 결과와 다른
새 디렉토리이며 기존 결과를 삭제하지 않는다. 이후 설정을 바꾸면 다른 이름을 쓴다.

### 1. Fixed 브랜치 build

```bash
export FIXED=/home/a/sum-motion-planning-fixed
export LUT=/home/a/ros2_ws/src/sum-motion-planning
export OUT=/home/a/ros2_ws/simp_planner_debug/research_offline_v1
source /opt/ros/jazzy/setup.bash
cd "$FIXED"
colcon build --base-paths . --symlink-install --packages-up-to simp_planner_tools --parallel-workers 2
source "$FIXED/install/setup.bash"
export MANIFEST="$FIXED/simp_planner_tools/experiments/gain_cases.json"
```

### 2. Tuning reference collection (13 cases, nominal fixed gains)

```bash
python3 -m simp_planner_tools.gain_experiment collect \
  --manifest "$MANIFEST" --output "$OUT/references" \
  --policy fixed --role tuning --domain 173 --sensor-delay-sec 0.0
```

각 내부 ROS 명령은 `ros2 launch simp_planner_tools simulation.launch.py ...
sensor_delay_sec:=0.0`이다. `--sensor-delay-sec`에는 0.0만 허용되며 manifest도 0.0이어야 한다.
Reference 입력 파일은 `$OUT/references/summary.json`; 실제 reference CSV는 summary가
가리키는 각 case의 `tracking_*/tracking.csv`다. 정상 scenario initial state를 쓴다.

### 3. Offline fixed optimization 및 한 candidate benchmark

```bash
python3 -m simp_planner_tools.gain_experiment benchmark \
  --manifest "$MANIFEST" --references "$OUT/references/summary.json" \
  --output "$OUT/benchmark_nominal" --gains 1 2 1
python3 -m simp_planner_tools.gain_experiment optimize-fixed \
  --manifest "$MANIFEST" --references "$OUT/references/summary.json" \
  --output "$OUT/fixed_offline"
```

두 명령은 ROS 없이도 실행 가능하다. 패키지 import 경로만 준비되면 된다.

### 4. Fixed 결과 확인

```bash
cat "$OUT/fixed_offline/best.json"
cat "$OUT/fixed_offline/optimizer_result.json"
```

### 5. LUT grid/initial LUT 준비

수집 단계에서 기존 operating report가 자동 생성된다. 아래 quantiles는 처음 쓰는
작은 grid 예시다. 실제 연구 grid는 operating distribution을 보고 조정한다.

```bash
python3 -m simp_planner_tools.gain_experiment grid \
  --statistics "$OUT/references/operating/operating_statistics.json" \
  --quantiles 0 0.5 1 --output "$OUT/grid_axes.json"
python3 -m simp_planner_tools.gain_experiment init-lut \
  --axes "$OUT/grid_axes.json" --fixed-result "$OUT/fixed_offline/best.json" \
  --output "$OUT/initial_lut.csv"
```

### 6. LUT 브랜치 build

```bash
cd /home/a/ros2_ws
colcon build --symlink-install --packages-up-to simp_planner_tools --parallel-workers 2
source /home/a/ros2_ws/install/setup.bash
export MANIFEST="$LUT/simp_planner_tools/experiments/gain_cases.json"
```

### 7. Offline LUT optimization

```bash
python3 -m simp_planner_tools.gain_experiment optimize-lut \
  --manifest "$MANIFEST" --references "$OUT/references/summary.json" \
  --initial-lut "$OUT/initial_lut.csv" \
  --samples "$OUT/references/operating/operating_points.csv" \
  --output "$OUT/lut_offline"
cat "$OUT/lut_offline/best.json"
ls -l "$OUT/lut_offline/optimized_lut.csv"
```

### 8. 同一 validation cases를 full ROS에서 실행

Fixed 검증은 fixed 설치, LUT 검증은 LUT 설치로 실행한다. 각 내부 launch는 항상
`sensor_delay_sec:=0.0`이고 gain 변화에 따라 planner가 reference를 다시 생성한다.
같은 shell에서 두 workspace를 번갈아 source하면 이미 들어 있는 prefix가 앞쪽으로
이동하지 않아 잘못된 controller가 선택될 수 있다. 검증은 각각 깨끗한 환경에서 실행한다.
`ros2 pkg prefix simp_controller`가 해당 설치를 가리키는지도 확인한다.

```bash
# Fixed validation: inherited ROS overlay를 제거한다.
env -i HOME="$HOME" USER="$USER" PATH=/usr/bin:/bin LANG=C.UTF-8 \
  FIXED="$FIXED" OUT="$OUT" bash --noprofile --norc <<'BASH'
source /opt/ros/jazzy/setup.bash
source "$FIXED/install/setup.bash"
test "$(ros2 pkg prefix simp_controller)" = "$FIXED/install/simp_controller" || exit 1
python3 -m simp_planner_tools.gain_experiment evaluate \
  --manifest "$FIXED/simp_planner_tools/experiments/gain_cases.json" \
  --fixed-result "$OUT/fixed_offline/best.json" --policy fixed --role validation \
  --output "$OUT/validation_fixed" --domain 173 --sensor-delay-sec 0.0
BASH

# LUT validation도 별도 환경에서 실행한다.
env -i HOME="$HOME" USER="$USER" PATH=/usr/bin:/bin LANG=C.UTF-8 \
  LUT="$LUT" OUT="$OUT" bash --noprofile --norc <<'BASH'
source /opt/ros/jazzy/setup.bash
source /home/a/ros2_ws/install/setup.bash
test "$(ros2 pkg prefix simp_controller)" = /home/a/ros2_ws/install/simp_controller || exit 1
python3 -m simp_planner_tools.gain_experiment evaluate \
  --manifest "$LUT/simp_planner_tools/experiments/gain_cases.json" \
  --fixed-result "$OUT/fixed_offline/best.json" --policy lut \
  --lut "$OUT/lut_offline/optimized_lut.csv" --role validation \
  --output "$OUT/validation_lut" --domain 173 --sensor-delay-sec 0.0
BASH
```

이미 잘못된 prefix로 fixed 검증이 실패했다면 그 로그는 보존하고 위 fixed 명령의
output을 `$OUT/validation_fixed_corrected`로 바꾼다. 기존 reference/fixed/LUT 최적화
결과를 그대로 사용한다. 진행 중인 LUT 검증이 끝난 뒤 fixed 검증을 재실행한다.

수동 한 scenario launch가 필요하면 실제 helper로 args를 만든다.

```bash
python3 -m simp_planner_tools.gain_experiment launch-args --fixed-result "$OUT/fixed_offline/best.json"
python3 -m simp_planner_tools.gain_experiment launch-args --lut "$OUT/lut_offline/optimized_lut.csv"
# 위 출력에는 sensor_delay_sec:=0.0이 포함된다. 예:
ros2 launch simp_planner_tools simulation.launch.py scenario:=straight_long target_speed:=1.0 \
  sensor_delay_sec:=0.0 use_rviz:=false
```

안전 회귀 검증은 evaluate의 `--role safety`로 같은 gain/LUT를 실행한다.
Full ROS 결과는 각 validation output의 `summary.json`, `case_metrics.csv`, case별
tracking/debug 기록에 저장된다. Offline objective를 최종 성능으로 해석하지 않는다.

## 한계 및 작은 검증

Offline에서는 ROS controller가 선택한 reference schedule이 고정된다. 실제 ROS에서는
actual pose와 mode 상태가 달라지면 이후 planner trajectory와 완료 시점도 달라진다.
따라서 offline은 gain search이고 full ROS가 최종 성능/완료/안전 검증이다. Offline의
mode gate는 기존 모델을 그대로 쓰지만 frozen planner는 전환 지연에 적응하지 않는다.

기존 FOPDT/AR2/kinematics/mode tests와 작은 연결 regression을 실행한다.

```bash
cd "$LUT"
source /opt/ros/jazzy/setup.bash
source /home/a/ros2_ws/install/setup.bash
PYTHONPATH="$PWD/simp_planner_tools:$PWD/planar_velocity_sim:$PYTHONPATH" \
  python3 -m pytest -q planar_velocity_sim/test/test_vehicle_response.py \
  planar_velocity_sim/test/test_kinematics.py planar_velocity_sim/test/test_mode_transition.py \
  simp_planner_tools/test/test_offline_gain_sim.py simp_planner_tools/test/test_gain_experiment.py \
  simp_planner_tools/test/test_tracking_shutdown_plot.py
```

짧은 실제 CLI 확인에는 `gain_cases.json` 대신 기존 `gain_smoke.json`을 모든 단계에서
일관되게 사용하고 별도 output을 쓴다. Smoke의 완료는 13 tuning/7 validation 연구 실행의
완료를 뜻하지 않는다.

이번 변경의 실행 확인 결과는 `/home/a/ros2_ws/simp_planner_debug/offline_backend_smoke_v1`에
보존했다. 두 브랜치 build, Python 61개 테스트, C++ fixed 1개/LUT 2개 테스트가 통과했다.
실제 ROS smoke collection → offline fixed/LUT search → 각각 full ROS smoke 재검증도 통과했다.
Smoke fixed best는 `[1.1865954112640338, 2.0, 1.0]`, offline objective는
`0.0037343767532030344`이며 LUT objective도 같은 값이다. 이는 smoke 결과다.

별도로 기본 tuning manifest의 `straight_long_3_s42` 한 case를 실제 ROS에서 수집했다.
90.420초/9,043 snapshots의 candidate J_total은 `0.00157969`, 계산 wall time은
`0.122551초`(약 738배 실시간), time-history CSV 저장 포함 benchmark는 `0.310336초`였다.
측정 파일은 `benchmark_90s_objective/summary.json`, `benchmark_90s_nominal/summary.json`,
통합 확인 기록은 `verification.json`이다. 전체 13 tuning/7 validation 연구 실행은 수행하지 않았다.
