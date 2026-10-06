# 궤적 전달 방식

계획 knot 간격 `trajectory_knot_dt_sec`와 코어 `LongitudinalConfig::dt`의 기본값은
0.01초다. 4초 계획은 처음부터 400개 구간 / 401개 점으로 계산한다.
재계획 빈도 상한과 추종용 궤적 발행 빈도는 각각 10 Hz다.
`/planner/tracking_trajectory`에는 현재 시각부터 0.3초까지의 31점을 전달한다.
`header.stamp`는 첫 점의 실행 시각이며 `sample_period`는 0.01초다.
수신 시각부터 배열을 재생하면 안 된다.

## 위치와 속도의 공통 해석해

각 계획 구간에서 종방향 jerk `j`와 곡률 `kappa`를 일정하게 둔다.
구간 시작으로부터 `tau`초 뒤의 진행거리·속도·가속도는 다음과 같다.

```text
s(tau) = v0*tau + a0*tau^2/2 + j*tau^3/6
v(tau) = v0 + a0*tau + j*tau^2/2
a(tau) = a0 + j*tau
chi(tau) = chi0 + kappa*s(tau)
```

위치는 이 진행거리를 직선/원호 해석해에 넣어 계산한다. 따라서 구간 내부에서
`dx/dt = v*cos(chi)`, `dy/dt = v*sin(chi)`가 성립한다.
정지하는 구간은 속도가 처음 0이 되는 시각까지만 적분하고 이후 위치를 유지한다.
속도만 clipping하거나 저속이라는 이유로 위치 진행 중에 속도를 0으로 바꾸지 않는다.

계획 knot 생성과 `sample_body_command()`가 같은 `integrate_motion()`을 사용한다.
발행 타이머와 계획 knot의 위상이 다를 때도 위치를 선형 보간하지 않고 해당 구간의
해석해를 평가한다. beta는 allocation의 구간별 beta_rate로 계산하고,
`body_yaw = chi - beta`, `yaw_rate = v*kappa - beta_rate`로 맞춘다.
계획 인계 상태도 같은 절대시각의 샘플을 사용하며, 안전정지 위치도 같은 식으로 적분한다.

곡선 전체의 곡률이 일정하다는 뜻은 아니다. 공간 후보에서 얻은 곡률을 각 0.01초
구간에 적용하는 근사이며, knot에서 곡률과 beta_rate가 바뀔 수 있다.
동적 검사는 구간 내부 해석해와 knot 사이 속도 변화 모두를 확인하고,
최종 충돌검사는 실제 적분한 위치와 차체 yaw를 사용한다.
일정 곡률 근사에 의한 원래 공간 경로와의 차이는 별도로 검증해야 한다.

## 책임 분리

- Planner의 10 Hz `tracking_callback()`이 31점 배열을 생성한다.
  `command_callback()`과 100 Hz 명령 타이머는 제거했다. `/cmd_vel`,
  `/planner/cmd_vel_stamped`, `/planner/executed_command`,
  `/vehicle/drive_mode_command`를 직접 발행하지 않는다.
- 계산 결과는 즉시 `latest_plan_`으로 등록한다. `previous_plan_`은 새
  계획의 `start_ns` 이전 구간에만 사용하고 현재 시각이 경계를 지나면 버린다.
  배열의 각 점을 절대시간으로 선택하며 계획 activation은 없다.
  미래 계획을 다시 계산하면 같은 handover 시각에서 교체하고,
  초기조건은 이전 nominal 계획을 `predict_handover_state()`로 예측한다.
  미래 계획의 존재로 planning을 막지 않는다. 재계산이 경계에 늦으면 버리고 재시도한다.
  기존 `/planner/execution_state`와 status의 `execution` 필드는 호환용
  참조 생성 진단이며 차량의 실제 실행 여부를 나타내지 않는다.
- odometry는 첫 1회 초기 자세·속도 설정에만 사용하며 이후 수신은 무시한다.
  이후 상태는 nominal 계획과 handover의 시간축으로 진행한다.
  지속적인 차량 피드백은 `DriveModeState`의 모드와 정렬 상태뿐이다.
- 정상 주행은 `sample_body_command()`, 계획 만료는 끝점부터 감속 모델을
  재생한다. 모드 변경·강제 정지는 nominal 상태에서 감속 배열을 생성한다.
  강제 정지는 nominal 감속 완료 후 재계획한다.
- 종점 도착·회전 완료도 nominal 모델로 판정하며 실제 도착 확인이 아니다.
  모드 정렬 완료만 차량 피드백으로 확인한다. 미래 배열은 복사본으로
  계산한다. 초기 자세와 모드가 없으면 기다리며 새 계획이 없어도 발행한다.
- 각 점의 `current_mode`는 마지막으로 확인한 차량 모드,
  `requested_mode`는 발행 시 플래너의 요청이다. 미래 정렬 완료나 모드
  전환 완료를 추측하지 않는다. 회전 완료 예측 뒤에는 정지해 기다린다.
- Tracker는 가장 최신 배열 하나만 저장하고 새 배열 수신 시 교체한다.
  Tracker만 odometry를 지속 수신해 예측 위치·자세와 참조의 오차를 계산한다.
  `tracker_node`의 독립 100 Hz 타이머가 입력 둘을 잠금 아래 복사하고 즉시
  잠금을 해제한다. trajectory와 odometry는 `shared_ptr<const ...>`로 보관하므로
  callback이 B를 등록해도 진행 중인 cycle은 A snapshot으로 끝난다.
- point는 `ceil((now - header.stamp) / sample_period)`로 선택하며 보간하지 않는다.
  pose와 속도, `requested_mode`는 같은 point를 사용한다. mode는 `UInt8`로
  그대로 전달하며 Tracker에서 `current_mode`나 READY를 판단하지 않는다.
- odometry의 측정 시각부터 **선택한 point의 절대시각**까지 실측 body-frame
  `vx`, `vy`, `yaw_rate`가 일정하다고 가정하고 중점법으로 위치·자세를 예측한다.
  `dt = header.stamp + index * sample_period - odom.header.stamp`이며,
  `yaw_mid = yaw + yaw_rate * dt / 2` 방향으로 병진 이동을 적분한다.
  매 cycle 최신 odometry 원본에서 계산하고, 이전 예측을 누적하지 않는다.
  명령 속도나 목표 속도를 실제 속도 대신 사용하지 않는다.
- `max_prediction_sec` 기본값은 0.05초이며 런타임 조정 가능하다. 측정 시각부터
  선택한 point까지의 전체 예측 시간이 이 값을 초과하면 zero command다.
  이는 초기 설정값으로, 실차 지연·운동 변화·허용 오차에 맞춰 검증해야 한다.
  미래 시각·잘못된 timestamp·역순/동일 timestamp의 odometry는 수신 시 무시한다.
  무시한 메시지가 기존 odometry의 유효기간을 연장하지는 않는다.
  `base_frame` 기본값은 `base_link`이며 odometry의 `child_frame_id`와 같아야 한다.
  pose와 twist는 같은 차체 기준점의 상태여야 하며 별도 TF 변환은 하지 않는다.
- 제어식은 Fractal Fract. 2023, 7, 121, Section 4.1의 식 (10), (14)이다.
  예측 body frame으로 위치 오차를 회전하고 yaw 오차를 [-pi, pi]로 감싼다.
  노드 gain parameter 기본값은 `kx=0.3, ky=0.2, ktheta=0.2`이며 런타임 조정 가능하다.
  ETC/ETM, PID, 속도 제한은 추가하지 않았다. 입력 위치는 같은 좌표계여야 한다.
- 입력 없음·빈 배열·잘못된 주기·시작 전·만료·NaN/Inf 입력/결과는 zero
  `/cmd_vel`을 발행한다. 유효 point가 없으면 모드도 새로 발행하지 않는다.
  유효 point가 있고 odometry가 없거나 예측 불가능하면 모드는 전달하되 속도는 zero다.
  양수가 아니거나 유한하지 않은 gain도 zero command로 처리한다.
- `simulation.launch.py`는 Planner → TrackingTrajectory → Tracker → `/cmd_vel`
  → simulator를 연결한다. 모드 명령은 reliable/transient-local QoS로 전달한다.

## 확인

종료 시 그래프는 `/tracker/control`의 `TrackingControl`을 기록한다. 매 제어 cycle에서
실제 선택한 목표점, 그 점의 시각까지 예측한 측위, 제어식에 넣은 오차, 발행 명령을
하나의 snapshot으로 전달한다. 그래프에서는 참조 선택이나 예측·오차 계산을 반복하지 않는다.
`header.stamp`는 제어 시각이며 `odom_stamp`, `reference_stamp`도 CSV에 보관한다.
위치 그래프의 `predicted`는 예측 위치이고, 속도의 `measured at odom time`은 예측에
사용한 원본 측위 속도다. `valid=false`인 정지 cycle은 오차를 NaN으로 기록해 선을 끊는다.
이는 원본 측위 시각의 실제 추종 오차가 아니라 **제어기에 사용된 예측 오차**다.
기존 CSV는 기존 계산 방식의 제목과 범례로 표시되며 새 오차로 소급 변환하지 않는다.

워크스페이스에서 빌드와 환경 설정 후 실행한다. 테스트가 별도 ROS domain에
플래너를 띄워 배열 크기·간격·발행 주기, 시작 전 계획 교체와 handover
경계의 연속성, odom-once, nominal 주행·감속·회전·모드 대기를 확인한다.

```bash
source install/setup.bash
python3 src/sum-motion-planning/simp_planner/simp_planner_cpp/test/test_tracking_publication.py install/simp_planner_cpp/lib/simp_planner_cpp/planner_node_cpp
```
