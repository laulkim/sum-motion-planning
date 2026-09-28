# 궤적 전달 방식

현재는 `sample_body_command()`를 재사용한다. `/planner/tracking_trajectory`에
10 Hz로 현재 시각부터 0.3초까지의 31점을 0.01초 간격으로 발행한다.
`header.stamp`는 첫 점의 실행 시각이며, `sample_period`의
`sec=0, nanosec=10000000`은 0.01초다. 수신 시각부터 배열을 재생하면 안 된다.

| 항목 | 현재 방식 | 향후 경로·시간 모델 통합 |
|---|---|---|
| 위치 | 기존 0.1초 점 사이의 선형 보간 | 시간별 진행 거리로 같은 경로 곡선 평가 |
| 속도 | 기존 가속도·jerk·beta 계산 | 위치·자세와 같은 모델의 미분 관계 사용 |
| 장점 | 기존 계산 재사용, 변경 범위가 작음 | 위치·자세·속도의 일관성 확보 |
| 한계/비용 | 위치의 미분과 속도가 다를 수 있음 | 경로 평가와 실행·충돌·동역학 검증 수정 필요 |

촘촘한 점은 새로운 GT가 아니다. 현재 방식의 오차가 허용 범위인지 먼저
속도 적분 결과와 참조 위치·자세를 비교한다. 오차가 추종 요구를 넘으면
공통 샘플러로 교체한다. 위치·접선·곡률을 각각 보간하는 기존 경로 함수를
그대로 쓰거나 계획 간격만 줄이는 것으로는 모델 일관성이 보장되지 않는다.
메시지 형식은 유지하고 내부 샘플링을 바꿀 수 있다.

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
  Tracker만 odometry를 지속 수신해 실제 위치·자세·속도 오차를 계산한다.
  `tracker_node`의 독립 100 Hz 타이머가 입력 둘을 잠금 아래 복사하고 즉시
  잠금을 해제한다. trajectory는 `shared_ptr<const ...>`로 보관하므로
  callback이 B를 등록해도 진행 중인 cycle은 A snapshot으로 끝난다.
- point는 `ceil((now - header.stamp) / sample_period)`로 선택하며 보간하지 않는다.
  pose와 속도, `requested_mode`는 같은 point를 사용한다. mode는 `UInt8`로
  그대로 전달하며 Tracker에서 `current_mode`나 READY를 판단하지 않는다.
- 제어식은 Fractal Fract. 2023, 7, 121, Section 4.1의 식 (10), (14)이다.
  실제 body frame으로 위치 오차를 회전하고 yaw 오차를 [-pi, pi]로 감싼다.
  gain parameter 기본값은 `kx=3.0, ky=4.0, ktheta=2.0`이며 런타임 조정 가능하다.
  ETC/ETM, PID, 속도 제한은 추가하지 않았다. 입력 위치는 같은 좌표계여야 한다.
- 입력 없음·빈 배열·잘못된 주기·시작 전·만료·NaN/Inf 입력/결과는 zero
  `/cmd_vel`을 발행한다. 유효 point가 없으면 모드도 새로 발행하지 않는다.
  유효 point가 있고 odometry가 없으면 모드는 전달하되 속도는 zero다.
  양수가 아니거나 유한하지 않은 gain도 zero command로 처리한다.
- `simulation.launch.py`는 Planner → TrackingTrajectory → Tracker → `/cmd_vel`
  → simulator를 연결한다. 모드 명령은 reliable/transient-local QoS로 전달한다.

## 확인

워크스페이스에서 빌드와 환경 설정 후 실행한다. 테스트가 별도 ROS domain에
플래너를 띄워 배열 크기·간격·발행 주기, 시작 전 계획 교체와 handover
경계의 연속성, odom-once, nominal 주행·감속·회전·모드 대기를 확인한다.

```bash
source install/setup.bash
python3 src/sum-motion-planning/simp_planner/simp_planner_cpp/test/test_tracking_publication.py install/simp_planner_cpp/lib/simp_planner_cpp/planner_node_cpp
```
