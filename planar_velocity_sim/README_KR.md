# planar_velocity_sim 현실화 모델

이 패키지는 Planner의 body-frame `vx`, `vy`, `yaw_rate` 명령을 유지하면서
명령 채널, 차량 응답, 상태추정 오차를 서로 분리합니다.

```text
/cmd_vel
  -> command delay / timeout
  -> actuator-level body velocity dynamics
  -> true state
       -> /sim/ground_truth/odom
       -> localization estimate -> /odom -> Planner
```

기본 `ideal` 프로파일은 기존의 `command == actual == odometry` 동작을
유지합니다. 현실화 모델은 다음처럼 실행합니다.

```bash
ros2 launch simp_planner_tools simulation.launch.py \
  scenario:=stadium target_speed:=2.0 sim_profile:=realistic
```

파라미터는 `config/realistic.yaml`에서 조정합니다. 예시값은 특정 실차를
식별한 값이 아니므로, 최종적으로는 명령과 ground truth 및 localization
로그의 시간 정렬된 잔차를 이용해 조정해야 합니다.

## 파라미터 그룹

- `command.*`: 순수 명령 지연과 command watchdog
- `dynamics.*`: 축별 응답 시정수, gain, 속도·가속도·저크 한계,
  구름저항과 deadband
- `estimator.*`: 상태추정 출력 지연, dropout, 백색잡음, 시간 상관 bias,
  양자화 분해능과 재현용 random seed

차량 모델은 바퀴 조향각이나 토크가 아니라 닫힌 저수준 속도 제어기의
응답을 모델링합니다. 따라서 현재 Planner 인터페이스와 전진, 후진, 좌우
crab 모드를 모두 유지합니다.

## 출력 토픽

- `/odom`: 지연과 추정 오차가 포함된 Planner 입력
- `/sim/ground_truth/odom`: 오차 없는 실제 시뮬레이션 상태
- `/sim/applied_twist`: 명령 지연 및 차량 응답 이후 실제 body twist

`/odom`의 `header.stamp`는 발행 시각이 아니라 지연된 추정 상태의 기준
시각입니다. `pose.covariance`와 `twist.covariance`에는 설정된 백색잡음과
bias 분산의 합이 기록됩니다.
