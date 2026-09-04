# 저장소 기여 가이드

## 프로젝트 구조 및 모듈 구성

이 저장소는 로컬 모션 플래닝 개념 검증을 위한 ROS 2 Jazzy 워크스페이스입니다. `simp_planner/simp_planner_cpp/`에는 C++17 플래너 코어, ROS 노드, 공개 헤더, launch 파일, C++ 테스트가 있습니다. `simp_planner/simp_planner_msgs/`는 사용자 정의 ROS 메시지를 정의합니다. `planar_velocity_sim/`은 Python 차량 시뮬레이터를 제공하며, `simp_planner_tools/`에는 시나리오 생성, 맵, launch 파일, 진단 및 플로팅 도구가 있습니다. 독립 실행형 회귀 검증 도구와 픽스처는 `validation/`에 있습니다. 이 디렉터리의 요약 및 CSV 결과가 변경되면 내용을 주의 깊게 검토하십시오.

## 빌드, 테스트 및 개발 명령어

ROS 명령은 워크스페이스 루트에서 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select planar_velocity_sim simp_planner_msgs simp_planner_cpp simp_planner_tools
source install/setup.bash
colcon test --packages-select planar_velocity_sim simp_planner_cpp simp_planner_tools
colcon test-result --verbose
```

통합 시나리오는 다음과 같이 실행합니다.

```bash
ros2 launch simp_planner_tools simulation.launch.py scenario:=stadium target_speed:=2.0
```

ROS 없이 회귀 검증을 실행하려면 `python3 validation/run_validation.py --jobs 4`를 사용하십시오. 검증기 바이너리가 최신일 때만 `--skip-build`를 사용합니다. 전체 매트릭스는 `python3 validation/run_all_scenarios.py`로 실행하며, 추적 중인 결과 파일을 덮어씁니다.

## 코딩 스타일 및 이름 규칙

기존 형식을 따릅니다. Python은 공백 4칸, C++은 공백 2칸으로 들여씁니다. Python 모듈·함수·변수는 `snake_case`, 클래스는 `PascalCase`, 상수는 `UPPER_SNAKE_CASE`를 사용합니다. C++ 타입은 `PascalCase`, 함수와 변수는 `snake_case`, 상수는 `kName` 형식을 사용합니다. 공개 C++ 선언은 `include/simp_planner/`, 구현은 `src/`에 둡니다. C++17 호환성을 유지하고 `-Wall -Wextra -Wpedantic` 경고 없이 빌드되도록 하십시오. 저장소 공통 포매터는 없으므로 인접 코드의 스타일을 유지합니다.

## 테스트 지침

Python 테스트는 `pytest`, C++ 코어 테스트는 CTest를 사용합니다. 파일 이름은 `test_<동작>.py` 또는 `test_<컴포넌트>.cpp` 형식으로 작성하고 테스트는 결정론적으로 유지하십시오. 변경한 패키지에 단위 테스트를 추가하고, 플래너·할당·충돌·시나리오 동작을 바꾼 경우 선택 회귀 검증도 실행합니다. 명시된 커버리지 기준은 없지만 경계 조건과 안전 정지 동작을 우선 검증하십시오.

## 커밋 및 풀 리퀘스트 지침

커밋 이력이 적고 Conventional Commits 규칙은 적용되지 않습니다. `Fix terminal stop tolerance`처럼 짧은 명령형 제목을 사용하고 관련 없는 변경은 분리하십시오. 풀 리퀘스트에는 동작 변경 사항, 실행한 검증 명령, 관련 이슈를 기재하고 변경된 검증 산출물을 명시합니다. 시각화 또는 궤적 출력이 달라졌다면 플롯이나 스크린샷을 첨부하십시오.
