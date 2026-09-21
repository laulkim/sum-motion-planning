"""Ao et al., Fractal Fract. 2023, 7, 121, equations (10) and (14).

시간 궤적과 odometry로 차체 속도를 보정하는 주기 제어기.
차륜별 명령 분배와 이벤트 기반 제어는 포함하지 않는다.

수신 콜백 → calculate(입력 검사·목표 보간·추종 계산) → tick(명령·진단 발행).
"""

from bisect import bisect_right
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from simp_planner_msgs.msg import DriveModeState, ExecutedCommand, Trajectory, TrajectoryPoint


# 정지 명령: 차체 x/y 방향 속도[m/s], yaw 각속도[rad/s].
ZERO = (0.0, 0.0, 0.0)


def stamp_ns(stamp):
    # ROS 시각의 초와 나노초를 하나의 정수 나노초 값으로 합친다.
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def wrap_angle(angle):
    # 동일한 방향을 나타내는 각도를 [-π, π] 범위로 맞춘다.
    return math.atan2(math.sin(angle), math.cos(angle))


def validate_trajectory(message):
    """시간 순서·수치·모드를 검사하고 잘못된 궤적이면 ValueError를 발생시킨다."""
    # 좌표계, 유효한 계획 ID, 보간에 필요한 최소 두 점을 요구한다.
    if not message.header.frame_id or message.plan_id == 0 or len(message.points) < 2:
        raise ValueError("trajectory needs a frame, plan_id and at least two points")
    # 시작 시각은 음수가 아니어야 하고 첫 점의 상대 시각은 0이어야 한다.
    if stamp_ns(message.start_time) < 0 or message.points[0].time_from_start != 0.0:
        raise ValueError("invalid trajectory time origin")
    # 첫 점의 시각 0도 증가 조건을 통과하도록 비교 기준을 잡는다.
    previous_time = -1.0
    # 궤적의 모든 점을 순서대로 검사한다.
    for point in message.points:
        # 위치·방향·속도·시간에 NaN 또는 무한대가 있으면 거부한다.
        if not all(math.isfinite(value) for value in (
            point.x, point.y, point.yaw, point.vx, point.vy,
            point.yaw_rate, point.time_from_start,
        )):
            raise ValueError("trajectory contains a non-finite value")
        # 시간은 엄격히 증가해야 하며 주행 모드는 0~4만 허용한다.
        if point.time_from_start <= previous_time or point.mode not in range(5):
            raise ValueError("trajectory times must increase and mode must be valid")
        # 다음 점의 시간과 비교할 기준을 갱신한다.
        previous_time = point.time_from_start
    # 한 궤적 안에서는 모드를 바꿀 수 없으며 전환에는 별도 계획이 필요하다.
    if any(point.mode != message.points[0].mode for point in message.points):
        raise ValueError("a mode transition requires a separate plan")


def reference_at(message, elapsed):
    """검증된 궤적을 경과 시간[초]에 맞춰 보간한다. 범위 밖이면 None을 반환한다."""
    # 검증을 마친 궤적의 점 목록을 사용한다.
    points = message.points
    # 계획 시작 전이나 마지막 점 이후에는 외삽하지 않는다.
    if not math.isfinite(elapsed) or elapsed < 0.0 or elapsed > points[-1].time_from_start:
        # 현재 시각에 사용할 목표점이 없음을 호출자에게 알린다.
        return None
    # 마지막 시각에서는 오른쪽 보간점이 없으므로 끝점을 그대로 사용한다.
    if elapsed == points[-1].time_from_start:
        # 궤적의 끝 시각도 유효한 추종 시각에 포함한다.
        return points[-1]
    # 현재 시각 이하인 마지막 점을 이진 탐색으로 찾아 왼쪽 점으로 삼는다.
    index = bisect_right(points, elapsed, key=lambda point: point.time_from_start) - 1
    # 현재 시각을 사이에 둔 두 목표점을 꺼낸다.
    left, right = points[index:index + 2]
    # 두 점 사이에서 현재 시각이 차지하는 비율을 계산한다(0~1).
    weight = (elapsed - left.time_from_start) / (right.time_from_start - left.time_from_start)
    # 보간 결과를 담을 목표점을 만든다.
    reference = TrajectoryPoint()
    # 위치와 속도 성분은 각 필드별로 선형 보간한다.
    for field in ("x", "y", "vx", "vy", "yaw_rate"):
        # 왼쪽과 오른쪽 목표점의 해당 필드 값을 읽는다.
        a, b = getattr(left, field), getattr(right, field)
        # 왼쪽 값에서 오른쪽 값까지 시간 비율만큼 이동한 값을 저장한다.
        setattr(reference, field, a + weight * (b - a))
    # yaw는 ±π 경계를 넘어도 최단 방향으로 보간하고 다시 각도를 정규화한다.
    reference.yaw = wrap_angle(left.yaw + weight * wrap_angle(right.yaw - left.yaw))
    # 보간한 점의 상대 시각을 현재 경과 시간[초]으로 기록한다.
    reference.time_from_start = elapsed
    # 검증된 궤적은 모든 점의 모드가 같으므로 왼쪽 점의 모드를 사용한다.
    reference.mode = left.mode
    # 현재 시각의 목표 위치·방향·속도를 반환한다.
    return reference


def tracking_command(reference, pose, gains):
    """식 (10)의 차체 축 오차와 식 (14)의 속도 명령을 계산한다. 속도 제한은 별도 적용한다."""
    # 궤적과 같은 좌표계에서 측정한 현재 위치[m]와 방향[rad]을 꺼낸다.
    x, y, yaw = pose
    # 현재 yaw의 회전 성분: 전역 위치 오차를 현재 차체 축으로 옮길 때 사용한다.
    c, s = math.cos(yaw), math.sin(yaw)
    # 전역 좌표계에서 목표 위치와 현재 위치의 차이를 구한다.
    dx, dy = reference.x - x, reference.y - y
    # 역회전 R(-yaw)을 적용해 차체 전후(ex)·좌우(ey) 위치 오차[m]로 바꾼다.
    ex, ey = c * dx + s * dy, -s * dx + c * dy
    # 방향 오차[rad]도 최단 회전각으로 계산한다.
    e_yaw = wrap_angle(reference.yaw - yaw)
    # 목표 차체 축의 속도를 현재 차체 축으로 회전시키기 위한 성분이다.
    ce, se = math.cos(e_yaw), math.sin(e_yaw)
    # 전후·좌우·방향 오차에 적용할 양의 비례 이득을 꺼낸다.
    kx, ky, k_yaw = gains
    # 회전한 목표 속도(feedforward)에 위치·방향 오차 보정량을 더한다.
    command = (
        # 현재 차체 x축 목표 속도에 전후 위치 오차 보정을 더한다.
        reference.vx * ce - reference.vy * se + kx * ex,
        # 현재 차체 y축 목표 속도에 좌우 위치 오차 보정을 더한다.
        reference.vy * ce + reference.vx * se + ky * ey,
        # 목표 yaw 각속도에 방향 오차 보정을 더한다.
        reference.yaw_rate + k_yaw * e_yaw,
    )
    # 제어 명령과 진단용 차체 좌표계 오차를 함께 반환한다.
    return command, (ex, ey, e_yaw)


def limit_command(command, max_speed, max_yaw_rate):
    """평면 이동 방향을 유지하며 선속도 크기와 각속도를 제한한다."""
    # 제한 전 차체 선속도[m/s]와 각속도[rad/s]를 분리한다.
    vx, vy, yaw_rate = command
    # 평면 속도의 방향은 유지하고 크기만 제한한다. 작은 분모로 0 나눗셈을 막는다.
    scale = min(1.0, max_speed / max(math.hypot(vx, vy), 1.0e-12))
    # 선속도에는 공통 축소 비율을 적용하고 각속도는 양방향 한계로 제한한다.
    return vx * scale, vy * scale, max(-max_yaw_rate, min(max_yaw_rate, yaw_rate))


class TrackingController(Node):
    """플래너 궤적과 차량 피드백을 연결해 /cmd_vel을 발행하는 ROS 노드."""

    def __init__(self):
        # ROS 노드 이름을 등록한다.
        super().__init__("tracking_controller")
        # 제어 주기, 비례 이득, 속도 한계, 입력 유효 시간의 기본값이다.
        defaults = {
            "control_frequency_hz": 50.0,  # 제어 주파수[Hz].
            "kx": 3.0,  # 차체 전후 위치 오차의 비례 이득.
            "ky": 4.0,  # 차체 좌우 위치 오차의 비례 이득.
            "k_yaw": 2.0,  # 방향 오차의 비례 이득.
            "max_linear_speed": 5.0,  # 평면 속도 크기 상한[m/s].
            "max_yaw_rate": 1.0,  # 회전 속도 절댓값 상한[rad/s].
            "odom_timeout_sec": 0.25,  # 측정 위치의 최대 허용 나이[초].
            "command_timeout_sec": 0.25,  # 실행 명령의 최대 허용 나이[초].
            "mode_timeout_sec": 0.5,  # 모드 피드백의 최대 허용 나이[초].
        }
        # 검증을 통과한 수치 파라미터를 저장한다.
        values = {}
        # 각 기본값을 등록한 뒤 launch/CLI에서 덮어쓴 실제 값을 읽는다.
        for name, default in defaults.items():
            # ROS 파라미터를 기본값과 함께 선언한다.
            self.declare_parameter(name, default)
            # 수치 파라미터는 실수로 통일한다.
            value = float(self.get_parameter(name).value)
            # 주기·이득·한계·timeout은 모두 유한한 양수여야 한다.
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
            # 검증한 값을 이후 노드 설정에 사용한다.
            values[name] = value
        # 차체 좌표계 이름의 기본값을 선언한다.
        self.declare_parameter("base_frame", "base_link")
        # 명령 및 odometry의 차체 프레임을 검사할 기준이다.
        self.base_frame = str(self.get_parameter("base_frame").value)
        # 전후·좌우·방향 순서로 제어 이득을 저장한다.
        self.gains = (values["kx"], values["ky"], values["k_yaw"])
        # 평면 선속도 벡터의 최대 크기[m/s]이다.
        self.max_speed = values["max_linear_speed"]
        # yaw 각속도의 최대 절댓값[rad/s]이다.
        self.max_yaw_rate = values["max_yaw_rate"]
        # odometry 메시지를 유효하게 볼 최대 나이[초]이다.
        self.odom_timeout = values["odom_timeout_sec"]
        # 플래너 실행 명령을 유효하게 볼 최대 나이[초]이다.
        self.command_timeout = values["command_timeout_sec"]
        # 차량 모드 피드백을 유효하게 볼 최대 나이[초]이다.
        self.mode_timeout = values["mode_timeout_sec"]

        # 계획 ID로 궤적을 찾는 캐시이며 최근 계획 두 개까지만 보관한다.
        self.plans = {}
        # odometry, 플래너 명령, 모드 피드백을 미수신 상태로 둔다.
        self.odom = self.nominal = self.mode = None
        # 이번 제어 주기에 유효한 목표점과 오차가 아직 없음을 표시한다.
        self.reference = self.error = None
        # 직전 제어 시각을 저장해 ROS 시각의 역행을 감지한다.
        self.last_tick_ns = None
        # 마지막 발행 상태를 보관해 같은 상태를 반복 발행하지 않는다.
        self.status = ""

        # 늦게 연결된 구독자도 보관된 최신 샘플을 받을 수 있도록 QoS를 지정한다.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # 플래너가 만든 시간 궤적을 수신한다.
        self.create_subscription(Trajectory, "/planner/trajectory", self.on_trajectory, latched)
        # 현재 차량 위치와 방향을 수신한다.
        self.create_subscription(Odometry, "/odom", self.on_odom, 1)
        # 플래너의 실행 상태·계획 ID·nominal 속도를 함께 수신한다.
        self.create_subscription(ExecutedCommand, "/planner/executed_command", self.on_command, 1)
        # 차량의 현재/요청 모드와 정렬 완료 여부를 수신한다.
        self.create_subscription(DriveModeState, "/vehicle/drive_mode_state", self.on_mode, latched)
        # 최종 차체 속도 명령을 발행한다.
        self.command_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        # 추종·대기·정지 이유를 발행하고 최신 상태를 보관한다.
        self.status_pub = self.create_publisher(String, "/tracker/status", latched)
        # 현재 보간한 목표 pose를 시각화용으로 발행한다.
        self.reference_pub = self.create_publisher(PoseStamped, "/tracker/reference", 1)
        # 차체 축의 위치·방향 오차를 진단용으로 발행한다.
        self.error_pub = self.create_publisher(Vector3Stamped, "/tracker/error", 1)
        # 설정한 주파수[Hz]의 역수를 주기[초]로 삼아 제어를 반복한다.
        self.create_timer(1.0 / values["control_frequency_hz"], self.tick)

    def on_trajectory(self, message):
        """유효하고 순서가 맞는 새 궤적을 받아 최근 두 계획을 보관한다."""
        try:
            # 캐시에 넣기 전에 궤적 구조와 수치의 유효성을 검사한다.
            validate_trajectory(message)
        # 잘못된 궤적은 경고만 남기고 기존 캐시를 유지한다.
        except ValueError as error:
            # 검증 실패 이유를 ROS 로그에 남긴다.
            self.get_logger().warning(str(error))
            return
        # 중복되거나 과거 ID인 계획이 새 계획을 덮어쓰지 못하게 한다.
        if self.plans and message.plan_id <= max(self.plans):
            return
        # ID가 새로워도 시작 시각이 기존 최신 시작 시각보다 과거이면 거부한다.
        if self.plans and stamp_ns(message.start_time) < max(
            stamp_ns(plan.start_time) for plan in self.plans.values()
        ):
            return
        # 검증과 순서 검사를 통과한 궤적을 ID별로 보관한다.
        self.plans[message.plan_id] = message
        # 캐시가 두 개를 넘으면 가장 작은 ID의 계획을 제거한다.
        if len(self.plans) > 2:
            # 최근 계획 두 개를 남긴다.
            del self.plans[min(self.plans)]

    def on_odom(self, message):
        """수신 순서가 아닌 메시지 시각을 기준으로 최신 odometry를 보관한다."""
        # 처음 받은 odometry이거나 기존 값보다 타임스탬프가 같거나 최신이면 수용한다.
        if self.odom is None or stamp_ns(message.header.stamp) >= stamp_ns(self.odom.header.stamp):
            # 다음 제어 주기에서 사용할 측정값을 교체한다.
            self.odom = message

    def on_command(self, message):
        """플래너의 최신 실행 상태·계획 ID·속도 명령을 보관한다."""
        # 실행 명령도 메시지 시각 기준으로 역순 수신을 걸러낸다.
        if self.nominal is None or stamp_ns(message.header.stamp) >= stamp_ns(self.nominal.header.stamp):
            # 실행 상태와 계획 ID를 속도 명령과 함께 갱신한다.
            self.nominal = message

    def on_mode(self, message):
        """차량의 최신 모드 피드백을 보관한다."""
        # 모드 피드백도 메시지 시각 기준으로 역순 수신을 걸러낸다.
        if self.mode is None or stamp_ns(message.header.stamp) >= stamp_ns(self.mode.header.stamp):
            # 현재/요청 모드와 준비 상태를 함께 갱신한다.
            self.mode = message

    @staticmethod
    def fresh(message, now_ns, timeout):
        """메시지 시각이 현재 이하이며 허용 나이[초] 이내인지 검사한다."""
        # 미수신·미래 시각·timeout 초과 메시지는 무효로 본다(초를 나노초로 변환).
        return message is not None and 0 <= now_ns - stamp_ns(message.header.stamp) <= timeout * 1e9

    def calculate(self, now_ns):
        """입력을 검사하고 (차체 속도 명령, 상태)를 반환하며 진단값을 갱신한다."""
        # 이번 제어 주기에 유효한 목표점과 오차가 아직 없음을 표시한다.
        self.reference = self.error = None
        # 시뮬레이션 재시작 등으로 ROS 시각이 과거로 돌아갔는지 검사한다.
        if self.last_tick_ns is not None and now_ns < self.last_tick_ns:
            # 이전 시간축의 궤적은 더 이상 사용하지 않는다.
            self.plans.clear()
            # odometry, 플래너 명령, 모드 피드백을 미수신 상태로 둔다.
            self.odom = self.nominal = self.mode = None
            # 이번 시각을 다음 주기의 역행 검사 기준으로 기록한다.
            self.last_tick_ns = now_ns
            # 시간축이 바뀐 주기에는 정지 명령을 반환한다.
            return ZERO, "CLOCK_RESET"
        # 이번 시각을 다음 주기의 역행 검사 기준으로 기록한다.
        self.last_tick_ns = now_ns

        # 플래너 실행 명령이 없거나 오래되었으면 정지한다.
        if not self.fresh(self.nominal, now_ns, self.command_timeout):
            return ZERO, "WAIT_COMMAND"
        # 측정 위치가 없거나 오래되었으면 정지한다.
        if not self.fresh(self.odom, now_ns, self.odom_timeout):
            return ZERO, "WAIT_ODOM"
        # 차량 모드 피드백이 없거나 오래되었으면 정지한다.
        if not self.fresh(self.mode, now_ns, self.mode_timeout):
            return ZERO, "WAIT_MODE"

        # 이 주기에서 검사할 플래너의 nominal 실행 명령이다.
        nominal = self.nominal
        # 플래너 속도가 설정된 차체 좌표계 기준인지 검사한다.
        if nominal.header.frame_id != self.base_frame:
            return ZERO, "COMMAND_FRAME_MISMATCH"
        # 플래너 속도 명령에 NaN/무한대가 있으면 정지한다.
        if not all(math.isfinite(value) for value in (nominal.vx, nominal.vy, nominal.yaw_rate)):
            return ZERO, "INVALID_COMMAND"
        # 차량의 모드 정렬이 완료된 경우에만 진행한다.
        if self.mode.status != DriveModeState.STATUS_READY:
            return ZERO, "MODE_ALIGNING"
        # 현재 모드가 유효하고 요청 모드와 일치해야 한다.
        if (
            self.mode.current_mode not in range(5)
            or self.mode.current_mode != self.mode.requested_mode
        ):
            return ZERO, "MODE_MISMATCH"

        # 계획 추종과 특수 명령 전달을 구분할 실행 상태를 읽는다.
        state = nominal.execution_state
        # 정지·제자리 회전 상태에서는 추종 보정 없이 플래너 명령을 제한해 전달한다.
        if state in {"SAFETY_STOP", "MODE_STOP", "SPOT_TURN_ROTATING"}:
            # 제자리 회전 명령은 실제 차량도 SPOT_TURN 모드일 때만 허용한다.
            if (
                state == "SPOT_TURN_ROTATING"
                and self.mode.current_mode != DriveModeState.SPOT_TURN
            ):
                return ZERO, "MODE_MISMATCH"
            # 이 분기는 궤적 없이도 실행되며 nominal 명령에 속도 한계만 적용한다.
            return limit_command(
                (nominal.vx, nominal.vy, nominal.yaw_rate),
                self.max_speed,
                self.max_yaw_rate,
            ), state
        # 계획 추종 이외의 나머지 상태는 모두 0 속도를 출력한다.
        if state != "ACTIVE_PLAN":
            # 상태가 비어 있으면 실행 상태 대기 사유를 대신 보고한다.
            return ZERO, state or "WAIT_EXECUTION_STATE"

        # 실행 명령이 지정한 ID와 정확히 일치하는 궤적만 선택한다.
        plan = self.plans.get(nominal.plan_id)
        # 해당 계획이 아직 도착하지 않았으면 수신을 기다리며 정지한다.
        if plan is None:
            return ZERO, "WAIT_TRAJECTORY"
        # 전역 프레임과 차체 프레임이 모두 맞아야 하며 여기서는 TF 변환하지 않는다.
        if (
            self.odom.header.frame_id != plan.header.frame_id
            or self.odom.child_frame_id != self.base_frame
        ):
            return ZERO, "ODOM_FRAME_MISMATCH"

        # 수신 시각이 아닌 계획 시작 시각부터의 경과 시간[초]을 계산한다.
        elapsed = (now_ns - stamp_ns(plan.start_time)) * 1e-9
        # 현재 시각에 대응하는 목표 위치·방향·속도를 보간한다.
        reference = reference_at(plan, elapsed)
        # 계획 시작 전 또는 종료 후이면 추종할 목표점이 없다.
        if reference is None:
            # 시작 대기와 유효 구간 만료를 구분해 정지 사유를 반환한다.
            return ZERO, "TRAJECTORY_NOT_STARTED" if elapsed < 0.0 else "TRAJECTORY_EXPIRED"
        # 궤적이 요구하는 모드와 차량의 실제 모드가 일치하는지 확인한다.
        if reference.mode != self.mode.current_mode:
            return ZERO, "MODE_MISMATCH"

        # odometry에서 실제 위치와 quaternion 자세를 꺼낸다.
        pose = self.odom.pose.pose
        # quaternion의 x/y/z/w 성분으로 평면 자세의 유효성을 검사한다.
        q = pose.orientation
        # 사용할 위치 및 quaternion 성분이 모두 유한해야 한다.
        if not all(math.isfinite(v) for v in (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w)):
            return ZERO, "INVALID_ODOM"
        # 평면 회전에서 쓰는 quaternion z/w 성분의 크기를 구한다.
        norm = math.hypot(q.z, q.w)
        # 퇴화한 quaternion이나 x/y 성분이 큰 비평면 자세는 거부한다.
        if norm < 1e-12 or abs(q.x) > 1e-6 or abs(q.y) > 1e-6:
            return ZERO, "INVALID_ODOM"
        # z/w 성분을 정규화한 뒤 현재 차체 방향[rad]을 복원한다.
        yaw = 2.0 * math.atan2(q.z / norm, q.w / norm)

        # 목표점과 실제 pose의 차이로 차체 속도 보정 명령을 계산한다.
        command, error = tracking_command(
            reference, (pose.position.x, pose.position.y, yaw), self.gains
        )
        # 계산 결과가 NaN/무한대이면 잘못된 속도를 내보내지 않는다.
        if not all(math.isfinite(value) for value in command):
            return ZERO, "INVALID_CONTROL"
        # 정상적으로 계산된 목표점과 오차만 이번 주기 진단값으로 보관한다.
        self.reference, self.error = reference, error
        # 속도 한계를 적용한 최종 명령과 정상 추종 상태를 반환한다.
        return limit_command(command, self.max_speed, self.max_yaw_rate), "TRACKING"

    def tick(self):
        """매 제어 주기마다 속도를 발행하고 상태 변화 및 추종 진단값을 알린다."""
        # 제어 계산과 진단 메시지에 같은 ROS 시각을 사용한다.
        now = self.get_clock().now()
        # 입력 검사와 제어 계산을 수행해 속도 및 상태를 얻는다.
        command, status = self.calculate(now.nanoseconds)
        # 최종 속도를 담을 ROS 메시지를 만든다. 미사용 축은 기본값 0이다.
        message = Twist()
        # 차체 x/y 선속도와 z축 회전 속도를 채운다.
        message.linear.x, message.linear.y, message.angular.z = command
        # 대기·오류 시의 0 속도도 매 제어 주기마다 발행한다.
        self.command_pub.publish(message)

        # 상태가 바뀔 때만 상태 토픽과 로그를 갱신한다.
        if status != self.status:
            # 중복 발행을 막기 위해 이번 상태를 기억한다.
            self.status = status
            # 새 상태를 구독자에게 알린다.
            self.status_pub.publish(String(data=status))
            # 상태 변화 이유를 로그에도 남긴다.
            self.get_logger().info(status)

        # 정상 추종에서 목표점이 계산된 경우에만 진단 메시지를 발행한다.
        if self.reference is not None:
            # 보간 목표점을 시각화할 pose 메시지를 만든다.
            reference = PoseStamped()
            # 이번 제어 주기의 시각을 기록한다.
            reference.header.stamp = now.to_msg()
            # 목표 위치는 odometry와 같은 전역 좌표계로 표시한다.
            reference.header.frame_id = self.odom.header.frame_id
            # 보간한 목표 x 위치[m]를 채운다.
            reference.pose.position.x = self.reference.x
            # 보간한 목표 y 위치[m]를 채운다.
            reference.pose.position.y = self.reference.y
            # 평면 yaw를 quaternion의 z 성분으로 변환한다(x/y는 0).
            reference.pose.orientation.z = math.sin(self.reference.yaw / 2.0)
            # 평면 yaw를 quaternion의 w 성분으로 변환한다.
            reference.pose.orientation.w = math.cos(self.reference.yaw / 2.0)
            # 현재 시각의 목표 pose를 발행한다.
            self.reference_pub.publish(reference)
            # 차체 좌표계의 추종 오차를 담을 진단 메시지를 만든다.
            error = Vector3Stamped()
            # 목표점과 같은 제어 시각을 기록한다.
            error.header.stamp = now.to_msg()
            # 위치 오차가 현재 차체 축 기준임을 표시한다.
            error.header.frame_id = self.base_frame
            # x/y는 위치 오차[m], z는 높이가 아닌 yaw 오차[rad]이다.
            error.vector.x, error.vector.y, error.vector.z = self.error
            # 위치·방향 오차를 진단 토픽으로 발행한다.
            self.error_pub.publish(error)


def main(args=None):
    """ROS 노드를 실행하고 종료 시 정지 명령 및 자원 정리를 수행한다."""
    # ROS 통신 컨텍스트를 초기화한다.
    rclpy.init(args=args)
    # 파라미터·입출력 토픽·제어 타이머를 구성한다.
    node = TrackingController()
    try:
        # 종료 요청까지 수신 콜백과 주기 제어 콜백을 실행한다.
        rclpy.spin(node)
    # Ctrl+C 또는 외부 ROS 종료 요청을 정상 종료로 처리한다.
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # 종료 예외는 전파하지 않고 아래 정리 절차로 넘어간다.
        pass
    finally:
        # ROS 컨텍스트가 살아 있을 때만 마지막 정지 명령을 발행한다.
        if rclpy.ok():
            # 모든 속도 성분이 0인 종료 명령을 보낸다.
            node.command_pub.publish(Twist())
        # 타이머와 publisher/subscription 등 노드 자원을 정리한다.
        node.destroy_node()
        # 이미 종료된 경우도 허용하며 ROS 컨텍스트를 종료한다.
        rclpy.try_shutdown()


# 모듈을 직접 실행한 경우에만 ROS 노드를 시작한다.
if __name__ == "__main__":
    # 노드 실행 진입점을 호출한다.
    main()
