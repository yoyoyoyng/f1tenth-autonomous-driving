#!/usr/bin/env python3
"""
Pure Pursuit Controller — F1TENTH RC카용
=========================================
전방 lookahead 포인트를 향해 조향하는 Pure Pursuit 알고리즘 기반 컨트롤러.
속도는 전방 경로 곡률 + 현재 조향각으로 계산하며, 비대칭 조향 rate limit으로
코너 직전 반대조향(counter-steer)을 억제한다.

[직진 안쪽 벽 붙는 문제 관련 파라미터 요약]
  원인1: LOOKAHEAD_MAX가 너무 낮아 직선에서 타겟이 너무 가까워 미세 조향 진동 발생
  원인2: STEER_WIND_RATE가 높아 직선에서 조향이 빠르게 커질 수 있음
  원인3: CORNER_EXIT_HOLD_SEC 직후 lookahead가 짧게 유지되며 안쪽으로 향함
  원인4: LOOKAHEAD_GAIN이 낮아 고속에서도 lookahead가 충분히 늘어나지 않음
  → 수정 우선순위: LOOKAHEAD_MAX ↑, LOOKAHEAD_GAIN ↑, STEER_WIND_RATE ↓
"""
import os, csv, math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf_transformations import euler_from_quaternion

# =====================================================================
#  설정값 (파라미터 튜닝 구간)
# =====================================================================

# ── 프레임 / 토픽 이름 ───────────────────────────────────────────────
GLOBAL_FRAME   = "map"      # 전역 좌표계
BASE_FRAME     = "base_link"  # 차량 로컬 좌표계
WAYPOINT_CSV   = os.path.join(os.path.dirname(__file__), "waypoints.csv")
ODOM_TOPIC     = "/odom"
DRIVE_TOPIC    = "/drive"
WAYPOINTS_MARKER_TOPIC     = "/waypoints_marker"
TARGET_MARKER_TOPIC        = "/pp_target_point"
LOOKAHEAD_LINE_TOPIC       = "/pp_lookahead_line"
SPEED_PREVIEW_MARKER_TOPIC = "/pp_speed_preview_path"
STEERING_ARC_MARKER_TOPIC  = "/pp_steering_arc"

CONTROL_RATE = 50.0   # 제어 루프 주파수 [Hz]. 높을수록 응답 빠름, CPU 부하 증가

WHEELBASE = 0.33      # 앞뒤 바퀴 축간 거리 [m]. 실차 측정값으로 정확히 맞출 것
                      # 틀리면 조향각-곡률 환산이 틀어져 모든 코너에서 오차 발생

# ── Lookahead 거리 설정 ──────────────────────────────────────────────
# Pure Pursuit의 핵심 파라미터. lookahead = 차량이 목표로 삼는 전방 거리.
# 너무 짧으면 → 진동/지그재그/안쪽 overshoot
# 너무 길면  → 코너 shortcut (apex 못 따라감)
#
# [직진 안쪽 벽 문제] LOOKAHEAD_MAX가 1.00으로 너무 낮음.
# 3~5 m/s 고속에서 1m 앞만 보면 미세한 경로 편차에도 급하게 반응해
# 조향 진동이 생기고 한쪽 벽으로 쏠릴 수 있음.
# → LOOKAHEAD_MAX를 1.5~2.5로 올리는 것을 강력 권장
#
# [직진 안쪽 벽 문제] LOOKAHEAD_GAIN이 0.18로 낮음.
# base_lookahead = LOOKAHEAD_MIN + LOOKAHEAD_GAIN × speed
# 속도 4 m/s일 때: 0.75 + 0.18×4 = 1.47 → 그러나 MAX=1.00에 클램프됨
# 결국 고속에서도 lookahead가 1.00에 묶여버림.
# → LOOKAHEAD_GAIN 0.25~0.35로 올리고 LOOKAHEAD_MAX 2.0으로 올릴 것
LOOKAHEAD_MIN  = 0.75    # lookahead 하한 [m]. 정지/극저속 시 최소 전방 거리
LOOKAHEAD_MAX  = 3.00    # lookahead 상한 [m] ← 직진 벽 문제: 너무 낮음. 1.5~2.5 권장
LOOKAHEAD_GAIN = 0.4  # 속도 비례 lookahead 증가율 [m/(m/s)] ← 0.25~0.35 권장

MAX_STEER = 0.7          # 조향각 절대 상한 [rad] (~40°). 서보 한계에 맞게 조정

WAYPOINTS_INTERVAL = 1   # CSV에서 몇 번째 줄마다 waypoint를 읽을지.
                          # 1 = 전부 읽음 (코너 형상 최대 보존)
                          # 2 이상으로 올리면 경로가 거칠어져 속도 계산 노이즈 증가

# ── 속도 제어 상수 ───────────────────────────────────────────────────
# 속도 결정 공식:
#   path_curvature     = 전방 SPEED_PREVIEW_DISTANCE 내 경로 곡률 (상위 CURVATURE_PERCENTILE%)
#   steering_curvature = abs(tan(steering) / WHEELBASE)
#   effective_curvature = max(path_curvature, STEER_CURVATURE_WEIGHT × steering_curvature)
#   target_speed        = sqrt(LATERAL_ACCEL_LIMIT / effective_curvature)
#   → SPEED_MIN~SPEED_MAX 사이로 클램프
#
# 직진 속도가 2초반에 묶이는 이유:
#   LATERAL_ACCEL_LIMIT=1.5, effective_curvature≈0.3 → sqrt(1.5/0.3) = 2.24 m/s
#   → LATERAL_ACCEL_LIMIT를 올리고 SPEED_MAX도 같이 올려야 함
SPEED_MAX              = 3.50    # 최고 목표 속도 [m/s] ← 직진 빠르게: 4.0~5.0
SPEED_MIN              = 0.80    # 최저 목표 속도 [m/s]. 급코너에서도 이 속도는 유지
SPEED_PREVIEW_DISTANCE = 3.0    # 전방 이 거리[m] 안의 경로 곡률을 속도 결정에 사용
                                  # 길수록 더 일찍 감속, 짧으면 코너 직전에야 감속
CURVATURE_SAMPLE_SPAN  = 3       # 3점 곡률 계산 시 양쪽 샘플 간격 (waypoint 인덱스 단위)
                                  # 클수록 노이즈 억제, 작을수록 코너 형상 민감하게 반응
LATERAL_ACCEL_LIMIT    = 2.50    # 허용 횡가속도 [m/s²] ← 전체 속도 결정의 핵심
                                  # 이 값 하나가 직선/코너 속도 모두를 지배함
                                  # 직진 속도 높이려면 2.5~4.0으로 올릴 것
                                  # 코너에서 밖으로 밀리면 낮출 것
STEER_CURVATURE_WEIGHT = 0.30    # 현재 조향각을 곡률로 환산해 effective_curvature에 반영하는 비율
                                  # 직선에서 조향이 조금만 있어도 속도를 낮추는 원인
                                  # 직진 속도 안 나오면 0.3~0.4로 낮출 것
CURVATURE_PERCENTILE   = 80.0    # 전방 경로 곡률 샘플 중 상위 N%를 사용
                                  # 100 = 최대값 사용 (노이즈에 민감)
                                  # 75~85로 낮추면 waypoint 노이즈성 급감속 줄어듦

# ── RViz 디버그 마커 (제어 로직에 영향 없음) ──────────────────────────
STEERING_ARC_LENGTH      = 3.00  # 예상 궤적 표시 길이 [m]
STEERING_ARC_SAMPLE_STEP = 0.08  # 예상 궤적 포인트 간격 [m]

# ── 가/감속 rate ────────────────────────────────────────────────────
# limit_speed_rate()에서 매 루프마다 속도 변화를 이 값으로 클램프함
# dt = 1/50 = 0.02s 기준으로 실제 가속도 = rate × dt
MAX_ACCEL         = 4.50   # 가속 한계 [m/s²]. 직선 진입 후 가속 속도
                            # 코너 탈출 후 가속이 느리면 올릴 것 (5.0~8.0)
MAX_DECEL         = 8.00   # 감속 한계 [m/s²]. 코너 전 제동 강도
                            # 코너에서 속도가 안 줄어 밖으로 밀리면 올릴 것 (12~20)
CORNER_EXIT_HOLD_SEC = 0.60  # 코너 탈출 직후 고속 복귀를 지연시키는 시간 [s]
                               # 이 시간 동안 lookahead가 1.70 이하로 제한됨
                               # 탈출 직후 안쪽 벽 칠 때 → 올리기 (0.5~0.8)
                               # 직선 가속이 늦을 때 → 줄이기 (0.1~0.2)

# ── Lookahead smoothing caps ─────────────────────────────────────────
# 코너에서 lookahead를 너무 줄이면 늦게/급하게 꺾어 apex 안쪽 overshoot 발생
# 각 코너 레벨에서 lookahead의 상한값을 설정
APPROACH_LOOKAHEAD_CAP = 1.20  # 코너 접근(level 1) 시 lookahead 상한 [m]
                                # 낮추면 더 일찍 꺾어 안쪽 컷, 올리면 늦게 완만하게 꺾음
CORNER_LOOKAHEAD_CAP   = 1.15  # 코너 진행(level 2) 시 lookahead 상한 [m]
                                # 낮추면 코너 안쪽 정밀 추종, 높으면 코너 shortcut

# ── 코너 감지 기준 (lookahead 안정화 전용) ────────────────────────────
# 전방 40 waypoint의 누적 꺾임각(turn_sum_abs)으로 코너 레벨을 결정
# 속도 계산에는 사용하지 않고, lookahead cap 선택에만 사용
PRECORNER_TURN_SUM_DEG = 26.0  # 이 각도 초과 → corner_level = 1 (코너 접근)
CORNER_TURN_SUM_DEG    = 55.0  # 이 각도 초과 → corner_level = 2 (코너 진행)
                                # 낮추면 더 일찍/자주 코너로 인식 (보수적 감속)
                                # 높이면 더 늦게/드물게 코너 인식 (공격적 속도)

# ── 비대칭 조향 rate limit ───────────────────────────────────────────
# 조향각의 변화 속도를 방향에 따라 다르게 제한
# UNWIND(풀기: 조향이 0 방향으로 줄어드는 것): 빠르게 → 코너 후 직진 복귀 빠름
# WIND  (꺾기: 조향이 커지거나 반대 방향으로 바뀌는 것): 느리게 → counter-steer 억제
#
# [직진 안쪽 벽 문제] STEER_WIND_RATE = 4.5 rad/s
# 직선에서 조향이 약간 발생하면 4.5 rad/s 속도로 빠르게 커질 수 있음
# 50Hz 기준 한 틱에 최대 4.5/50 = 0.09 rad씩 꺾임 허용 → 직선 진동 원인
# → STEER_WIND_RATE를 2.0~3.0으로 낮추면 직선 조향 진동 줄어듦
STEER_UNWIND_RATE = 14.00   # [rad/s] 조향 0으로 풀리는 속도 — 빠르게
STEER_WIND_RATE   =  2.0   # [rad/s] 조향 키우기/반대로 꺾기 속도 — 느리게
                              # ← 직진 안쪽 벽 문제: 2.0~3.0으로 낮출 것

# ── Lookahead rate limit ─────────────────────────────────────────────
# lookahead 거리가 급격히 변하면 타겟 포인트가 튀어 조향 진동 발생
# 변화 속도를 제한해 부드럽게 전환
LOOKAHEAD_DROP_RATE = 4.50  # [m/s²] 코너 진입 시 lookahead 줄어드는 속도
                              # 낮추면 더 천천히 줄어들어 코너 진입 부드러움
LOOKAHEAD_RISE_RATE = 6.50  # [m/s²] 코너 탈출 후 lookahead 늘어나는 속도
                              # 높이면 직선 타겟을 빠르게 앞에 얹어 조향 빨리 풀림


class PurePursuitOnly(Node):
    def __init__(self):
        super().__init__("pure_pursuit_only_node")

        # waypoint 로드: CSV에서 x, y 좌표만 읽음
        self.wp_x, self.wp_y = self.load_waypoints(WAYPOINT_CSV, WAYPOINTS_INTERVAL)
        self.num_waypoints = len(self.wp_x)

        # TF2 리스너: map→base_link 변환으로 차량 위치 취득
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 상태 변수 초기화
        self.current_x      = 0.0   # 현재 차량 x 위치 [m] (map 프레임)
        self.current_y      = 0.0   # 현재 차량 y 위치 [m] (map 프레임)
        self.current_yaw    = 0.0   # 현재 차량 heading [rad]
        self.current_speed  = 0.0   # 현재 속도 [m/s] (odom에서 취득)
        self.prev_steering  = 0.0   # 이전 루프의 조향 명령 [rad] (rate limit용)
        self.prev_cmd_speed = 0.0   # 이전 루프의 속도 명령 [m/s] (rate limit용)
        self.corner_hold_ticks = 0  # 코너 탈출 후 hold 남은 틱 수
        self.smoothed_lookahead = LOOKAHEAD_MIN  # rate limit 적용된 현재 lookahead [m]
        self.prev_x        = 0.0    # 이전 루프 x (TF 점프 감지용)
        self.prev_y        = 0.0    # 이전 루프 y (TF 점프 감지용)
        self.closest_idx   = 0      # 현재 차량에 가장 가까운 waypoint 인덱스
        self.target_x      = None   # 현재 lookahead 타겟 포인트 x (마커용)
        self.target_y      = None   # 현재 lookahead 타겟 포인트 y (마커용)
        self.first_loop    = True   # 첫 루프 여부 (closest_idx 전체 탐색용)

        # 퍼블리셔
        self.drive_pub            = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.waypoints_marker_pub = self.create_publisher(Marker, WAYPOINTS_MARKER_TOPIC, 10)
        self.target_marker_pub    = self.create_publisher(Marker, TARGET_MARKER_TOPIC, 10)
        self.line_marker_pub      = self.create_publisher(Marker, LOOKAHEAD_LINE_TOPIC, 10)
        self.speed_preview_marker_pub = self.create_publisher(
            Marker, SPEED_PREVIEW_MARKER_TOPIC, 10)
        self.steering_arc_marker_pub = self.create_publisher(
            Marker, STEERING_ARC_MARKER_TOPIC, 10)

        # 서브스크라이버: odom에서 현재 속도 취득
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_callback, 10)

        # 제어 루프 타이머 (50 Hz)
        self.timer = self.create_timer(1.0 / CONTROL_RATE, self.control_loop)

        self.get_logger().info("Pure Pursuit ANTI-APEX-CUT node started.")
        self.get_logger().info(f"Waypoints: {self.num_waypoints} from {WAYPOINT_CSV}")
        self.get_logger().info(f"TF: {GLOBAL_FRAME} -> {BASE_FRAME}")
        self.get_logger().info(
            f"SPEED: max={SPEED_MAX} min={SPEED_MIN} "
            f"preview={SPEED_PREVIEW_DISTANCE}m lat_accel={LATERAL_ACCEL_LIMIT} | "
            f"Ld corner_cap={CORNER_LOOKAHEAD_CAP} approach_cap={APPROACH_LOOKAHEAD_CAP}")

    # -----------------------------------------------------------------
    def load_waypoints(self, path, interval=1):
        """
        CSV에서 waypoint x, y 좌표를 읽는다.
        - 속도 컬럼이 있어도 무시 (속도는 곡률로 자체 계산)
        - interval > 1이면 일부 waypoint를 건너뜀 (경로 거칠어질 수 있음)
        """
        if not os.path.exists(path):
            raise FileNotFoundError(f"Waypoint CSV not found: {path}")
        xs, ys = [], []
        with open(path, "r") as f:
            reader = csv.reader(f)
            for i, row in enumerate(reader):
                if not row:
                    continue
                if any(c.isalpha() for c in row[0]):   # 헤더 행 스킵
                    continue
                if i % interval != 0:                  # interval 간격으로 샘플링
                    continue
                vals = [float(v) for v in row]
                xs.append(vals[0])
                ys.append(vals[1])
        if len(xs) < 3:
            raise ValueError("At least 3 waypoints required.")
        return np.array(xs), np.array(ys)

    # -----------------------------------------------------------------
    def global_to_local(self, gx, gy):
        """
        전역 좌표(map 프레임)를 차량 로컬 좌표로 변환.
        로컬 좌표: x=전방, y=좌측
        Pure Pursuit은 로컬 y값(횡방향 오차)을 이용해 조향각을 계산함.
        """
        dx = gx - self.current_x
        dy = gy - self.current_y
        cos_y = math.cos(-self.current_yaw)
        sin_y = math.sin(-self.current_yaw)
        return dx*cos_y - dy*sin_y, dx*sin_y + dy*cos_y

    def local_to_global(self, lx, ly):
        """
        차량 로컬 좌표를 전역 좌표로 변환.
        RViz 마커 표시용 (제어 로직에는 직접 사용 안 함).
        """
        cos_y = math.cos(self.current_yaw)
        sin_y = math.sin(self.current_yaw)
        return (self.current_x + lx*cos_y - ly*sin_y,
                self.current_y + lx*sin_y + ly*cos_y)

    # -----------------------------------------------------------------
    def odom_callback(self, msg):
        """
        odom 메시지에서 현재 속도를 취득.
        vx, vy를 합산해 실제 주행 속도를 구함.
        속도 기반 lookahead 계산과 로그 출력에 사용됨.
        """
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.hypot(vx, vy)

    def get_map_pose(self):
        """
        TF2에서 map→base_link 변환을 읽어 차량 위치·방향을 취득.
        TF가 없으면 None 반환 → 제어 루프에서 정지 명령 발행.
        Time(0,0) = 가장 최신 TF 사용 (실시간 위치).
        """
        try:
            t = self.tf_buffer.lookup_transform(
                GLOBAL_FRAME, BASE_FRAME,
                Time(seconds=0, nanoseconds=0))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF 없음 {GLOBAL_FRAME}->{BASE_FRAME}: {e}",
                throttle_duration_sec=2.0)
            return None
        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return x, y, yaw

    # -----------------------------------------------------------------
    def find_closest_idx(self):
        """
        현재 차량 위치에 가장 가까운 waypoint 인덱스를 찾는다.

        전략:
        - 첫 루프: 전체 waypoint에서 최소거리 탐색 (O(n))
        - 이후: 이전 closest 주변 ±5~+15 범위에서만 탐색 (O(1), 고속 대응)
        - TF 점프(>1.5m) 감지 시: 전체 재탐색으로 리셋

        [직진 안쪽 벽 문제와 관련]
        closest_idx가 실제 차량보다 뒤처지면 find_target_local에서
        이미 지나친 waypoint 방향으로 타겟이 잡혀 안쪽으로 꺾일 수 있음.
        탐색 범위 range(-5, 16)에서 전방 탐색(+16)이 후방(-5)보다 넓어
        고속에서 closest_idx가 잘 따라오도록 설계됨.
        """
        dx = self.wp_x - self.current_x
        dy = self.wp_y - self.current_y
        d2 = dx*dx + dy*dy

        if self.first_loop:
            self.first_loop  = False
            self.closest_idx = int(np.argmin(d2))
            self.prev_x      = self.current_x
            self.prev_y      = self.current_y
            return self.closest_idx

        n = self.num_waypoints

        # TF 위치 점프 감지: 1.5m 이상 순간이동하면 전체 재탐색
        jump = math.hypot(self.current_x - self.prev_x,
                          self.current_y - self.prev_y)
        if jump > 1.5:
            self.closest_idx = int(np.argmin(d2))
            self.prev_x = self.current_x
            self.prev_y = self.current_y
            return self.closest_idx

        self.prev_x = self.current_x
        self.prev_y = self.current_y

        # 주변 탐색: 후방 5개 ~ 전방 15개 waypoint만 확인
        best_idx = self.closest_idx
        min_d2   = d2[self.closest_idx]
        for i in range(-5, 16):
            idx = (self.closest_idx + i) % n
            if d2[idx] < min_d2:
                min_d2   = d2[idx]
                best_idx = idx

        self.closest_idx = best_idx
        return best_idx

    # -----------------------------------------------------------------
    def find_target_local(self, lookahead):
        """
        lookahead 원과 경로의 교점을 로컬 좌표로 반환한다.

        동작:
        1. closest_idx부터 최대 40 waypoint를 수집해 경로 polyline 구성
        2. _intersect_circle()로 lookahead 원과의 교점 탐색
        3. 교점 없으면 fallback1: 가장 가까운 전방 포인트
        4. 그것도 없으면 fallback2: closest+3번째 포인트 강제 타겟

        [직진 안쪽 벽 문제와 관련]
        lookahead가 짧으면(1.00m) 수집하는 polyline도 짧아져
        교점이 차량 바로 앞 1m에 잡힘 → 아주 작은 경로 편차에도 크게 반응
        → LOOKAHEAD_MAX를 높여 타겟을 충분히 앞에 잡아야 안정됨
        """
        n    = self.num_waypoints
        poly = []
        idx  = self.closest_idx

        # lookahead 원을 벗어날 때까지 포인트 수집 (최대 40개)
        max_search_steps = min(n, 40)
        for _ in range(max_search_steps):
            lx, ly = self.global_to_local(self.wp_x[idx], self.wp_y[idx])
            poly.append((lx, ly))
            # 충분히 수집하고 lookahead보다 멀어지면 중단 (1.35배 여유)
            if len(poly) >= 6 and math.hypot(lx, ly) >= lookahead * 1.35:
                break
            idx = (idx + 1) % n

        hit = self._intersect_circle(poly, lookahead)
        if hit is not None:
            return hit

        # fallback1: 전방 포인트 중 lookahead에 거리가 가장 가까운 것
        forward = [(lx, ly) for (lx, ly) in poly if lx > 0.0]
        if forward:
            return min(forward, key=lambda p: abs(math.hypot(p[0], p[1]) - lookahead))

        # fallback2: 3 waypoint 앞 포인트 강제 타겟 (후진/U턴 등 극단 상황)
        fwd_idx = (self.closest_idx + 3) % n
        lx, ly = self.global_to_local(self.wp_x[fwd_idx], self.wp_y[fwd_idx])
        return lx, ly

    def _intersect_circle(self, poly, lookahead):
        """
        polyline(경로 선분 목록)과 반지름 lookahead인 원의 교점을 찾는다.
        경로 순서 기준으로 첫 번째(가장 가까운 전방) 교점을 반환.
        로컬 x > 0.03 조건: 차량 뒤쪽 교점은 무시.

        수학적 원리:
        선분 P(t) = A + t*(B-A), t∈[0,1]
        |P(t)|² = lookahead² 를 풀면 2차 방정식 → 판별식(disc) ≥ 0이면 교점 존재
        """
        L2 = lookahead * lookahead

        for i in range(len(poly) - 1):
            ax, ay = poly[i];  bx, by = poly[i+1]
            dx, dy = bx-ax, by-ay
            a = dx*dx + dy*dy
            if a < 1e-12:
                continue

            b    = 2.0*(ax*dx + ay*dy)
            c    = ax*ax + ay*ay - L2
            disc = b*b - 4.0*a*c
            if disc < 0.0:
                continue

            sq = math.sqrt(disc)
            candidates = []
            for t in ((-b-sq)/(2*a), (-b+sq)/(2*a)):
                if 0.0 <= t <= 1.0:
                    px, py = ax+t*dx, ay+t*dy
                    if px > 0.03:   # 차량 전방 포인트만
                        candidates.append((t, px, py))

            if candidates:
                # t가 작은 것 = 선분 시작점에 가까운 것 = 경로상 먼저 만나는 교점
                _, px, py = min(candidates, key=lambda v: v[0])
                return px, py

        return None

    # -----------------------------------------------------------------
    #  속도 로직: 전방 경로 곡률 + 현재 조향각으로 목표 속도 결정
    # -----------------------------------------------------------------
    @staticmethod
    def _three_point_curvature(ax, ay, bx, by, cx, cy):
        """
        세 점을 지나는 원의 곡률 절댓값 [1/m]을 계산한다.
        공식: κ = 2|cross| / (|AB| × |BC| × |AC|)
        여기서 cross = (B-A) × (C-A) (2D 외적)
        곡률이 클수록 더 급한 코너.
        """
        ab = math.hypot(bx - ax, by - ay)
        bc = math.hypot(cx - bx, cy - by)
        ac = math.hypot(cx - ax, cy - ay)
        denominator = ab * bc * ac
        if denominator < 1e-9:
            return 0.0

        cross = abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax))
        return 2.0 * cross / denominator

    def get_preview_path_curvature(self):
        """
        closest_idx부터 SPEED_PREVIEW_DISTANCE까지 전방 경로의 곡률을 계산한다.

        - 각 waypoint에서 CURVATURE_SAMPLE_SPAN 간격으로 3점 곡률을 계산
        - 가까운 포인트는 100% 반영, 먼 포인트는 최소 65% 반영 (거리 가중치)
        - 전체 샘플의 상위 CURVATURE_PERCENTILE%를 반환 (노이즈 억제)

        [직진 안쪽 벽 문제와 관련]
        직선 구간에서 이 함수가 0에 가까운 값을 반환하면
        calculate_target_speed에서 SPEED_MAX까지 속도가 올라감.
        하지만 waypoint 노이즈로 간간이 높은 곡률이 나오면
        순간적으로 속도를 낮춰 불안정해짐.
        → CURVATURE_PERCENTILE을 80 정도로 낮추면 노이즈 내성 증가
        """
        n = self.num_waypoints
        span = CURVATURE_SAMPLE_SPAN
        weighted_curvatures = []
        accumulated_distance = 0.0
        previous_idx = self.closest_idx

        for step in range(1, n):
            idx = (self.closest_idx + step) % n
            segment_length = math.hypot(
                self.wp_x[idx] - self.wp_x[previous_idx],
                self.wp_y[idx] - self.wp_y[previous_idx])
            accumulated_distance += segment_length
            previous_idx = idx

            if accumulated_distance > SPEED_PREVIEW_DISTANCE:
                break

            # 3점 곡률: idx 앞뒤로 span 간격의 waypoint 사용
            idx_a = (idx - span) % n
            idx_b = idx
            idx_c = (idx + span) % n

            curvature = self._three_point_curvature(
                self.wp_x[idx_a], self.wp_y[idx_a],
                self.wp_x[idx_b], self.wp_y[idx_b],
                self.wp_x[idx_c], self.wp_y[idx_c])

            # 거리 가중치: 가까울수록 100%, 멀수록 65%까지 감소
            distance_ratio = accumulated_distance / SPEED_PREVIEW_DISTANCE
            distance_weight = 1.0 - 0.35 * float(np.clip(distance_ratio, 0.0, 1.0))
            weighted_curvatures.append(curvature * distance_weight)

        if not weighted_curvatures:
            return 0.0

        return float(np.percentile(weighted_curvatures, CURVATURE_PERCENTILE))

    def calculate_target_speed(self, steering):
        """
        목표 속도를 계산한다.

        공식: v = sqrt(LATERAL_ACCEL_LIMIT / effective_curvature)
        - effective_curvature = max(경로 곡률, STEER_CURVATURE_WEIGHT × 조향 곡률)
        - 경로 곡률과 조향 곡률 중 더 큰 값을 사용해 보수적으로 속도를 결정
        - SPEED_MIN~SPEED_MAX 사이로 클램프

        직선(곡률≈0): effective_curvature < 1e-6 → SPEED_MAX 반환
        코너(곡률 큼): sqrt(LATERAL_ACCEL_LIMIT / 큰값) → 느린 속도

        예시:
          LATERAL_ACCEL_LIMIT=1.5, effective_curvature=0.3 → sqrt(5.0) = 2.24 m/s
          LATERAL_ACCEL_LIMIT=3.5, effective_curvature=0.3 → sqrt(11.7) = 3.42 m/s
        """
        path_curvature     = self.get_preview_path_curvature()
        steering_curvature = abs(math.tan(steering) / WHEELBASE)

        # 경로 곡률과 조향 곡률 중 큰 값을 사용 (더 보수적인 속도 선택)
        effective_curvature = max(
            path_curvature,
            STEER_CURVATURE_WEIGHT * steering_curvature)

        # 직선 구간: 곡률이 0에 가까우면 최고속 반환
        if effective_curvature < 1e-6:
            return SPEED_MAX

        target_speed = math.sqrt(LATERAL_ACCEL_LIMIT / effective_curvature)
        return float(np.clip(target_speed, SPEED_MIN, SPEED_MAX))

    # -----------------------------------------------------------------
    #  RViz 디버그 마커용 (제어값에 영향 없음)
    # -----------------------------------------------------------------
    def get_speed_preview_path_points(self):
        """
        속도 계산이 참조하는 전방 waypoint 경로를 RViz 마커용으로 반환.
        주황색 선으로 표시됨. 속도 계산 범위 시각화용.
        """
        n = self.num_waypoints
        points = [Point(
            x=float(self.wp_x[self.closest_idx]),
            y=float(self.wp_y[self.closest_idx]),
            z=0.12)]

        accumulated_distance = 0.0
        previous_idx = self.closest_idx

        for step in range(1, n):
            idx = (self.closest_idx + step) % n
            segment_length = math.hypot(
                self.wp_x[idx] - self.wp_x[previous_idx],
                self.wp_y[idx] - self.wp_y[previous_idx])
            accumulated_distance += segment_length

            if accumulated_distance > SPEED_PREVIEW_DISTANCE:
                break

            points.append(Point(
                x=float(self.wp_x[idx]),
                y=float(self.wp_y[idx]),
                z=0.12))
            previous_idx = idx

        return points

    def get_steering_arc_points(self, steering):
        """
        현재 조향각을 유지할 때의 bicycle model 예상 궤적을 반환.
        보라색 선으로 표시됨. 조향 미래 경로 시각화용.

        bicycle model:
          - 곡률 κ = tan(δ) / L  (δ=조향각, L=wheelbase)
          - 호 길이 s에서 위치: x=sin(κs)/κ, y=(1-cos(κs))/κ
        """
        points = []
        curvature = math.tan(float(steering)) / WHEELBASE
        sample_count = max(2, int(STEERING_ARC_LENGTH / STEERING_ARC_SAMPLE_STEP) + 1)

        for i in range(sample_count):
            s = min(i * STEERING_ARC_SAMPLE_STEP, STEERING_ARC_LENGTH)

            if abs(curvature) < 1e-6:
                lx = s
                ly = 0.0
            else:
                heading_change = curvature * s
                lx = math.sin(heading_change) / curvature
                ly = (1.0 - math.cos(heading_change)) / curvature

            gx, gy = self.local_to_global(lx, ly)
            points.append(Point(x=float(gx), y=float(gy), z=0.16))

        return points

    # -----------------------------------------------------------------
    def compute_control(self):
        """
        메인 제어 계산 함수. 매 루프(50Hz)마다 호출.

        순서:
        1. find_closest_idx(): 현재 위치 기준 가장 가까운 waypoint
        2. 코너 감지: 전방 40 waypoint 누적 꺾임각으로 corner_level 결정
        3. lookahead 결정: corner_level에 따라 cap 적용 후 rate limit
        4. find_target_local(): lookahead 원과 경로 교점 = 타겟 포인트
        5. Pure Pursuit 조향각 계산: steering = atan(L × 2y/Ld²)
        6. 비대칭 조향 rate limit 적용
        7. calculate_target_speed(): 목표 속도 계산
        """
        n = self.num_waypoints
        self.closest_idx = self.find_closest_idx()

        # ── 1단계: 코너 감지 ─────────────────────────────────────────
        # 전방 40 waypoint의 누적 꺾임각(turn_sum_abs)으로 코너 레벨 결정
        # lookahead cap 선택에만 사용되고 속도 계산에는 영향 없음
        look_ahead_steps = 40
        max_angle_diff   = 0.0
        turn_sum_abs     = 0.0

        for j in range(look_ahead_steps):
            idx_a = (self.closest_idx + j)     % n
            idx_b = (self.closest_idx + j + 1) % n
            idx_c = (self.closest_idx + j + 2) % n

            # 연속된 세 waypoint가 이루는 벡터 쌍
            v0 = [self.wp_x[idx_b] - self.wp_x[idx_a],
                  self.wp_y[idx_b] - self.wp_y[idx_a]]
            v1 = [self.wp_x[idx_c] - self.wp_x[idx_b],
                  self.wp_y[idx_c] - self.wp_y[idx_b]]

            norm_v0 = math.hypot(v0[0], v0[1]) + 1e-6
            norm_v1 = math.hypot(v1[0], v1[1]) + 1e-6

            dot_prod  = v0[0]*v1[0] + v0[1]*v1[1]
            cos_theta = float(np.clip(dot_prod / (norm_v0 * norm_v1), -1.0, 1.0))
            angle     = math.acos(cos_theta)   # 두 벡터 사이 각도 [rad]

            max_angle_diff  = max(max_angle_diff, angle)
            turn_sum_abs   += angle

        # 누적 꺾임각으로 코너 레벨 결정
        if turn_sum_abs > math.radians(CORNER_TURN_SUM_DEG):
            corner_level = 2   # 급코너
        elif turn_sum_abs > math.radians(PRECORNER_TURN_SUM_DEG):
            corner_level = 1   # 코너 접근
        else:
            corner_level = 0   # 직선

        # ── 2단계: 코너 탈출 hold 카운터 관리 ───────────────────────
        # 코너 탈출 직후 일정 시간 동안 lookahead를 짧게 유지해
        # 차량이 직선으로 안정화된 후 고속으로 전환
        if corner_level > 0:
            # 코너 중에는 카운터를 계속 리셋 (hold 시작점 갱신)
            self.corner_hold_ticks = int(CORNER_EXIT_HOLD_SEC * CONTROL_RATE)
        else:
            # 직선 구간에서는 카운터를 1씩 감소
            self.corner_hold_ticks = max(0, self.corner_hold_ticks - 1)

        # ── 3단계: Lookahead 결정 ────────────────────────────────────
        # base_lookahead: 현재 속도에 비례해 기본 lookahead 계산
        # [직진 안쪽 벽 문제] LOOKAHEAD_MAX=1.00이 너무 낮음
        # 3~5 m/s에서 base_lookahead = 0.75 + 0.18×4 = 1.47 이지만
        # clip에서 1.00으로 잘림 → 고속에서 lookahead 부족 → 진동/쏠림
        base_lookahead = LOOKAHEAD_MIN + LOOKAHEAD_GAIN * self.current_speed
        base_lookahead = float(np.clip(base_lookahead, LOOKAHEAD_MIN, LOOKAHEAD_MAX))

        # 코너 레벨에 따라 lookahead cap 적용
        if corner_level >= 2:
            # 급코너: 80%로 줄이되 CORNER_LOOKAHEAD_CAP 이하로
            desired_lookahead = min(base_lookahead * 0.80, CORNER_LOOKAHEAD_CAP)
        elif corner_level == 1:
            # 코너 접근: 90%로 줄이되 APPROACH_LOOKAHEAD_CAP 이하로
            desired_lookahead = min(base_lookahead * 0.90, APPROACH_LOOKAHEAD_CAP)
        elif self.corner_hold_ticks > 0:
            # 코너 탈출 직후: 1.70 이하로 제한하면서 안정화
            # 이 구간에서 lookahead가 짧으면 안쪽 벽을 칠 수 있음
            # [직진 안쪽 벽 문제] 1.70 → LOOKAHEAD_MAX까지 바로 올릴 것
            desired_lookahead = min(base_lookahead, 1.70)
        else:
            # 완전 직선: base_lookahead 그대로 사용
            desired_lookahead = base_lookahead

        # Lookahead rate limit: 급격한 변화 억제
        dt = 1.0 / CONTROL_RATE
        if desired_lookahead < self.smoothed_lookahead:
            max_change = LOOKAHEAD_DROP_RATE * dt   # 줄어드는 속도 제한
        else:
            max_change = LOOKAHEAD_RISE_RATE * dt   # 늘어나는 속도 제한

        diff = desired_lookahead - self.smoothed_lookahead
        diff = float(np.clip(diff, -max_change, max_change))
        self.smoothed_lookahead = float(np.clip(
            self.smoothed_lookahead + diff,
            LOOKAHEAD_MIN,
            LOOKAHEAD_MAX))

        lookahead = self.smoothed_lookahead

        # ── 4단계: 타겟 포인트 탐색 ─────────────────────────────────
        target = self.find_target_local(lookahead)
        if target is None:
            self.target_x = self.target_y = None
            return 0.0, 0.0, lookahead

        lx, ly = target
        gx, gy = self.local_to_global(lx, ly)
        self.target_x, self.target_y = gx, gy

        # ── 5단계: Pure Pursuit 조향각 계산 ─────────────────────────
        # 수식: curvature = 2y / Ld²
        #       steering  = atan(L × curvature)
        # y > 0: 타겟이 왼쪽 → 왼쪽으로 꺾음
        # y < 0: 타겟이 오른쪽 → 오른쪽으로 꺾음
        Ld        = max(math.hypot(lx, ly), 1e-6)   # 타겟까지 실제 거리
        curvature = 2.0 * ly / (Ld * Ld)
        steering  = math.atan(WHEELBASE * curvature)
        steering  = float(np.clip(steering, -MAX_STEER, MAX_STEER))

        # ── 6단계: 비대칭 조향 rate limit ───────────────────────────
        # 풀기(unwind): 조향이 0 방향으로 줄어드는 것 (같은 부호, 절댓값 감소)
        #   → STEER_UNWIND_RATE(14 rad/s)로 빠르게 허용 (코너 후 직진 복귀)
        # 꺾기/반대: 조향이 커지거나 반대 방향 (다른 부호 또는 절댓값 증가)
        #   → STEER_WIND_RATE(4.5 rad/s)로 느리게 허용 (counter-steer 억제)
        #
        # [직진 안쪽 벽 문제] 직선에서 Pure Pursuit이 살짝 한쪽 조향을 계산하면
        # STEER_WIND_RATE 속도로 조향이 커져 한쪽으로 쏠릴 수 있음
        # → STEER_WIND_RATE를 2.0~3.0으로 낮추면 직선 조향 진동 감소
        prev     = self.prev_steering
        unwinding = (prev * steering > 0.0) and (abs(steering) < abs(prev))
        rate      = STEER_UNWIND_RATE if unwinding else STEER_WIND_RATE
        max_steer_step = rate / CONTROL_RATE
        steer_delta = float(np.clip(steering - prev, -max_steer_step, max_steer_step))
        steering = prev + steer_delta
        self.prev_steering = steering

        # ── 7단계: 목표 속도 계산 ───────────────────────────────────
        # corner_level / corner_hold_ticks는 속도 계산에 사용하지 않음
        # (lookahead 안정화 전용)
        speed = self.calculate_target_speed(steering)

        return speed, steering, lookahead

    # -----------------------------------------------------------------
    def limit_speed_rate(self, desired_speed):
        """
        속도 변화율 제한 (가속/감속 부드럽게).

        매 루프(dt=0.02s)마다 속도 변화를 MAX_ACCEL/MAX_DECEL로 클램프.
        실제 물리적 구동계 특성에 맞게 튜닝해야 함.

        예: MAX_ACCEL=3.5 → 매 루프 최대 3.5×0.02 = 0.07 m/s씩 가속
            1 m/s → 5 m/s 달성에 약 1.14초 소요
        """
        dt = 1.0 / CONTROL_RATE
        desired_speed = max(0.0, float(desired_speed))
        delta = desired_speed - self.prev_cmd_speed

        if delta > 0.0:
            delta = min(delta, MAX_ACCEL * dt)   # 가속 제한
        else:
            delta = max(delta, -MAX_DECEL * dt)  # 감속 제한

        self.prev_cmd_speed = max(0.0, self.prev_cmd_speed + delta)
        return self.prev_cmd_speed

    # -----------------------------------------------------------------
    def control_loop(self):
        """
        메인 제어 루프 (50 Hz 타이머 콜백).

        순서:
        1. TF에서 차량 위치/방향 취득
        2. compute_control()으로 목표 속도·조향 계산
        3. limit_speed_rate()으로 가속도 제한
        4. 드라이브 명령 발행
        5. RViz 마커 발행
        6. 디버그 로그 출력 (0.5초마다)
        """
        pose = self.get_map_pose()
        if pose is None:
            self.publish_stop()
            return
        self.current_x, self.current_y, self.current_yaw = pose
        try:
            speed, steering, lookahead = self.compute_control()
        except Exception as e:
            self.get_logger().warn(f"control failed: {e}")
            self.publish_stop()
            return
        speed = self.limit_speed_rate(speed)
        self.publish_drive(speed, steering)
        self.publish_markers(steering)
        self.get_logger().info(
            f"speed={speed:.2f} steer={math.degrees(steering):+.1f}deg "
            f"closest={self.closest_idx} Ld={lookahead:.2f}",
            throttle_duration_sec=0.5)

    # -----------------------------------------------------------------
    def publish_drive(self, speed, steering):
        """AckermannDriveStamped 메시지로 속도·조향 명령 발행."""
        msg = AckermannDriveStamped()
        msg.header.stamp         = self.get_clock().now().to_msg()
        msg.header.frame_id      = BASE_FRAME
        msg.drive.speed          = float(speed)
        msg.drive.steering_angle = float(steering)
        self.drive_pub.publish(msg)

    def publish_stop(self):
        """긴급 정지: 속도 0, 조향 0 발행. prev_cmd_speed도 0으로 리셋."""
        self.prev_cmd_speed = 0.0
        msg = AckermannDriveStamped()
        msg.header.stamp         = self.get_clock().now().to_msg()
        msg.header.frame_id      = BASE_FRAME
        msg.drive.speed          = 0.0
        msg.drive.steering_angle = 0.0
        self.drive_pub.publish(msg)

    def publish_markers(self, steering):
        """
        RViz 시각화 마커 발행 (5종).
        제어 로직과 완전 분리되어 있어 주석 처리해도 주행에 영향 없음.

        마커 목록:
        - waypoints (파란 점): 전체 경로
        - pp_target (빨간 구): 현재 lookahead 타겟 포인트
        - pp_line (초록 선): 차량→타겟 lookahead 선
        - pp_speed_preview (주황 선): 속도 계산이 참조하는 전방 경로
        - pp_steering_arc (보라 선): 현재 조향 유지 시 예상 궤적
        """
        now = self.get_clock().now().to_msg()

        # 전체 waypoint (파란 점)
        wp = Marker()
        wp.header.stamp = now; wp.header.frame_id = GLOBAL_FRAME
        wp.ns = "waypoints"; wp.id = 0
        wp.type = Marker.POINTS; wp.action = Marker.ADD
        wp.scale.x = wp.scale.y = 0.08
        wp.color.a = 1.0; wp.color.b = 1.0
        wp.points = [Point(x=float(x), y=float(y), z=0.0)
                     for x, y in zip(self.wp_x, self.wp_y)]
        self.waypoints_marker_pub.publish(wp)

        # 현재 타겟 포인트 (빨간 구)
        tp = Marker()
        tp.header.stamp = now; tp.header.frame_id = GLOBAL_FRAME
        tp.ns = "pp_target"; tp.id = 10; tp.type = Marker.SPHERE
        tp.scale.x = tp.scale.y = tp.scale.z = 0.30
        tp.color.a = 1.0; tp.color.r = 1.0
        if self.target_x is None:
            tp.action = Marker.DELETE
        else:
            tp.action = Marker.ADD
            tp.pose.position.x = self.target_x
            tp.pose.position.y = self.target_y
            tp.pose.position.z = 0.15
            tp.pose.orientation.w = 1.0
        self.target_marker_pub.publish(tp)

        # 차량→타겟 선 (초록)
        ln = Marker()
        ln.header.stamp = now; ln.header.frame_id = GLOBAL_FRAME
        ln.ns = "pp_line"; ln.id = 20; ln.type = Marker.LINE_STRIP
        ln.scale.x = 0.06; ln.color.a = 1.0; ln.color.g = 1.0
        if self.target_x is None:
            ln.action = Marker.DELETE
        else:
            ln.action = Marker.ADD
            ln.points = [
                Point(x=float(self.current_x), y=float(self.current_y), z=0.1),
                Point(x=float(self.target_x),  y=float(self.target_y),  z=0.1)]
        self.line_marker_pub.publish(ln)

        # 속도 판단 전방 경로 (주황)
        preview = Marker()
        preview.header.stamp = now
        preview.header.frame_id = GLOBAL_FRAME
        preview.ns = "pp_speed_preview"
        preview.id = 30
        preview.type = Marker.LINE_STRIP
        preview.action = Marker.ADD
        preview.pose.orientation.w = 1.0
        preview.scale.x = 0.13
        preview.color.a = 1.0
        preview.color.r = 1.0
        preview.color.g = 0.45
        preview.color.b = 0.0
        preview.points = self.get_speed_preview_path_points()
        self.speed_preview_marker_pub.publish(preview)

        # 현재 조향 유지 시 예상 궤적 (보라)
        arc = Marker()
        arc.header.stamp = now
        arc.header.frame_id = GLOBAL_FRAME
        arc.ns = "pp_steering_arc"
        arc.id = 40
        arc.type = Marker.LINE_STRIP
        arc.action = Marker.ADD
        arc.pose.orientation.w = 1.0
        arc.scale.x = 0.09
        arc.color.a = 1.0
        arc.color.r = 0.75
        arc.color.g = 0.10
        arc.color.b = 1.0
        arc.points = self.get_steering_arc_points(steering)
        self.steering_arc_marker_pub.publish(arc)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitOnly()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
