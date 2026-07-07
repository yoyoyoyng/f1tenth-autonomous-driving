#!/usr/bin/env python3
"""
Disparity Extender — F1TENTH RC Car (v4-speedup)
================================================
기존에 잘 되던 v4 구조는 유지하고, 직선구간 속도만 더 공격적으로 올린 버전.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker


class GapFollow(Node):
    def __init__(self):
        super().__init__('gap_follow_node')

        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.best_marker_pub   = self.create_publisher(Marker, '/best_point_marker', 10)
        self.bubble_marker_pub = self.create_publisher(Marker, '/bubble_point_marker', 10)

        # ── LiDAR ────────────────────────────────────────────
        self.max_range        = 4.5
        # LiDAR 최대 인식 거리 [m]
        # 올리면 더 멀리 보지만 노이즈 증가
        # 낮추면 가까운 장애물에만 집중

        self.fov_deg          = 160.0
        # 좌우 시야각 [deg] — 이 각도 안쪽 데이터만 사용
        # 올리면 더 넓게 보지만 측면 벽에 민감해짐
        # 낮추면 전방에만 집중 → 코너에서 늦게 반응할 수 있음

        self.smoothing_window = 5
        # LiDAR 데이터 스무딩 윈도우 크기
        # 올리면 노이즈 감소하지만 장애물 경계가 뭉개짐
        # 낮추면 날카로운 장애물 감지 가능하지만 노이즈에 민감

        # ── disparity + bubble ────────────────────────────────
        self.car_width            = 0.30
        # 차량 폭 [m] — 실제 차 폭으로 정확히 맞출 것
        # 틀리면 bubble/disparity 크기가 잘못 계산되어 벽 충돌 위험

        self.safety_margin        = 0.12
        # 차폭에 추가하는 안전 여유 [m]
        # 올리면 더 안전하지만 좁은 통로 통과 어려움
        # 낮추면 좁은 공간 통과 가능하지만 충돌 위험 증가

        self.disparity_threshold  = 0.25
        # 인접 LiDAR 빔 간 거리 차이가 이 값 이상이면 disparity로 판단 [m]
        # 낮추면 더 많은 곳을 disparity로 인식 → 더 보수적
        # 올리면 큰 단차만 disparity로 인식 → 벽 가까이 붙을 수 있음

        # ── gap 선택 ─────────────────────────────────────────
        self.gap_min_dist = 0.4
        # gap으로 인정하는 최소 거리 [m]
        # 올리면 더 넓고 먼 gap만 선택 → 안전하지만 좁은 공간 못 통과
        # 낮추면 가까운 gap도 선택 → 장애물 가까이 붙을 수 있음

        # ── 조향 안정화 ───────────────────────────────────────
        self.MAX_STEER        = 0.36
        # 최대 조향각 [rad] (~21도) — 서보 한계에 맞게 조정
        # 올리면 급코너 가능하지만 과조향 위험
        # 낮추면 완만하게 꺾음

        self.steer_deadzone   = np.radians(1.5)
        # 이 각도 이하의 조향 명령은 0으로 처리 [rad]
        # 직진 중 미세 진동 억제용
        # 올리면 직진 안정화되지만 미세 조향 반응 없어짐

        self.steer_alpha      = 0.2
        # 조향 EMA 필터 계수 (0~1)
        # 낮을수록 이전 조향값을 많이 유지 → 부드럽지만 반응 느림
        # 높을수록 새 값을 빠르게 반영 → 반응 빠르지만 진동 가능
        # 직진에서 흔들리면 낮추기 (0.15~0.20)
        # 코너 반응이 느리면 올리기 (0.30~0.40)

        self.steer_rate_limit = np.radians(5.0)
        # 매 루프당 조향 변화 한계 [rad]
        # 낮추면 조향이 천천히 바뀜 → 안정적이지만 급코너 대응 느림
        # 올리면 조향이 빠르게 바뀜 → 급코너 대응 빠르지만 진동 가능

        self.steer_gain       = 0.9
        # 최종 조향각에 곱하는 이득값
        # 1.0 미만이면 조향을 약간 줄임 → 과조향 방지
        # 조향이 너무 크면 낮추기 (0.85~0.95)
        # 조향이 부족하면 올리기 (1.0~1.05)

        self.prev_steering    = 0.0

        # ── 직진 강제 ─────────────────────────────────────────
        self.straight_drive_dist = 3.8
        # 전방 이 거리 이상이면 직진 강제 모드 진입 [m]
        # 올리면 더 멀리 열려있을 때만 직진 → 보수적
        # 낮추면 조금만 열려도 직진 → 공격적이지만 코너 진입 늦음
        # 직진에서 불필요하게 조향하면 올리기
        # 코너 진입이 너무 늦으면 낮추기 (3.5~4.0)

        self.straight_steer_max  = np.radians(8.0)
        # 직진 강제 모드에서 허용하는 최대 조향각 [rad]
        # 낮추면 직진에서 거의 안 꺾음 → 벽 충돌 위험
        # 올리면 직진에서도 많이 꺾을 수 있음 → 직진 불안정

        # ── 직진 중 측면 벽 회피 ──────────────────────────────
        self.wall_safe_dist  = 0.2
        # 측면 벽과 이 거리 이하로 가까워지면 회피 조향 [m]
        # 올리면 벽에서 더 멀리 떨어짐 → 안전하지만 좁은 공간에서 과민반응
        # 낮추면 벽 가까이 붙어도 회피 안 함 → 좁은 공간 통과 가능

        self.wall_push_gain  = 1.0
        # 벽 회피 조향 강도
        # 올리면 벽에서 강하게 밀어냄 → 급격한 조향 변화 가능
        # 낮추면 부드럽게 회피 → 벽에 더 가까이 붙을 수 있음

        self.wall_fov_deg    = 40.0
        # 측면 벽 감지 시야각 [deg]
        # 올리면 더 넓은 각도에서 측면 벽 감지
        # 낮추면 좁은 각도만 감지 → 측면 벽 반응 줄어듦

        # ── 속도 ─────────────────────────────────────────────
        self.speed_max          = 10.0
        # 최고 목표 속도 [m/s]
        # 너무 빠르면 코너에서 밖으로 밀림 → 낮추기 (6.0~7.0)
        # 더 빠르게 달리려면 올리기 (9.0~10.0)

        self.speed_min          = 1.2
        # 최저 목표 속도 [m/s]
        # 코너에서 너무 빠르면 낮추기 (1.2~1.5)
        # 너무 느리게 달리면 올리기 (2.0~2.5)

        self.slow_dist          = 2.8
        # 전방 이 거리 이하부터 감속 시작 [m]
        # 코너에서 속도가 안 줄면 올리기 (3.5~4.0)
        # 직선에서 너무 일찍 감속하면 낮추기 (2.8~3.0)

        self.angle_spd_penalty  = 0.91
        # 조향각이 클수록 속도를 줄이는 비율 (0~1)
        # 코너에서 밖으로 밀리면 올리기 (0.85~0.95)
        # 코너 속도를 올리려면 낮추기 (0.70~0.80)

        # ── 직선 전용 공격 속도 ───────────────────────────────
        self.straight_fast_dist  = 6.0
        # 전방이 이 거리 이상 열려있을 때 직선 boost 적용 [m]
        # 올리면 더 넓게 열렸을 때만 boost → 보수적
        # 낮추면 조금만 열려도 boost → 공격적

        self.straight_fast_steer = np.radians(4.0)
        # 조향각이 이 값 이하일 때만 직선 boost 적용 [rad]
        # 낮추면 거의 직진일 때만 boost
        # 올리면 약간 꺾인 상태에서도 boost → 코너 진입 시 과속 위험

        self.straight_fast_speed = 23.0
        # 직선 boost 시 목표 속도 [m/s]
        # speed_max로 클램프되므로 사실상 직선에서 speed_max까지 밀어주는 역할

        self.get_logger().info('GapFollow v4-straight-aggressive-corner-plus initialized')

    # ─────────────────────────────────────────────────────────
    # marker
    # ─────────────────────────────────────────────────────────
    def _sphere(self, idx, dist, r, amin, ainc, ns, rgba):
        ang = amin + idx * ainc
        m = Marker()
        m.header.frame_id = 'laser'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(dist * np.cos(ang))
        m.pose.position.y = float(dist * np.sin(ang))
        m.pose.orientation.w = 1.0
        m.scale.x = float(max(0.05, r))
        m.scale.y = float(max(0.05, r))
        m.scale.z = float(max(0.05, r))
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        return m

    # ─────────────────────────────────────────────────────────
    # 1. preprocess
    # ─────────────────────────────────────────────────────────
    def preprocess(self, ranges):
        p = np.array(ranges, dtype=np.float32)
        p[np.isnan(p)] = 0.0
        p[np.isinf(p)] = self.max_range
        p = np.clip(p, 0.0, self.max_range)
        k = np.ones(self.smoothing_window) / self.smoothing_window
        return np.convolve(p, k, mode='same')

    def fov_slice(self, data, n):
        s = max(
            0,
            int(np.floor((-np.deg2rad(self.fov_deg) - data.angle_min) / data.angle_increment))
        )
        e = min(
            n - 1,
            int(np.ceil((np.deg2rad(self.fov_deg) - data.angle_min) / data.angle_increment))
        )
        return s, e

    # ─────────────────────────────────────────────────────────
    # 2. safety bubble
    # ─────────────────────────────────────────────────────────
    def apply_bubble(self, win, ainc):
        out = np.copy(win)
        valid = np.where(out > 1e-3)[0]
        if len(valid) == 0:
            return out, len(out) // 2, 0.0, 0.0
        near_i = valid[np.argmin(out[valid])]
        nd = float(out[near_i])
        half = self.car_width / 2.0 + self.safety_margin
        npts = self._n_pts(nd, half, ainc)
        out[max(0, near_i - npts): min(len(out), near_i + npts + 1)] = 0.0
        return out, near_i, nd, half

    def _n_pts(self, dist, half, ainc):
        if dist < 1e-3:
            return 1
        arc = 2.0 * np.arcsin(np.clip(half / max(dist, 1e-3), 0.0, 1.0))
        return max(1, int(np.ceil(arc / ainc)))

    # ─────────────────────────────────────────────────────────
    # 3. disparity extension
    # ─────────────────────────────────────────────────────────
    def extend_disparities(self, win, ainc):
        out = np.copy(win)
        half = self.car_width / 2.0 + self.safety_margin
        diff = np.abs(np.diff(out))
        for i in np.where(diff > self.disparity_threshold)[0]:
            l = out[i]
            r = out[i + 1]
            close = min(l, r)
            if close < 1e-3:
                continue
            npts = self._n_pts(close, half, ainc)
            if l < r:
                s = i + 1
                e = min(len(out), i + 1 + npts)
            else:
                s = max(0, i - npts + 1)
                e = i + 1
            out[s:e] = np.minimum(out[s:e], close)
        return out

    # ─────────────────────────────────────────────────────────
    # 4. 가장 넓은 gap의 중심 선택
    # ─────────────────────────────────────────────────────────
    def best_gap_center(self, safe):
        mask = safe >= self.gap_min_dist
        best_s = -1
        best_e = -1
        best_len = 0
        cur_s = None
        for i, v in enumerate(mask):
            if v and cur_s is None:
                cur_s = i
            elif not v and cur_s is not None:
                if i - cur_s > best_len:
                    best_len = i - cur_s
                    best_s = cur_s
                    best_e = i - 1
                cur_s = None
        if cur_s is not None and len(mask) - cur_s > best_len:
            best_s = cur_s
            best_e = len(mask) - 1
        if best_s < 0:
            return int(np.argmax(safe))
        seg = safe[best_s:best_e + 1]
        thresh = max(self.gap_min_dist, np.max(seg) * 0.85)
        deep = np.where(seg >= thresh)[0]
        if len(deep) > 0:
            return best_s + int(round(deep.mean()))
        return best_s + int(np.argmax(seg))

    # ─────────────────────────────────────────────────────────
    # 직진 중 측면 벽 감지 → 조향 보정
    # ─────────────────────────────────────────────────────────
    def wall_avoidance(self, data, proc):
        n = len(proc)
        angles = data.angle_min + np.arange(n) * data.angle_increment
        left_idx = np.where(angles > np.deg2rad(self.wall_fov_deg))[0]
        right_idx = np.where(angles < -np.deg2rad(self.wall_fov_deg))[0]

        def min_dist(idx):
            if len(idx) == 0:
                return np.inf
            v = proc[idx]
            v = v[v > 1e-3]
            return float(np.min(v)) if len(v) else np.inf

        ld = min_dist(left_idx)
        rd = min_dist(right_idx)
        correction = 0.0
        if ld < self.wall_safe_dist:
            correction -= self.wall_push_gain * (self.wall_safe_dist - ld)
        if rd < self.wall_safe_dist:
            correction += self.wall_push_gain * (self.wall_safe_dist - rd)
        return float(np.clip(correction, -self.straight_steer_max, self.straight_steer_max))

    # ─────────────────────────────────────────────────────────
    # 조향 안정화
    # ─────────────────────────────────────────────────────────
    def stabilize(self, raw):
        t = raw * self.steer_gain
        if abs(t) < self.steer_deadzone:
            t = 0.0
        sm = self.steer_alpha * t + (1.0 - self.steer_alpha) * self.prev_steering
        delta = np.clip(
            sm - self.prev_steering,
            -self.steer_rate_limit,
            self.steer_rate_limit
        )
        sm = float(np.clip(
            self.prev_steering + delta,
            -self.MAX_STEER,
            self.MAX_STEER
        ))
        self.prev_steering = sm
        return sm

    # ─────────────────────────────────────────────────────────
    # 속도 계산
    # ─────────────────────────────────────────────────────────
    def calc_speed(self, steer, front_dist):
        d_ratio = np.clip(
            (front_dist - self.slow_dist) / (self.max_range - self.slow_dist),
            0.0,
            1.0
        )
        base = self.speed_min + d_ratio * (self.speed_max - self.speed_min)
        if front_dist >= self.straight_fast_dist and abs(steer) <= self.straight_fast_steer:
            base = max(base, self.straight_fast_speed)
        a_ratio = np.clip(abs(steer) / self.MAX_STEER, 0.0, 1.0)
        base *= (1.0 - self.angle_spd_penalty * a_ratio)
        return float(np.clip(base, self.speed_min, self.speed_max))

    # ─────────────────────────────────────────────────────────
    # 메인 콜백
    # ─────────────────────────────────────────────────────────
    def scan_callback(self, data):
        n = len(data.ranges)
        proc = self.preprocess(data.ranges)
        s_idx, e_idx = self.fov_slice(data, n)
        win = np.copy(proc[s_idx:e_idx + 1])
        win, near_i, near_d, bubble_r = self.apply_bubble(win, data.angle_increment)
        safe = self.extend_disparities(win, data.angle_increment)
        best_local = self.best_gap_center(safe)
        best_global = s_idx + best_local
        best_dist = float(safe[best_local])
        raw_steer = data.angle_min + best_global * data.angle_increment
        c0 = max(
            0,
            int((-np.deg2rad(5.0) - data.angle_min) / data.angle_increment)
        )
        c1 = min(
            n - 1,
            int((np.deg2rad(5.0) - data.angle_min) / data.angle_increment)
        )
        fseg = proc[c0:c1 + 1]
        fseg = fseg[fseg > 1e-3]
        front_dist = float(np.min(fseg)) if len(fseg) else self.max_range
        if front_dist >= self.straight_drive_dist:
            wall_corr = self.wall_avoidance(data, proc)
            base = (1.0 - self.steer_alpha) * self.prev_steering
            steering = float(np.clip(
                base + wall_corr,
                -self.straight_steer_max,
                self.straight_steer_max
            ))
            self.prev_steering = steering
        else:
            steering = self.stabilize(raw_steer)
        speed = self.calc_speed(steering, front_dist)
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = steering
        msg.drive.speed = speed
        self.drive_pub.publish(msg)
        self.best_marker_pub.publish(self._sphere(
            best_global, best_dist, 0.3,
            data.angle_min, data.angle_increment,
            'best_point', (0.0, 1.0, 0.0, 1.0)
        ))
        self.bubble_marker_pub.publish(self._sphere(
            s_idx + near_i, near_d, bubble_r * 2.0,
            data.angle_min, data.angle_increment,
            'bubble_point', (1.0, 0.0, 0.0, 0.5)
        ))


def main(args=None):
    rclpy.init(args=args)
    node = GapFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()