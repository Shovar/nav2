import math


class PurePursuitLocalPlanner:
    def __init__(self, lookahead_distance=0.5, max_linear_vel=0.22, min_linear_vel=0.05,
                 max_angular_vel=1.0, goal_tolerance=0.15, yaw_goal_tolerance=0.15,
                 avoid_min_dist=0.4, avoid_forward_angle=30, avoid_max_speed_red=0.3):
        self.lookahead_distance = lookahead_distance
        self.max_linear_vel = max_linear_vel
        self.min_linear_vel = min_linear_vel
        self.max_angular_vel = max_angular_vel
        self.goal_tolerance = goal_tolerance
        self.yaw_goal_tolerance = yaw_goal_tolerance
        self.avoid_min_dist = avoid_min_dist
        self.avoid_forward_angle = avoid_forward_angle
        self.avoid_max_speed_red = avoid_max_speed_red

        self.global_path = []
        self.path_index = 0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0

        self.smooth_alpha = 0.12
        self.smooth_vx = 0.0
        self.smooth_wz = 0.0

    def set_path(self, path):
        self.global_path = list(path)
        self.path_index = 0

    def set_goal(self, x, y, yaw):
        self.goal_x = x
        self.goal_y = y
        self.goal_yaw = yaw

    def compute_velocity(self, rx, ry, robot_yaw, laser_scan, nearest_obs_dist):
        if not self.global_path:
            return None

        if self._is_goal_reached(rx, ry, robot_yaw):
            return 0.0, 0.0, True, 0.0, 0.0, 0.0, 1.0

        self.path_index = self._find_nearest_index(rx, ry)

        if self.path_index >= len(self.global_path) - 1:
            return 0.0, 0.0, True, 0.0, 0.0, 0.0, 1.0

        lookahead = self._get_lookahead_point(rx, ry)
        if lookahead is None:
            return None

        lx, ly = lookahead

        path_yaw = math.atan2(ly - ry, lx - rx)
        alpha = self._normalize_angle(path_yaw - robot_yaw)
        ld = max(0.01, math.sqrt((lx - rx) ** 2 + (ly - ry) ** 2))
        curvature = 2.0 * math.sin(alpha) / ld
        angular_vel = curvature * self.max_linear_vel
        angular_vel = max(min(angular_vel, self.max_angular_vel), -self.max_angular_vel)

        dist_to_goal = math.sqrt((rx - self.goal_x) ** 2 + (ry - self.goal_y) ** 2)

        vel = self.max_linear_vel
        if nearest_obs_dist < 0.3:
            vel *= max(0.2, nearest_obs_dist / 0.3)
        if dist_to_goal < 0.8 * self.lookahead_distance:
            vel = max(self.min_linear_vel,
                      self.max_linear_vel * (dist_to_goal / (0.8 * self.lookahead_distance)))
        if abs(alpha) > math.radians(60):
            vel = min(vel, self.max_linear_vel * 0.4)

        avoid_steer, avoid_speed = self._compute_avoidance(laser_scan, robot_yaw, path_yaw)
        angular_vel += avoid_steer * 0.5
        angular_vel = max(min(angular_vel, self.max_angular_vel), -self.max_angular_vel)
        vel *= avoid_speed

        vx = max(0.0, min(vel, self.max_linear_vel))
        wz = angular_vel

        if vx > 0.01:
            self.smooth_vx += self.smooth_alpha * (vx - self.smooth_vx)
            self.smooth_wz += self.smooth_alpha * (wz - self.smooth_wz)
            vx = self.smooth_vx
            wz = self.smooth_wz
        else:
            self.smooth_vx = 0.0
            self.smooth_wz = 0.0

        return vx, wz, False, lx, ly, avoid_steer, avoid_speed

    def _find_nearest_index(self, rx, ry):
        min_dist = float('inf')
        min_idx = 0
        for i, (px, py) in enumerate(self.global_path):
            d = (rx - px) ** 2 + (ry - py) ** 2
            if d < min_dist:
                min_dist = d
                min_idx = i
        return min_idx

    def _get_lookahead_point(self, rx, ry):
        for i in range(self.path_index, len(self.global_path)):
            px, py = self.global_path[i]
            d = math.sqrt((rx - px) ** 2 + (ry - py) ** 2)
            if d >= self.lookahead_distance:
                return px, py
        return self.global_path[-1]

    def _is_goal_reached(self, rx, ry, robot_yaw):
        dist = math.sqrt((rx - self.goal_x) ** 2 + (ry - self.goal_y) ** 2)
        if dist > self.goal_tolerance:
            return False
        yaw_diff = abs(self._normalize_angle(robot_yaw - self.goal_yaw))
        return yaw_diff < self.yaw_goal_tolerance

    def _compute_avoidance(self, laser_scan, robot_yaw, path_yaw):
        if laser_scan is None:
            return 0.0, 1.0
        scan = laser_scan

        min_forward = float('inf')
        blocked = False
        for i, r in enumerate(scan.ranges):
            if r < scan.range_min or r > scan.range_max:
                continue
            angle_deg = math.degrees(scan.angle_min + i * scan.angle_increment)
            if abs(angle_deg) <= self.avoid_forward_angle:
                if r < min_forward:
                    min_forward = r
                if r < self.avoid_min_dist:
                    blocked = True

        if not blocked:
            return 0.0, 1.0

        sector_angle = 20
        num_sectors = int(360 / sector_angle)
        sector_ranges = [0.0] * num_sectors
        sector_counts = [0] * num_sectors
        for i, r in enumerate(scan.ranges):
            if r < scan.range_min or r > scan.range_max:
                continue
            angle_deg = math.degrees(scan.angle_min + i * scan.angle_increment)
            idx = int((angle_deg + 180) / sector_angle) % num_sectors
            sector_ranges[idx] += r
            sector_counts[idx] += 1

        for i in range(num_sectors):
            if sector_counts[i] > 0:
                sector_ranges[i] /= sector_counts[i]

        rel_path = math.degrees(self._normalize_angle(path_yaw - robot_yaw))
        for i in range(num_sectors):
            sector_center = i * sector_angle - 180 + sector_angle / 2
            angle_diff = abs(self._normalize_angle(math.radians(sector_center - rel_path)))
            weight = max(0.25, 1.0 - math.degrees(angle_diff) / 180.0)
            sector_ranges[i] *= weight

        best_sector = max(range(num_sectors), key=lambda i: sector_ranges[i])
        best_center = best_sector * sector_angle - 180 + sector_angle / 2

        steer = self._normalize_angle(math.radians(best_center))
        steer = max(min(steer, self.max_angular_vel * 0.5), -self.max_angular_vel * 0.5)

        dist_factor = min_forward / self.avoid_min_dist
        speed_red = self.avoid_max_speed_red + (1.0 - self.avoid_max_speed_red) * dist_factor

        return steer, max(speed_red, self.avoid_max_speed_red)

    @staticmethod
    def _normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a
