import copy
import heapq
import math


class AstarPlanner:
    def __init__(self, obstacle_threshold=60, inflation_radius_cells=8):
        self.obstacle_threshold = obstacle_threshold
        self.inflation_radius_cells = inflation_radius_cells

        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        self.dynamic_obstacles = set()

    def set_map(self, msg):
        self.map_data = msg
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

    def clear_dynamic(self):
        self.dynamic_obstacles.clear()

    def add_dynamic_obstacle(self, wx, wy):
        gx, gy = self.world_to_grid(wx, wy)
        if self.is_valid(gx, gy):
            self.dynamic_obstacles.add((gx, gy))

    def world_to_grid(self, wx, wy):
        gx = int((wx - self.map_origin_x) / self.map_resolution)
        gy = int((wy - self.map_origin_y) / self.map_resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = gx * self.map_resolution + self.map_origin_x
        wy = gy * self.map_resolution + self.map_origin_y
        return wx, wy

    def is_valid(self, gx, gy):
        return 0 <= gx < self.map_width and 0 <= gy < self.map_height

    def is_occupied_static(self, gx, gy):
        if not self.is_valid(gx, gy):
            return True
        idx = gy * self.map_width + gx
        val = self.map_data.data[idx]
        return val >= self.obstacle_threshold or val == -1

    def is_occupied_dynamic(self, gx, gy):
        return (gx, gy) in self.dynamic_obstacles

    def is_occupied(self, gx, gy):
        return self.is_occupied_static(gx, gy) or self.is_occupied_dynamic(gx, gy)

    def get_cost(self, gx, gy):
        if not self.is_valid(gx, gy):
            return 100
        if self.is_occupied_dynamic(gx, gy):
            return 100
        idx = gy * self.map_width + gx
        val = self.map_data.data[idx]
        if val < 0:
            return 50
        return val

    def is_definitely_occupied(self, gx, gy):
        if self.is_occupied_dynamic(gx, gy):
            return True
        if not self.is_valid(gx, gy):
            return True
        idx = gy * self.map_width + gx
        val = self.map_data.data[idx]
        return val >= 0 and val >= self.obstacle_threshold

    def nearest_obstacle_distance(self, wx, wy, max_search=50):
        if self.map_data is None:
            return max_search * self.map_resolution
        gx, gy = self.world_to_grid(wx, wy)
        if not self.is_valid(gx, gy):
            return 0.0
        best = None
        for dy in range(-max_search, max_search + 1):
            for dx in range(-max_search, max_search + 1):
                nx, ny = gx + dx, gy + dy
                if self.is_valid(nx, ny) and self.is_definitely_occupied(nx, ny):
                    d = dx * dx + dy * dy
                    if best is None or d < best:
                        best = d
        if best is None:
            return max_search * self.map_resolution
        return math.sqrt(best) * self.map_resolution

    @staticmethod
    def heuristic(ax, ay, bx, by):
        return math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)

    def inflate_obstacles(self):
        inflated = copy.copy(self.map_data.data)
        r = self.inflation_radius_cells
        kernel = []
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    kernel.append((dx, dy))
        for gy in range(self.map_height):
            for gx in range(self.map_width):
                idx = gy * self.map_width + gx
                if self.map_data.data[idx] >= self.obstacle_threshold:
                    for kx, ky in kernel:
                        nx, ny = gx + kx, gy + ky
                        if self.is_valid(nx, ny):
                            nidx = ny * self.map_width + nx
                            if inflated[nidx] < self.obstacle_threshold:
                                inflated[nidx] = self.obstacle_threshold
                if (gx, gy) in self.dynamic_obstacles:
                    for kx, ky in kernel:
                        nx, ny = gx + kx, gy + ky
                        if self.is_valid(nx, ny):
                            nidx = ny * self.map_width + nx
                            if inflated[nidx] < self.obstacle_threshold:
                                inflated[nidx] = self.obstacle_threshold
        return inflated

    def find_nearest_free(self, gx, gy):
        best = None
        best_dist = float('inf')
        for dy in range(-20, 21):
            for dx in range(-20, 21):
                nx, ny = gx + dx, gy + dy
                if self.is_valid(nx, ny) and not self.is_occupied(nx, ny):
                    d = dx * dx + dy * dy
                    if d < best_dist:
                        best_dist = d
                        best = (nx, ny)
        return best

    @staticmethod
    def smooth_path(path, weight=0.15, passes=1):
        if len(path) <= 2:
            return list(path)
        smoothed = list(path)
        for _ in range(passes):
            new_path = [smoothed[0]]
            for i in range(1, len(smoothed) - 1):
                px = weight * smoothed[i-1][0] + (1 - 2*weight) * smoothed[i][0] + weight * smoothed[i+1][0]
                py = weight * smoothed[i-1][1] + (1 - 2*weight) * smoothed[i][1] + weight * smoothed[i+1][1]
                new_path.append((px, py))
            new_path.append(smoothed[-1])
            smoothed = new_path
        return smoothed

    def plan(self, start_wx, start_wy, goal_wx, goal_wy):
        if self.map_data is None:
            return None

        sx, sy = self.world_to_grid(start_wx, start_wy)
        gx, gy = self.world_to_grid(goal_wx, goal_wy)

        if not self.is_valid(sx, sy) or not self.is_valid(gx, gy):
            return None

        if self.is_occupied(sx, sy):
            sx, sy = self.find_nearest_free(sx, sy)
            if sx is None:
                return None
        if self.is_occupied(gx, gy):
            gx, gy = self.find_nearest_free(gx, gy)
            if gx is None:
                return None

        inflated = self.inflate_obstacles()

        def is_occupied_inflated(cx, cy):
            if not self.is_valid(cx, cy):
                return True
            idx = cy * self.map_width + cx
            return inflated[idx] >= self.obstacle_threshold

        def get_neighbors_inflated(cx, cy):
            result = []
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                           (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                nx, ny = cx + dx, cy + dy
                if not self.is_valid(nx, ny):
                    continue
                if is_occupied_inflated(nx, ny):
                    continue
                if abs(dx) == 1 and abs(dy) == 1:
                    if is_occupied_inflated(cx + dx, cy):
                        continue
                    if is_occupied_inflated(cx, cy + dy):
                        continue
                result.append((nx, ny))
            return result

        goal_node = (gx, gy)
        open_set = [(0.0, (sx, sy))]
        came_from = {}
        g_score = {(sx, sy): 0.0}
        f_score = {(sx, sy): self.heuristic(sx, sy, gx, gy)}
        open_set_set = {(sx, sy)}

        while open_set:
            _, current = heapq.heappop(open_set)
            open_set_set.discard(current)

            if current == goal_node:
                path = []
                node = current
                while node in came_from:
                    path.append(node)
                    node = came_from[node]
                path.reverse()
                world_path = [(start_wx, start_wy)]
                for px, py in path[1:-1]:
                    wx, wy = self.grid_to_world(px, py)
                    world_path.append((wx, wy))
                world_path.append((goal_wx, goal_wy))
                world_path = self.smooth_path(world_path)
                return world_path

            cx, cy = current
            for neighbor in get_neighbors_inflated(cx, cy):
                nx, ny = neighbor
                step_cost = math.sqrt((nx - cx) ** 2 + (ny - cy) ** 2)
                cost = self.get_cost(nx, ny) / 100.0
                tentative_g = g_score[current] + step_cost * (0.5 + 0.5 * cost)

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h = self.heuristic(nx, ny, gx, gy)
                    f_score[neighbor] = tentative_g + h * 1.2
                    if neighbor not in open_set_set:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
                        open_set_set.add(neighbor)

        return None
