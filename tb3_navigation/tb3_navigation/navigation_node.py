import math
import time
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from nav2_msgs.action import NavigateToPose

from tb3_navigation.astar_planner import AstarPlanner
from tb3_navigation.local_planner import PurePursuitLocalPlanner


class PurePursuitNavigator(Node):
    STATE_IDLE = 0
    STATE_COMPUTING_PATH = 1
    STATE_FOLLOWING_PATH = 2

    def __init__(self):
        super().__init__('pure_pursuit_navigator')

        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('max_linear_vel', 0.22)
        self.declare_parameter('min_linear_vel', 0.05)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('yaw_goal_tolerance', 0.15)
        self.declare_parameter('controller_frequency', 20.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')

        self.declare_parameter('obstacle_threshold', 60)
        self.declare_parameter('inflation_radius_cells', 8)

        self.declare_parameter('avoidance_min_dist', 0.4)
        self.declare_parameter('avoidance_forward_angle', 30)
        self.declare_parameter('avoidance_max_speed_red', 0.3)

        self.global_frame = self.get_parameter('global_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        self.state = self.STATE_IDLE
        self.goal_handle = None
        self.goal_pose_map = None
        self.goal_yaw = 0.0
        self.goal_reached_success = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.planner = AstarPlanner(
            self.get_parameter('obstacle_threshold').value,
            self.get_parameter('inflation_radius_cells').value)

        self.local_planner = PurePursuitLocalPlanner(
            lookahead_distance=self.get_parameter('lookahead_distance').value,
            max_linear_vel=self.get_parameter('max_linear_vel').value,
            min_linear_vel=self.get_parameter('min_linear_vel').value,
            max_angular_vel=self.get_parameter('max_angular_vel').value,
            goal_tolerance=self.get_parameter('goal_tolerance').value,
            yaw_goal_tolerance=self.get_parameter('yaw_goal_tolerance').value,
            avoid_min_dist=self.get_parameter('avoidance_min_dist').value,
            avoid_forward_angle=self.get_parameter('avoidance_forward_angle').value,
            avoid_max_speed_red=self.get_parameter('avoidance_max_speed_red').value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        qos_map = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.cmd_pub = self.create_publisher(
            TwistStamped, self.get_parameter('cmd_vel_topic').value, qos)
        self.marker_pub = self.create_publisher(Marker, '/lookahead_point', qos)
        self.plan_pub = self.create_publisher(
            Path, self.get_parameter('plan_topic').value, qos)

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos_map)

        self.nav_action_server = ActionServer(
            self, NavigateToPose, '/navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos)

        self.latest_scan = None
        self.scan_time = None

        dt = 1.0 / self.get_parameter('controller_frequency').value
        self.control_timer = self.create_timer(dt, self.control_loop)

        self.path_timer = self.create_timer(0.1, self.compute_path_timer_cb)
        self.path_timer.cancel()

        self.replan_timer = self.create_timer(0.5, self.replan_timer_cb)
        self.replan_timer.cancel()

        self.get_logger().info('=== Pure Pursuit Navigator Ready ===')
        self.get_logger().info(f'  Action server: /navigate_to_pose')
        self.get_logger().info(
            f'  Lookahead: {self.get_parameter("lookahead_distance").value}m')
        self.get_logger().info(
            f'  Max vel: {self.get_parameter("max_linear_vel").value} m/s')

    def map_callback(self, msg):
        self.planner.set_map(msg)

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.scan_time = self.get_clock().now()
        try:
            trans = self.tf_buffer.lookup_transform(
                self.global_frame, msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return
        self.planner.clear_dynamic()
        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        q = trans.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)
                wx = tx + cosy * lx - siny * ly
                wy = ty + siny * lx + cosy * ly
                self.planner.add_dynamic_obstacle(wx, wy)
            angle += msg.angle_increment

    def replan_timer_cb(self):
        if self.state != self.STATE_FOLLOWING_PATH or self.goal_pose_map is None:
            return
        if self.goal_reached_success:
            return
        try:
            trans = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, rclpy.time.Time())
        except Exception:
            return
        rx = trans.transform.translation.x
        ry = trans.transform.translation.y
        gx, gy = self.goal_pose_map
        path = self.planner.plan(rx, ry, gx, gy)
        if path:
            self.local_planner.set_path(path)
            self.publish_path()
            self.get_logger().info(f'Replanned: {len(path)} waypoints')

    def goal_callback(self, goal_request):
        self.get_logger().info('Goal request received (auto-accept)')
        return GoalResponse.ACCEPT

    def goal_pose_callback(self, msg):
        if self.state != self.STATE_IDLE:
            self.get_logger().info('Already navigating, ignoring /goal_pose')
            return
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        gq = msg.pose.orientation
        siny = 2.0 * (gq.w * gq.z + gq.x * gq.y)
        cosy = 1.0 - 2.0 * (gq.y * gq.y + gq.z * gq.z)
        self.goal_yaw = math.atan2(siny, cosy)
        self.goal_pose_map = (gx, gy)
        self.get_logger().info(
            f'Goal from /goal_pose: x={gx:.2f}, y={gy:.2f}, yaw={self.goal_yaw:.2f}')
        self.start_navigation()

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('=== EXECUTING NAVIGATION GOAL ===')
        self.goal_handle = goal_handle
        goal = goal_handle.request

        gx = goal.pose.pose.position.x
        gy = goal.pose.pose.position.y
        gq = goal.pose.pose.orientation
        siny = 2.0 * (gq.w * gq.z + gq.x * gq.y)
        cosy = 1.0 - 2.0 * (gq.y * gq.y + gq.z * gq.z)
        self.goal_yaw = math.atan2(siny, cosy)
        self.goal_pose_map = (gx, gy)

        self.get_logger().info(
            f'  Goal: x={gx:.2f}, y={gy:.2f}, yaw={self.goal_yaw:.2f}')

        self.start_navigation()

        result = NavigateToPose.Result()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancelled by client')
                self.state = self.STATE_IDLE
                self.stop_robot()
                goal_handle.canceled()
                return result
            if self.state == self.STATE_IDLE:
                if self.goal_reached_success:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return result
            time.sleep(0.05)

        return result

    def start_navigation(self):
        self.state = self.STATE_COMPUTING_PATH
        self.path_timer_calls = 0
        self.path_timer.reset()
        self.replan_timer.reset()

    def compute_path_timer_cb(self):
        self.path_timer_calls += 1
        self.path_timer.cancel()

        try:
            trans = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF lookup failed (attempt {self.path_timer_calls}): {e}')
            if self.path_timer_calls < 40:
                self.path_timer.reset()
            else:
                self.get_logger().error('Giving up on TF lookup, aborting goal')
                self.goal_reached_success = False
                self.state = self.STATE_IDLE
            return

        rx = trans.transform.translation.x
        ry = trans.transform.translation.y
        self.get_logger().info(f'  Robot in map: x={rx:.2f}, y={ry:.2f}')

        gx, gy = self.goal_pose_map
        dx = gx - rx
        dy = gy - ry
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < self.goal_tolerance:
            self.get_logger().info('Already at goal!')
            self.goal_reached_success = True
            self.state = self.STATE_IDLE
            return

        path = self.planner.plan(rx, ry, gx, gy)
        if path is None:
            self.get_logger().error(
                'A* failed to find path, falling back to straight line')
            num_pts = max(10, int(dist / 0.1))
            path = []
            for i in range(num_pts + 1):
                t = i / num_pts
                path.append((rx + dx * t, ry + dy * t))

        self.local_planner.set_path(path)
        self.local_planner.set_goal(gx, gy, self.goal_yaw)
        self.state = self.STATE_FOLLOWING_PATH
        self.get_logger().info(
            f'  Path built: {len(path)} waypoints, dist={dist:.2f}m')
        self.publish_path()

    def publish_path(self):
        msg = Path()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for px, py in self.local_planner.global_path:
            pose = PoseStamped()
            pose.header.frame_id = self.global_frame
            pose.header.stamp = msg.header.stamp
            pose.pose.position.x = px
            pose.pose.position.y = py
            msg.poses.append(pose)
        self.plan_pub.publish(msg)

    def get_robot_pose_map(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        return trans.transform.translation.x, trans.transform.translation.y

    def get_robot_yaw(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, rclpy.time.Time())
        except Exception:
            return None
        q = trans.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def publish_lookahead_marker(self, lx, ly):
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'pure_pursuit'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = lx
        marker.pose.position.y = ly
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

    def stop_robot(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.global_frame
        self.cmd_pub.publish(cmd)
        self.goal_pose_map = None
        self.replan_timer.cancel()

    def control_loop(self):
        if self.state != self.STATE_FOLLOWING_PATH:
            return

        pose = self.get_robot_pose_map()
        if pose is None:
            return
        rx, ry = pose

        robot_yaw = self.get_robot_yaw()
        if robot_yaw is None:
            return

        obs_dist = self.planner.nearest_obstacle_distance(rx, ry, 20)
        result = self.local_planner.compute_velocity(
            rx, ry, robot_yaw, self.latest_scan, obs_dist)
        if result is None:
            return

        vx, wz, goal_reached, lx, ly, avoid_steer, avoid_speed = result

        if goal_reached:
            self.get_logger().info('Goal reached! Stopping.')
            self.stop_robot()
            self.goal_reached_success = True
            self.state = self.STATE_IDLE
            return

        self.publish_lookahead_marker(lx, ly)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.global_frame
        cmd.twist.linear.x = vx
        cmd.twist.angular.z = wz
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'vel={vx:.2f} ang={wz:.2f} '
            f'av steer={avoid_steer:.2f} spd={avoid_speed:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNavigator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
