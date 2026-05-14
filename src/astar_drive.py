#!/usr/bin/env python3

import os
import math
import heapq
import yaml

import cv2 as cv
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from nav_msgs.msg import Path

from tf2_ros import Buffer, TransformListener


class AStarDrive(Node):
    def __init__(self):
        super().__init__('astar_drive')

        # =========================
        # PARAMETRY
        # =========================
        self.map_yaml_path = os.path.expanduser(
            '~/slam_astar_project/maps/slam_map.yaml'
        )

        # Inflacja przeszkód.
        # Dla tego dużego robota zacznij od 1.0 m.
        # Jeśli A* nie znajduje trasy, zmniejsz np. do 0.7.
        # Jeśli robot jedzie za blisko ścian, zwiększ np. do 1.2.
        self.inflation_radius_m = 1.6

        # Sterowanie
        self.distance_tolerance = 0.45
        self.angle_tolerance = 0.30

        self.max_linear_speed = 0.6
        self.max_angular_speed = 0.8

        # Co ile metrów wybierać waypoint ze ścieżki A*
        self.waypoint_spacing_m = 1.0

        # =========================
        # MAPA
        # =========================
        self.load_map()

        # =========================
        # ROS
        # =========================
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/astar_path', 10)

        self.goal_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.goal_callback,
            10
        )

        # TF: potrzebujemy pozycji robota w układzie map
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.waypoints = []
        self.current_waypoint_idx = 0
        self.driving_active = False

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('A* drive node started.')
        self.get_logger().info('Use RViz Publish Point tool on /clicked_point.')

    # ============================================================
    # MAPA
    # ============================================================

    def load_map(self):
        with open(self.map_yaml_path, 'r') as f:
            map_info = yaml.safe_load(f)

        image_name = map_info['image']
        map_dir = os.path.dirname(self.map_yaml_path)
        image_path = os.path.join(map_dir, image_name)

        self.resolution = float(map_info['resolution'])
        self.origin_x = float(map_info['origin'][0])
        self.origin_y = float(map_info['origin'][1])

        self.map_img = cv.imread(image_path, cv.IMREAD_GRAYSCALE)

        if self.map_img is None:
            raise RuntimeError(f'Could not read map image: {image_path}')

        self.height, self.width = self.map_img.shape

        self.get_logger().info(f'Map loaded: {image_path}')
        self.get_logger().info(f'Map size: {self.width} x {self.height}')
        self.get_logger().info(f'Resolution: {self.resolution}')
        self.get_logger().info(f'Origin: {self.origin_x}, {self.origin_y}')

        # Mapa z map_saver:
        # biały = wolne
        # czarny = przeszkoda
        # szary = nieznane
        #
        # Traktujemy tylko bardzo jasne piksele jako wolne.
        free_raw = self.map_img >= 250

        # Wszystko inne: ściany albo nieznane.
        blocked_raw = np.logical_not(free_raw).astype(np.uint8) * 255

        inflation_radius_px = int(self.inflation_radius_m / self.resolution)
        inflation_radius_px = max(inflation_radius_px, 1)

        kernel_size = 2 * inflation_radius_px + 1
        kernel = cv.getStructuringElement(
            cv.MORPH_ELLIPSE,
            (kernel_size, kernel_size)
        )

        inflated_blocked = cv.dilate(blocked_raw, kernel)
        self.free = inflated_blocked == 0

        self.get_logger().info(
            f'Obstacle inflation: {self.inflation_radius_m} m = {inflation_radius_px} px'
        )

    # ============================================================
    # KONWERSJE MAPA <-> ŚWIAT
    # ============================================================

    def world_to_pixel(self, x, y):
        px = int((x - self.origin_x) / self.resolution)

        # YAML/map_server ma origin w lewym dolnym rogu mapy,
        # a obraz PGM ma indeksowanie od lewego górnego rogu.
        py_from_bottom = int((y - self.origin_y) / self.resolution)
        py = self.height - 1 - py_from_bottom

        return px, py

    def pixel_to_world(self, px, py):
        x = self.origin_x + (px + 0.5) * self.resolution
        y = self.origin_y + (self.height - 1 - py + 0.5) * self.resolution
        return x, y

    def in_bounds(self, px, py):
        return 0 <= px < self.width and 0 <= py < self.height

    def is_free(self, px, py):
        if not self.in_bounds(px, py):
            return False
        return bool(self.free[py, px])

    def find_nearest_free(self, start_px, start_py, max_radius_px=80):
        if self.is_free(start_px, start_py):
            return start_px, start_py

        for r in range(1, max_radius_px + 1):
            for dx in range(-r, r + 1):
                for dy in [-r, r]:
                    px = start_px + dx
                    py = start_py + dy
                    if self.is_free(px, py):
                        return px, py

            for dy in range(-r + 1, r):
                for dx in [-r, r]:
                    px = start_px + dx
                    py = start_py + dy
                    if self.is_free(px, py):
                        return px, py

        return None

    # ============================================================
    # TF / POZYCJA ROBOTA
    # ============================================================

    def get_robot_pose_map(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                'jetbot/chassis',
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f'Cannot get robot pose in map frame: {e}')
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y

        q = tf.transform.rotation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        return x, y, yaw

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # ============================================================
    # CALLBACK CELU
    # ============================================================

    def goal_callback(self, msg):
        self.get_logger().info(
            f'Clicked goal: x={msg.point.x:.3f}, y={msg.point.y:.3f}, frame={msg.header.frame_id}'
        )

        robot_pose = self.get_robot_pose_map()
        if robot_pose is None:
            self.get_logger().error('Cannot plan: robot pose unavailable.')
            return

        start_x, start_y, _ = robot_pose
        goal_x = msg.point.x
        goal_y = msg.point.y

        start_px, start_py = self.world_to_pixel(start_x, start_y)
        goal_px, goal_py = self.world_to_pixel(goal_x, goal_y)

        self.get_logger().info(f'Start pixel: {start_px}, {start_py}')
        self.get_logger().info(f'Goal pixel:  {goal_px}, {goal_py}')

        start_free = self.find_nearest_free(start_px, start_py)
        goal_free = self.find_nearest_free(goal_px, goal_py)

        if start_free is None:
            self.get_logger().error('Start is not in free space, even after nearest-free search.')
            return

        if goal_free is None:
            self.get_logger().error('Goal is not in free space, even after nearest-free search.')
            return

        if start_free != (start_px, start_py):
            self.get_logger().warn(f'Start moved to nearest free cell: {start_free}')

        if goal_free != (goal_px, goal_py):
            self.get_logger().warn(f'Goal moved to nearest free cell: {goal_free}')

        path_px = self.astar(start_free, goal_free)

        if path_px is None:
            self.get_logger().error('A* did not find a path.')
            return

        self.get_logger().info(f'A* path length: {len(path_px)} cells')

        path_world = [self.pixel_to_world(px, py) for px, py in path_px]

        self.save_debug_path_image(path_px, start_free, goal_free)
        self.publish_path(path_world)
        self.waypoints = self.make_waypoints(path_world)

        self.current_waypoint_idx = 0
        self.driving_active = True

        self.get_logger().info(f'Waypoints: {len(self.waypoints)}')
        self.get_logger().info('Driving started.')

    # ============================================================
    # A*
    # ============================================================

    def astar(self, start, goal):
        sx, sy = start
        gx, gy = goal

        g_cost = np.full((self.height, self.width), np.inf, dtype=np.float32)
        closed = np.zeros((self.height, self.width), dtype=bool)

        came_from = {}

        g_cost[sy, sx] = 0.0

        open_heap = []
        counter = 0

        h0 = self.heuristic(sx, sy, gx, gy)
        heapq.heappush(open_heap, (h0, counter, (sx, sy)))

        neighbors = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        ]

        while open_heap:
            _, _, (cx, cy) = heapq.heappop(open_heap)

            if closed[cy, cx]:
                continue

            closed[cy, cx] = True

            if (cx, cy) == (gx, gy):
                return self.reconstruct_path(came_from, (gx, gy))

            for dx, dy, move_cost in neighbors:
                nx = cx + dx
                ny = cy + dy

                if not self.is_free(nx, ny):
                    continue

                # Zakaz przechodzenia po skosie przez narożnik ściany.
                if dx != 0 and dy != 0:
                    if not self.is_free(cx + dx, cy):
                        continue
                    if not self.is_free(cx, cy + dy):
                        continue

                if closed[ny, nx]:
                    continue

                tentative_g = g_cost[cy, cx] + move_cost

                if tentative_g < g_cost[ny, nx]:
                    came_from[(nx, ny)] = (cx, cy)
                    g_cost[ny, nx] = tentative_g

                    f = tentative_g + self.heuristic(nx, ny, gx, gy)
                    counter += 1
                    heapq.heappush(open_heap, (f, counter, (nx, ny)))

        return None

    @staticmethod
    def heuristic(x, y, gx, gy):
        return math.hypot(gx - x, gy - y)

    @staticmethod
    def reconstruct_path(came_from, current):
        path = [current]

        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()
        return path

    # ============================================================
    # ŚCIEŻKA / WAYPOINTY
    # ============================================================

    def save_debug_path_image(self, path_px, start_px, goal_px):
        debug = cv.cvtColor(self.map_img, cv.COLOR_GRAY2BGR)

        # Pokaż przeszkody po inflacji na czerwono
        inflated_obstacles = np.logical_not(self.free)
        debug[inflated_obstacles] = [0, 0, 120]

        # Oryginalna mapa jako tło:
        # czarne ściany zostają czarne, wolne pole jasne.
        original_obstacles = self.map_img < 100
        debug[original_obstacles] = [0, 0, 0]

        # Ścieżka A* na zielono
        for i in range(1, len(path_px)):
            p1 = path_px[i - 1]
            p2 = path_px[i]
            cv.line(debug, p1, p2, (0, 255, 0), 2)

        # Start na niebiesko
        cv.circle(debug, start_px, 6, (255, 0, 0), -1)

        # Cel na czerwono
        cv.circle(debug, goal_px, 6, (0, 0, 255), -1)

        out_dir = os.path.expanduser('~/slam_astar_project/results')
        os.makedirs(out_dir, exist_ok=True)

        out_path = os.path.join(out_dir, 'astar_plan.png')
        cv.imwrite(out_path, debug)

        self.get_logger().info(f'A* debug image saved to: {out_path}')

    def publish_path(self, path_world):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in path_world:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)

    def make_waypoints(self, path_world):
        if not path_world:
            return []

        waypoints = [path_world[0]]
        last_x, last_y = path_world[0]

        for x, y in path_world[1:]:
            dist = math.hypot(x - last_x, y - last_y)
            if dist >= self.waypoint_spacing_m:
                waypoints.append((x, y))
                last_x, last_y = x, y

        if waypoints[-1] != path_world[-1]:
            waypoints.append(path_world[-1])

        return waypoints

    # ============================================================
    # STEROWANIE ROBOTEM
    # ============================================================

    def control_loop(self):
        if not self.driving_active:
            return

        if self.current_waypoint_idx >= len(self.waypoints):
            self.stop_robot()
            self.driving_active = False
            self.get_logger().info('Goal reached.')
            return

        pose = self.get_robot_pose_map()
        if pose is None:
            self.stop_robot()
            return

        x, y, yaw = pose
        target_x, target_y = self.waypoints[self.current_waypoint_idx]

        dx = target_x - x
        dy = target_y - y

        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - yaw)

        if distance < self.distance_tolerance:
            self.current_waypoint_idx += 1
            self.get_logger().info(
                f'Next waypoint: {self.current_waypoint_idx}/{len(self.waypoints)}'
            )
            return

        cmd = Twist()

        if abs(angle_error) > self.angle_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = max(
                -self.max_angular_speed,
                min(self.max_angular_speed, 1.2 * angle_error)
            )
        else:
            cmd.linear.x = min(self.max_linear_speed, 0.5 * distance)
            cmd.angular.z = max(
                -self.max_angular_speed,
                min(self.max_angular_speed, 0.8 * angle_error)
            )

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AStarDrive()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()