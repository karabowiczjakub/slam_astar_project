#!/usr/bin/env python3

import os
import math
import time
import heapq
import yaml

import cv2 as cv
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from tf2_ros import Buffer, TransformListener


class AStarDrive(Node):
    def __init__(self):
        super().__init__('astar_drive')

        # ============================================================
        # PLIKI
        # ============================================================
        self.map_yaml_path = os.path.expanduser(
            '~/slam_astar_project/maps/slam_map.yaml'
        )

        self.results_dir = os.path.expanduser(
            '~/slam_astar_project/results'
        )
        os.makedirs(self.results_dir, exist_ok=True)

        # ============================================================
        # PARAMETRY PLANOWANIA
        # ============================================================

        # Obszar całkowicie zabroniony wokół ścian.
        # Jeśli A* nie znajduje trasy, zmniejsz np. na 1.10.
        # Jeśli dalej jedzie za blisko rogów, zwiększ np. na 1.50.
        self.hard_inflation_m = 1.35

        # Obszar "niechętny" wokół ścian.
        # Nie blokuje przejazdu, ale mocno zwiększa koszt.
        self.soft_inflation_m = 3.0

        # Im większe, tym bardziej A* unika jazdy blisko ścian.
        # Wcześniej było 25.0, co było za słabe.
        self.cost_weight = 150.0

        # Nie wygładzamy agresywnie ścieżki, żeby nie ścinać rogów.
        self.enable_path_smoothing = False

        # Mały odstęp waypointów zmniejsza ścinanie zakrętów.
        self.waypoint_spacing_m = 0.35

        # Okna OpenCV:
        # tylko astar_plan ma się wyświetlać.
        self.show_base_windows = False
        self.show_plan_window = True

        # ============================================================
        # PARAMETRY STEROWANIA
        # ============================================================
        self.distance_tolerance = 0.35

        self.max_linear_speed = 0.38
        self.max_angular_speed = 0.85

        # Robot rusza do przodu dopiero, kiedy jest prawie ustawiony.
        self.turn_in_place_angle = 0.38
        self.slow_down_angle = 0.22

        # Awaryjne zatrzymanie przed przeszkodą z lidara.
        self.enable_lidar_safety = True
        self.front_stop_distance = 0.75
        self.front_slow_distance = 1.20
        self.front_min_range = float('inf')

        # ============================================================
        # MAPA
        # ============================================================
        self.load_map()
        self.build_planning_matrices()

        # WAŻNE:
        # Nie zapisujemy ani nie wypisujemy macierzy przy starcie.
        # Macierze/debug zapiszą się dopiero po kliknięciu punktu.

        # ============================================================
        # ROS
        # ============================================================
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/astar_path', 10)
        
        self.robot_marker_pub = self.create_publisher(Marker, '/astar_robot_marker', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/astar_goal_marker', 10)

        self.goal_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.goal_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/lidar',
            self.scan_callback,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.waypoints = []
        self.current_waypoint_idx = 0
        self.driving_active = False

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('A* drive node started.')
        self.get_logger().info('Click a goal in RViz using Publish Point.')
        self.get_logger().info('Matrices will be saved/printed only after clicked goal.')

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

    def build_planning_matrices(self):
        # Biały = wolne, czarny = przeszkoda, szary = nieznane.
        # Do planowania tylko prawie białe piksele są wolne.
        free_raw = self.map_img >= 250
        occupied_or_unknown = np.logical_not(free_raw)

        # binary_occupancy:
        # 0 = wolne
        # 1 = przeszkoda albo nieznane
        self.binary_occupancy = occupied_or_unknown.astype(np.uint8)

        occupied_img = (self.binary_occupancy * 255).astype(np.uint8)

        # ============================================================
        # TWARDA INFLACJA
        # ============================================================
        hard_px = max(1, int(self.hard_inflation_m / self.resolution))
        hard_kernel = cv.getStructuringElement(
            cv.MORPH_ELLIPSE,
            (2 * hard_px + 1, 2 * hard_px + 1)
        )

        hard_inflated_img = cv.dilate(occupied_img, hard_kernel)

        self.hard_blocked = hard_inflated_img > 0
        self.free = np.logical_not(self.hard_blocked)

        # ============================================================
        # ODLEGŁOŚĆ OD ŚCIAN
        # ============================================================
        obstacle_mask = (occupied_img > 0).astype(np.uint8)
        distance_input = (1 - obstacle_mask).astype(np.uint8)

        dist_px = cv.distanceTransform(distance_input, cv.DIST_L2, 5)
        self.dist_m = dist_px * self.resolution

        # ============================================================
        # MIĘKKA COSTMAPA
        # ============================================================
        # 0.0 = daleko od ścian
        # 1.0 = bardzo blisko ściany / obszar zablokowany
        self.costmap = np.zeros_like(self.dist_m, dtype=np.float32)

        near = self.dist_m < self.soft_inflation_m

        normalized = np.zeros_like(self.dist_m, dtype=np.float32)
        normalized[near] = (
            (self.soft_inflation_m - self.dist_m[near]) / self.soft_inflation_m
        )

        # Kara rośnie mocniej przy samej ścianie.
        self.costmap = normalized ** 2

        # Komórki twardo zablokowane mają maksymalny koszt wizualny,
        # ale A* i tak nie może tam wejść przez self.free.
        self.costmap[self.hard_blocked] = 1.0

        self.get_logger().info(
            f'Hard inflation: {self.hard_inflation_m} m'
        )
        self.get_logger().info(
            f'Soft cost radius: {self.soft_inflation_m} m'
        )
        self.get_logger().info(
            f'Cost weight: {self.cost_weight}'
        )

    # ============================================================
    # KONWERSJE MAPA <-> ŚWIAT
    # ============================================================

    def world_to_pixel(self, x, y):
        px = int((x - self.origin_x) / self.resolution)

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

    def find_nearest_free(self, start_px, start_py, max_radius_px=160):
        if self.is_free(start_px, start_py):
            return start_px, start_py

        for r in range(1, max_radius_px + 1):
            for dx in range(-r, r + 1):
                for dy in (-r, r):
                    px = start_px + dx
                    py = start_py + dy
                    if self.is_free(px, py):
                        return px, py

            for dy in range(-r + 1, r):
                for dx in (-r, r):
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
    # LIDAR SAFETY
    # ============================================================

    def scan_callback(self, msg):
        if not msg.ranges:
            return

        n = len(msg.ranges)

        # Zakładamy skan 360 stopni. Bierzemy około +/- 25 stopni z przodu.
        front_width = max(5, int(n * 25.0 / 360.0))

        front_ranges = list(msg.ranges[:front_width]) + list(msg.ranges[-front_width:])

        valid = [
            r for r in front_ranges
            if math.isfinite(r) and msg.range_min < r < msg.range_max
        ]

        if valid:
            self.front_min_range = min(valid)
        else:
            self.front_min_range = float('inf')

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
        self.publish_goal_marker(goal_x, goal_y)

        start_px, start_py = self.world_to_pixel(start_x, start_y)
        goal_px, goal_py = self.world_to_pixel(goal_x, goal_y)

        self.get_logger().info(f'Start pixel: {start_px}, {start_py}')
        self.get_logger().info(f'Goal pixel:  {goal_px}, {goal_py}')

        start_free = self.find_nearest_free(start_px, start_py)
        goal_free = self.find_nearest_free(goal_px, goal_py)

        if start_free is None:
            self.get_logger().error('Start is not in free space after nearest-free search.')
            return

        if goal_free is None:
            self.get_logger().error('Goal is not in free space after nearest-free search.')
            return

        if start_free != (start_px, start_py):
            self.get_logger().warn(f'Start moved to nearest free cell: {start_free}')

        if goal_free != (goal_px, goal_py):
            self.get_logger().warn(f'Goal moved to nearest free cell: {goal_free}')

        astar_result = self.astar(start_free, goal_free)

        if astar_result is None:
            self.get_logger().error('A* did not find a path.')
            self.save_base_debug_files()
            self.save_plan_debug_image([], start_free, goal_free)
            return

        path_px, g_matrix, f_matrix, closed_matrix = astar_result

        self.get_logger().info(f'Raw A* path length: {len(path_px)} cells')

        if self.enable_path_smoothing:
            path_px = self.smooth_path(path_px)
            self.get_logger().info(f'Smoothed path length: {len(path_px)} cells')
        else:
            self.get_logger().info('Path smoothing disabled to avoid corner cutting.')

        # Dopiero po kliknięciu celu zapisujemy mapy bazowe i macierze.
        # Dzięki temu po samym uruchomieniu skryptu nie pojawiają się "puste" debugi.
        self.save_base_debug_files()
        self.save_astar_matrices(
            g_matrix,
            f_matrix,
            closed_matrix,
            start_free,
            goal_free,
            path_px
        )

        path_world = [self.pixel_to_world(px, py) for px, py in path_px]

        self.save_plan_debug_image(path_px, start_free, goal_free)
        self.publish_path(path_world)

        self.waypoints = self.make_waypoints(path_world)

        # Pomijamy pierwszy waypoint, jeśli jest praktycznie pod robotem.
        if len(self.waypoints) > 1:
            rx, ry, _ = robot_pose
            d0 = math.hypot(self.waypoints[0][0] - rx, self.waypoints[0][1] - ry)
            if d0 < 0.4:
                self.waypoints = self.waypoints[1:]

        self.current_waypoint_idx = 0
        self.driving_active = True

        self.get_logger().info(f'Waypoints: {len(self.waypoints)}')
        self.get_logger().info('Driving started.')

    # ============================================================
    # A* Z COSTMAPĄ
    # ============================================================

    def astar(self, start, goal):
        sx, sy = start
        gx, gy = goal

        g_cost = np.full((self.height, self.width), np.inf, dtype=np.float32)
        f_cost = np.full((self.height, self.width), np.inf, dtype=np.float32)
        closed = np.zeros((self.height, self.width), dtype=bool)

        came_from = {}

        g_cost[sy, sx] = 0.0
        f_cost[sy, sx] = self.heuristic(sx, sy, gx, gy)

        open_heap = []
        counter = 0

        heapq.heappush(open_heap, (f_cost[sy, sx], counter, (sx, sy)))

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
                path = self.reconstruct_path(came_from, (gx, gy))
                return path, g_cost, f_cost, closed

            for dx, dy, move_cost in neighbors:
                nx = cx + dx
                ny = cy + dy

                if not self.is_free(nx, ny):
                    continue

                # Nie przechodzimy po skosie przez narożnik.
                if dx != 0 and dy != 0:
                    if not self.is_free(cx + dx, cy):
                        continue
                    if not self.is_free(cx, cy + dy):
                        continue

                if closed[ny, nx]:
                    continue

                proximity_cost = float(self.costmap[ny, nx]) * self.cost_weight

                # Koszt ruchu + mocna kara za bliskość ściany.
                # Dzięki temu A* wybiera trasę bardziej środkiem korytarza,
                # nawet jeśli ścieżka będzie trochę dłuższa.
                tentative_g = g_cost[cy, cx] + move_cost * (1.0 + proximity_cost)

                if tentative_g < g_cost[ny, nx]:
                    came_from[(nx, ny)] = (cx, cy)
                    g_cost[ny, nx] = tentative_g

                    f = tentative_g + self.heuristic(nx, ny, gx, gy)
                    f_cost[ny, nx] = f

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
    # WYGŁADZANIE — DOMYŚLNIE WYŁĄCZONE
    # ============================================================

    def smooth_path(self, path):
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        i = 0

        while i < len(path) - 1:
            j = len(path) - 1

            while j > i + 1:
                if self.has_line_of_sight(path[i], path[j]):
                    break
                j -= 1

            smoothed.append(path[j])
            i = j

        return smoothed

    def has_line_of_sight(self, p1, p2):
        x1, y1 = p1
        x2, y2 = p2

        points = self.bresenham_line(x1, y1, x2, y2)

        for px, py in points:
            if not self.is_free(px, py):
                return False

            # Bardzo konserwatywnie: nie wygładzamy przez drogie obszary.
            if self.costmap[py, px] > 0.25:
                return False

        return True

    @staticmethod
    def bresenham_line(x0, y0, x1, y1):
        points = []

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        err = dx - dy

        x, y = x0, y0

        while True:
            points.append((x, y))

            if x == x1 and y == y1:
                break

            e2 = 2 * err

            if e2 > -dy:
                err -= dy
                x += sx

            if e2 < dx:
                err += dx
                y += sy

        return points

    # ============================================================
    # ŚCIEŻKA / WAYPOINTY
    # ============================================================

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



    def publish_sphere_marker(self, publisher, marker_id, x, y, z, radius, r, g, b, name):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = name
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0

        marker.scale.x = float(radius)
        marker.scale.y = float(radius)
        marker.scale.z = float(radius)

        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = 0.9

        publisher.publish(marker)

    def publish_robot_marker(self):
        pose = self.get_robot_pose_map()
        if pose is None:
            return

        x, y, _ = pose

        # Czerwona kula = pozycja robota według TF/map.
        self.publish_sphere_marker(
            self.robot_marker_pub,
            0,
            x,
            y,
            0.25,
            0.45,
            1.0,
            0.0,
            0.0,
            'robot_position'
        )

    def publish_goal_marker(self, x, y):
        # Niebieska kula = kliknięty punkt/cel.
        self.publish_sphere_marker(
            self.goal_marker_pub,
            0,
            x,
            y,
            0.30,
            0.45,
            0.0,
            0.2,
            1.0,
            'clicked_goal'
        )









    # ============================================================
    # STEROWANIE
    # ============================================================

    def control_loop(self):
        
        self.publish_robot_marker()
        
        if not self.driving_active:
            return
        
        
        
        if self.enable_lidar_safety and self.front_min_range < self.front_stop_distance:
            self.get_logger().warn(
                f'Obstacle too close in front: {self.front_min_range:.2f} m. Stopping.'
            )
            self.safe_stop()
            self.driving_active = False
            return

        if self.current_waypoint_idx >= len(self.waypoints):
            self.safe_stop()
            self.driving_active = False
            self.get_logger().info('Goal reached.')
            return

        pose = self.get_robot_pose_map()
        if pose is None:
            self.safe_stop()
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

        # Duży błąd kąta: obrót w miejscu, bez jazdy do przodu.
        if abs(angle_error) > self.turn_in_place_angle:
            cmd.linear.x = 0.0
            cmd.angular.z = self.clamp(
                1.2 * angle_error,
                -self.max_angular_speed,
                self.max_angular_speed
            )

        # Średni błąd kąta: bardzo wolny ruch.
        elif abs(angle_error) > self.slow_down_angle:
            cmd.linear.x = 0.06
            cmd.angular.z = self.clamp(
                0.9 * angle_error,
                -self.max_angular_speed,
                self.max_angular_speed
            )

        # Mały błąd kąta: normalna jazda.
        else:
            speed = min(self.max_linear_speed, 0.65 * distance)

            if self.enable_lidar_safety and self.front_min_range < self.front_slow_distance:
                speed = min(speed, 0.08)

            cmd.linear.x = speed
            cmd.angular.z = self.clamp(
                0.55 * angle_error,
                -self.max_angular_speed,
                self.max_angular_speed
            )

        self.cmd_pub.publish(cmd)

    @staticmethod
    def clamp(value, vmin, vmax):
        return max(vmin, min(vmax, value))

    def safe_stop(self):
        cmd = Twist()

        for _ in range(10):
            self.cmd_pub.publish(cmd)
            time.sleep(0.03)

    # ============================================================
    # DEBUG — OBRAZY I MACIERZE
    # ============================================================

    def save_base_debug_files(self):
        binary_vis = np.zeros_like(self.map_img)
        binary_vis[self.binary_occupancy == 0] = 255
        binary_vis[self.binary_occupancy == 1] = 0

        inflated_vis = np.zeros_like(self.map_img)
        inflated_vis[self.free] = 255
        inflated_vis[self.hard_blocked] = 0

        cost_vis = (255 - (self.costmap * 255.0)).astype(np.uint8)

        dist_norm = cv.normalize(
            self.dist_m,
            None,
            0,
            255,
            cv.NORM_MINMAX
        ).astype(np.uint8)

        cv.imwrite(os.path.join(self.results_dir, 'binary_map.png'), binary_vis)
        cv.imwrite(os.path.join(self.results_dir, 'inflated_map.png'), inflated_vis)
        cv.imwrite(os.path.join(self.results_dir, 'costmap.png'), cost_vis)
        cv.imwrite(os.path.join(self.results_dir, 'distance_to_wall.png'), dist_norm)

        np.savetxt(
            os.path.join(self.results_dir, 'binary_matrix.txt'),
            self.binary_occupancy.astype(np.uint8),
            fmt='%d'
        )

        np.savetxt(
            os.path.join(self.results_dir, 'inflated_matrix.txt'),
            self.hard_blocked.astype(np.uint8),
            fmt='%d'
        )

        np.savetxt(
            os.path.join(self.results_dir, 'cost_matrix.txt'),
            self.costmap.astype(np.float32),
            fmt='%.2f'
        )

        np.savetxt(
            os.path.join(self.results_dir, 'distance_to_wall_matrix.txt'),
            self.dist_m.astype(np.float32),
            fmt='%.2f'
        )

        self.get_logger().info('Saved base maps and matrices to results/.')

    def save_astar_matrices(self, g_matrix, f_matrix, closed_matrix, start_px, goal_px, path_px):
        # Inf zamieniamy na -1, żeby plik txt był czytelny.
        g_save = np.where(np.isfinite(g_matrix), g_matrix, -1.0)
        f_save = np.where(np.isfinite(f_matrix), f_matrix, -1.0)

        path_matrix = np.zeros((self.height, self.width), dtype=np.uint8)

        for px, py in path_px:
            if self.in_bounds(px, py):
                path_matrix[py, px] = 1

        np.savetxt(
            os.path.join(self.results_dir, 'astar_g_matrix.txt'),
            g_save,
            fmt='%.2f'
        )

        np.savetxt(
            os.path.join(self.results_dir, 'astar_f_matrix.txt'),
            f_save,
            fmt='%.2f'
        )

        np.savetxt(
            os.path.join(self.results_dir, 'astar_closed_matrix.txt'),
            closed_matrix.astype(np.uint8),
            fmt='%d'
        )

        np.savetxt(
            os.path.join(self.results_dir, 'astar_path_matrix.txt'),
            path_matrix,
            fmt='%d'
        )

        preview_path = os.path.join(self.results_dir, 'astar_matrix_preview.txt')

        with open(preview_path, 'w') as f:
            f.write('A* MATRICES PREVIEW AFTER CLICKED GOAL\n')
            f.write('binary: 0=free, 1=occupied/unknown\n')
            f.write('inflated: 0=free, 1=blocked after hard inflation\n')
            f.write('distance: distance to nearest wall in meters\n')
            f.write('cost: 0.00=safe, 1.00=near obstacle/blocked\n')
            f.write('g: cost from start, -1 means not reached\n')
            f.write('f: g+h, -1 means not reached\n')
            f.write('closed: 1 means cell expanded by A*\n')
            f.write('path: 1 means final A* path\n\n')

            for name, point in [('START', start_px), ('GOAL', goal_px)]:
                px, py = point
                f.write(f'\n===== {name} CROP around pixel ({px}, {py}) =====\n')

                f.write('\nBINARY:\n')
                f.write(str(self.crop_matrix(self.binary_occupancy, px, py, 12)))
                f.write('\n')

                f.write('\nINFLATED:\n')
                f.write(str(self.crop_matrix(self.hard_blocked.astype(np.uint8), px, py, 12)))
                f.write('\n')

                f.write('\nDISTANCE TO WALL [m]:\n')
                f.write(str(np.round(self.crop_matrix(self.dist_m, px, py, 12), 2)))
                f.write('\n')

                f.write('\nCOST:\n')
                f.write(str(np.round(self.crop_matrix(self.costmap, px, py, 12), 2)))
                f.write('\n')

                f.write('\nG MATRIX:\n')
                f.write(str(np.round(self.crop_matrix(g_save, px, py, 12), 2)))
                f.write('\n')

                f.write('\nF MATRIX:\n')
                f.write(str(np.round(self.crop_matrix(f_save, px, py, 12), 2)))
                f.write('\n')

                f.write('\nCLOSED:\n')
                f.write(str(self.crop_matrix(closed_matrix.astype(np.uint8), px, py, 12)))
                f.write('\n')

                f.write('\nPATH:\n')
                f.write(str(self.crop_matrix(path_matrix, px, py, 12)))
                f.write('\n')

            if path_px:
                path_arr = np.array(path_px)

                min_x = max(0, int(np.min(path_arr[:, 0])) - 8)
                max_x = min(self.width - 1, int(np.max(path_arr[:, 0])) + 8)

                min_y = max(0, int(np.min(path_arr[:, 1])) - 8)
                max_y = min(self.height - 1, int(np.max(path_arr[:, 1])) + 8)

                f.write('\n===== CROP AROUND WHOLE PATH =====\n')
                f.write(f'x={min_x}:{max_x}, y={min_y}:{max_y}\n')

                f.write('\nPATH MATRIX:\n')
                f.write(str(path_matrix[min_y:max_y + 1, min_x:max_x + 1]))
                f.write('\n')

                f.write('\nCOST MATRIX:\n')
                f.write(str(np.round(self.costmap[min_y:max_y + 1, min_x:max_x + 1], 2)))
                f.write('\n')

                f.write('\nCLOSED MATRIX:\n')
                f.write(str(closed_matrix[min_y:max_y + 1, min_x:max_x + 1].astype(np.uint8)))
                f.write('\n')

        self.get_logger().info('Saved A* matrices after clicked goal:')
        self.get_logger().info('  results/astar_g_matrix.txt')
        self.get_logger().info('  results/astar_f_matrix.txt')
        self.get_logger().info('  results/astar_closed_matrix.txt')
        self.get_logger().info('  results/astar_path_matrix.txt')
        self.get_logger().info('  results/astar_matrix_preview.txt')

        # W terminalu pokazujemy macierze obliczone PO KLIKNIĘCIU,
        # a nie żadne zerowe macierze ze startu programu.
        print('\n==================== A* MATRICES AFTER CLICKED GOAL ====================')
        print(f'START pixel: {start_px}')
        print(f'GOAL pixel:  {goal_px}')
        print(f'Path length: {len(path_px)} cells')

        print('\n--- G MATRIX CROP AROUND START ---')
        print(np.round(self.crop_matrix(g_save, start_px[0], start_px[1], 8), 2))

        print('\n--- F MATRIX CROP AROUND START ---')
        print(np.round(self.crop_matrix(f_save, start_px[0], start_px[1], 8), 2))

        print('\n--- CLOSED MATRIX CROP AROUND START ---')
        print(self.crop_matrix(closed_matrix.astype(np.uint8), start_px[0], start_px[1], 8))

        print('\n--- PATH MATRIX CROP AROUND GOAL ---')
        print(self.crop_matrix(path_matrix, goal_px[0], goal_px[1], 8))

        print('\n--- COST MATRIX CROP AROUND GOAL ---')
        print(np.round(self.crop_matrix(self.costmap, goal_px[0], goal_px[1], 8), 2))

        print('=======================================================================\n')

    @staticmethod
    def crop_matrix(matrix, px, py, half):
        h, w = matrix.shape

        x1 = max(0, px - half)
        x2 = min(w, px + half + 1)

        y1 = max(0, py - half)
        y2 = min(h, py + half + 1)

        return matrix[y1:y2, x1:x2]

    def save_plan_debug_image(self, path_px, start_px, goal_px):
        debug = cv.cvtColor(self.map_img, cv.COLOR_GRAY2BGR)

        # Costmapa jako fioletowa poświata.
        cost_overlay = (self.costmap * 180).astype(np.uint8)
        debug[:, :, 0] = np.maximum(debug[:, :, 0], cost_overlay)
        debug[:, :, 2] = np.maximum(debug[:, :, 2], cost_overlay)

        # Twardo zabronione obszary na fioletowo.
        debug[self.hard_blocked] = [160, 0, 160]

        # Oryginalne ściany na czarno.
        original_obstacles = self.map_img < 100
        debug[original_obstacles] = [0, 0, 0]

        # Ścieżka A* na zielono.
        if path_px:
            for i in range(1, len(path_px)):
                cv.line(debug, path_px[i - 1], path_px[i], (0, 255, 0), 2)

        # Start na niebiesko.
        cv.circle(debug, start_px, 7, (255, 0, 0), -1)

        # Cel na czerwono.
        cv.circle(debug, goal_px, 7, (0, 0, 255), -1)

        out_path = os.path.join(self.results_dir, 'astar_plan.png')
        cv.imwrite(out_path, debug)

        self.get_logger().info(f'Saved plan image: {out_path}')

        # Wyświetlamy tylko końcowy plan, żadnych map pośrednich.
        if self.show_plan_window:
            cv.namedWindow('astar_plan', cv.WINDOW_NORMAL)
            cv.imshow('astar_plan', debug)
            cv.resizeWindow('astar_plan', 900, 900)
            cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = AStarDrive()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt detected. Stopping robot...')
    finally:
        node.safe_stop()
        cv.waitKey(200)
        cv.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()