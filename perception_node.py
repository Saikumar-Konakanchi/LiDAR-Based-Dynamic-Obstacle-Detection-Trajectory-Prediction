#!/usr/bin/env python3
"""
Perception node: builds a static point-cloud map of the environment,
classifies incoming LiDAR points as static/dynamic, clusters dynamic
points, and runs the rover's reactive patrol state machine.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, PoseArray, Pose
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
from rviz_2d_overlay_msgs.msg import OverlayText

STATIC_SAMPLES = 100
DYNAMIC_THRESHOLD = 0.3
CLUSTER_DIST = 0.6
MIN_CLUSTER_SIZE = 3
SLOW_DISTANCE = 2.5
STOP_DISTANCE = 2.5
FORWARD_SPEED = 0.4
SLOW_SPEED = 0.1

END_POINT_X = 5.0


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.set_parameters(
            [rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)]
        )

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.dist_sub = self.create_subscription(
            Float32, '/closest_object_distance', self.dist_cb, 10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cluster_pub = self.create_publisher(PoseArray, '/detected_clusters', 10)
        self.ui_pub = self.create_publisher(MarkerArray, '/ui_elements', 10)
        self.hud_pub = self.create_publisher(OverlayText, '/hud', 10)

        self.rover_x = 0.0
        self.rover_y = 0.0
        self.rover_yaw = 0.0
        self.current_speed = 0.0

        self.static_points_odom = None
        self.scan_buffer = []
        self.scan_count = 0
        self.building_map = True
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.num_ranges = 0
        self.range_min = 0.0
        self.range_max = 0.0

        self.closest_dynamic_dist = float('inf')

        self.actual_path = []

        self.state = 'WAITING'
        self.state_start = self.get_clock().now()

        self.get_logger().info('Keep rover STILL — building static map...')

    def odom_cb(self, msg):
        self.rover_x = msg.pose.pose.position.x
        self.rover_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.rover_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def dist_cb(self, msg):
        self.closest_dynamic_dist = msg.data

    def scan_cb(self, msg):
        ranges = np.array(msg.ranges)

        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.num_ranges = len(ranges)
        self.range_max = msg.range_max
        self.range_min = msg.range_min

        angles = np.linspace(self.angle_min, self.angle_max, self.num_ranges)

        if self.building_map:
            self._accumulate_static_map(ranges, angles, msg.range_max)
            self.publish_hud(999.0)
            return

        self.publish_ui()

        if self.state == 'FINISHED':
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.cmd_pub.publish(stop_msg)
            self.current_speed = 0.0
            self.publish_hud(999.0)
            return

        ranges = np.where(np.isinf(ranges), msg.range_max, ranges)
        ranges = np.where(np.isnan(ranges), msg.range_max, ranges)

        valid_mask = (ranges > self.range_min) & (ranges < self.range_max)
        r_valid = ranges[valid_mask]
        a_valid = angles[valid_mask]

        local_x = r_valid * np.cos(a_valid)
        local_y = r_valid * np.sin(a_valid)

        world_x = self.rover_x + local_x * np.cos(self.rover_yaw) - local_y * np.sin(self.rover_yaw)
        world_y = self.rover_y + local_x * np.sin(self.rover_yaw) + local_y * np.cos(self.rover_yaw)
        curr_odom = np.column_stack((world_x, world_y))

        if len(curr_odom) == 0 or self.static_points_odom is None or len(self.static_points_odom) == 0:
            self.patrol(self.closest_dynamic_dist)
            self.publish_hud(self.closest_dynamic_dist)
            return

        diff = curr_odom[:, np.newaxis, :] - self.static_points_odom[np.newaxis, :, :]
        dist_sq = np.sum(diff ** 2, axis=-1)
        min_dist_sq = np.min(dist_sq, axis=1)
        dynamic_mask = min_dist_sq > (DYNAMIC_THRESHOLD ** 2)

        dyn_world_x = world_x[dynamic_mask]
        dyn_world_y = world_y[dynamic_mask]
        dynamic_points_world = list(zip(dyn_world_x, dyn_world_y))

        clusters = self.cluster(dynamic_points_world)
        self.publish_clusters(clusters)

        self.patrol(self.closest_dynamic_dist)
        self.publish_hud(self.closest_dynamic_dist)

    def _accumulate_static_map(self, ranges, angles, range_max):
        self.scan_buffer.append(ranges)
        self.scan_count += 1
        if self.scan_count % 10 == 0:
            self.get_logger().info(f'Building static map... {self.scan_count}/{STATIC_SAMPLES}')

        if self.scan_count >= STATIC_SAMPLES:
            static_r = np.median(self.scan_buffer, axis=0)
            static_r = np.where(np.isinf(static_r), range_max, static_r)
            static_r = np.where(np.isnan(static_r), range_max, static_r)

            valid_mask = static_r < range_max
            r_valid = static_r[valid_mask]
            a_valid = angles[valid_mask]

            local_x = r_valid * np.cos(a_valid)
            local_y = r_valid * np.sin(a_valid)

            ox = self.rover_x + local_x * np.cos(self.rover_yaw) - local_y * np.sin(self.rover_yaw)
            oy = self.rover_y + local_x * np.sin(self.rover_yaw) + local_y * np.cos(self.rover_yaw)

            self.static_points_odom = np.column_stack((ox, oy))
            self.building_map = False

            self.get_logger().info('Static point cloud built — starting patrol')
            self.state = 'FORWARD'
            self.state_start = self.get_clock().now()

    def cluster(self, points):
        if not points:
            return []
        clusters = []
        used = [False] * len(points)
        for i in range(len(points)):
            if used[i]:
                continue
            cluster = [points[i]]
            used[i] = True
            for j in range(i + 1, len(points)):
                if not used[j]:
                    d = math.hypot(points[i][0] - points[j][0], points[i][1] - points[j][1])
                    if d < CLUSTER_DIST:
                        cluster.append(points[j])
                        used[j] = True
            if len(cluster) >= MIN_CLUSTER_SIZE:
                clusters.append(cluster)
        return clusters

    def publish_clusters(self, clusters):
        array = PoseArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = 'odom'

        for cluster in clusters:
            cx = float(np.mean([p[0] for p in cluster]))
            cy = float(np.mean([p[1] for p in cluster]))
            pose = Pose()
            pose.position.x = cx
            pose.position.y = cy
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            array.poses.append(pose)

        self.cluster_pub.publish(array)

    def patrol(self, min_dist):
        msg = Twist()
        now = self.get_clock().now()

        if self.rover_x >= END_POINT_X:
            if self.state != 'FINISHED':
                self.get_logger().info(
                    f'Destination reached at X: {self.rover_x:.2f}m! Mission complete.'
                )
                self.state = 'FINISHED'
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.current_speed = 0.0
            self.cmd_pub.publish(msg)
            return

        if min_dist <= STOP_DISTANCE:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.current_speed = 0.0
            if self.state != 'STOPPED':
                self.get_logger().info(f'STOPPED — dynamic object at {min_dist:.2f}m')
                self.state = 'STOPPED'
                self.state_start = now

        elif min_dist <= SLOW_DISTANCE:
            msg.linear.x = SLOW_SPEED
            msg.angular.z = 0.0
            self.current_speed = SLOW_SPEED
            if self.state != 'SLOWING':
                self.get_logger().info(f'SLOWING — dynamic object at {min_dist:.2f}m')
                self.state = 'SLOWING'
                self.state_start = now

        else:
            if self.state in ('STOPPED', 'SLOWING'):
                self.get_logger().info('Path clear — resuming')
            msg.linear.x = FORWARD_SPEED
            msg.angular.z = 0.0
            self.current_speed = FORWARD_SPEED
            self.state = 'FORWARD'

        self.cmd_pub.publish(msg)

    def publish_hud(self, min_dist):
        hud = OverlayText()
        hud.action = OverlayText.ADD
        hud.width = 400
        hud.height = 180
        hud.horizontal_alignment = OverlayText.LEFT
        hud.horizontal_distance = 10
        hud.vertical_alignment = OverlayText.TOP
        hud.vertical_distance = 10
        hud.text_size = 14.0
        hud.line_width = 3
        hud.font = "DejaVu Sans Mono"

        if self.state == 'STOPPED':
            hud.bg_color.r, hud.bg_color.g, hud.bg_color.b, hud.bg_color.a = 1.0, 0.0, 0.0, 0.9
            hud.fg_color.r, hud.fg_color.g, hud.fg_color.b, hud.fg_color.a = 1.0, 1.0, 1.0, 1.0
        elif self.state == 'SLOWING':
            hud.bg_color.r, hud.bg_color.g, hud.bg_color.b, hud.bg_color.a = 1.0, 0.6, 0.0, 0.9
            hud.fg_color.r, hud.fg_color.g, hud.fg_color.b, hud.fg_color.a = 0.0, 0.0, 0.0, 1.0
        elif self.state == 'FINISHED':
            hud.bg_color.r, hud.bg_color.g, hud.bg_color.b, hud.bg_color.a = 0.0, 0.4, 1.0, 0.9
            hud.fg_color.r, hud.fg_color.g, hud.fg_color.b, hud.fg_color.a = 1.0, 1.0, 1.0, 1.0
        else:
            hud.bg_color.r, hud.bg_color.g, hud.bg_color.b, hud.bg_color.a = 0.1, 0.1, 0.1, 0.8
            hud.fg_color.r, hud.fg_color.g, hud.fg_color.b, hud.fg_color.a = 0.2, 1.0, 0.2, 1.0

        dist_str = f"{min_dist:.2f} m" if min_dist != float('inf') and min_dist != 999.0 else "CLEAR"

        hud.text = (
            f"=== ROVER TELEMETRY ===\n\n"
            f"SYSTEM STATE : {self.state}\n"
            f"ROVER SPEED  : {self.current_speed:.2f} m/s\n"
            f"POSITION X   : {self.rover_x:.2f} m\n"
            f"TARGET DIST  : {dist_str}"
        )
        self.hud_pub.publish(hud)

    def publish_ui(self):
        array = MarkerArray()
        now = self.get_clock().now().to_msg()

        rover_label = Marker()
        rover_label.header.frame_id = 'base_link'
        rover_label.ns = 'rover_status'
        rover_label.id = 999
        rover_label.type = Marker.TEXT_VIEW_FACING
        rover_label.action = Marker.ADD
        rover_label.pose.position.z = 1.2
        rover_label.scale.z = 0.35

        if self.state == 'STOPPED':
            rover_label.color.r, rover_label.color.g, rover_label.color.b, rover_label.color.a = 1.0, 0.0, 0.0, 1.0
        elif self.state == 'SLOWING':
            rover_label.color.r, rover_label.color.g, rover_label.color.b, rover_label.color.a = 1.0, 0.8, 0.0, 1.0
        else:
            rover_label.color.r, rover_label.color.g, rover_label.color.b, rover_label.color.a = 0.0, 1.0, 1.0, 1.0

        rover_label.text = f"ROVER: {self.current_speed:.2f} m/s\n[{self.state}]"
        array.markers.append(rover_label)

        ring = Marker()
        ring.header.frame_id = 'base_link'
        ring.ns = 'sensor_range'
        ring.id = 998
        ring.type = Marker.CYLINDER
        ring.action = Marker.ADD
        ring.scale.x = 6.0
        ring.scale.y = 6.0
        ring.scale.z = 0.01
        ring.pose.position.z = 0.02
        ring.color.r, ring.color.g, ring.color.b, ring.color.a = 0.0, 0.5, 1.0, 0.1
        array.markers.append(ring)

        self.actual_path.append((self.rover_x, self.rover_y))
        if len(self.actual_path) > 1000:
            self.actual_path = self.actual_path[-1000:]

        path_marker = Marker()
        path_marker.header.frame_id = 'odom'
        path_marker.header.stamp = now
        path_marker.ns = 'actual_path'
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.05
        path_marker.color.r = 0.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        path_marker.color.a = 1.0
        for px, py in self.actual_path:
            pt = Point()
            pt.x = px
            pt.y = py
            pt.z = 0.05
            path_marker.points.append(pt)
        array.markers.append(path_marker)

        plan_marker = Marker()
        plan_marker.header.frame_id = 'odom'
        plan_marker.ns = 'planned_path'
        plan_marker.id = 1
        plan_marker.type = Marker.LINE_STRIP
        plan_marker.action = Marker.ADD
        plan_marker.scale.x = 0.05
        plan_marker.color.r = 0.0
        plan_marker.color.g = 0.5
        plan_marker.color.b = 1.0
        plan_marker.color.a = 0.5
        pt1 = Point()
        pt1.x = -3.0
        pt1.y = 0.0
        pt1.z = 0.05
        pt2 = Point()
        pt2.x = END_POINT_X
        pt2.y = 0.0
        pt2.z = 0.05
        plan_marker.points = [pt1, pt2]
        array.markers.append(plan_marker)

        dest = Marker()
        dest.header.frame_id = 'odom'
        dest.ns = 'destination'
        dest.id = 2
        dest.type = Marker.CYLINDER
        if self.state == 'FINISHED':
            dest.scale.x = 1.0
            dest.scale.y = 1.0
            dest.scale.z = 0.1
            dest.color.r = 0.0
            dest.color.g = 1.0
            dest.color.b = 0.0
            dest.color.a = 0.8
        else:
            dest.scale.x = 0.5
            dest.scale.y = 0.5
            dest.scale.z = 0.1
            dest.color.r = 1.0
            dest.color.g = 0.0
            dest.color.b = 0.0
            dest.color.a = 0.5
        dest.pose.position.x = END_POINT_X
        dest.pose.position.y = 0.0
        dest.pose.position.z = 0.05
        array.markers.append(dest)

        self.ui_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PerceptionNode())


if __name__ == '__main__':
    main()
