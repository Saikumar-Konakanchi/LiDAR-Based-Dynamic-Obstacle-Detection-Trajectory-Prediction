#!/usr/bin/env python3
"""
Tracker node: runs a constant-velocity Kalman filter per detected dynamic
cluster, associates new detections to existing tracks, predicts future
positions, and publishes live + predicted visualization markers.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Point
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry

MAX_MATCH_DIST = 1.0
TRACK_TIMEOUT = 1.0
STATIC_SPEED_THRESH = 0.2
PREDICTION_HORIZONS = [0.5, 1.0, 1.5, 2.0]

PROCESS_NOISE_POS = 0.01
PROCESS_NOISE_VEL = 0.1
MEASUREMENT_NOISE = 0.15


class KalmanTrack:
    def __init__(self, track_id, x, y, now):
        self.id = track_id
        self.last_seen = now
        self.x = np.array([x, y, 0.0, 0.0])
        self.P = np.diag([0.1, 0.1, 1.0, 1.0])
        self.Q = np.diag([
            PROCESS_NOISE_POS, PROCESS_NOISE_POS,
            PROCESS_NOISE_VEL, PROCESS_NOISE_VEL,
        ])
        self.R = np.diag([MEASUREMENT_NOISE, MEASUREMENT_NOISE])
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ])

    def predict(self, dt):
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

    @property
    def pos(self):
        return self.x[0], self.x[1]

    @property
    def vel(self):
        return self.x[2], self.x[3]

    @property
    def speed(self):
        return math.hypot(self.x[2], self.x[3])

    def predict_future(self, t):
        fx = self.x[0] + self.x[2] * t
        fy = self.x[1] + self.x[3] * t
        return fx, fy


class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')
        self.set_parameters(
            [rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)]
        )

        self.cluster_sub = self.create_subscription(
            PoseArray, '/detected_clusters', self.cluster_cb, 10
        )
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.dist_pub = self.create_publisher(Float32, '/closest_object_distance', 10)
        self.track_pub = self.create_publisher(MarkerArray, '/tracked_objects', 10)
        self.predict_pub = self.create_publisher(MarkerArray, '/predicted_markers', 10)

        self.tracks = {}
        self.next_id = 0
        self.last_time = None

        self.rover_x = 0.0
        self.rover_y = 0.0
        self.rover_yaw = 0.0

        self.get_logger().info('Tracker node started — waiting for detections')

    def odom_cb(self, msg):
        self.rover_x = msg.pose.pose.position.x
        self.rover_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.rover_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def cluster_cb(self, msg):
        now_msg = self.get_clock().now()
        now = now_msg.nanoseconds / 1e9

        if self.last_time is None:
            self.last_time = now
        dt = now - self.last_time
        if dt <= 0:
            dt = 0.05
        self.last_time = now

        detections = [(p.position.x, p.position.y) for p in msg.poses]

        for track in self.tracks.values():
            track.predict(dt)

        unmatched_detections = list(range(len(detections)))
        matched_track_ids = set()

        for det_idx in list(unmatched_detections):
            dx, dy = detections[det_idx]
            best_id = None
            best_dist = MAX_MATCH_DIST
            for tid, track in self.tracks.items():
                if tid in matched_track_ids:
                    continue
                tx, ty = track.pos
                d = math.hypot(dx - tx, dy - ty)
                if d < best_dist:
                    best_dist = d
                    best_id = tid

            if best_id is not None:
                self.tracks[best_id].update(np.array([dx, dy]))
                self.tracks[best_id].last_seen = now
                matched_track_ids.add(best_id)
                unmatched_detections.remove(det_idx)

        for det_idx in unmatched_detections:
            dx, dy = detections[det_idx]
            new_track = KalmanTrack(self.next_id, dx, dy, now)
            self.tracks[self.next_id] = new_track
            self.next_id += 1

        stale_ids = [tid for tid, t in self.tracks.items() if (now - t.last_seen) > TRACK_TIMEOUT]
        for tid in stale_ids:
            del self.tracks[tid]

        moving_tracks = [t for t in self.tracks.values() if t.speed >= STATIC_SPEED_THRESH]

        self.publish_distance(moving_tracks)
        self.publish_tracks(moving_tracks)
        self.publish_predictions(moving_tracks)

    def publish_distance(self, moving_tracks):
        msg = Float32()
        if not moving_tracks:
            msg.data = float('inf')
        else:
            min_d = float('inf')
            for t in moving_tracks:
                tx, ty = t.pos
                lx, ly = self.world_to_local(tx, ty)
                d = math.hypot(lx, ly)
                if d < min_d:
                    min_d = d
            msg.data = min_d
        self.dist_pub.publish(msg)

    def world_to_local(self, wx, wy):
        dx = wx - self.rover_x
        dy = wy - self.rover_y
        lx = dx * math.cos(self.rover_yaw) + dy * math.sin(self.rover_yaw)
        ly = -dx * math.sin(self.rover_yaw) + dy * math.cos(self.rover_yaw)
        return lx, ly

    def publish_tracks(self, moving_tracks):
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        header_frame = 'base_link'
        stamp = self.get_clock().now().to_msg()

        for track in moving_tracks:
            wx, wy = track.pos
            vx, vy = track.vel
            lx, ly = self.world_to_local(wx, wy)
            dist = math.hypot(lx, ly)

            cyl = Marker()
            cyl.header.frame_id = header_frame
            cyl.header.stamp = stamp
            cyl.ns = 'tracked'
            cyl.id = track.id
            cyl.type = Marker.CYLINDER
            cyl.action = Marker.ADD
            cyl.pose.position.x = lx
            cyl.pose.position.y = ly
            cyl.pose.position.z = 0.9
            cyl.pose.orientation.w = 1.0
            cyl.scale.x = 0.6
            cyl.scale.y = 0.6
            cyl.scale.z = 1.8
            cyl.color.r = 1.0
            cyl.color.g = 0.0 if dist <= 1.0 else 0.5
            cyl.color.b = 0.0
            cyl.color.a = 0.7
            array.markers.append(cyl)

            local_vx = vx * math.cos(self.rover_yaw) + vy * math.sin(self.rover_yaw)
            local_vy = -vx * math.sin(self.rover_yaw) + vy * math.cos(self.rover_yaw)

            arrow = Marker()
            arrow.header.frame_id = header_frame
            arrow.header.stamp = stamp
            arrow.ns = 'velocity'
            arrow.id = track.id + 1000
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = 0.1
            arrow.scale.y = 0.2
            arrow.scale.z = 0.2
            arrow.color.r = 0.0
            arrow.color.g = 1.0
            arrow.color.b = 1.0
            arrow.color.a = 1.0
            p0 = Point()
            p0.x = lx
            p0.y = ly
            p0.z = 1.8
            p1 = Point()
            p1.x = lx + local_vx * 1.5
            p1.y = ly + local_vy * 1.5
            p1.z = 1.8
            arrow.points = [p0, p1]
            array.markers.append(arrow)

            txt = Marker()
            txt.header.frame_id = header_frame
            txt.header.stamp = stamp
            txt.ns = 'labels'
            txt.id = track.id + 2000
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = lx
            txt.pose.position.y = ly
            txt.pose.position.z = 2.3
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.25
            txt.color.r = 1.0
            txt.color.g = 1.0
            txt.color.b = 1.0
            txt.color.a = 1.0
            state_str = 'STOP' if dist <= 1.0 else 'SLOW' if dist <= 2.5 else 'TRACKING'
            txt.text = f'TRACK #{track.id}: {dist:.1f}m\nspeed: {track.speed:.2f}m/s [{state_str}]'
            array.markers.append(txt)

        self.track_pub.publish(array)

    def publish_predictions(self, moving_tracks):
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        header_frame = 'base_link'
        stamp = self.get_clock().now().to_msg()
        marker_id = 0

        for track in moving_tracks:
            ghost_points_local = []

            for t_ahead in PREDICTION_HORIZONS:
                fx, fy = track.predict_future(t_ahead)
                lx, ly = self.world_to_local(fx, fy)
                ghost_points_local.append((lx, ly))

                fade = 1.0 - (t_ahead / (max(PREDICTION_HORIZONS) + 0.5))

                ghost = Marker()
                ghost.header.frame_id = header_frame
                ghost.header.stamp = stamp
                ghost.ns = 'ghost'
                ghost.id = marker_id
                marker_id += 1
                ghost.type = Marker.SPHERE
                ghost.action = Marker.ADD
                ghost.pose.position.x = lx
                ghost.pose.position.y = ly
                ghost.pose.position.z = 0.9
                ghost.pose.orientation.w = 1.0
                ghost.scale.x = 0.4
                ghost.scale.y = 0.4
                ghost.scale.z = 1.2
                ghost.color.r = 1.0
                ghost.color.g = 1.0
                ghost.color.b = 0.0
                ghost.color.a = max(0.15, fade * 0.6)
                array.markers.append(ghost)

                label = Marker()
                label.header.frame_id = header_frame
                label.header.stamp = stamp
                label.ns = 'ghost_label'
                label.id = marker_id
                marker_id += 1
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position.x = lx
                label.pose.position.y = ly
                label.pose.position.z = 1.6
                label.pose.orientation.w = 1.0
                label.scale.z = 0.18
                label.color.r = 1.0
                label.color.g = 1.0
                label.color.b = 0.6
                label.color.a = max(0.3, fade)
                label.text = f'+{t_ahead:.1f}s'
                array.markers.append(label)

            if ghost_points_local:
                cur_lx, cur_ly = self.world_to_local(*track.pos)
                trail = Marker()
                trail.header.frame_id = header_frame
                trail.header.stamp = stamp
                trail.ns = 'ghost_trail'
                trail.id = marker_id
                marker_id += 1
                trail.type = Marker.LINE_STRIP
                trail.action = Marker.ADD
                trail.scale.x = 0.04
                trail.color.r = 1.0
                trail.color.g = 1.0
                trail.color.b = 0.0
                trail.color.a = 0.5
                pt0 = Point()
                pt0.x = cur_lx
                pt0.y = cur_ly
                pt0.z = 0.9
                trail.points = [pt0]
                for lx, ly in ghost_points_local:
                    pt = Point()
                    pt.x = lx
                    pt.y = ly
                    pt.z = 0.9
                    trail.points.append(pt)
                array.markers.append(trail)

        self.predict_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TrackerNode())


if __name__ == '__main__':
    main()
