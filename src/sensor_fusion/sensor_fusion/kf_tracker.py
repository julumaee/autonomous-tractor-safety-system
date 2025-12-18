# Copyright 2025 Eemil Kulmala, University of Oulu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

from geometry_msgs.msg import TwistWithCovarianceStamped as TWCS
import numpy as np
import rclpy
from rclpy.node import Node
from tractor_safety_system_interfaces.msg import (
    FusedDetection,
    FusedDetectionArray
)


def rot2d(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s],
                     [s,  c]], dtype=float)


class Track:
    __slots__ = ('x', 'P', 'age', 'hits', 'miss', 'id', 'last_stamp', 'was_updated')

    def __init__(self, x0, P0, stamp, tid):
        self.x = x0.copy()      # [x, y, vx, vy]
        self.P = P0.copy()
        self.age = 1
        self.hits = 1
        self.miss = 0
        self.id = tid
        self.last_stamp = stamp
        self.was_updated = True


class EgoKFTracker(Node):

    def __init__(self):
        super().__init__('ego_kf_tracker')

        # --- Control->vehicle parameters
        self.declare_parameter('wheelbase', 2.5)
        self.declare_parameter('steer_unit_per_rad', 180.0/math.pi)  # int units per rad
        self.declare_parameter('speed_unit_per_mps', 1.0)            # int units per m/s
        self.declare_parameter('control_latency', 0.10)              # seconds
        self.declare_parameter('max_yaw_rate', 1.0)                  # rad/s safety clamp

        # --- KF modelling
        self.declare_parameter('sigma_accel', 3.0)    # m/s^2 (constant-accel white noise)
        self.declare_parameter('R_meas_xy', 0.3)      # m std for fused measurement (per-axis)
        self.declare_parameter('gate_chi2', 10.597)   # 99.5% in 2D

        # --- Track mgmt
        self.declare_parameter('spawn_hits', 3)          # how many hits to confirm a track
        self.declare_parameter('spawn_hit_ratio', 0.75)  # min ratio of hits to age to confirm
        self.declare_parameter('max_miss', 10)           # max consecutive misses before deletion
        self.declare_parameter('init_speed_std', 3.0)    # m/s initial std on velocity
        self.declare_parameter('update_rate', 20.0)      # Hz
        self.declare_parameter('enable_ego_drag', True)

        # Read params
        self.L = float(self.get_parameter('wheelbase').value)
        self.k_steer = float(self.get_parameter('steer_unit_per_rad').value)
        self.k_speed = float(self.get_parameter('speed_unit_per_mps').value)
        self.control_latency = float(self.get_parameter('control_latency').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)

        self.sigma_accel = float(self.get_parameter('sigma_accel').value)
        self.R_meas = (float(self.get_parameter('R_meas_xy').value)**2) * np.eye(2)
        self.gate_chi2 = float(self.get_parameter('gate_chi2').value)

        self.spawn_hits = int(self.get_parameter('spawn_hits').value)
        self.spawn_hit_ratio = float(self.get_parameter('spawn_hit_ratio').value)
        self.max_miss = int(self.get_parameter('max_miss').value)
        self.init_speed_std = float(self.get_parameter('init_speed_std').value)

        self.dt_nom = 1.0/float(self.get_parameter('update_rate').value)
        self.enable_ego_drag = bool(self.get_parameter('enable_ego_drag').value)

        self.ego_x = 0.0
        self.ego_y = 0.0
        self.ego_yaw = 0.0

        # Subscriptions
        self.sub_det = self.create_subscription(
            FusedDetection, '/fused_detections', self.on_detection, 50)
        self.sub_odom = self.create_subscription(
            TWCS, '/ego_motion', self.on_ego_odom, 100)

        # Publisher
        self.pub_tracks = self.create_publisher(FusedDetectionArray, '/tracked_detections', 50)

        # Timer loop
        self.timer = self.create_timer(self.dt_nom, self.on_timer)

        # Internal state
        self.tracks = []     # list[Track]
        self.next_id = 1
        self.last_timer_stamp = None

        # Latest motion
        self.u_v = 0.0       # m/s
        self.u_delta = 0.0   # rad
        self.u_stamp = None

        self.get_logger().info('KF tracker running.')

    # ------------------ Inputs ------------------

    def on_detection(self, msg: FusedDetection):
        """Receive and store a fused detection in ego frame."""
        position_ego = np.array([msg.position.x, msg.position.y], dtype=float)
        position_world = self.ego_to_world_pos(position_ego)
        stamp = msg.header.stamp
        source = msg.detection_type
        # store immediately in a small buffer for this cycle
        # Simple approach: push onto a list; the timer will consume and clear.
        if not hasattr(self, '_meas_buf'):
            self._meas_buf = []
        self._meas_buf.append((position_world, self.R_meas, stamp, source))

    def on_ego_odom(self, twist_msg: TWCS):
        """Receive ego vehicle twist (body frame)."""
        self.u_v = float(twist_msg.twist.twist.linear.x)
        self.u_delta = float(twist_msg.twist.twist.angular.z)
        self.u_stamp = twist_msg.header.stamp.sec + twist_msg.header.stamp.nanosec * 1e-9

    def publish_tracks(self):
        """Publish confirmed tracks in ego (base_link) frame."""
        now = self.get_clock().now().to_msg()
        track_array = FusedDetectionArray()
        track_array.header.stamp = now
        for tr in self.tracks:
            if tr.hits < self.spawn_hits:
                continue  # only publish confirmed
            msg = FusedDetection()
            msg.header.stamp = tr.last_stamp
            msg.header.frame_id = 'base_link'
            position_ego = self.world_to_ego_pos(tr.x[0:2])
            velocity_ego = self.world_to_ego_vel(tr.x[2:4])
            msg.position.x = float(position_ego[0])
            msg.position.y = float(position_ego[1])
            msg.position.z = 0.0
            msg.distance = float(np.hypot(tr.x[0], tr.x[1]))
            vx = float(velocity_ego[0])
            vy = float(velocity_ego[1])
            msg.speed = float(np.hypot(vx, vy))
            msg.is_tracking = True
            msg.tracking_id = f'object_{tr.id}'
            msg.detection_type = 'tracked'
            msg.age = tr.age
            msg.consecutive_misses = tr.miss
            track_array.detections.append(msg)
        self.pub_tracks.publish(track_array)

    def update_ego_pose(self, dt: float) -> None:
        """
        Integrate ego pose in a local world (odometry) frame.

        use current body-frame twist:
        v = self.u_v [m/s], yaw_rate = self.u_delta [rad/s].
        """
        if dt <= 0.0:
            return

        v = float(self.u_v)        # forward speed in base_link
        yaw_rate = float(self.u_delta)  # THIS IS YAW RATE [rad/s], NOT steering angle

        # small-step integration in ego/body frame
        dx_body = v * dt
        dy_body = 0.0
        dth = yaw_rate * dt

        # rotate body displacement into world frame using current heading
        c = math.cos(self.ego_yaw)
        s = math.sin(self.ego_yaw)
        dx_world = c * dx_body - s * dy_body
        dy_world = s * dx_body + c * dy_body

        # update ego pose in world frame
        self.ego_x += dx_world
        self.ego_y += dy_world
        self.ego_yaw += dth

    def ego_to_world_pos(self, p_ego: np.ndarray) -> np.ndarray:
        """
        Convert a 2D position from ego (base_link) to world (odometry) frame.

        p_ego: shape (2,)
        """
        c = math.cos(self.ego_yaw)
        s = math.sin(self.ego_yaw)
        R = np.array([[c, -s],
                      [s,  c]], dtype=float)
        t = np.array([self.ego_x, self.ego_y], dtype=float)
        return t + R @ p_ego

    def world_to_ego_pos(self, p_world: np.ndarray) -> np.ndarray:
        """
        Convert a 2D position from world (odometry) to ego (base_link) frame.

        p_world: shape (2,)
        """
        c = math.cos(self.ego_yaw)
        s = math.sin(self.ego_yaw)
        R_T = np.array([[c, s],
                        [-s, c]], dtype=float)
        t = np.array([self.ego_x, self.ego_y], dtype=float)
        return R_T @ (p_world - t)

    def ego_to_world_vel(self, v_ego: np.ndarray) -> np.ndarray:
        """
        Convert a 2D velocity from ego frame to world frame.

        Note: we assume small dt, so we just rotate the vector.
        """
        c = math.cos(self.ego_yaw)
        s = math.sin(self.ego_yaw)
        R = np.array([[c, -s],
                      [s,  c]], dtype=float)
        return R @ v_ego

    def world_to_ego_vel(self, v_world: np.ndarray) -> np.ndarray:
        """Convert a 2D velocity from world frame to ego frame."""
        c = math.cos(self.ego_yaw)
        s = math.sin(self.ego_yaw)
        R_T = np.array([[c, s],
                        [-s, c]], dtype=float)
        return R_T @ v_world

    def on_timer(self):
        """
        Perform all operations in the timer loop.

        1. Ego update
        2. KF predict/update
        3. track mgmt
        4. publish.
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_timer_stamp is None:
            self.last_timer_stamp = now

        dt = max(1e-3, now - self.last_timer_stamp)

        # 1. Update ego pose
        self.update_ego_pose(dt)

        # 2. KF predict for all tracks over dt
        if self.tracks:
            self.kf_predict_all(dt)
            # Reset update flags
            for tr in self.tracks:
                tr.was_updated = False

        # 3. Data association & updates using buffered detections
        meas = getattr(self, '_meas_buf', [])
        if meas:
            self.associate_and_update(meas)
            self._meas_buf.clear()
        for tr in self.tracks:
            if not tr.was_updated:
                tr.miss += 1

        # 4. Aging / deletion
        self.maintain_tracks()
        self.publish_tracks()
        self.last_timer_stamp = now
        # for tr in self.tracks:
        #    self.get_logger().info(f'Track {tr.id}: pos=({tr.x[0]:.2f},{tr.x[1]:.2f}) '
        #                           f'vel=({tr.x[2]:.2f},{tr.x[3]:.2f}) '
        #                           f'hits={tr.hits} miss={tr.miss} age={tr.age}')

    # ------------------ Ego dead-reckoning ------------------

    def integrate_twist(v, yaw_rate, dt):
        """Dead-reckon pose change over dt given body-frame twist."""
        # motion expressed in the *previous* body frame
        dx = v * dt         # move forward in body x
        dy = 0.0            # no lateral motion in body frame
        dth = yaw_rate * dt  # heading change

        return dx, dy, dth

    # ------------------ KF predict/update ------------------

    def kf_predict_all(self, dt):
        """KF predict step for all tracks over dt."""
        # Constant-velocity model with discrete white accel noise
        a = self.sigma_accel  # Acceleration std
        # State transition matrix:
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]], dtype=float)
        q = a*a  # Acceleration variance
        # Process noise covariance:
        Q = q * np.array([[dt**4/4,     0, dt**3/2,    0],
                          [0,     dt**4/4,    0, dt**3/2],
                          [dt**3/2,     0,   dt**2,    0],
                          [0,     dt**3/2,    0,   dt**2]], dtype=float)
        # Predict all tracks
        for tr in self.tracks:
            tr.x = F @ tr.x
            tr.P = F @ tr.P @ F.T + Q
            tr.age += 1

    def kf_update_track(self, tr, position, Rm, H, stamp):
        """KF update step for a single track with one measurement."""
        # KF update
        y = position - H @ tr.x  # Innovation
        S = H @ tr.P @ H.T + Rm   # Innovation covariance
        K = tr.P @ H.T @ np.linalg.inv(S)  # Kalman gain
        identity = np.eye(4, dtype=float)

        tr.x = tr.x + K @ y  # State update

        # Joseph stabilized covariance update
        KH = K @ H
        tr.P = (identity - KH) @ tr.P @ (identity - KH).T + K @ Rm @ K.T

        # Mark as updated
        tr.was_updated = True
        tr.hits += 1
        tr.miss = 0
        tr.last_stamp = stamp

    def associate_and_update(self, meas_list):
        """Associate measurements to tracks and perform KF updates."""
        # Simple NN gating per measurement.
        # Map state to measurement
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)

        # Sort detection array
        order = {'fused': 0, 'camera': 1, 'radar': 2}
        meas_list.reverse()  # Newest first
        # Sort based on type priority (fused > camera > radar)
        meas_list.sort(key=lambda d: order.get(str(d[3]).casefold().strip(), 99))

        # Update loop
        used_tracks = set()
        for position, Rm, stamp, src in meas_list:
            best_i = -1
            best_d2 = None
            for i, tr in enumerate(self.tracks):
                if i in used_tracks:
                    continue
                y = position - H @ tr.x
                S = H @ tr.P @ H.T + Rm
                try:
                    Sinv = np.linalg.inv(S)
                except np.linalg.LinAlgError:
                    Sinv = np.linalg.pinv(S)
                d2 = float(y.T @ Sinv @ y)
                if d2 < self.gate_chi2 and (best_d2 is None or d2 < best_d2):
                    best_d2 = d2
                    best_i = i

            if best_i >= 0:
                tr = self.tracks[best_i]
                self.kf_update_track(tr, position, Rm, H, stamp)
                used_tracks.add(best_i)
                continue

            # Suppress duplicates from lower-priority sources
            duplicate = False
            for i, tr in enumerate(self.tracks):
                yj = position - H @ tr.x
                Sj = H @ tr.P @ H.T + Rm
                try:
                    Sinvj = np.linalg.inv(Sj)
                except np.linalg.LinAlgError:
                    Sinvj = np.linalg.pinv(Sj)
                d2j = float(yj.T @ Sinvj @ yj)
                if d2j < self.gate_chi2:
                    duplicate = True
                    break
            if duplicate:
                continue  # Don't spawn a new track

            # No association with existing tracks, spawn a new track with zero vel
            P0 = np.diag([0.5, 0.5, self.init_speed_std**2, self.init_speed_std**2])
            x0 = np.array([position[0], position[1], 0.0, 0.0], dtype=float)
            self.tracks.append(Track(x0, P0, stamp, self.next_id))
            self.next_id += 1

    def maintain_tracks(self):
        """Age and delete tracks based on hits/misses."""
        kept = []
        for tr in self.tracks:
            # confirm criterion
            confirmed = tr.hits >= self.spawn_hits
            alive = tr.miss <= self.max_miss
            if confirmed and alive:
                kept.append(tr)
            elif not confirmed and alive and tr.age * self.spawn_hit_ratio <= self.spawn_hits:
                # brief grace for newborns
                kept.append(tr)
        self.tracks = kept


def main():
    rclpy.init()
    node = EgoKFTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
