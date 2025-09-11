#!/usr/bin/env python3
# ============================================================
# File: main.py
# Project: Doppler ICP ROS 2 Node
# Role: Launches a ROS 2 node for LiDAR frame stitching using
#       Doppler-ICP algorithm with velocity constraints.
#
# Description:
#   - Loads sequential CSV point cloud frames (x, y, z, v_radial).
#   - Applies preprocessing: filtering, downsampling, normal estimation.
#   - Runs Doppler-ICP to align frames and estimate motion.
#   - Publishes:
#       * Stitched point cloud (/stitched_cloud)
#       * Current pose (/icp_pose)
#       * Trajectory history (/icp_trajectory)
#       * Linear acceleration (/linear_acceleration)
#       * Angular velocity (/angular_velocity)
#
# Author: Farness AI
# Copyright: 2025
# ============================================================
import os
import glob
import numpy as np
import pandas as pd
import open3d as o3d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Vector3Stamped
from sensor_msgs_py import point_cloud2
from scipy.spatial.transform import Rotation
from sklearn.neighbors import NearestNeighbors
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from numpy.linalg import inv
import time

# ---------- Math helpers ----------
def skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]], dtype=np.float64)


def exp_so3(omega):
    theta = np.linalg.norm(omega)
    if theta < 1e-12:
        return np.eye(3)
    k = omega / theta
    K = skew(k)
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)

def se3_exp(omega, v, dt):
    R = exp_so3(omega * dt)
    t = v * dt
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

# ---------- Node ----------
class DopplerICPStitcher(Node):
    def __init__(self):
        super().__init__('doppler_icp_stitcher')

        # Parameters
        # All configurable parameters (can be set via YAML )
        self.declare_parameter("frames_directory", "/home/farness/Téléchargements/csv_point_clouds")
        self.declare_parameter("velocity_threshold", 20)
        self.declare_parameter("downsample_factor", 2)
        self.declare_parameter("max_iterations", 100)
        self.declare_parameter("icp_tolerance", 1e-6)
        self.declare_parameter("publish_rate", 10)
        self.declare_parameter("lambda_doppler_start", 0.02)
        self.declare_parameter("lambda_doppler_end", 0.1)
        self.declare_parameter("lambda_schedule_iters", 20)
        self.declare_parameter("frame_dt", 0.1)
        self.declare_parameter("t_vl_x", 1)
        self.declare_parameter("t_vl_y", 1)
        self.declare_parameter("t_vl_z", 1)
        self.declare_parameter("reject_outliers", True)
        self.declare_parameter("outlier_thresh", 1.0)
        self.declare_parameter("rejection_min_iters", 2)
        self.declare_parameter("geometric_min_iters", 0)
        self.declare_parameter("doppler_min_iters", 5)
        self.declare_parameter("geometric_k", 0.8) 
        self.declare_parameter("doppler_k", 0.01)
        self.declare_parameter("max_corr_distance", 1)
        self.declare_parameter("min_inliers", 30)

        self.frames_dir = self.get_parameter("frames_directory").value
        self.velocity_threshold = float(self.get_parameter("velocity_threshold").value)
        self.frame_dt = float(self.get_parameter("frame_dt").value)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        # Publishers
        self.publisher_cloud = self.create_publisher(PointCloud2, 'stitched_cloud', qos_profile)
        self.pose_pub = self.create_publisher(PoseStamped, 'icp_pose', qos_profile)
        self.trajectory_pub = self.create_publisher(PoseArray, 'icp_trajectory', qos_profile)
        self.linear_accel_pub = self.create_publisher(Vector3Stamped, 'linear_acceleration', qos_profile)
        self.angular_vel_pub = self.create_publisher(Vector3Stamped, 'angular_velocity', qos_profile)

        # State
        self.stitched_pts = np.empty((0, 4))   # cumulative stitched cloud
        self.current_pose = np.eye(4)          # current estimated pose
        self.trajectory = []                   # trajectory history
        self.frame_files = sorted(glob.glob(os.path.join(self.frames_dir, "*.csv")))
        self.previous_frame = None
        self.current_frame_idx = 0
        self.prev_linear_velocity = None
        self.prev_angular_velocity = None

        publish_rate = float(self.get_parameter("publish_rate").value)
        self.timer = self.create_timer(1.0 / publish_rate, self.frame_loop)
        self.initial_delay_counter = 0
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.timer = self.create_timer(1.0 / publish_rate, self.frame_loop)

    def frame_loop(self):
        if self.initial_delay_counter < 5:
            self.initial_delay_counter += 1
            self.get_logger().info(f"Waiting for Foxglove... ({self.initial_delay_counter}/5)")
            return
        self.process_next_frame()    


    # ---------- IO ----------
    def load_frame(self, filename):
        try:
            df = pd.read_csv(filename)
            if 'radial_vel' in df.columns:
                df = df.rename(columns={'radial_vel': 'v_radial'})
            filtered_df = df[np.abs(df['v_radial']) < self.velocity_threshold][['x', 'y', 'z', 'v_radial']]
            self.get_logger().info(f"Loaded {filename}: {len(df)} points, {len(filtered_df)} after velocity filtering (|v_radial| < {self.velocity_threshold})")
            return filtered_df.to_numpy(dtype=np.float64)
        except Exception as e:
            self.get_logger().error(f"Error loading {filename}: {e}")
            return None

    def preprocess_point_cloud(self, points, velocities):
        downsample_factor = int(self.get_parameter("downsample_factor").value)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Uniform downsampling
        if downsample_factor > 1:
            pcd_down = pcd.uniform_down_sample(downsample_factor)
        else:
            pcd_down = pcd
        # Edge case: empty cloud
        if len(pcd_down.points) == 0:
            return np.empty((0, 3)), np.empty((0, 3)), np.empty((0,)), pcd_down
        # Estimate normals
        pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=10.0, max_nn=30))
        down_pts = np.asarray(pcd_down.points)
        down_normals = np.asarray(pcd_down.normals)
        # Match velocities to downsampled points
        if len(down_pts) < len(points):
            nbrs = NearestNeighbors(n_neighbors=1).fit(points)
            _, idx = nbrs.kneighbors(down_pts)
            down_vels = velocities[idx.flatten()]
        else:
            down_vels = velocities

        return down_pts, down_normals, down_vels, pcd_down

    def huber_weights(self, residuals, k):
        """
        Robust weighting function (Huber loss).
        Reduces influence of outliers in residuals.
        """
        abs_r = np.abs(residuals)
        eps = 1e-12
        return np.where(abs_r <= k, 1.0, k / (abs_r + eps))

    # ---------- Doppler-ICP ----------
    """
        Doppler-ICP algorithm:
          - Iteratively aligns source to target with both geometric and Doppler constraints.
          - Builds and solves a weighted least-squares system.
        Returns:
          transformation, inlier_rmse, linear_velocity, angular_velocity
    """
    
    def doppler_icp(self, source_frame, target_frame):
        max_iter = int(self.get_parameter("max_iterations").value)
        tol = float(self.get_parameter("icp_tolerance").value)
        lambda_start = float(self.get_parameter("lambda_doppler_start").value)
        lambda_end = float(self.get_parameter("lambda_doppler_end").value)
        lambda_iters = int(self.get_parameter("lambda_schedule_iters").value)
        dt = float(self.get_parameter("frame_dt").value)
        t_vl = np.array([self.get_parameter("t_vl_x").value,
                         self.get_parameter("t_vl_y").value,
                         self.get_parameter("t_vl_z").value], dtype=np.float64)
        reject_outliers = bool(self.get_parameter("reject_outliers").value)
        outlier_thresh = float(self.get_parameter("outlier_thresh").value)
        rejection_min_iters = int(self.get_parameter("rejection_min_iters").value)
        geometric_min_iters = int(self.get_parameter("geometric_min_iters").value)
        doppler_min_iters = int(self.get_parameter("doppler_min_iters").value)
        geometric_k = float(self.get_parameter("geometric_k").value)
        doppler_k = float(self.get_parameter("doppler_k").value)
        max_corr_distance = float(self.get_parameter("max_corr_distance").value)
        min_inliers = int(self.get_parameter("min_inliers").value)

        src_pts_raw = source_frame[:, :3].astype(np.float64, copy=False)
        src_vel_raw = source_frame[:, 3].astype(np.float64, copy=False)
        tgt_pts_raw = target_frame[:, :3].astype(np.float64, copy=False)
        tgt_vel_raw = target_frame[:, 3].astype(np.float64, copy=False)

        src_pts, src_normals, src_vels, _ = self.preprocess_point_cloud(src_pts_raw, src_vel_raw)
        tgt_pts, tgt_normals, tgt_vels, _ = self.preprocess_point_cloud(tgt_pts_raw, tgt_vel_raw)

        if len(src_pts) == 0 or len(tgt_pts) == 0 or len(src_normals) == 0 or len(tgt_normals) == 0:
            self.get_logger().warn("Insufficient points/normals after preprocess; returning identity")
            return np.eye(4), np.inf, np.zeros(3), np.zeros(3)

        nbrs = NearestNeighbors(n_neighbors=1).fit(tgt_pts)
        transformation = np.eye(4)
        prev_error = np.inf

        for it in range(max_iter):
            lam = lambda_start + (lambda_end - lambda_start) * min(1.0, it / lambda_iters) if lambda_iters > 0 else lambda_end
            R = transformation[:3, :3]
            t = transformation[:3, 3]
            src_tf = (R @ src_pts.T + t.reshape(3, 1)).T
            dists, indices = nbrs.kneighbors(src_tf)
            dists = dists.flatten()
            idx = indices.flatten()
            closest_pts = tgt_pts[idx]
            closest_normals = tgt_normals[idx]

            r_g = np.sum((src_tf - closest_pts) * closest_normals, axis=1)
            norms = np.linalg.norm(src_pts, axis=1)
            norms = np.where(norms > 1e-12, norms, 1.0)
            d_unit = (src_pts / norms[:, None])
            r_vecs = src_pts + t_vl
            geom_inliers = dists < max_corr_distance
            doppler_inliers = np.ones_like(src_vels, dtype=bool)
            if reject_outliers and (it + 1) >= rejection_min_iters:
                doppler_inliers &= (np.abs(src_vels) < outlier_thresh)
            mask = geom_inliers & doppler_inliers
            if np.count_nonzero(mask) < min_inliers:
                self.get_logger().warn(f"Too few combined inliers ({np.count_nonzero(mask)}) at iter {it+1}; stopping")
                break

            A_rows = []
            b_rows = []
            w_g = self.huber_weights(r_g, geometric_k) if (it + 1) >= geometric_min_iters else np.ones_like(r_g)
            w_d = self.huber_weights(src_vels, doppler_k) if (it + 1) >= doppler_min_iters else np.ones_like(src_vels)
            p_skews_tf = np.stack([skew(p) for p in src_tf], axis=0)

            for j in np.where(mask)[0]:
                n = closest_normals[j]
                Jg_omega = - (n @ p_skews_tf[j]) * dt
                Jg_v = n * dt
                wg = np.sqrt((1.0 - lam) * w_g[j])
                A_rows.append(np.hstack([Jg_omega, Jg_v]) * wg)
                b_rows.append((-r_g[j]) * wg)

            rx_d = np.cross(r_vecs, d_unit)
            for j in np.where(mask)[0]:
                wd = np.sqrt(lam * w_d[j])
                row = np.hstack([rx_d[j], d_unit[j]]) * wd
                A_rows.append(row)
                b_rows.append(src_vels[j] * wd)

            A = np.vstack(A_rows) if len(A_rows) else None
            b = np.array(b_rows, dtype=np.float64) if len(b_rows) else None
            if A is None or A.shape[0] < 6:
                self.get_logger().warn("Insufficient linear equations (A) to solve; stopping ICP")
                break

            try:
                x, *_ = np.linalg.lstsq(A, b, rcond=None)
            except np.linalg.LinAlgError:
                self.get_logger().warn("Linear least squares failed; stopping ICP")
                break

            omega = x[:3]
            v = x[3:]
            delta_T = se3_exp(omega, v, dt)
            transformation = delta_T @ transformation

            R = transformation[:3, :3]
            t = transformation[:3, 3]
            src_tf = (R @ src_pts.T + t.reshape(3, 1)).T
            dists_new, _ = nbrs.kneighbors(src_tf)
            r_g_new = np.sum((src_tf - closest_pts) * closest_normals, axis=1)
            v_pred = (d_unit @ v) + (rx_d @ omega)
            r_d_new = src_vels - v_pred
            wgeom = self.huber_weights(np.abs(r_g_new), geometric_k)
            wdop = self.huber_weights(np.abs(r_d_new), doppler_k)
            total_error = np.mean((1 - lam) * wgeom * (r_g_new * 2) + lam * wdop * (r_d_new * 2))

            self.get_logger().info(f"ICP iter {it+1}/{max_iter}  inliers={np.count_nonzero(mask)}  lam={lam:.3f}  error={total_error:.6e}")

            if abs(prev_error - total_error) < tol:
                self.get_logger().info(f"Converged after {it+1} iterations")
                break
            prev_error = total_error

        R = transformation[:3, :3]
        t = transformation[:3, 3]
        src_tf = (R @ src_pts.T + t.reshape(3, 1)).T
        dists_final, _ = nbrs.kneighbors(src_tf)
        inlier_rmse = float(np.sqrt(np.mean(dists_final ** 2)))

        return transformation, inlier_rmse, v, omega
    
    def build_pointcloud2(self, cloud_np):
        from sensor_msgs_py import point_cloud2
        from std_msgs.msg import Header
        from sensor_msgs.msg import PointField

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
       ]

        cloud_data = np.hstack((
            cloud_np[:, :3],
            cloud_np[:, 3:4]
        )).astype(np.float32)

        return point_cloud2.create_cloud(header, fields, cloud_data)

        
    # ---------- Main loop ----------
    def process_next_frame(self):
        if self.current_frame_idx >= len(self.frame_files):
            self.get_logger().info("All frames processed")
            self.timer.cancel()
            return

        frame_data = self.load_frame(self.frame_files[self.current_frame_idx])
        if frame_data is None:
            self.current_frame_idx += 1
            return

        self.get_logger().info(f"Processing frame {self.current_frame_idx + 1}/{len(self.frame_files)}")

        if self.previous_frame is None:
            self.stitched_pts = frame_data
            self.current_pose = np.eye(4)
            self.trajectory.append(self.current_pose.copy())
            self.previous_frame = frame_data
        else:
            start_time = time.time()
            transform, fitness, linear_velocity, angular_velocity = self.doppler_icp(self.previous_frame, frame_data)
            end_time = time.time()
            self.get_logger().info(f"Doppler-ICP time: {end_time - start_time:.2f}s  inlier RMSE: {fitness:.4f}")

            # Relative pose
            relative_pose = inv(transform)
            self.current_pose = self.current_pose @ relative_pose
            self.trajectory.append(self.current_pose.copy())

            # Transform current frame into map frame
            transformed = (self.current_pose[:3, :3] @ frame_data[:, :3].T + self.current_pose[:3, 3:4]).T
            merged_pts = np.hstack((transformed, frame_data[:, 3:]))
            self.stitched_pts = np.vstack((self.stitched_pts, merged_pts))
            self.previous_frame = frame_data

            # Compute linear acceleration
            if self.prev_linear_velocity is not None:
                linear_acceleration = (linear_velocity - self.prev_linear_velocity) / self.frame_dt
            else:
                linear_acceleration = np.zeros(3)

            # Publish linear acceleration and angular velocity
            self.publish_vector3(linear_acceleration, self.linear_accel_pub)
            self.publish_vector3(angular_velocity, self.angular_vel_pub)

            # Update previous velocities
            self.prev_linear_velocity = linear_velocity
            self.prev_angular_velocity = angular_velocity

        cloud_msg = self.build_pointcloud2(self.stitched_pts)
        self.publisher_cloud.publish(cloud_msg)
        self.publish_current_pose()
        self.publish_trajectory()
        self.current_frame_idx += 1

    # ---------- Publishers ----------
    def publish_pointcloud(self):
        if len(self.stitched_pts) == 0:
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        cloud_data = np.hstack((self.stitched_pts[:, :3], self.stitched_pts[:, 3].reshape(-1, 1))).astype(np.float32)
        cloud_msg = point_cloud2.create_cloud(header, fields, cloud_data)
        self.publisher_cloud.publish(cloud_msg)
        self.get_logger().info(f"Published stitched cloud with {len(cloud_data)} points")

    def publish_current_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        translation = self.current_pose[:3, 3]
        rotation = Rotation.from_matrix(self.current_pose[:3, :3]).as_quat()
        pose_msg.pose.position.x = float(translation[0])
        pose_msg.pose.position.y = float(translation[1])
        pose_msg.pose.position.z = float(translation[2])
        pose_msg.pose.orientation.x = float(rotation[0])
        pose_msg.pose.orientation.y = float(rotation[1])
        pose_msg.pose.orientation.z = float(rotation[2])
        pose_msg.pose.orientation.w = float(rotation[3])
        self.pose_pub.publish(pose_msg)

    def publish_trajectory(self):
        if not self.trajectory:
            return
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "map"
        for pose in self.trajectory:
            p = Pose()
            translation = pose[:3, 3]
            rotation = Rotation.from_matrix(pose[:3, :3]).as_quat()
            p.position.x = float(translation[0])
            p.position.y = float(translation[1])
            p.position.z = float(translation[2])
            p.orientation.x = float(rotation[0])
            p.orientation.y = float(rotation[1])
            p.orientation.z = float(rotation[2])
            p.orientation.w = float(rotation[3])
            pose_array.poses.append(p)
        self.trajectory_pub.publish(pose_array)

    def publish_vector3(self, vec, publisher):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.vector.x = float(vec[0])
        msg.vector.y = float(vec[1])
        msg.vector.z = float(vec[2])
        publisher.publish(msg)

# ---------- Main ----------
def main(args=None):
    rclpy.init(args=args)
    node = DopplerICPStitcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__== "_main_":
    main()
