# ==============================================
# File: test/test_dicp_launch.py
# Role: ROS 2 launch test for dicp_stitcher node (multi-topic + params + dynamic updates, type-robust)
# Copyright (c) 2025 Farness AI — Internal Use Only
# ==============================================

import os
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, PoseArray

from ament_index_python.packages import get_package_share_directory

import launch
import launch_testing.actions
import launch_testing.markers


@pytest.mark.launch_test
def generate_test_description():
    """
    Include the dicp_stitcher.launch.py file and prepare the test environment.
    """
    pkg_dir = get_package_share_directory('farness_dicp')
    launch_file = os.path.join(pkg_dir, 'launch', 'dicp_stitcher.launch.py')

    return (
        launch.LaunchDescription([
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(launch_file)
            ),
            launch_testing.actions.ReadyToTest()
        ]),
        {}
    )


class MultiTopicListener(Node):
    """
    Test node that subscribes to topics and verifies parameters.
    """

    def __init__(self):
        rclpy.init()
        super().__init__('dicp_multi_test_listener')

        # --- Topic flags ---
        self.cloud_received = False
        self.pose_received = False
        self.traj_received = False

        # --- Subscriptions ---
        self.create_subscription(PointCloud2, '/stitched_cloud', self.cloud_cb, 10)
        self.create_subscription(PoseStamped, '/current_pose', self.pose_cb, 10)
        self.create_subscription(PoseArray, '/trajectory', self.traj_cb, 10)

    # --- Callbacks ---
    def cloud_cb(self, msg):
        self.get_logger().info("✅ Received stitched cloud")
        self.cloud_received = True

    def pose_cb(self, msg):
        self.get_logger().info("✅ Received current pose")
        self.pose_received = True

    def traj_cb(self, msg):
        self.get_logger().info("✅ Received trajectory")
        self.traj_received = True

    # --- Spin until all topics are seen ---
    def spin_until_all_received(self, timeout_sec=10.0):
        import time
        start = time.time()
        while time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.cloud_received and self.pose_received and self.traj_received:
                break
        return self.cloud_received, self.pose_received, self.traj_received

    # --- Check parameters ---
    def check_parameters(self, expected_params):
        """
        Ensures all expected parameters are declared and match values.
        Auto-casts expected values to match node types.
        """
        for param, expected_value in expected_params.items():
            node_value = self.get_parameter(param).value
            # Cast expected type to match node type
            if isinstance(node_value, float):
                expected_value = float(expected_value)
            elif isinstance(node_value, int):
                expected_value = int(expected_value)
            assert node_value == expected_value, (
                f"❌ Param {param}={node_value}, expected {expected_value}"
            )

    # --- Change parameters dynamically ---
    def update_parameter(self, param, new_value):
        """
        Update a parameter at runtime and confirm it changed.
        Handles type automatically (int vs float).
        """
        from rcl_interfaces.srv import SetParameters
        from rclpy.parameter import Parameter

        client = self.create_client(SetParameters, 'set_parameters')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for parameter service...")

        # Detect type from current parameter
        current_value = self.get_parameter(param).value
        if isinstance(current_value, float):
            new_value = float(new_value)
        elif isinstance(current_value, int):
            new_value = int(new_value)

        req = SetParameters.Request()
        req.parameters = [Parameter(name=param, value=new_value).to_parameter_msg()]

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        # Re-check param value
        result = self.get_parameter(param).value
        assert result == new_value, f"❌ Param {param} failed to update (got {result})"


@launch_testing.markers.keep_alive
def test_all_topics_and_params():
    """
    Ensure dicp_stitcher publishes all topics and loads parameters.
    """
    listener = MultiTopicListener()

    # --- Spin and check topics ---
    cloud_ok, pose_ok, traj_ok = listener.spin_until_all_received()
    assert cloud_ok, "❌ No /stitched_cloud messages received"
    assert pose_ok, "❌ No /current_pose messages received"
    assert traj_ok, "❌ No /trajectory messages received"

    # --- Check parameters (values from YAML, type auto-corrected) ---
    expected = {
        "publish_rate": 5,
        "downsample_factor": 10,
        "max_iterations": 5,
        "reject_outliers": True
    }
    listener.check_parameters(expected)

    rclpy.shutdown()


@launch_testing.markers.keep_alive
def test_dynamic_parameter_update():
    """
    Test that parameters can be updated dynamically at runtime.
    Example: change publish_rate and confirm update.
    """
    listener = MultiTopicListener()

    # Spin a little to ensure node is alive
    listener.spin_until_all_received(timeout_sec=5.0)

    # Update publish_rate dynamically (type auto-detected)
    new_rate = 10
    listener.update_parameter("publish_rate", new_rate)

    # Verify update applied
    result = listener.get_parameter("publish_rate").value
    assert result == new_rate, f"❌ Dynamic update failed: publish_rate={result}"

    rclpy.shutdown()
