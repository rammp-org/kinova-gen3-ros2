"""description.launch.py, live: the model on /arm/robot_description, latched, and TF
rooted at arm_mount with the mount's rotation."""

import math
import os
import time
import unittest

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

TILTED = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mounts", "tilted.yaml")
LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


@pytest.mark.launch_test
def generate_test_description():
    desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("kinova_gen3_description"), "launch", "description.launch.py"]
            )
        ),
        launch_arguments={"mount_file": TILTED}.items(),
    )
    return LaunchDescription([desc, launch_testing.actions.ReadyToTest()])


class TestDescription(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("description_launch_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, done, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline and not done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return done()

    def test_model_is_latched_on_arm_robot_description(self):
        # Subscribe AFTER robot_state_publisher has published: only a transient-local
        # subscription still gets it, which is what the driver and cuRobo rely on.
        time.sleep(3.0)
        got = []
        self.node.create_subscription(
            String, "/arm/robot_description", lambda m: got.append(m.data), LATCHED
        )
        self.assertTrue(self._spin_until(lambda: got, 10.0), "nothing latched")
        self.assertIn('name="arm_tool_frame"', got[0])
        self.assertNotIn("ros2_control", got[0])

    def test_not_published_on_the_global_topic(self):
        # Wait until the remapped topic exists, so "absent" is not just "too early".
        def names():
            return dict(self.node.get_topic_names_and_types())

        self.assertTrue(
            self._spin_until(lambda: "/arm/robot_description" in names(), 10.0)
        )
        self.assertNotIn("/robot_description", names())

    def test_tf_carries_the_mount_rotation(self):
        buf = Buffer()
        TransformListener(buf, self.node)
        tf = []

        def lookup():
            try:
                tf.append(
                    buf.lookup_transform("arm_mount", "arm_base_link", rclpy.time.Time())
                )
            except Exception:
                pass
            return tf

        self.assertTrue(self._spin_until(lookup, 10.0), "no arm_mount -> arm_base_link")
        q = tf[0].transform.rotation
        # +30 deg about Y: (x, y, z, w) = (0, sin 15deg, 0, cos 15deg)
        self.assertAlmostEqual(q.x, 0.0, places=6)
        self.assertAlmostEqual(q.y, math.sin(math.radians(15.0)), places=6)
        self.assertAlmostEqual(q.z, 0.0, places=6)
        self.assertAlmostEqual(q.w, math.cos(math.radians(15.0)), places=6)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
