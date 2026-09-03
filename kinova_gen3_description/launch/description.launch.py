"""Publish /robot_description and run robot_state_publisher.

Deliberately contains nothing arm-specific and starts no driver, so a client that wants
only TF -- a planner, a perception node, RViz -- can include it without owning the arm.

robot_state_publisher DOES derive <mimic> joints -- verified 2026-09-03 with a two-joint
URDF: publishing only the driver joint moved the mimic link by exactly -0.6 rad for a
+0.6 rad drive. The articulated model has 13 movable joints but only EIGHT independent
ones (seven arm plus robotiq_85_left_knuckle_joint); RSP derives the other five.

The driver publishes that knuckle joint as of the gripper tier, so `articulated` now
defaults to true. It is safe on a gripper-less build: the joint is not in the model and
RSP ignores joint states for joints it does not know.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _rsp(model_file, condition):
    share = FindPackageShare("kinova_gen3_description")
    path = PathJoinSubstitution([share, "urdf", model_file])
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        condition=condition,
        # value_type=str is required: without it launch_ros guesses the type of the
        # Command substitution and robot_state_publisher receives a mangled or empty
        # description, failing as silently as the bug this package exists to fix.
        parameters=[{
            "robot_description": ParameterValue(Command(["cat ", path]), value_type=str)
        }],
    )


def generate_launch_description():
    articulated = LaunchConfiguration("articulated")
    return LaunchDescription([
        DeclareLaunchArgument(
            "articulated", default_value="true",
            description="use the 13-DOF model with a moving gripper. Requires the driver to "
                        "publish robotiq_85_left_knuckle_joint (the gripper tier does); "
                        "robot_state_publisher derives the five mimic joints itself."),
        _rsp("kinova_gen3_7dof.urdf", UnlessCondition(articulated)),
        _rsp("kinova_gen3.urdf", IfCondition(articulated)),
    ])
