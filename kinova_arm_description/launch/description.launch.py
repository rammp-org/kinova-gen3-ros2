"""Publish /robot_description and run robot_state_publisher.

Deliberately contains nothing arm-specific and starts no driver, so a client that wants
only TF -- a planner, a perception node, RViz -- can include it without owning the arm.

DEFAULTS TO THE 7-DOF MODEL, and that is not arbitrary. robot_state_publisher in Humble
does NOT derive mimic joints: given 7 of the articulated model's 13 movable joints it
publishes NOTHING on /tf -- not a partial tree, nothing. Measured, not assumed:

    7 arm joints                      -> 0 /tf messages
    7 arm + the actuated knuckle      -> 0 /tf messages
    all 13 movable joints             -> transforms appear

The driver publishes 7 joint states, so the frozen-gripper model is the one that
actually produces TF today. Set articulated:=true once something publishes the gripper's
six joints -- either the driver computing the mimics itself, or joint_state_publisher in
the chain.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _rsp(model_file, condition):
    share = FindPackageShare("kinova_arm_description")
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
            "articulated", default_value="false",
            description="use the 13-DOF model with a moving gripper. Requires something "
                        "to publish all six Robotiq joints, or TF stays empty."),
        _rsp("kinova_arm_7dof.urdf", UnlessCondition(articulated)),
        _rsp("kinova_arm.urdf", IfCondition(articulated)),
    ])
