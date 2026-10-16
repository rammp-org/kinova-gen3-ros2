"""Render the arm's model for one robot and run robot_state_publisher on it.

Deliberately contains nothing arm-specific and starts no driver, so a client that wants
only TF -- a planner, a perception node, RViz -- can include it without owning the arm.

mount_file is REQUIRED and has no default: it is the robot's mount.yaml (the rotation
of arm_base_link in arm_mount), and a default would silently render a level arm on a
tilted robot. The model is rendered HERE, at launch, and published on
/arm/robot_description (latched); the driver and cuRobo read it from there, so all
three use one model by construction.

robot_state_publisher DOES derive <mimic> joints -- verified 2026-09-03 with a two-joint
URDF: publishing only the driver joint moved the mimic link by exactly -0.6 rad for a
+0.6 rad drive. The model has 13 movable joints but only EIGHT independent ones (seven
arm plus arm_robotiq_85_left_knuckle_joint), all of which the driver publishes.
"""

from kinova_gen3_render import embodiment_line, render
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _rsp(context):
    xml, mount = render(LaunchConfiguration("mount_file").perform(context))
    return [
        LogInfo(msg=embodiment_line(mount)),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            # value_type=str: without it launch_ros guesses the type of the string
            # and robot_state_publisher can receive a mangled description.
            parameters=[{"robot_description": ParameterValue(xml, value_type=str)}],
            # The model belongs to the arm; /tf and /joint_states stay global.
            remappings=[("robot_description", "/arm/robot_description")],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mount_file",
                description="the robot's mount.yaml (in containers "
                "/config/robot/mount.yaml). Required: no default.",
            ),
            OpaqueFunction(function=_rsp),
        ]
    )
