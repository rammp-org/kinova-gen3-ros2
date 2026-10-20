"""Start the driver and TF together, on one model.

description.launch.py renders the model from mount_file and publishes it on
/arm/robot_description; the driver reads it from there. So robot_state_publisher and
the driver cannot disagree -- which is the point of the package.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("kinova_gen3_description")
    sim = LaunchConfiguration("sim")

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([share, "launch", "description.launch.py"])
        ),
        launch_arguments={"mount_file": LaunchConfiguration("mount_file")}.items(),
    )

    params = [
        {
            "arbitration_mode": LaunchConfiguration("arbitration_mode"),
            "estop_clear_max_age_s": LaunchConfiguration("estop_clear_max_age_s"),
            "joint_torque_damping": LaunchConfiguration("joint_torque_damping"),
        }
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mount_file",
                description="the robot's mount.yaml. Required: no default.",
            ),
            DeclareLaunchArgument(
                "sim",
                default_value="true",
                description="run against SimTransport instead of the arm",
            ),
            DeclareLaunchArgument(
                "ip",
                default_value="192.168.1.10",
                description="arm IP; used only when sim:=false",
            ),
            # Dynamics resolves this by name and throws if it is absent from the model.
            DeclareLaunchArgument("ee_frame", default_value="arm_end_effector_link"),
            DeclareLaunchArgument(
                "arbitration_mode",
                default_value="disabled",
                description="enforced | disabled; read-only at runtime",
            ),
            DeclareLaunchArgument("estop_clear_max_age_s", default_value="1.0"),
            DeclareLaunchArgument(
                "joint_torque_damping",
                default_value="0.0",
                description="joint-torque mode damping [N*m*s/rad]; read-only at runtime",
            ),
            description,
            # Two Nodes rather than one with a conditional argument list: the driver takes
            # --sim OR --ip, never both, and launch substitutions cannot build a variable
            # length argv.
            Node(
                package="kinova_gen3_ros2",
                executable="kinova_gen3_node",
                output="screen",
                condition=IfCondition(sim),
                arguments=["--sim", "--ee-frame", LaunchConfiguration("ee_frame")],
                parameters=params,
            ),
            Node(
                package="kinova_gen3_ros2",
                executable="kinova_gen3_node",
                output="screen",
                condition=UnlessCondition(sim),
                arguments=[
                    "--ip",
                    LaunchConfiguration("ip"),
                    "--ee-frame",
                    LaunchConfiguration("ee_frame"),
                ],
                parameters=params,
            ),
        ]
    )
