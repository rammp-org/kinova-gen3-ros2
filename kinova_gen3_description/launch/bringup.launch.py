"""Start the driver and TF together.

`urdf` defaults to this package's expanded model, so robot_state_publisher and the
driver use the SAME model by default -- which is the point of the package. Point it
elsewhere and you own keeping them consistent.
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
    # The 7-DOF variant: core's Dynamics asserts nv == 7 and aborts on the
    # articulated model. robot_state_publisher gets the articulated one.
    default_urdf = PathJoinSubstitution([share, "urdf", "kinova_gen3_7dof.urdf"])
    sim = LaunchConfiguration("sim")

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([share, "launch", "description.launch.py"])
        ),
        launch_arguments={
            "articulated": LaunchConfiguration("articulated"),
        }.items(),
    )

    params = [
        {
            "arbitration_mode": LaunchConfiguration("arbitration_mode"),
            "estop_clear_max_age_s": LaunchConfiguration("estop_clear_max_age_s"),
        }
    ]

    return LaunchDescription(
        [
            # The model's gripper/camera/prefix are BUILD-time xacro args, expanded by
            # CMakeLists; only the articulated/frozen choice is a launch-time one.
            DeclareLaunchArgument(
                "articulated",
                default_value="true",
                description="13-DOF model with a moving gripper; needs the driver to publish "
                "robotiq_85_left_knuckle_joint (the gripper tier does)",
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
            DeclareLaunchArgument("urdf", default_value=default_urdf),
            # Must match the model above: Dynamics resolves this by name and throws if it is
            # absent. The generated model drops the gen3_ prefix core still defaults to.
            DeclareLaunchArgument("ee_frame", default_value="end_effector_link"),
            DeclareLaunchArgument(
                "arbitration_mode",
                default_value="disabled",
                description="enforced | disabled; read-only at runtime",
            ),
            DeclareLaunchArgument("estop_clear_max_age_s", default_value="1.0"),
            description,
            # Two Nodes rather than one with a conditional argument list: the driver takes
            # --sim OR --ip, never both, and launch substitutions cannot build a variable
            # length argv.
            Node(
                package="kinova_gen3_ros2",
                executable="kinova_gen3_node",
                output="screen",
                condition=IfCondition(sim),
                arguments=[
                    "--sim",
                    "--urdf",
                    LaunchConfiguration("urdf"),
                    "--ee-frame",
                    LaunchConfiguration("ee_frame"),
                ],
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
                    "--urdf",
                    LaunchConfiguration("urdf"),
                    "--ee-frame",
                    LaunchConfiguration("ee_frame"),
                ],
                parameters=params,
            ),
        ]
    )
