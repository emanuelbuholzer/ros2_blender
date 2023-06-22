import launch
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    EnvironmentVariable,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare

from ros2_blender import installation


def generate_launch_description():
    declared_arguments = []

    blender_executable = LaunchConfiguration("blender_executable")
    declared_arguments.append(
        DeclareLaunchArgument(
            name="blender_executable",
            description="The blender executable to use",
            default_value=str(installation.find_blender_executable()),
        )
    )

    blend_file = LaunchConfiguration("blend_file")
    declared_arguments.append(
        DeclareLaunchArgument(
            name="blend_file",
            description="Open given blend file, instead of the default startup file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ros2_blender"), "launch", "default.blend"]
            ),
        )
    )

    addon_paths = LaunchConfiguration("addon_paths")
    declared_arguments.append(
        DeclareLaunchArgument(
            name="addon_paths",
            description="Comma separated list of paths of blender addons",
            default_value="",
        )
    )

    addons = LaunchConfiguration("addons")
    declared_arguments.append(
        DeclareLaunchArgument(
            name="addons",
            description="Comma separated list of addons to enable",
            default_value="",
        )
    )

    read_prefs = LaunchConfiguration("read_prefs")
    declared_arguments.append(
        DeclareLaunchArgument(
            name="read_prefs",
            description="Read user preferences",
            default_value="false",
        )
    )

    domain_id = EnvironmentVariable(name="ROS_DOMAIN_ID", default_value="0")

    link_addons_action = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution(
                [FindPackageShare("ros2_blender"), "launch", "link_addons.py"]
            ),
            blender_executable,
            addon_paths,
        ],
        output="both",
    )

    exec_blender_file_action = ExecuteProcess(
        cmd=[
            blender_executable,
            "--factory-startup",
            "--python-expr",
            PythonExpression(
                [  # fmt: off
                    "'\"",
                    "import ros2_blender; ros2_blender.bootstrap_launch_compat(",
                    domain_id,
                    ", addons_str=\\'",
                    addons,
                    "\\'",
                    ", read_prefs_str=\\'",
                    read_prefs,
                    "\\'" ")" "\"'",
                ]  # fmt: on
            ),
            blend_file,
        ],
        additional_env={"ROS_DOMAIN_ID": domain_id},
        name=["Blender"],
        log_cmd=True,
        output="both",
        shell=True,
    )

    exec_blender_file_after_link_addons_action = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=link_addons_action, on_exit=[exec_blender_file_action]
        )
    )

    unlink_addons_action = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=exec_blender_file_action,
            on_exit=[
                ExecuteProcess(
                    cmd=[
                        "python3",
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ros2_blender"),
                                "launch",
                                "unlink_addons.py",
                            ]
                        ),
                        blender_executable,
                        addon_paths,
                    ],
                    output="both",
                )
            ],
        )
    )

    return launch.LaunchDescription(
        declared_arguments
        + [
            link_addons_action,
            exec_blender_file_after_link_addons_action,
            unlink_addons_action,
        ]
    )
