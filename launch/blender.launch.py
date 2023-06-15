import launch
import launch.actions as actions
import launch.substitutions as substitutions


def generate_launch_description():
    declared_arguments = []

    blend_file = substitutions.LaunchConfiguration(
        variable_name="blend_file",
    )
    blend_file_arg = actions.DeclareLaunchArgument(
        name="blend_file",
        description="Open given blend file, instead of the default startup file",
        default_value="false",
    )
    declared_arguments.append(blend_file_arg)

    domain_id = substitutions.EnvironmentVariable(
        name="ROS_DOMAIN_ID", default_value="0"
    )

    exec_blender_file_action = actions.ExecuteProcess(
        cmd=[
            "blender",
            "--python-expr",
            '"import ros2_blender; ros2_blender.bootstrap()"',
            blend_file,
        ],
        additional_env={"ROS_DOMAIN_ID": domain_id},
        name=["Blender"],
        log_cmd=True,
        output="both",
        shell=True,
    )

    return launch.LaunchDescription(
        declared_arguments
        + [
            exec_blender_file_action,
        ]
    )
