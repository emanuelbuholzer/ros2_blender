import launch
import launch.actions as actions
import launch.substitutions as substitutions
from launch import conditions


def generate_launch_description():
    declared_arguments = []

    blend_file = substitutions.LaunchConfiguration(
        variable_name="blend_file",
    )
    blend_file_arg = actions.DeclareLaunchArgument(
        name="blend_file",
        description="Open given blend file, instead of the default startup file",
        default_value="false"
    )
    declared_arguments.append(blend_file_arg)

    no_blend_file_cond = conditions.UnlessCondition(
        substitutions.PythonExpression(["'true' if '", blend_file, "' != 'false' else 'false'"])
    )

    blend_file_cond = conditions.UnlessCondition(
        substitutions.NotSubstitution(
            substitutions.PythonExpression(["'true' if '", blend_file, "' != '' else 'false'"])
        )
    )

    # skip_user_prefs = PythonExpression([  # noqa: F841
    #     LaunchConfiguration(variable_name="skip_user_prefs", default=False),
    #     " is 'True'"
    # ]),

    # skip_user_prefs_arg = DeclareLaunchArgument(
    #     name="skip_user_prefs",
    #     default_value="False",
    #     description="TODO",
    #     choices=["True", "False"]
    # )
    # declared_arguments.append(skip_user_prefs_arg)

    exec_blender_file_action = actions.ExecuteProcess(
        cmd=["blender", blend_file],
        name=["Blender"],
        condition=blend_file_cond
    )

    exec_blender_action = actions.ExecuteProcess(
        cmd=["blender"],
        name=["Blender"],
        condition=no_blend_file_cond
    )

    return launch.LaunchDescription(declared_arguments + [
        exec_blender_file_action,
        exec_blender_action
    ])
