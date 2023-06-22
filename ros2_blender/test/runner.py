import pytest
import ros2_blender


if __name__ == "__main__":
    argv = [
        "-p",
        "no:ros2_blender.test.launch",
    ]

    def run_pytest():
        ros2_blender.bootstrap(0, "", "false")
        pytest.main(argv, plugins=[])

    run_pytest()
