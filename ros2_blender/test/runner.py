import pytest
from ros2_blender import bootstrap


if __name__ == "__main__":
    argv = [
        "-p",
        "no:ros2_blender.test.launch",
    ]

    def run_pytest():
        bootstrap(0, "", "false")
        pytest.main(argv, plugins=[])

    run_pytest()
