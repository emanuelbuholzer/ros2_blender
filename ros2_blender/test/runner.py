import pytest


if __name__ == "__main__":
    argv = [
        "-p",
        "no:ros2_blender.test.launch",
    ]

    def run_pytest():
        pytest.main(argv, plugins=[])

    run_pytest()
