import io
import logging
import os
import signal
import subprocess
import sys

import _pytest
import _pytest.config
import pytest

from ros2_blender import installation


@pytest.hookimpl(tryfirst=True)
def pytest_addoption(parser):
    parser.addoption("--hello", action="store_true", help="a help", required=False)


@pytest.hookimpl(tryfirst=True)
def pytest_configure(config: _pytest.config.Config):

    config.addinivalue_line(
        "markers", "blender: configure the ros2_blender pytest runner"
    )

    comm = os.path.basename(sys.argv[0])
    if comm == "blender":
        return None

    blender_executable = installation.find_blender_executable()
    logging.info(f"Using {blender_executable} to run tests with ros2_blender")

    args = [
        blender_executable,
        "--background",
        "--factory-startup",
        "--python",
        os.path.join(
            os.path.abspath(os.path.dirname(__file__)),
            "runner.py",
        ),
    ]

    with subprocess.Popen(
        args, stdout=sys.stdout, stderr=sys.stderr, env=os.environ
    ) as proc:

        def handled_exit():
            # hide "Exit:" message shown by pytest on exit
            sys.stderr = io.StringIO()
            pytest.exit(" ", returncode=proc.returncode)

        def on_sigint(signum, _frame):
            proc.send_signal(signum)
            handled_exit()

        signal.signal(signal.SIGINT, on_sigint)
        signal.signal(signal.SIGTERM, on_sigint)

        proc.communicate()

        handled_exit()
