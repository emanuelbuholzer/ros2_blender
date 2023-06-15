import subprocess
from pathlib import Path
import shutil


def find_blender_executable() -> Path:
    blender_executable_path = shutil.which("blender")
    if blender_executable_path:
        return Path(blender_executable_path)
    else:
        raise RuntimeError("blender executable could not be found")


def find_addons_directory(blender_executable_path: Path) -> Path:
    stdio = subprocess.check_output(
        [
            blender_executable_path,
            "--background",
            "--python-expr",
            "import bpy;print(bpy.utils.script_path_user())",
        ],
        stderr=subprocess.STDOUT,
    )

    scripts_dir = None
    for line in stdio.decode("utf-8").splitlines():
        if line.endswith("scripts"):
            scripts_dir = line
            break
    if scripts_dir:
        return Path(scripts_dir).joinpath("addons")
    else:
        raise RuntimeError("blender addons directory could not be found")
