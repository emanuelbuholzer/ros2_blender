import logging
import sys
from pathlib import Path
from ros2_blender import installation

blender_executable = Path(sys.argv[1])

m_addon_paths = sys.argv[2]
if m_addon_paths == "":
    logging.info("No addons to install")
    exit(0)

addon_paths = [Path(addon_path) for addon_path in m_addon_paths.split(",")]

addons_directory: Path = installation.find_addons_directory(blender_executable)

for addon_path in addon_paths:
    if addons_directory.joinpath(addon_path.name).is_symlink():
        print("OUuuu noed gued")

    addons_directory.joinpath(addon_path.name).symlink_to(
        addon_path.resolve(), target_is_directory=True
    )
