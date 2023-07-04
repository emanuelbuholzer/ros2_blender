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
    resolved_addon_path = addon_path.resolve()
    if not resolved_addon_path.is_dir():
        egg_link_path = resolved_addon_path.parent.joinpath(
            resolved_addon_path.name.replace("_", "-") + ".egg-link"
        )
        if egg_link_path.is_file():
            base_dir = Path(egg_link_path.read_text().replace("\n.", ""))
            resolved_addon_path = base_dir.joinpath(resolved_addon_path.name)

    addons_directory.joinpath(addon_path.name).symlink_to(
        resolved_addon_path, target_is_directory=True
    )
