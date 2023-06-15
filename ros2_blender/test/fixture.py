from pathlib import Path

import pytest
from _pytest.fixtures import FixtureRequest


@pytest.fixture
def blender(request: FixtureRequest):
    from ros2_blender.test.addon import find_addon_in_path

    addon_paths = []
    marker = request.node.get_closest_marker("blender")
    if marker is not None:
        addon_paths_arg = marker.kwargs.get("addons")
        if not isinstance(addon_paths_arg, list):
            raise TypeError(
                "received unknown type for addons argument from blender marker"
            )
        for addon_path in addon_paths_arg:
            if not isinstance(addon_path, str) and not isinstance(addon_path, Path):
                raise TypeError(
                    "received unknown type for addon within addons argument from blender marker"
                )
            addon_paths.append(Path(addon_path))

    addons = [find_addon_in_path(addon_path) for addon_path in addon_paths]

    for addon in addons:
        addon.install()

    yield "WIP"

    for addon in addons:
        addon.uninstall()
