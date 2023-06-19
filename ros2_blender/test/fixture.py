from pathlib import Path

import pytest


class BlenderFixture:
    def __init__(self):
        self.addons = []

    def install_addon(self, addon_path: str | Path):
        from ros2_blender.test.addon import find_addon_in_path

        path: Path
        if isinstance(addon_path, str):
            path = Path(addon_path)
        elif isinstance(addon_path, Path):
            path = addon_path
        else:
            raise TypeError(
                "received unknown type for addon within addons argument from blender marker"
            )

        addon = find_addon_in_path(path)
        self.addons.append(addon)

        addon.install()

    def uninstall_addons(self):
        for addon in self.addons:
            addon.uninstall()


@pytest.fixture
def blender():
    blender_fixture = BlenderFixture()

    yield blender_fixture

    blender_fixture.uninstall_addons()
