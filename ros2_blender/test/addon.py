import os
import tempfile
import zipfile
from pathlib import Path

import addon_utils
import bpy


class Addon:
    path: Path

    def __init__(self, name: str):
        self.name = name

    def install(self):
        bpy.ops.preferences.addon_install(filepath=str(self.path))
        addon_utils.enable(
            self.name, default_set=True, persistent=True
        )

    def uninstall(self):
        addon_utils.disable(module_name=self.name, default_set=False)
        try:
            bpy.ops.preferences.addon_remove(module=self.name)
        except Exception as err:
            # This happens as we use factory startup, addons still get removed
            if str(err).find("'tag_redraw'") == -1:
                raise err
            else:
                pass


class PythonModuleAddon(Addon):
    def __init__(self, path: Path):
        super().__init__(path.name.removesuffix(".py"))
        self.path = path


class ZippedAddon(Addon):
    def __init__(self, path: Path):
        super().__init__(path.name.removesuffix(".zip"))
        self.path = path


class PackageAddon(Addon):
    def __init__(self, path: Path):
        super().__init__(path.name)
        self.path = path

    def to_zipped_addon(self, path: Path) -> ZippedAddon:
        if not path.name.endswith(".zip"):
            raise Exception(
                f"the given path {path} to convert a package addon to"
                f" a zipped addon is not a zip file"
            )

        with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as zipped_addon:
            for root, _, files in os.walk(self.path):
                for file in files:
                    filepath = os.path.join(root, file)
                    zipped_addon.write(
                        filepath,
                        os.path.relpath(
                            filepath,
                            os.path.join(self.path, ".."),
                        ),
                    )

        return ZippedAddon(path)

    def install(self):
        temp_dir = Path(tempfile.mkdtemp(f"ros2_blender_{self.name}"))
        zipped_addon = self.to_zipped_addon(temp_dir.joinpath(f"{self.name}.zip"))
        zipped_addon.install()


def find_addon_in_path(path: Path) -> Addon:
    if path.is_dir():
        for child_path in [path.joinpath(p) for p in os.listdir(path)]:
            if child_path.name == "__init__.py":
                return PackageAddon(path)
        raise Exception(
            f"could not find a __init__.py within the given addon directory {path}"
        )
    elif path.is_file():
        if path.name.endswith(".zip"):
            return ZippedAddon(path)
        elif path.name.endswith(".py"):
            return PythonModuleAddon(path)
        else:
            raise Exception(f"{path} is not a *.py or *.zip file")
    else:
        raise TypeError(
            "could not determine addon type, neither a file nor a directory"
        )
