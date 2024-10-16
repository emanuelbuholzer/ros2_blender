import rclpy
import pytest


@pytest.fixture
def addon_minimal(blender):
    blender.bootstrap()
    blender.install_addon("test/addon_minimal")
    yield blender
    blender.teardown()


def test_registered(addon_minimal):
    import addon_utils

    addon_names = [mod.bl_info.get("name", "") for mod in addon_utils.modules()]

    assert "addon_minimal" in addon_names


def test_enabled(addon_minimal):
    import bpy

    enabled_addon_names = bpy.context.preferences.addons.keys()

    assert "addon_minimal" in enabled_addon_names


def test_ensure_no_rclpy_shutdown_after_unregister(addon_minimal):
    import addon_utils

    addon_utils.disable("addon_minimal")

    assert rclpy.ok()

    addon_utils.enable("addon_minimal")


def test_ensure_cannot_enable_with_no_rclpy_context(addon_minimal):
    import addon_utils
    import bpy

    addon_minimal.teardown()
    addon_utils.disable("addon_minimal", default_set=True)

    assert not rclpy.ok()
    addon_utils.enable("addon_minimal", default_set=True)

    enabled_addon_names = bpy.context.preferences.addons.keys()
    assert "addon_minimal" not in enabled_addon_names


def test_restart_and_reload_preferences(addon_minimal):
    import bpy

    assert bpy.context.preferences.addons["addon_minimal"].preferences.domain_id == 0
    assert rclpy.utilities.get_default_context().get_domain_id() == 0

    bpy.context.preferences.addons["addon_minimal"].preferences.domain_id = 42
    bpy.ops.addon_minimal.restart_and_reload_preferences("EXEC_DEFAULT")

    assert bpy.context.preferences.addons["addon_minimal"].preferences.domain_id == 42
