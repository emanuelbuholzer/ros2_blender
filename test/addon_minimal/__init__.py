import sys

bl_info = {
    "name": "addon_minimal",
    "description": "This is a minimal addon for testing purposes within ros2_blender",
    "author": "Emanuel Buholzer",
    "version": (0, 1, 0),
    "blender": (3, 0, 0),
    "location": "View3D > Tool Shelf > addon_minimal",
    "doc_url": "https://https://github.com/emanuelbuholzer/ros2_blender",
    "tracker_url": "https://github.com/emanuelbuholzer/ros2_blender/issues",
    "support": "COMMUNITY",
    "category": "Test",
}

# Support reloading non-bpy dependent modules
if "rclpy" in locals():
    import importlib

    rclpy = importlib.reload(rclpy)  # noqa: F821
    rclpy.logging = importlib.reload(rclpy.logging)
else:
    import rclpy
    import rclpy.logging

logger = rclpy.logging.get_logger(__name__)


def register():
    # Support reloading for bpy dependent modules
    if "addon_minimal.preferences" in sys.modules:
        import importlib

        def reload_module(name):
            module_name = "%s.%s" % (__name__, name)
            module = importlib.reload(sys.modules[module_name])
            sys.modules[module_name] = module
            return module

        preferences = reload_module("preferences")
    else:
        from . import preferences

    assert rclpy.ok()

    preferences.register()


def unregister():
    from . import preferences

    preferences.unregister()
