import atexit
import rclpy
import rclpy.logging


# Compatibility to work with ros2 launch
def bootstrap_launch_compat(domain_id: int, addons_str: str, read_prefs_str: str):
    addons = []
    if addons_str != "":
        import addon_utils

        available_addons = [
            mod.bl_info.get("name", "") for mod in addon_utils.modules()
        ]

        addon_names = addons_str.split(",")
        for addon_name in addon_names:
            assert addon_name in available_addons
            addons.append(addon_name)

    read_prefs = read_prefs_str == "true"

    bootstrap(domain_id, addons, read_prefs)


def bootstrap(domain_id: int = 0, addons: [str] = [], read_prefs: bool = False):
    import addon_utils
    import bpy

    rclpy.init(domain_id=domain_id)
    if not rclpy.ok():
        raise RuntimeError("Could not initialize rclpy context")

    logger = rclpy.logging.get_logger(__name__)
    logger.info(f"Initialized rclpy with domain id {domain_id}")

    def atexit_shutdown():
        rclpy.try_shutdown()

    atexit.register(atexit_shutdown)

    for addon in addons:
        logger.info(f"enabling addon {addon}")
        addon_utils.enable(addon, default_set=True, persistent=True)

    if read_prefs:
        logger.info("Reading user preferences")
        bpy.ops.wm.read_userpref()


def teardown():
    rclpy.try_shutdown()
