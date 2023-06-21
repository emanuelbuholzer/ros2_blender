import atexit
import rclpy
import rclpy.logging


def bootstrap(domain_id, addons_str: str, read_prefs: str):
    rclpy.init(domain_id=domain_id)
    if not rclpy.ok():
        raise RuntimeError("Could not initialize rclpy context")

    logger = rclpy.logging.get_logger(__name__)
    logger.info(f"Initialized rclpy with domain id {domain_id}")

    def atexit_shutdown():
        rclpy.try_shutdown()

    atexit.register(atexit_shutdown)

    if addons_str != "":
        import addon_utils

        addon_names = [mod.bl_info.get("name", "") for mod in addon_utils.modules()]

        addons = addons_str.split(",")
        logger.info(str(addons))
        for addon in addons:
            assert addon in addon_names

            logger.info(f"Enabling addon {addon}")
            addon_utils.enable(addon, default_set=True, persistent=True)

    if read_prefs == "true":
        import bpy
        logger.info("Reading user preferences")
        bpy.ops.wm.read_userpref()
