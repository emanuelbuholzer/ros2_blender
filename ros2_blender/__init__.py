import atexit
import os
import rclpy
import rclpy.logging


def bootstrap():
    logger = rclpy.logging.get_logger(__name__)

    domain_id = int(os.getenv("ROS_DOMAIN_ID", "0"))
    logger.info(f"Initializing rclpy context with domain id {domain_id}")
    rclpy.init(domain_id=domain_id)

    if not rclpy.ok():
        raise RuntimeError("Could not initialize rclpy context")

    def atexit_shutdown():
        rclpy.try_shutdown()

    atexit.register(atexit_shutdown)