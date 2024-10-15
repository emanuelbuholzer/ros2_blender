import os
from glob import glob

from setuptools import setup

package_name = "ros2_blender"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.blend")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Emanuel Buholzer",
    maintainer_email="emanuel0xb@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    test_suite="test",
    entry_points={
        "console_scripts": [],
        "pytest11": [
            "ros2_blender_launch = ros2_blender.test.launch",
            "ros2_blender = ros2_blender.test",
        ],
    },
)
