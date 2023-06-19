import os
from glob import glob

import unittest
from setuptools import setup

package_name = "ros2_blender"


def test_suite():
    test_loader = unittest.TestLoader()
    test_suite = test_loader.discover('test', pattern='test_*.py')
    return test_suite


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="oxb",
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
