import os
from glob import glob

from setuptools import find_packages, setup


package_name = "avstack_perception"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="spencer",
    maintainer_email="20426598+roshambo919@users.noreply.github.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mmdetection2d = avstack_perception.camera.mmdetection2d:main",
            "mmdetection3d = avstack_perception.lidar.mmdetection3d:main",
            "laserscan_box_detection = avstack_perception.lidar.laserscan_box_detection:main",
            "lidar_concave_hull = avstack_perception.fov.lidar_concave_hull:main",
        ],
    },
)
