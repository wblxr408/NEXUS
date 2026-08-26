from glob import glob
import os
from setuptools import find_packages, setup

package_name = "nexus_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "preflight_node = nexus_bringup.preflight_node:main",
        "uav_route_node = nexus_bringup.uav_route_node:main",
    ]},
)
