from setuptools import find_packages, setup

packages = [p for p in find_packages(where="src", exclude=["test", "launch"]) if p != "launch"]

package_name = "nexus_uwb_simulation"

setup(
    name=package_name,
    version="0.1.0",
    packages=packages,
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/uwb_simulation.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nexus",
    maintainer_email="nexus@todo.todo",
    description="NEXUS UWB simulation adapter: Gazebo GT -> ranges -> Trilateration -> ROS2.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "uwb_simulation_node = nexus_uwb_simulation.uwb_simulation_node:main",
        ],
    },
)
