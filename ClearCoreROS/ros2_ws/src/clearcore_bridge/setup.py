from setuptools import setup

package_name = "clearcore_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Adam G. Sweeney",
    author_email="agsweeney@gmail.com",
    description="ROS 2 joint bridge for ClearCoreROS firmware.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge = clearcore_bridge.bridge_node:main",
        ],
    },
)
