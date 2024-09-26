import os
from glob import glob
from setuptools import setup, find_packages

package_name = "reactive_arm_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share/", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools", "os", "yaml",],
    zip_safe=True,
    maintainer="Robert van de Ven",
    maintainer_email="robert.vandeven@wur.nl",
    description="Wrapper for launching simulated UR5e arm with custom settings",
    license="GPL-3.0-only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
        ],
    },
)
