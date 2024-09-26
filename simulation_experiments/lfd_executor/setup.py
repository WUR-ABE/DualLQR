import os
from glob import glob
from setuptools import setup, find_packages

package_name = "lfd_executor"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share/", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.urdf")),
    ],
    install_requires=["setuptools", "python-math", "numpy", "transforms3d", "pickle-mixin", "roboticstoolbox-python",],
    zip_safe=True,
    maintainer="agro-legion",
    maintainer_email="robert.vandeven@wur.nl",
    description="Packages containing scripts to test DualLQR and InfLQR in a simulation enviroment",
    license="GPL-3.0-only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lfd_controller = lfd_executor.lfd_controller:main",
            "traj_executor = lfd_executor.traj_executor:main",
            "traj_executor_reactive = lfd_executor.traj_executor_reactive:main",
            "traj_pred_gmr = lfd_executor.traj_predictor_gmr:main",
            "action_client_test = lfd_executor.action_client_test:main",
            "relaxed_frame_broadcaster = lfd_executor.relaxed_frame_broadcaster:main",
            "tcp_for_ik_broadcaster = lfd_executor.tcp_for_ik_broadcaster:main",
            "goal_pose_broadcaster = lfd_executor.goal_pose_broadcaster:main",
            "dynamic_base_frame_broadcaster = lfd_executor.dynamic_base_frame_broadcaster:main",
            "simulation_testing = lfd_executor.simulation_testing:main",
            "controlled_goal_pose_broadcaster = lfd_executor.controlled_goal_pose_broadcaster:main",
        ],
    },
)
