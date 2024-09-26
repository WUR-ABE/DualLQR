#!/usr/bin/env python

import numpy as np
from roboticstoolbox.robot.Robot import Robot
from math import pi
import os


class UR5e(Robot):
    """
    Class that imports a UR5e URDF model

    ``UR5e()`` is a class which imports a Universal Robotics UR5e robot
    definition from a URDF file.  The model describes its kinematic and
    graphical characteristics.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.URDF.UR5()
        >>> print(robot)

    Defined joint configurations are:

    - qz, zero joint angle configuration, 'L' shaped configuration
    - qr, vertical 'READY' configuration

    .. codeauthor:: Jesse Haviland
    .. sectionauthor:: Peter Corke
    """

    def __init__(self):
        file_path = os.getcwd() + "/ur5e.urdf"
        links, name, urdf_string, urdf_filepath = self.URDF_read(file_path=file_path)
        # for link in links:
        #     print(link)

        super().__init__(
            links,
            name=name.upper(),
            manufacturer="Universal Robotics",
            gripper_links=links[-1],
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

        self.qr = np.array([np.pi, 0, 0, 0, np.pi / 2, 0])
        self.qz = np.zeros(6)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)


if __name__ == "__main__":  # pragma nocover
    robot = UR5e()
    print(robot)
    print(robot.ets())
