import math

import wpilib as wp
from wpimath.controller import PIDController
import wpilib.drive
from navx import AHRS
import rev

from DriveSubsystem import DriveSubsystem
import constants


class Robot(wp.TimedRobot):
    def __init__(self) -> None:
        # 100hz update rate instead of default 50hz
        super().__init__(0.01)

    def robotInit(self):
        driver = wp.XboxController(0)
        self.drive = DriveSubsystem(driver)

        # self.operator = wp.XboxController(1)

    def robotPeriodic(self):
        self.drive.periodic()

    def teleopInit(self):
        pass

    def teleopPeriodic(self):

        self.drive.drive()
