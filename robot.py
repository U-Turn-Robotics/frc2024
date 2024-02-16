import wpilib as wp

from subsystems.DriveSubsystem import DriveSubsystem
from pilots import Driver, Operator


class Robot(wp.TimedRobot):
    def robotInit(self):
        driver = Driver()
        self.drive = DriveSubsystem(driver)

        # self.operator = wp.XboxController(1)

    def robotPeriodic(self):
        self.drive.periodic()

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        self.drive.drive()
