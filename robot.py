import wpilib as wp
from commands2 import CommandScheduler

import constants
from RobotContainer import RobotContainer


class Robot(wp.TimedRobot):
    def __init__(self):
        super().__init__(constants.Robot.period)

    def robotInit(self):
        self.robot = RobotContainer()

    def robotPeriodic(self):
        CommandScheduler.getInstance().run()

    def autonomousInit(self):
        self.autoCommand = self.robot.getAutoCommand()

        self.autoCommand.schedule()

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        self.autoCommand.cancel()

    def teleopPeriodic(self):
        pass

    def testInit(self):
        CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self):
        pass
