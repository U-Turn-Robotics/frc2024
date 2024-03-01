import wpilib as wp
from commands2 import CommandScheduler
from wpinet import PortForwarder

import constants
from RobotContainer import RobotContainer


class Robot(wp.TimedRobot):
    def __init__(self):
        super().__init__(constants.Robot.period)

        self.autoCommand = None

    def robotInit(self):
        PortForwarder.getInstance().add(5800, "photonvision.local", 5800)
        wp.CameraServer.launch()
        self.robot = RobotContainer()

    def robotPeriodic(self):
        CommandScheduler.getInstance().run()

    def autonomousInit(self):
        self.autoCommand = self.robot.getAutoCommand()

        self.autoCommand.schedule()

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        if self.autoCommand:
            self.autoCommand.cancel()

    def teleopPeriodic(self):
        self.robot.teleopPeriodic()

    def testInit(self):
        CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self):
        pass
